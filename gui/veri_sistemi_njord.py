import copy
import importlib.util
import ipaddress
import math
import platform
import re
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock, Thread

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage
from pymavlink import mavutil

try:
    from gui.backend_client_njord import BackendClient, BackendClientError
except ImportError:
    from backend_client_njord import BackendClient, BackendClientError


ARDUROVER_MODS = {
    0: "MANUAL",
    1: "ACRO",
    2: "LEARNING",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    8: "DOCK",
    9: "CIRCLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}
MODE_NAME_TO_ID = {name: mode_id for mode_id, name in ARDUROVER_MODS.items()}

HEARTBEAT_TIMEOUT = 5.0

JETSON_IP = None
JETSON_MAC = "8c:b8:7e:04:20:a9"
JETSON_VIDEO_PORT = 5000
NETWORK_SCAN_INTERVAL = 30.0


def _get_grap_video_path():
    from pathlib import Path

    repo_root = Path(__file__).resolve().parent.parent
    candidates = [
        repo_root / "Fetch Data" / "grap_video.py",
        repo_root / "fetch_data" / "grap_video.py",
        repo_root / "Fetch_Data" / "grap_video.py",
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def _load_grap_video():
    try:
        import importlib as _importlib

        client_side = _importlib.import_module("zed.task1.client_side")
        return getattr(client_side, "grap_video", client_side)
    except Exception:
        pass

    gv_path = _get_grap_video_path()
    if not gv_path:
        return None

    try:
        spec = importlib.util.spec_from_file_location("grap_video", str(gv_path))
        if spec is None or spec.loader is None:
            return None
        module = importlib.util.module_from_spec(spec)
        sys.modules["grap_video"] = module
        spec.loader.exec_module(module)
        return module
    except Exception:
        return None


grap_video = _load_grap_video()


class NjordVeriSistemi(QObject):
    veri_guncelle = pyqtSignal(dict)
    log_sinyali = pyqtSignal(str)
    kamera_sinyali = pyqtSignal(QImage)
    baglanti_kesildi = pyqtSignal()
    waypoint_guncelle = pyqtSignal(list)

    def __init__(self):
        super().__init__()

        self.connection = None
        self._lock = Lock()
        self._aktif = True
        self._last_hb = 0.0
        self._watchdog_started = False

        self._camera_started = False
        self._camera_running = False
        self._last_network_scan = 0.0
        self._last_logged_jetson_ip = None
        self._last_video_wait_log = 0.0
        self.backend_client = BackendClient()
        self._backend_mission_id = None
        self._mission_waypoints = []

        self._durum = {
            "baglanti": False,
            "armed": False,
            "mod": "UNKNOWN",
            "mod_id": -1,
            "hiz": 0.0,
            "yaw": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "gps": 0,
            "gps_uydu": 0,
            "mesafe": 0.0,
            "decision_log": "System ready. Waiting for connection...",
            "active_mission": None,
            "link_ok": False,
            "arm_change_pending": False,
            "requested_arm_state": False,
            "mode_change_pending": False,
            "requested_mode": -1,
            "wifi_aktif": False,
            "jetson_ip": "Searching...",
            "battery": {
                "total_voltage": 0.0,
                "current": 0.0,
                "percentage": 0,
            },
        }

        Thread(target=self._wifi_kontrol_dongusu, daemon=True, name="WiFiWatch").start()

    def _set(self, **kwargs):
        with self._lock:
            self._durum.update(kwargs)
        self._emit_durum()

    def _snapshot(self):
        with self._lock:
            return copy.deepcopy(self._durum)

    def gorev_noktalarini_al(self):
        with self._lock:
            return copy.deepcopy(self._mission_waypoints)

    def gorev_noktalarini_guncelle(self, waypoints):
        normalized = []
        for index, item in enumerate(waypoints, start=1):
            if isinstance(item, dict):
                name = item.get("name") or item.get("id") or item.get("label") or f"WP_{index:02d}"
                lat = item.get("lat", item.get("latitude"))
                lon = item.get("lon", item.get("lng", item.get("longitude")))
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                name, lat, lon = item[0], item[1], item[2]
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                name, lat, lon = f"WP_{index:02d}", item[0], item[1]
            else:
                continue

            try:
                normalized.append(
                    {
                        "name": str(name),
                        "lat": float(lat),
                        "lon": float(lon),
                    }
                )
            except (TypeError, ValueError):
                continue

        with self._lock:
            self._mission_waypoints = normalized
        self.waypoint_guncelle.emit(copy.deepcopy(normalized))

    def _emit_durum(self):
        with self._lock:
            kopya = copy.deepcopy(self._durum)

        battery = kopya.get("battery", {})
        kopya["voltaj"] = battery.get("total_voltage", 0.0)
        kopya["akim"] = battery.get("current", 0.0)
        kopya["pil_yuzde"] = battery.get("percentage", 0)
        self.veri_guncelle.emit(kopya)

    def _log(self, mesaj):
        print(f"[NJORD] {mesaj}")
        self.log_sinyali.emit(mesaj)

    def _command_output(self, command):
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                creationflags=self._subprocess_flags(),
            )
        except Exception:
            return None

        if platform.system().lower() == "windows":
            return result.stdout.decode("cp857", errors="replace")

        return result.stdout.decode(errors="replace")

    def get_ip_from_mac(self, target_mac):
        return self._arp_cache_ip(target_mac)

    def _arp_cache_ip(self, target_mac):
        normalized_mac = target_mac.lower().replace("-", ":")

        result = self._command_output(["arp", "-a"])
        if result is None:
            return None

        for line in result.splitlines():
            normalized_line = line.lower().replace("-", ":")
            if normalized_mac in normalized_line:
                ip_match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
                if ip_match:
                    return ip_match.group()
        return None

    def _local_ipv4_networks(self):
        result = self._command_output(["ipconfig"])
        if result is None:
            return []

        networks = []
        current_ip = None
        for line in result.splitlines():
            ipv4_match = re.search(r"IPv4.*?:\s*([0-9.]+)", line, re.IGNORECASE)
            mask_match = None
            if "mask" in line.lower() or "maske" in line.lower():
                mask_match = re.search(r":\s*([0-9.]+)", line)

            if ipv4_match:
                current_ip = ipv4_match.group(1)
            elif current_ip and mask_match:
                try:
                    network = ipaddress.IPv4Network(
                        f"{current_ip}/{mask_match.group(1)}",
                        strict=False,
                    )
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass
                current_ip = None

        if current_ip:
            try:
                networks.append(ipaddress.IPv4Network(f"{current_ip}/24", strict=False))
            except Exception:
                pass

        return networks

    def _scan_network_for_mac(self, target_mac):
        networks = self._local_ipv4_networks()
        if not networks:
            return None

        for network in networks:
            hosts = [str(ip) for ip in network.hosts()]
            if len(hosts) > 4096:
                continue

            self._log(f"Searching Jetson MAC on network: {network}")
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = [executor.submit(self._ping_ip, ip) for ip in hosts]
                for future in as_completed(futures):
                    future.result()

            ip = self._arp_cache_ip(target_mac)
            if ip:
                self._log(f"Jetson MAC matched IP: {ip}")
                return ip

            self._log(f"Jetson MAC not found on network: {network}")

        return None

    def _find_video_stream_ip(self):
        networks = self._local_ipv4_networks()
        if not networks:
            return None

        for network in networks:
            hosts = [str(ip) for ip in network.hosts()]
            if len(hosts) > 4096:
                self._log(f"Skipping large network for video scan: {network}")
                continue

            self._log(f"Searching Jetson video service on {network}")
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = {
                    executor.submit(self._is_port_open, ip, JETSON_VIDEO_PORT): ip
                    for ip in hosts
                }
                for future in as_completed(futures):
                    ip = futures[future]
                    try:
                        if future.result():
                            self._log(f"Jetson video service found at IP: {ip}")
                            return ip
                    except Exception:
                        pass

            self._log(f"Jetson video service not found on {network}")

        return None

    def _is_port_open(self, ip, port):
        try:
            with socket.create_connection((ip, port), timeout=0.25):
                return True
        except OSError:
            return False

    def _jetson_reachable(self, ip):
        return self._ping_ip(ip) or self._is_port_open(ip, JETSON_VIDEO_PORT)

    def _valid_ip(self, value):
        return bool(re.fullmatch(r"(?:[0-9]{1,3}\.){3}[0-9]{1,3}", str(value)))

    def _subprocess_flags(self):
        if platform.system().lower() == "windows":
            return getattr(subprocess, "CREATE_NO_WINDOW", 0)
        return 0

    def _ping_ip(self, ip):
        if not ip:
            return False

        if platform.system().lower() == "windows":
            command = ["ping", "-n", "1", "-w", "1000", ip]
        else:
            command = ["ping", "-c", "1", "-W", "1", ip]

        try:
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=self._subprocess_flags(),
            )
            return result.returncode == 0
        except Exception:
            return False

    def _wifi_kontrol_dongusu(self):
        while self._aktif:
            bulunan_ip = None

            if JETSON_IP and self._ping_ip(JETSON_IP):
                self._set(wifi_aktif=True, jetson_ip=JETSON_IP)
            else:
                bulunan_ip = self.get_ip_from_mac(JETSON_MAC)
                if bulunan_ip and bulunan_ip != self._last_logged_jetson_ip:
                    self._last_logged_jetson_ip = bulunan_ip
                    self._log(f"Jetson found in ARP cache: {bulunan_ip}")
                if not bulunan_ip and time.time() - self._last_network_scan > NETWORK_SCAN_INTERVAL:
                    self._last_network_scan = time.time()
                    bulunan_ip = self._scan_network_for_mac(JETSON_MAC)
                    if not bulunan_ip:
                        bulunan_ip = self._find_video_stream_ip()

                if bulunan_ip and self._jetson_reachable(bulunan_ip):
                    self._set(wifi_aktif=True, jetson_ip=bulunan_ip)
                else:
                    self._set(wifi_aktif=False, jetson_ip=bulunan_ip or "Not found")

            time.sleep(2.0)

    def baglanti_kur(self, tip, baud, port):
        self._aktif = True
        self._log(f"CONNECTION STARTING: {tip} -> {port}")

        try:
            if tip == "UDP":
                address = f"udp:{port}" if ":" in port else f"udp:127.0.0.1:{port}"
                self.connection = mavutil.mavlink_connection(address)
            elif tip == "TCP":
                self.connection = mavutil.mavlink_connection(f"tcp:{port}")
            else:
                self.connection = mavutil.mavlink_connection(port, baud=int(baud))

            self._log("Waiting for Pixhawk heartbeat...")

            Thread(target=self._dinleme_dongusu, daemon=True, name="MAVLink").start()
            if not self._watchdog_started:
                self._watchdog_started = True
                Thread(target=self._guvenlik_dongusu, daemon=True, name="Watchdog").start()

            self._set(
                baglanti=True,
                decision_log="Connection established. Waiting for heartbeat...",
            )
            self._log("Telemetry connection started.")
            self._kamera_baslat()

        except Exception as exc:
            self._log(f"CONNECTION ERROR: {exc}")
            self._set(
                baglanti=False,
                link_ok=False,
                decision_log=f"CONNECTION ERROR: {exc}",
            )

    def baglanti_kes(self):
        self._aktif = False
        self._camera_running = False
        self._camera_started = False
        self._watchdog_started = False

        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None

        self._set(
            baglanti=False,
            link_ok=False,
            mod="UNKNOWN",
            mod_id=-1,
            armed=False,
            decision_log="CONNECTION CLOSED",
        )
        self._log("CONNECTION CLOSED")

    def kapat(self):
        self.baglanti_kes()

    def kamera_oto_baslat(self):
        self._kamera_baslat()

    def _kamera_baslat(self):
        if self._camera_started:
            return

        self._camera_started = True
        self._camera_running = True
        Thread(target=self._kamera_dongusu, daemon=True, name="Camera").start()

    def _kamera_dongusu(self):
        if grap_video is None:
            self._log("CAMERA WARNING: grap_video was not found. ZED2 stream could not start.")
            self._set(decision_log="Camera could not start: grap_video was not found.")
            self._camera_started = False
            self._camera_running = False
            return

        try:
            self._log("Camera waiting for Jetson IP...")
            jetson_ip = None
            while self._camera_running:
                with self._lock:
                    candidate_ip = self._durum.get("jetson_ip")
                    wifi_active = self._durum.get("wifi_aktif")

                if wifi_active and self._valid_ip(candidate_ip):
                    jetson_ip = candidate_ip
                    break

                time.sleep(1.0)

            if not jetson_ip:
                self._set(decision_log="Camera stopped before Jetson IP was found.")
                return

            while self._camera_running and not self._is_port_open(jetson_ip, JETSON_VIDEO_PORT):
                now = time.time()
                if now - self._last_video_wait_log > 5.0:
                    self._last_video_wait_log = now
                    self._log(
                        f"Waiting for Jetson video service: "
                        f"http://{jetson_ip}:{JETSON_VIDEO_PORT}/video_feed"
                    )
                    self._set(
                        decision_log=(
                            f"Jetson IP found ({jetson_ip}), "
                            f"waiting for video service on port {JETSON_VIDEO_PORT}..."
                        )
                    )
                time.sleep(1.0)

            if not self._camera_running:
                self._set(decision_log="Camera stopped before video service opened.")
                return

            self._set(decision_log=f"Camera connecting to Jetson IP: {jetson_ip}")

            try:
                grap_video.start(
                    jetson_ip=jetson_ip,
                    frame_callback=self.kamera_sinyali.emit,
                    log_callback=self._log,
                    stop_callback=lambda: self._camera_running,
                )
            except TypeError:
                grap_video.start(
                    frame_callback=self.kamera_sinyali.emit,
                    log_callback=self._log,
                    stop_callback=lambda: self._camera_running,
                )

        except Exception as exc:
            self._log(f"CAMERA THREAD ERROR: {exc}")
            self._set(decision_log=f"Camera error: {exc}")

        finally:
            self._camera_running = False
            self._camera_started = False
            self._log("CAMERA OFFLINE")

    def _dinleme_dongusu(self):
        while self._aktif and self.connection:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=0.1)
                if msg:
                    self._islenmis_mesaj(msg)
            except Exception:
                pass

    def _guvenlik_dongusu(self):
        while self._aktif:
            time.sleep(1)

            with self._lock:
                gecen_sure = time.time() - self._last_hb
                baglanti_var = self._durum["baglanti"]

            if baglanti_var and gecen_sure > HEARTBEAT_TIMEOUT:
                self._log("!!! WARNING: HEARTBEAT LOST !!!")
                self._set(
                    baglanti=False,
                    link_ok=False,
                    decision_log=f"TELEMETRY LOST! No heartbeat for {HEARTBEAT_TIMEOUT:.0f}s.",
                )
                self.baglanti_kesildi.emit()
            elif not baglanti_var and gecen_sure <= HEARTBEAT_TIMEOUT and self._last_hb > 0:
                self._log("Heartbeat restored. Connection is stable.")
                self._set(baglanti=True, link_ok=True)

        self._watchdog_started = False

    def _islenmis_mesaj(self, msg):
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            self._last_hb = time.time()
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mod_id = int(msg.custom_mode)
            mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")

            with self._lock:
                if self._durum.get("arm_change_pending"):
                    if self._durum.get("requested_arm_state") == armed:
                        self._durum["arm_change_pending"] = False
                        durum_str = "ARMED (ACTIVE)" if armed else "DISARMED (LOCKED)"
                        self._durum["decision_log"] = f"CONFIRMED: {durum_str}"
                        self._log(f"SUCCESS: VEHICLE IS NOW {durum_str}")

                if self._durum.get("mode_change_pending"):
                    if self._durum.get("requested_mode") == mod_id:
                        self._durum["mode_change_pending"] = False
                        self._durum["decision_log"] = f"CONFIRMED: {mod_name} MODE ACTIVE"
                        self._log(f"SUCCESS: MODE CHANGED TO {mod_name}")

            self._set(
                armed=armed,
                mod_id=mod_id,
                mod=mod_name,
                baglanti=True,
                link_ok=True,
            )

        elif msg_type == "VFR_HUD":
            self._set(yaw=msg.heading, hiz=msg.groundspeed)

        elif msg_type == "ATTITUDE":
            self._set(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch))

        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            self._set(mesafe=msg.wp_dist)

        elif msg_type == "GLOBAL_POSITION_INT":
            self._set(lat=msg.lat / 1e7, lon=msg.lon / 1e7)

        elif msg_type == "GPS_RAW_INT":
            self._set(gps=msg.fix_type, gps_uydu=msg.satellites_visible)

        elif msg_type == "SYS_STATUS":
            with self._lock:
                self._durum["battery"]["total_voltage"] = msg.voltage_battery / 1000.0
                self._durum["battery"]["current"] = msg.current_battery / 100.0
                self._durum["battery"]["percentage"] = msg.battery_remaining
            self._emit_durum()

    def _komut_gonder(
        self,
        komut_id,
        p1=0.0,
        p2=0.0,
        p3=0.0,
        p4=0.0,
        p5=0.0,
        p6=0.0,
        p7=0.0,
    ):
        if not self.connection:
            self._log("ERROR: No connection. Command was not sent.")
            return False

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            komut_id,
            0,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        )
        return True

    def arm_yap(self):
        try:
            response = self._backend_arm_ayarla(True)
            self._handle_backend_command_response(response, "ARM command")
        except BackendClientError as exc:
            self._log(f"ERROR: ARM command failed: {exc}")

    def _arm_yap_mavlink_fallback(self):
        with self._lock:
            if self._durum["mod"] == "EMERGENCY":
                self._log("ERROR: Reset emergency state before arming.")
                return
            if self._durum["armed"]:
                self._log("INFO: Vehicle is already armed.")
                return

        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            self._set(
                requested_arm_state=True,
                arm_change_pending=True,
                decision_log="ARM command sent. Waiting for heartbeat confirmation...",
            )
            self._log("ARM command sent -> waiting for heartbeat confirmation")

    def disarm_yap(self):
        try:
            response = self._backend_arm_ayarla(False)
            self._handle_backend_command_response(response, "DISARM command")
        except BackendClientError as exc:
            self._log(f"ERROR: DISARM command failed: {exc}")

    def _disarm_yap_mavlink_fallback(self):
        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0):
            self._set(
                requested_arm_state=False,
                arm_change_pending=True,
                hiz=0.0,
                decision_log="DISARM command sent. Waiting for heartbeat confirmation...",
            )
            self._log("DISARM command sent -> waiting for heartbeat confirmation")

    def mod_ayarla(self, mod_id):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        try:
            response = self._backend_mod_ayarla(mod_name)
            self._handle_backend_command_response(response, f"MODE {mod_name} command")
            self._set(
                requested_mode=mod_id,
                mode_change_pending=True,
                decision_log=f"MODE CHANGE REQUESTED: {mod_name}",
            )
        except BackendClientError as exc:
            self._log(f"ERROR: MODE {mod_name} command failed: {exc}")

    def mod_ayarla_ad(self, mod_name):
        mode_id = MODE_NAME_TO_ID.get(str(mod_name).upper())
        if mode_id is None:
            self._log(f"ERROR: Unknown mode selected: {mod_name}")
            return
        self.mod_ayarla(mode_id)

    def _mod_ayarla_mavlink_fallback(self, mod_id):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if not self.connection:
            self._log("ERROR: No connection for mode change.")
            return

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mod_id,
        )
        self._set(
            requested_mode=mod_id,
            mode_change_pending=True,
            decision_log=f"MODE CHANGE: {mod_name} | Waiting for heartbeat confirmation...",
        )
        self._log(f"MODE command sent: {mod_name} ({mod_id})")

    def _backend_arm_ayarla(self, armed):
        jetson_ip = self._require_backend_ip()
        self._log(f"Sending backend ARM command: {armed}")
        return self.backend_client.set_arm(armed, jetson_ip=jetson_ip)

    def _backend_mod_ayarla(self, mode):
        jetson_ip = self._require_backend_ip()
        self._log(f"Sending backend MODE command: {mode}")
        return self.backend_client.set_mode(mode, jetson_ip=jetson_ip)

    def _backend_acil_durum(self):
        jetson_ip = self._require_backend_ip()
        self._log("Sending backend EMERGENCY STOP command")
        return self.backend_client.emergency_stop(jetson_ip=jetson_ip)

    def _require_backend_ip(self):
        with self._lock:
            jetson_ip = self._durum.get("jetson_ip")
            wifi_aktif = self._durum.get("wifi_aktif")

        if not wifi_aktif or not self._valid_ip(jetson_ip):
            raise BackendClientError("Jetson IP is not ready. Wait for Wi-Fi/Jetson connection first.")
        return jetson_ip

    def _handle_backend_command_response(self, response, label):
        ok = bool(response.get("ok", response.get("success", False)))
        message = response.get("message") or response.get("detail") or f"{label} completed."
        if ok:
            self._log(f"SUCCESS: {message}")
        else:
            self._log(f"ERROR: {message}")

    def gorev_baslat(self, gorev_adi):
        try:
            response = self.gorev_backend_baslat(gorev_adi)
        except BackendClientError as exc:
            self._log(f"ERROR: Mission start failed: {exc}")
            return

        ok = bool(response.get("ok", response.get("success", False)))
        message = response.get("message") or response.get("detail") or "Mission start request completed."
        if ok:
            self._set(
                active_mission=gorev_adi,
                decision_log=f"MISSION STARTED BY BACKEND: {gorev_adi}",
            )
            self._log(f"SUCCESS: {message}")
        else:
            self._log(f"ERROR: {message}")

    def _gorev_baslat_mavlink_fallback(self, gorev_adi):
        with self._lock:
            armed = self._durum["armed"]

        if not armed:
            self._log("ERROR: Arm the vehicle before switching to AUTO.")
            return

        self.mod_ayarla(10)
        self._set(
            active_mission=gorev_adi,
            decision_log=(
                f"MISSION STARTED: {gorev_adi} | "
                "AUTO mode transition command sent..."
            ),
        )
        self._log(f"MISSION STARTED: {gorev_adi}")

    def gorev_txt_yukle(self, txt_yolu, mission_name=None):
        jetson_ip = self._require_backend_ip()

        self._log(f"Uploading mission TXT to backend: {txt_yolu}")
        response = self.backend_client.upload_mission_txt(
            txt_yolu,
            jetson_ip=jetson_ip,
            mission_name=mission_name,
        )

        ok = bool(response.get("ok", response.get("success", False)))
        message = response.get("message") or response.get("detail") or "Mission TXT upload completed."
        if ok:
            self._backend_mission_id = (
                response.get("mission_id")
                or response.get("id")
                or response.get("mission", {}).get("id")
            )
            self._log(f"SUCCESS: {message}")
        else:
            self._log(f"ERROR: {message}")

        return response

    def gorev_backend_baslat(self, gorev_adi):
        jetson_ip = self._require_backend_ip()

        self._log(f"Starting mission through backend: {gorev_adi}")
        return self.backend_client.start_mission(
            gorev_adi,
            mission_id=self._backend_mission_id,
            jetson_ip=jetson_ip,
        )

    def acil_durum(self):
        try:
            response = self._backend_acil_durum()
            self._handle_backend_command_response(response, "EMERGENCY STOP")
            self._set(mod="EMERGENCY", decision_log="!!! EMERGENCY STOP REQUESTED !!!")
        except BackendClientError as exc:
            self._log(f"ERROR: EMERGENCY STOP failed: {exc}")

    def _acil_durum_mavlink_fallback(self):
        self._disarm_yap_mavlink_fallback()
        self._set(mod="EMERGENCY", decision_log="!!! EMERGENCY STOP ACTIVE !!!")
        self._log("!!! EMERGENCY STOP !!!")

    def durum_al(self):
        return self._snapshot()

    def update_battery(self, battery_data):
        with self._lock:
            self._durum["battery"].update(battery_data)
        self._emit_durum()
