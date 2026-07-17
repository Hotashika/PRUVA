import copy
import ipaddress
import math
import platform
import re
import socket
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from threading import Lock, Thread

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage
from pymavlink import mavutil

from ..services.backend_client import BackendClient
from ..streaming import video_client as grap_video


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

HEARTBEAT_TIMEOUT = 15.0
ARDUPILOT_FORCE_ARM_MAGIC = 21196

JETSON_IP = None
JETSON_MAC = "8c:b8:7e:04:20:a9"
JETSON_VIDEO_PORT = 5000
JETSON_BACKEND_PORT = 8000
NETWORK_SCAN_INTERVAL = 30.0




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
        self._connection_started_at = 0.0
        self._heartbeat_seen = False
        self._telemetry_lost_reported = False
        self._watchdog_started = False
        self._streams_requested = False
        self._seen_message_types = set()
        self._mission_messages = []
        self._last_position_for_cog = None
        self._last_radio_failsafe = 0.0

        self._camera_started = False
        self._camera_running = False
        self._last_network_scan = 0.0
        self._last_logged_jetson_ip = None
        self._last_video_wait_log = 0.0
        self.backend_client = BackendClient()
        self._mission_id = None
        self._mission_uploaded_to_pixhawk = False
        self._mission_component_zero = False
        self._mission_waypoints = []

        self._durum = {
            "baglanti": False,
            "armed": False,
            "mod": "UNKNOWN",
            "mod_id": -1,
            "system_status": "UNKNOWN",
            "hiz": 0.0,
            "yaw": 0.0,
            "cog": 0.0,
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
            "heartbeat_seen": False,
            "telemetry_lost": False,
            "radio_failsafe": False,
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
                "power_w": 0.0,
                "remaining_wh": 0.0,
                "capacity_wh": 0.0,
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

    def _battery_state(self):
        battery = self._durum.get("battery")
        if not isinstance(battery, dict):
            battery = {}
            self._durum["battery"] = battery

        defaults = {
            "total_voltage": 0.0,
            "current": 0.0,
            "percentage": 0,
            "power_w": 0.0,
            "remaining_wh": 0.0,
            "capacity_wh": 0.0,
        }
        for key, value in defaults.items():
            battery.setdefault(key, value)
        return battery

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
            self._battery_state()
            kopya = copy.deepcopy(self._durum)

        battery = kopya.get("battery") or {}
        kopya["voltaj"] = battery.get("total_voltage", 0.0)
        kopya["akim"] = battery.get("current", 0.0)
        kopya["pil_yuzde"] = battery.get("percentage", 0)
        kopya["power_w"] = battery.get("power_w", 0.0)
        kopya["remaining_wh"] = battery.get("remaining_wh", 0.0)
        kopya["capacity_wh"] = battery.get("capacity_wh", 0.0)
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

    @staticmethod
    def _ipv4_from_text(text):
        return re.findall(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", text or "")

    @staticmethod
    def _normalize_mac(value):
        return str(value or "").lower().replace("-", ":")

    def _arp_cache_ip(self, target_mac):
        normalized_mac = self._normalize_mac(target_mac)

        outputs = []
        for command in (["arp", "-a"], ["ip", "neigh"], ["ip", "neighbor"], ["netsh", "interface", "ip", "show", "neighbors"]):
            result = self._command_output(command)
            if result:
                outputs.append(result)

        for output in outputs:
            for line in output.splitlines():
                normalized_line = self._normalize_mac(line)
                if normalized_mac in normalized_line:
                    ips = self._ipv4_from_text(line)
                    if ips:
                        return ips[0]
        return None

    def _local_ipv4_networks(self):
        system = platform.system().lower()
        if system == "windows":
            return self._local_ipv4_networks_windows()
        return self._local_ipv4_networks_posix()

    def _local_ipv4_networks_windows(self):
        result = self._command_output(["ipconfig"])
        networks = []
        current_ip = None
        for line in (result or "").splitlines():
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

    def _local_ipv4_networks_posix(self):
        networks = []
        ip_output = self._command_output(["ip", "-4", "addr", "show"])
        if ip_output:
            for match in re.finditer(r"\binet\s+([0-9.]+/\d+)", ip_output):
                try:
                    network = ipaddress.IPv4Network(match.group(1), strict=False)
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass

        if not networks:
            ifconfig_output = self._command_output(["ifconfig"])
            for match in re.finditer(
                r"inet\s+(?:addr:)?([0-9.]+).*?(?:netmask\s+(0x[0-9a-fA-F]+|[0-9.]+))?",
                ifconfig_output or "",
            ):
                ip = match.group(1)
                mask = match.group(2) or "255.255.255.0"
                if mask.startswith("0x"):
                    mask_int = int(mask, 16)
                    mask = ".".join(str((mask_int >> shift) & 0xFF) for shift in (24, 16, 8, 0))
                try:
                    network = ipaddress.IPv4Network(f"{ip}/{mask}", strict=False)
                    if not network.is_loopback:
                        networks.append(network)
                except Exception:
                    pass

        unique = []
        seen = set()
        for network in networks:
            if network not in seen:
                unique.append(network)
                seen.add(network)
        return unique

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
        # Avoid showing "Wi-Fi active" just because a stale ARP/ping entry exists.
        # Jetson is treated as active only when a service used by the GUI is open.
        backend_port = int(self.backend_client.config.get("backend", {}).get("http_port", JETSON_BACKEND_PORT))
        return self._is_port_open(ip, JETSON_VIDEO_PORT) or self._is_port_open(ip, backend_port)

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

            if JETSON_IP and self._jetson_reachable(JETSON_IP):
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
            self._last_hb = 0.0
            self._connection_started_at = time.time()
            self._heartbeat_seen = False
            self._telemetry_lost_reported = False
            self._streams_requested = False

            Thread(target=self._dinleme_dongusu, daemon=True, name="MAVLink").start()
            if not self._watchdog_started:
                self._watchdog_started = True
                Thread(target=self._guvenlik_dongusu, daemon=True, name="Watchdog").start()

            self._set(
                baglanti=True,
                link_ok=False,
                heartbeat_seen=False,
                telemetry_lost=False,
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
            heartbeat_seen=False,
            telemetry_lost=False,
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
            except Exception as exc:
                self._log(f"WARNING: MAVLink read error: {exc}")

    def _guvenlik_dongusu(self):
        while self._aktif:
            time.sleep(1)

            with self._lock:
                gecen_sure = time.time() - self._last_hb
                baglanti_var = self._durum["baglanti"]
                heartbeat_seen = self._heartbeat_seen
                baslangictan_beri = time.time() - self._connection_started_at if self._connection_started_at else 0.0
                telemetry_lost_reported = self._telemetry_lost_reported
                radio_failsafe = bool(self._durum.get("radio_failsafe"))

            if baglanti_var and not heartbeat_seen and baslangictan_beri > HEARTBEAT_TIMEOUT:
                if not telemetry_lost_reported:
                    self._log(
                        "ERROR: No Pixhawk heartbeat received. Check COM port, baud rate, telemetry radio power, and close Mission Planner."
                    )
                self._set(
                    link_ok=False,
                    heartbeat_seen=False,
                    telemetry_lost=True,
                    decision_log="No Pixhawk heartbeat. Check COM/baud and telemetry radio.",
                )
                self._telemetry_lost_reported = True
            elif baglanti_var and heartbeat_seen and gecen_sure > HEARTBEAT_TIMEOUT:
                if not telemetry_lost_reported:
                    self._log("!!! WARNING: HEARTBEAT LOST !!!")
                self._set(
                    baglanti=True,
                    link_ok=False,
                    heartbeat_seen=True,
                    telemetry_lost=True,
                    decision_log=f"TELEMETRY LOST! No heartbeat for {HEARTBEAT_TIMEOUT:.0f}s.",
                )
                self._telemetry_lost_reported = True
            elif not baglanti_var and gecen_sure <= HEARTBEAT_TIMEOUT and self._last_hb > 0:
                self._log("Heartbeat restored. Connection is stable.")
                self._set(baglanti=True, link_ok=True, heartbeat_seen=True, telemetry_lost=False)
            elif radio_failsafe and self._last_radio_failsafe and time.time() - self._last_radio_failsafe > 12.0:
                self._set(
                    radio_failsafe=False,
                    system_status="MAV_STATE_ACTIVE",
                    decision_log="Radio failsafe message cleared. Vehicle commands can be retried.",
                )

        self._watchdog_started = False

    def _islenmis_mesaj(self, msg):
        msg_type = msg.get_type()
        if msg_type != "BAD_DATA" and msg_type not in self._seen_message_types:
            self._seen_message_types.add(msg_type)
            if len(self._seen_message_types) <= 20:
                self._log(f"MAVLink message received: {msg_type}")

        if msg_type == "HEARTBEAT":
            self._last_hb = time.time()
            self._heartbeat_seen = True
            self._telemetry_lost_reported = False
            if not self._streams_requested:
                self._mavlink_streamlerini_iste()

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mod_id = int(msg.custom_mode)
            mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
            system_status = self._mavlink_state_name(getattr(msg, "system_status", None))

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
                system_status=system_status,
                baglanti=True,
                link_ok=True,
                heartbeat_seen=True,
                telemetry_lost=False,
            )

        elif msg_type == "VFR_HUD":
            self._set(yaw=msg.heading, hiz=msg.groundspeed)

        elif msg_type == "ATTITUDE":
            self._set(
                roll=math.degrees(msg.roll),
                pitch=math.degrees(msg.pitch),
                yaw=(math.degrees(msg.yaw) + 360.0) % 360.0,
            )

        elif msg_type in ("AHRS", "AHRS2"):
            updates = {}
            if hasattr(msg, "roll"):
                updates["roll"] = math.degrees(msg.roll)
            if hasattr(msg, "pitch"):
                updates["pitch"] = math.degrees(msg.pitch)
            if hasattr(msg, "yaw"):
                updates["yaw"] = (math.degrees(msg.yaw) + 360.0) % 360.0
            lat_raw = getattr(msg, "lat", None)
            lon_raw = getattr(msg, "lng", getattr(msg, "lon", None))
            if lat_raw is not None and lon_raw is not None:
                lat = float(lat_raw) / 1e7 if abs(float(lat_raw)) > 1000 else float(lat_raw)
                lon = float(lon_raw) / 1e7 if abs(float(lon_raw)) > 1000 else float(lon_raw)
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    updates["lat"] = lat
                    updates["lon"] = lon
            if updates:
                self._set(**updates)

        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            self._set(mesafe=msg.wp_dist)

        elif msg_type == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            cog = self._cog_hesapla(lat, lon)
            updates = {"lat": lat, "lon": lon}
            hdg = getattr(msg, "hdg", 65535)
            if hdg != 65535:
                updates["yaw"] = float(hdg) / 100.0
            if cog is not None:
                updates["cog"] = cog
            self._set(**updates)

        elif msg_type == "GPS_RAW_INT":
            updates = {
                "gps": msg.fix_type,
                "gps_uydu": msg.satellites_visible,
            }
            cog_raw = getattr(msg, "cog", 65535)
            if cog_raw != 65535:
                updates["cog"] = float(cog_raw) / 100.0
            self._set(**updates)

        elif msg_type == "SYS_STATUS":
            with self._lock:
                battery = self._battery_state()
                voltage = msg.voltage_battery / 1000.0
                current = msg.current_battery / 100.0
                battery["total_voltage"] = voltage
                battery["current"] = current
                battery["percentage"] = msg.battery_remaining
                battery["power_w"] = voltage * current
                capacity_wh = float(battery.get("capacity_wh", 0.0) or 0.0)
                if capacity_wh > 0:
                    battery["remaining_wh"] = capacity_wh * max(0, min(msg.battery_remaining, 100)) / 100.0
            self._emit_durum()

        elif msg_type == "COMMAND_ACK":
            command = getattr(msg, "command", None)
            result = getattr(msg, "result", None)
            command_name = self._mavlink_command_name(command)
            result_name = self._mavlink_result_name(result)
            self._log(f"COMMAND ACK: {command_name} -> {result_name}")

        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            text = str(text).strip("\x00").strip()
            if text:
                if text.upper().startswith("JETSON:"):
                    self._log(text)
                else:
                    self._log(f"PIXHAWK: {text}")
                lower_text = text.lower()
                if "radio failsafe" in lower_text:
                    self._last_radio_failsafe = time.time()
                    self._set(
                        radio_failsafe=True,
                        system_status="FAILSAFE_RADIO",
                        decision_log=(
                            "RADIO FAILSAFE ACTIVE: Mission start is blocked. "
                            "Check RC receiver or Pixhawk failsafe settings in Mission Planner."
                        ),
                    )

        elif msg_type in (
            "MISSION_COUNT",
            "MISSION_ITEM",
            "MISSION_ITEM_INT",
            "MISSION_REQUEST",
            "MISSION_REQUEST_INT",
            "MISSION_ACK",
            "PARAM_VALUE",
        ):
            with self._lock:
                self._mission_messages.append(msg)
                self._mission_messages = self._mission_messages[-30:]

    def _mavlink_streamlerini_iste(self):
        if not self.connection:
            return

        self._streams_requested = True
        try:
            target_system = self.connection.target_system or 1

            for stream_id in (
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            ):
                self.connection.mav.request_data_stream_send(
                    target_system,
                    0,
                    stream_id,
                    2,
                    1,
                )

            self._set(decision_log="MAVLink telemetry streams requested at low rate.")
            self._log("MAVLink telemetry streams requested at low rate.")
        except Exception as exc:
            self._log(f"ERROR: MAVLink stream request failed: {exc}")

    def _cog_hesapla(self, lat, lon):
        if abs(lat) < 0.000001 and abs(lon) < 0.000001:
            return None

        onceki = self._last_position_for_cog
        self._last_position_for_cog = (lat, lon)
        if onceki is None:
            return None

        onceki_lat, onceki_lon = onceki
        if abs(lat - onceki_lat) < 0.000002 and abs(lon - onceki_lon) < 0.000002:
            return None

        lat1 = math.radians(onceki_lat)
        lat2 = math.radians(lat)
        dlon = math.radians(lon - onceki_lon)
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    def _mesaj_araliklarini_iste(self, target_system, target_component):
        mesajlar = {
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 1,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 5,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 10,
        }

        for message_id, hz in mesajlar.items():
            interval_us = int(1_000_000 / hz)
            self.connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )

    def _mavlink_command_name(self, command):
        try:
            return mavutil.mavlink.enums["MAV_CMD"][int(command)].name
        except Exception:
            return str(command)

    def _mavlink_result_name(self, result):
        try:
            return mavutil.mavlink.enums["MAV_RESULT"][int(result)].name
        except Exception:
            return str(result)

    def _mavlink_state_name(self, state):
        try:
            return mavutil.mavlink.enums["MAV_STATE"][int(state)].name
        except Exception:
            return str(state)

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
        target_component=None,
    ):
        if not self.connection:
            self._log("ERROR: No connection. Command was not sent.")
            return False

        target_system = self.connection.target_system or 1
        target_component = target_component
        if target_component is None:
            target_component = self.connection.target_component or 1

        self.connection.mav.command_long_send(
            target_system,
            target_component,
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

    def _mavlink_hedefleri(self, component_zero=False):
        target_system = self.connection.target_system or 1
        if component_zero:
            target_component = 0
        else:
            target_component = self.connection.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        return target_system, target_component

    def _arm_disarm_gonder(self, armed, force=False, repeat=1):
        if not self.connection:
            self._log("ERROR: No connection. ARM/DISARM command was not sent.")
            return False

        target_system, target_component = self._mavlink_hedefleri(component_zero=False)
        force_param = float(ARDUPILOT_FORCE_ARM_MAGIC) if armed and force else 0.0
        for _ in range(max(1, int(repeat))):
            self.connection.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if armed else 0.0,
                force_param,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            time.sleep(0.15)
        return True

    def arm_yap(self):
        self._arm_yap_mavlink()

    def _arm_yap_mavlink(self):
        with self._lock:
            if self._durum["mod"] == "EMERGENCY":
                self._log("ERROR: Reset emergency state before arming.")
                return
            if self._durum["armed"]:
                self._log("INFO: Vehicle is already armed.")
                return

        if self._arm_disarm_gonder(armed=True, force=True, repeat=3):
            self._set(
                requested_arm_state=True,
                arm_change_pending=True,
                decision_log="MAVLink FORCE ARM command sent. Waiting for heartbeat confirmation...",
            )
            self._log("MAVLink FORCE ARM command sent -> waiting for heartbeat confirmation")

    def disarm_yap(self):
        self._disarm_yap_mavlink()

    def _disarm_yap_mavlink(self):
        if self._arm_disarm_gonder(armed=False, force=False, repeat=2):
            self._set(
                requested_arm_state=False,
                arm_change_pending=True,
                hiz=0.0,
                decision_log="MAVLink DISARM command sent. Waiting for heartbeat confirmation...",
            )
            self._log("MAVLink DISARM command sent -> waiting for heartbeat confirmation")

    def mod_ayarla(self, mod_id):
        self._mod_ayarla_mavlink(mod_id)

    def mod_ayarla_ad(self, mod_name):
        mode_id = MODE_NAME_TO_ID.get(str(mod_name).upper())
        if mode_id is None:
            self._log(f"ERROR: Unknown mode selected: {mod_name}")
            return
        self.mod_ayarla(mode_id)

    def _mod_ayarla_mavlink(self, mod_id):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if not self.connection:
            self._log("ERROR: No connection for mode change.")
            return

        try:
            target_system = self.connection.target_system or 1
            self.connection.mav.set_mode_send(
                target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mod_id,
            )
            self._set(
                requested_mode=mod_id,
                mode_change_pending=True,
                decision_log=f"MAVLink MODE command sent: {mod_name}. Waiting for heartbeat confirmation...",
            )
            self._log(f"MAVLink MODE command sent: {mod_name} ({mod_id})")
        except Exception as exc:
            self._log(f"ERROR: MAVLink MODE command failed: {exc}")

    def _mission_mesaji_bekle(self, beklenen_tipler, timeout=8.0):
        deadline = time.time() + timeout
        beklenen_tipler = set(beklenen_tipler)
        while time.time() < deadline:
            with self._lock:
                for index, msg in enumerate(self._mission_messages):
                    if msg.get_type() in beklenen_tipler:
                        return self._mission_messages.pop(index)
            time.sleep(0.05)
        return None

    def _mission_kuyrugunu_temizle(self):
        with self._lock:
            self._mission_messages.clear()

    def _waypoints_dosyasini_oku(self, waypoints_yolu):
        waypoints_yolu = Path(waypoints_yolu)
        if waypoints_yolu.suffix.lower() != ".waypoints":
            raise ValueError("Mission file must have a .waypoints extension.")
        text = waypoints_yolu.read_text(encoding="utf-8", errors="replace")
        waypoints = []
        qgc_wpl = False
        for line_no, line in enumerate(text.splitlines(), start=1):
            temiz = line.strip()
            if not temiz or temiz.startswith("#"):
                continue

            if temiz.upper().startswith("QGC WPL"):
                qgc_wpl = True
                continue

            if qgc_wpl:
                waypoint = self._qgc_wpl_satiri_oku(temiz, len(waypoints) + 1, line_no)
                if waypoint is not None:
                    waypoints.append(waypoint)
                continue

            sayilar = re.findall(r"[-+]?\d+(?:\.\d+)?", temiz)
            if len(sayilar) < 2:
                continue

            candidates = []
            for index in range(len(sayilar) - 1):
                try:
                    lat = float(sayilar[index])
                    lon = float(sayilar[index + 1])
                except ValueError:
                    continue
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    candidates.append((index, lat, lon))

            if not candidates:
                continue

            _index, lat, lon = self._en_olasi_lat_lon_cifti(candidates)
            waypoints.append(
                {
                    "name": f"WP_{len(waypoints) + 1:02d}",
                    "lat": lat,
                    "lon": lon,
                    "alt": 100.0,
                    "line": line_no,
                }
            )

        if not waypoints:
            raise ValueError("No valid latitude/longitude waypoint found in .waypoints file.")
        return waypoints

    def _txt_waypointlerini_oku(self, txt_yolu):
        """Geriye dönük uyumluluk için eski okuyucu adı."""
        return self._waypoints_dosyasini_oku(txt_yolu)

    def _qgc_wpl_satiri_oku(self, line, waypoint_index, line_no):
        parts = re.split(r"[\t,; ]+", line.strip())
        if len(parts) < 12:
            return None

        try:
            seq = int(float(parts[0]))
            current = int(float(parts[1]))
            frame = int(float(parts[2]))
            command = int(float(parts[3]))
            param1, param2, param3, param4 = (float(value) for value in parts[4:8])
            lat = float(parts[8])
            lon = float(parts[9])
            alt = float(parts[10])
            autocontinue = int(float(parts[11]))
        except (TypeError, ValueError, IndexError):
            return None

        if command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            return None
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return None

        is_home = seq == 0 and current == 1
        return {
            "name": "HOME" if is_home else f"WP_{seq:02d}",
            "seq": seq,
            "current": current,
            "frame": frame,
            "command": command,
            "param1": param1,
            "param2": param2,
            "param3": param3,
            "param4": param4,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "autocontinue": autocontinue,
            "is_home": is_home,
            "line": line_no,
        }

    def _en_olasi_lat_lon_cifti(self, candidates):
        def score(item):
            index, lat, lon = item
            value = index * 0.01
            if 35.0 <= abs(lat) <= 72.0:
                value += 4.0
            if -20.0 <= lon <= 45.0:
                value += 4.0
            if abs(lat) >= abs(lon):
                value += 1.0
            return value

        return max(candidates, key=score)

    def _mission_count_gonder(self, target_system, target_component, count):
        try:
            self.connection.mav.mission_count_send(
                target_system,
                target_component,
                count,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_count_send(target_system, target_component, count)

    def _mission_item_gonder(self, target_system, target_component, seq, waypoint, use_int=True):
        lat_int = int(float(waypoint["lat"]) * 1e7)
        lon_int = int(float(waypoint["lon"]) * 1e7)
        command = int(waypoint.get("command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        current = int(waypoint.get("current", 0))
        autocontinue = int(waypoint.get("autocontinue", 1))
        frame = int(waypoint.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        int_frames = {
            mavutil.mavlink.MAV_FRAME_GLOBAL: mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        }
        int_frame = int_frames.get(frame, frame)
        params = [float(waypoint.get(f"param{index}", 0.0) or 0.0) for index in range(1, 5)]
        altitude = float(waypoint.get("alt", waypoint.get("altitude", 0.0)) or 0.0)

        if not use_int:
            self._mission_item_float_gonder(
                target_system, target_component, seq, waypoint, frame,
                command, current, autocontinue, params, altitude,
            )
            return

        args = (
            target_system, target_component, seq, int_frame, command,
            current, autocontinue, *params, lat_int, lon_int, altitude,
        )
        try:
            self.connection.mav.mission_item_int_send(
                *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_item_int_send(*args)

    def _mission_item_float_gonder(
        self, target_system, target_component, seq, waypoint, frame,
        command, current, autocontinue, params, altitude,
    ):
        args = (
            target_system, target_component, seq, frame, command,
            current, autocontinue, *params,
            float(waypoint["lat"]), float(waypoint["lon"]), altitude,
        )
        try:
            self.connection.mav.mission_item_send(
                *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_item_send(*args)

    def _mavlink_gorev_baglantisi_hazir_mi(self):
        with self._lock:
            baglanti = bool(self._durum.get("baglanti"))
            link_ok = bool(self._durum.get("link_ok"))
            heartbeat_seen = bool(self._durum.get("heartbeat_seen"))
            telemetry_lost = bool(self._durum.get("telemetry_lost"))

        return bool(self.connection and baglanti and link_ok and heartbeat_seen and not telemetry_lost)

    def _mavlink_gorev_yukle(self, waypoints):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            raise RuntimeError(
                "No healthy Pixhawk MAVLink telemetry connection. Use CONNECT and wait for heartbeat first."
            )

        son_hata = None
        for component_zero in (False, True):
            try:
                self._mavlink_gorev_yukle_hedef(waypoints, component_zero=component_zero)
                self._mission_component_zero = component_zero
                return
            except RuntimeError as exc:
                son_hata = exc
                if not component_zero:
                    self._log(f"WARNING: Mission upload retrying with target component 0 after: {exc}")
                    time.sleep(0.5)
                    continue
                raise
        if son_hata:
            raise son_hata

    def _mission_items_hazirla(self, waypoints):
        items = list(waypoints)
        if items and items[0].get("is_home"):
            return items
        return [self._home_waypoint_yap(items)] + items

    def _mavlink_gorev_yukle_hedef(self, waypoints, component_zero=False):
        target_system, target_component = self._mavlink_hedefleri(component_zero=component_zero)
        mission_items = self._mission_items_hazirla(waypoints)
        self._mission_kuyrugunu_temizle()

        try:
            self.connection.mav.mission_clear_all_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_clear_all_send(target_system, target_component)

        clear_ack = self._mission_mesaji_bekle(("MISSION_ACK",), timeout=2.0)
        if clear_ack is not None:
            result = getattr(clear_ack, "type", None)
            self._log(f"MAVLink mission clear ACK: {self._mavlink_mission_result_name(result)}")

        self._mission_kuyrugunu_temizle()
        self._mission_count_gonder(target_system, target_component, len(mission_items))
        self._log(
            f"MAVLink mission upload started: {len(mission_items)} mission item(s), "
            f"target_component={target_component}"
        )

        gonderilenler = set()
        resend_count = 0
        deadline = time.time() + max(12.0, len(mission_items) * 3.0)
        while time.time() < deadline:
            msg = self._mission_mesaji_bekle(
                ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"),
                timeout=2.0,
            )
            if msg is None:
                if not gonderilenler:
                    resend_count += 1
                    self._mission_count_gonder(target_system, target_component, len(mission_items))
                    self._log(f"MAVLink mission count resent ({resend_count})")
                continue

            msg_type = msg.get_type()
            if msg_type == "MISSION_ACK":
                result = getattr(msg, "type", None)
                result_name = self._mavlink_mission_result_name(result)
                if len(gonderilenler) < len(mission_items):
                    self._log(f"INFO: Ignoring early mission ACK before all waypoints were sent: {result_name}")
                    if not gonderilenler:
                        self._mission_count_gonder(target_system, target_component, len(mission_items))
                    continue
                if int(result or 0) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self._log("MAVLink mission upload accepted by Pixhawk.")
                    return
                raise RuntimeError(f"Mission upload rejected: {result_name}")

            seq = int(getattr(msg, "seq", -1))
            if seq < 0 or seq >= len(mission_items):
                raise RuntimeError(f"Pixhawk requested invalid mission item: {seq}")

            self._mission_item_gonder(
                target_system,
                target_component,
                seq,
                mission_items[seq],
                use_int=True,
            )
            gonderilenler.add(seq)
            if seq == 0:
                self._log(f"MAVLink mission item sent: HOME/{len(mission_items)}")
            else:
                self._log(f"MAVLink mission item sent: WP {seq}/{len(waypoints)}")

        raise RuntimeError("Mission upload timed out before Pixhawk ACK.")

    def _home_waypoint_yap(self, waypoints):
        first = waypoints[0] if waypoints else {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        with self._lock:
            lat = float(self._durum.get("lat", 0.0) or 0.0)
            lon = float(self._durum.get("lon", 0.0) or 0.0)

        if abs(lat) < 0.000001 and abs(lon) < 0.000001:
            lat = float(first.get("lat", 0.0) or 0.0)
            lon = float(first.get("lon", 0.0) or 0.0)

        return {
            "name": "HOME",
            "seq": 0,
            "current": 1,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "lat": lat,
            "lon": lon,
            "alt": float(first.get("alt", first.get("altitude", 0.0)) or 0.0),
            "autocontinue": 1,
            "is_home": True,
        }

    def _mission_request_list_gonder(self, target_system, target_component):
        try:
            self.connection.mav.mission_request_list_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.connection.mav.mission_request_list_send(target_system, target_component)

    def _mission_request_item_gonder(self, target_system, target_component, seq):
        try:
            self.connection.mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except AttributeError:
            self.connection.mav.mission_request_send(target_system, target_component, seq)
        except TypeError:
            self.connection.mav.mission_request_int_send(target_system, target_component, seq)

    def _mission_item_waypoint_yap(self, msg, seq):
        msg_type = msg.get_type()
        if msg_type == "MISSION_ITEM_INT":
            lat = getattr(msg, "x", 0) / 1e7
            lon = getattr(msg, "y", 0) / 1e7
        else:
            lat = getattr(msg, "x", getattr(msg, "lat", 0.0))
            lon = getattr(msg, "y", getattr(msg, "lon", 0.0))

        command = int(getattr(msg, "command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT))
        if command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            return None
        if not (-90 <= float(lat) <= 90 and -180 <= float(lon) <= 180):
            return None

        current = int(getattr(msg, "current", 0) or 0)
        is_home = seq == 0
        return {
            "name": "HOME" if is_home else f"WP_{seq:02d}",
            "seq": seq,
            "current": current,
            "frame": int(getattr(msg, "frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)),
            "command": command,
            "param1": float(getattr(msg, "param1", 0.0) or 0.0),
            "param2": float(getattr(msg, "param2", 0.0) or 0.0),
            "param3": float(getattr(msg, "param3", 0.0) or 0.0),
            "param4": float(getattr(msg, "param4", 0.0) or 0.0),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(getattr(msg, "z", 0.0) or 0.0),
            "autocontinue": int(getattr(msg, "autocontinue", 1) or 0),
            "is_home": is_home,
        }

    def _mission_item_gecerli_mi(self, msg, beklenen_seq):
        msg_type = msg.get_type()
        if msg_type not in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            return False
        if int(getattr(msg, "seq", -1)) != int(beklenen_seq):
            return False
        return True

    def _waypointler_eslesiyor(self, beklenen, okunan, tolerans=0.00001):
        beklenen = [item for item in beklenen if not item.get("is_home")]
        okunan = [item for item in okunan if not item.get("is_home")]
        if len(beklenen) != len(okunan):
            self._log(
                f"ERROR: Mission verify failed. Uploaded {len(beklenen)} waypoint(s), "
                f"Pixhawk returned {len(okunan)}."
            )
            return False

        for index, (exp, got) in enumerate(zip(beklenen, okunan), start=1):
            lat_fark = abs(float(exp["lat"]) - float(got["lat"]))
            lon_fark = abs(float(exp["lon"]) - float(got["lon"]))
            if lat_fark > tolerans or lon_fark > tolerans:
                self._log(
                    f"ERROR: Mission verify mismatch at WP {index}: "
                    f"expected {float(exp['lat']):.7f}, {float(exp['lon']):.7f}; "
                    f"got {float(got['lat']):.7f}, {float(got['lon']):.7f}"
                )
                return False

        return True

    def _mavlink_gorev_oku(self):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            raise RuntimeError(
                "No healthy Pixhawk MAVLink telemetry connection. Use CONNECT and wait for heartbeat first."
            )

        target_system, target_component = self._mavlink_hedefleri(component_zero=self._mission_component_zero)
        self._mission_kuyrugunu_temizle()
        self._mission_request_list_gonder(target_system, target_component)
        self._log("Reading mission list from Pixhawk...")

        count_msg = self._mission_mesaji_bekle(("MISSION_COUNT",), timeout=6.0)
        if count_msg is None:
            raise RuntimeError("Pixhawk did not return MISSION_COUNT.")

        count = int(getattr(count_msg, "count", 0) or 0)
        waypoints = []
        for seq in range(count):
            self._mission_request_item_gonder(target_system, target_component, seq)
            item_msg = None
            read_deadline = time.time() + 5.0
            while time.time() < read_deadline:
                candidate = self._mission_mesaji_bekle(("MISSION_ITEM_INT", "MISSION_ITEM"), timeout=1.0)
                if candidate is None:
                    continue
                if self._mission_item_gecerli_mi(candidate, seq):
                    item_msg = candidate
                    break
                self._log(
                    f"INFO: Ignoring stale mission item seq "
                    f"{getattr(candidate, 'seq', None)} while waiting for {seq}."
                )
            if item_msg is None:
                raise RuntimeError(f"Pixhawk did not return mission item {seq}.")
            waypoint = self._mission_item_waypoint_yap(item_msg, seq)
            if waypoint is not None:
                if seq == 0:
                    self._log(
                        f"Mission item read: HOME/{count} -> "
                        f"{waypoint['lat']:.7f}, {waypoint['lon']:.7f}"
                    )
                else:
                    self._log(
                        f"Mission item read: WP {seq}/{count - 1} -> "
                        f"{waypoint['lat']:.7f}, {waypoint['lon']:.7f}"
                    )
                waypoints.append(waypoint)

        self._log(f"Mission read from Pixhawk: {len(waypoints)} waypoint(s)")
        return waypoints

    def _mavlink_mission_result_name(self, result):
        try:
            return mavutil.mavlink.enums["MAV_MISSION_RESULT"][int(result)].name
        except Exception:
            return str(result)

    def gorev_baslat(self, gorev_adi):
        if not self._mavlink_gorev_baglantisi_hazir_mi():
            self._log("ERROR: No healthy Pixhawk MAVLink telemetry connection for mission start. Use CONNECT and wait for heartbeat first.")
            return
        if not self._mission_uploaded_to_pixhawk:
            self._log("ERROR: Mission is not confirmed on Pixhawk. Load a .waypoints mission over telemetry before EXECUTE.")
            self._set(
                active_mission=None,
                decision_log="Mission start blocked. Upload a .waypoints file to Pixhawk over telemetry first.",
            )
            return

        mission_number = self._gorev_numarasi(gorev_adi)
        if mission_number is None:
            self._log(f"ERROR: Unknown mission selection: {gorev_adi}")
            return

        if not self._scr_user1_ayarla(mission_number):
            self._log(f"ERROR: SCR_USER1 could not be confirmed for mission: {gorev_adi}")
            return

        self._set(
            active_mission=gorev_adi,
            decision_log=(
                f"Mission selected: {gorev_adi}. Pixhawk mission is loaded. "
                f"SCR_USER1 was confirmed as {mission_number}."
            ),
        )
        self._log(f"MISSION SELECTED VIA SCR_USER1: {gorev_adi}")

    def _gorev_numarasi(self, gorev_adi):
        text = str(gorev_adi or "").strip().upper()
        if text.startswith("M"):
            text = text[1:]
        try:
            number = int(text)
        except ValueError:
            return None
        if 1 <= number <= 4:
            return number
        return None

    def _scr_user1_ayarla(self, mission_number, timeout=4.0):
        if not self.connection:
            return False

        target_system, target_component = self._mavlink_hedefleri(component_zero=False)
        param_id = b"SCR_USER1"
        expected = float(mission_number)
        self._mission_kuyrugunu_temizle()

        try:
            self.connection.mav.param_set_send(
                target_system,
                target_component,
                param_id,
                expected,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            self._log(f"MAVLink PARAM_SET sent: SCR_USER1={expected:.0f}")
        except Exception as exc:
            self._log(f"ERROR: SCR_USER1 PARAM_SET failed: {exc}")
            return False

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mission_mesaji_bekle(("PARAM_VALUE",), timeout=0.5)
            if msg is None:
                continue
            received_id = getattr(msg, "param_id", "")
            if isinstance(received_id, bytes):
                received_id = received_id.decode(errors="replace")
            received_id = str(received_id).strip("\x00").strip()
            if received_id != "SCR_USER1":
                continue
            received_value = float(getattr(msg, "param_value", float("nan")))
            if abs(received_value - expected) <= 0.001:
                self._log(f"SUCCESS: SCR_USER1 confirmed as {received_value:.0f}")
                return True
            self._log(
                f"ERROR: SCR_USER1 verification mismatch: "
                f"expected {expected:.0f}, got {received_value}"
            )
            return False

        self._log("ERROR: SCR_USER1 PARAM_VALUE confirmation timed out.")
        return False

    def _mission_start_gonder(self):
        if not self.connection:
            self._log("ERROR: No connection for mission start.")
            return False

        ok = self._komut_gonder(
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_component=0,
        )
        if ok:
            self._log("MAVLink MISSION_START command sent.")
        return ok

    def _rc_stop_gonder(self, repeat=5):
        if not self.connection:
            return
        target_system, target_component = self._mavlink_hedefleri(component_zero=False)
        for _ in range(repeat):
            try:
                self.connection.mav.rc_channels_override_send(
                    target_system,
                    target_component,
                    1500,
                    0,
                    1500,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                self._log(f"WARNING: RC stop override failed: {exc}")
                return
            time.sleep(0.03)

    def gorev_waypoints_yukle(self, waypoints_yolu, mission_name=None):
        waypoints_yolu = str(waypoints_yolu)
        mission_name = mission_name or Path(waypoints_yolu).stem
        local_waypoints = self._waypoints_dosyasini_oku(waypoints_yolu)
        mission_items = self._mission_items_hazirla(local_waypoints)
        display_waypoints = [item for item in local_waypoints if not item.get("is_home")]
        pixhawk_waypoints = None

        response = {
            "ok": True,
            "success": True,
            "mission_id": mission_name,
            "mission_name": mission_name,
            "message": "Mission waypoints parsed locally.",
            "waypoints": display_waypoints,
            "backend_used": False,
            "pixhawk_uploaded": False,
            "pixhawk_confirmed": False,
        }
        self._mission_id = mission_name

        with self._lock:
            jetson_ip = self._durum.get("jetson_ip")
        if not self._valid_ip(jetson_ip):
            jetson_ip = None

        try:
            self._log(
                "Uploading mission waypoint file to Jetson backend: "
                f"mission={mission_name}, host={jetson_ip or 'configured backend host'}"
            )
            backend_response = self.backend_client.upload_mission_waypoints(
                waypoints_yolu,
                jetson_ip=jetson_ip,
                mission_name=mission_name,
            )
            response["backend_used"] = True
            response["backend_response"] = backend_response
            self._log(
                "SUCCESS: Mission waypoint file saved by Jetson backend: "
                f"{backend_response.get('filename', mission_name)}"
            )
        except Exception as exc:
            response["backend_error"] = str(exc)
            self._log(f"WARNING: Jetson waypoint file upload failed: {exc}")

        if self._mavlink_gorev_baglantisi_hazir_mi():
            try:
                self._log("Uploading mission waypoints directly to Pixhawk over MAVLink telemetry.")
                self._mavlink_gorev_yukle(local_waypoints)
                self._mission_uploaded_to_pixhawk = True
                response["pixhawk_uploaded"] = True
                self._log("SUCCESS: Mission uploaded directly to Pixhawk by GUI.")
                try:
                    pixhawk_waypoints = self._mavlink_gorev_oku()
                    if self._waypointler_eslesiyor(mission_items, pixhawk_waypoints):
                        response["waypoints"] = display_waypoints
                        response["pixhawk_confirmed"] = True
                        self._log("SUCCESS: Mission list verified from Pixhawk by GUI.")
                    else:
                        response["pixhawk_confirmed"] = False
                        response["pixhawk_error"] = "Pixhawk mission verification mismatch."
                        self._log("WARNING: Mission upload ACK was accepted, but read-back verification did not match exactly. Keeping uploaded mission active.")
                except Exception as exc:
                    self._log(f"WARNING: Direct Pixhawk mission read failed: {exc}")
            except Exception as exc:
                self._mission_uploaded_to_pixhawk = False
                response["pixhawk_uploaded"] = False
                response["pixhawk_error"] = str(exc)
                self._log(f"ERROR: Direct Pixhawk mission upload failed: {exc}")
        else:
            self._mission_uploaded_to_pixhawk = False
            response["pixhawk_error"] = "Pixhawk MAVLink telemetry is not connected or heartbeat is not healthy."
            self._log("WARNING: Mission parsed locally, but Pixhawk telemetry heartbeat is not ready. Use CONNECT before uploading to vehicle.")

        if response.get("pixhawk_confirmed") or response.get("pixhawk_uploaded"):
            self.gorev_noktalarini_guncelle(response.get("waypoints") or display_waypoints)
            self._set(
                active_mission=self._mission_id,
                decision_log="Mission uploaded to vehicle and displayed on map.",
            )
        else:
            self.gorev_noktalarini_guncelle([])
            self._set(
                active_mission=None,
                decision_log="Mission waypoints parsed, but vehicle connection is not ready. Route is not displayed.",
            )
            self._log("INFO: Mission waypoints were parsed only. Vehicle/Pixhawk upload was not confirmed, so map route was not displayed.")
        return response

    def gorev_waypoints_onizle(self, waypoints_yolu):
        """Bir mission dosyasını araca göndermeden yerelde ayrıştırır."""
        items = self._waypoints_dosyasini_oku(waypoints_yolu)
        return [item for item in items if not item.get("is_home")]

    def gorev_txt_yukle(self, txt_yolu, mission_name=None):
        """Geriye dönük uyumluluk için eski yükleyici adı."""
        return self.gorev_waypoints_yukle(txt_yolu, mission_name=mission_name)

    def acil_durum(self):
        self._acil_durum_mavlink()

    def _acil_durum_mavlink(self):
        self._rc_stop_gonder()
        self._mod_ayarla_mavlink(4)
        self._disarm_yap_mavlink()
        self._set(
            mod="HOLD",
            mod_id=4,
            requested_mode=4,
            mode_change_pending=True,
            decision_log="!!! EMERGENCY STOP ACTIVE: HOLD + DISARM requested !!!",
        )
        self._log("!!! MAVLink EMERGENCY STOP: HOLD + DISARM !!!")

    def durum_al(self):
        return self._snapshot()

    def update_battery(self, battery_data):
        with self._lock:
            self._battery_state().update(battery_data or {})
        self._emit_durum()
