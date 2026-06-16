import copy
import importlib.util
import ipaddress
import math
import platform
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock, Thread

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage
from pymavlink import mavutil


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

HEARTBEAT_TIMEOUT = 5.0

JETSON_IP = None
JETSON_MAC = "8c:b8:7e:04:20:a9"
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
            "decision_log": "Sistem hazir. Baglanti bekleniyor...",
            "active_mission": None,
            "link_ok": False,
            "arm_change_pending": False,
            "requested_arm_state": False,
            "mode_change_pending": False,
            "requested_mode": -1,
            "wifi_aktif": False,
            "jetson_ip": "Araniyor...",
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

    def get_ip_from_mac(self, target_mac):
        return self._arp_cache_ip(target_mac)

    def _arp_cache_ip(self, target_mac):
        is_windows = platform.system().lower() == "windows"
        formatted_mac = target_mac.lower().replace(":", "-" if is_windows else ":")

        try:
            result = subprocess.check_output(
                ["arp", "-a"],
                text=True,
                stderr=subprocess.DEVNULL,
                creationflags=self._subprocess_flags(),
            )
        except Exception:
            return None

        for line in result.splitlines():
            if formatted_mac in line.lower():
                ip_match = re.search(r"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", line)
                if ip_match:
                    return ip_match.group()
        return None

    def _local_ipv4_networks(self):
        try:
            result = subprocess.check_output(
                ["ipconfig"],
                text=True,
                stderr=subprocess.DEVNULL,
                creationflags=self._subprocess_flags(),
            )
        except Exception:
            return []

        networks = []
        current_ip = None
        for line in result.splitlines():
            ipv4_match = re.search(r"IPv4.*?:\s*([0-9.]+)", line)
            mask_match = re.search(r"Subnet Mask.*?:\s*([0-9.]+)", line)

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

        return networks

    def _scan_network_for_mac(self, target_mac):
        networks = self._local_ipv4_networks()
        if not networks:
            return None

        for network in networks:
            hosts = [str(ip) for ip in network.hosts()]
            if len(hosts) > 4096:
                continue

            self._log(f"Jetson MAC aranıyor: {network}")
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = [executor.submit(self._ping_ip, ip) for ip in hosts]
                for future in as_completed(futures):
                    future.result()

            ip = self._arp_cache_ip(target_mac)
            if ip:
                return ip

        return None

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
                if not bulunan_ip and time.time() - self._last_network_scan > NETWORK_SCAN_INTERVAL:
                    self._last_network_scan = time.time()
                    bulunan_ip = self._scan_network_for_mac(JETSON_MAC)

                if bulunan_ip and self._ping_ip(bulunan_ip):
                    self._set(wifi_aktif=True, jetson_ip=bulunan_ip)
                else:
                    self._set(wifi_aktif=False, jetson_ip=bulunan_ip or "Bulunamadi")

            time.sleep(2.0)

    def baglanti_kur(self, tip, baud, port):
        self._aktif = True
        self._log(f"BAGLANTI BASLATILIYOR: {tip} -> {port}")

        try:
            if tip == "UDP":
                address = f"udp:{port}" if ":" in port else f"udp:127.0.0.1:{port}"
                self.connection = mavutil.mavlink_connection(address)
            elif tip == "TCP":
                self.connection = mavutil.mavlink_connection(f"tcp:{port}")
            else:
                self.connection = mavutil.mavlink_connection(port, baud=int(baud))

            self._log("Pixhawk heartbeat bekleniyor...")

            Thread(target=self._dinleme_dongusu, daemon=True, name="MAVLink").start()
            if not self._watchdog_started:
                self._watchdog_started = True
                Thread(target=self._guvenlik_dongusu, daemon=True, name="Watchdog").start()

            self._set(
                baglanti=True,
                decision_log="Baglanti basarili. Heartbeat bekleniyor...",
            )
            self._log("Telemetri baglantisi baslatildi.")
            self._kamera_baslat()

        except Exception as exc:
            self._log(f"BAGLANTI HATASI: {exc}")
            self._set(
                baglanti=False,
                link_ok=False,
                decision_log=f"BAGLANTI HATASI: {exc}",
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
            decision_log="BAGLANTI KESILDI",
        )
        self._log("BAGLANTI KAPATILDI")

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
            self._log("KAMERA UYARISI: grap_video bulunamadi. ZED2 goruntusu baslatilamadi.")
            self._set(decision_log="Kamera baslatilamadi: grap_video bulunamadi.")
            self._camera_started = False
            self._camera_running = False
            return

        try:
            with self._lock:
                jetson_ip = self._durum.get("jetson_ip")
                if not self._durum.get("wifi_aktif"):
                    jetson_ip = None

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
            hata = str(exc).lower()
            if "winpcap" in hata or "libpcap" in hata or "sniffing" in hata:
                self._log(
                    "KAMERA UYARISI: Windows packet sniff hatasi. "
                    "Direkt Jetson IP kullanin veya Npcap kurun."
                )
                self._set(decision_log="Kamera hatasi: Windows packet sniff / Npcap sorunu.")
            else:
                self._log(f"KAMERA THREAD HATASI: {exc}")
                self._set(decision_log=f"Kamera hatasi: {exc}")

        finally:
            self._camera_running = False
            self._camera_started = False
            self._log("KAMERA CEVRIMDISI")

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
                self._log("!!! UYARI: HEARTBEAT KAYBEDILDI !!!")
                self._set(
                    baglanti=False,
                    link_ok=False,
                    decision_log=f"TELEMETRI KAYBI! {HEARTBEAT_TIMEOUT:.0f}s heartbeat yok.",
                )
                self.baglanti_kesildi.emit()
            elif not baglanti_var and gecen_sure <= HEARTBEAT_TIMEOUT and self._last_hb > 0:
                self._log("Heartbeat yeniden alindi. Baglanti stabil.")
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
                        durum_str = "ARMED (AKTIF)" if armed else "DISARMED (KILITLI)"
                        self._durum["decision_log"] = f"ONAYLANDI: {durum_str}"
                        self._log(f"BASARILI: IDA SU AN {durum_str}")

                if self._durum.get("mode_change_pending"):
                    if self._durum.get("requested_mode") == mod_id:
                        self._durum["mode_change_pending"] = False
                        self._durum["decision_log"] = f"ONAYLANDI: {mod_name} MODU AKTIF"
                        self._log(f"BASARILI: MOD {mod_name} OLARAK DEGISTIRILDI")

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
            self._log("HATA: Baglanti yok, komut gonderilemedi.")
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
        with self._lock:
            if self._durum["mod"] == "EMERGENCY":
                self._log("HATA: ARM OLMADAN ONCE ACIL DURUMU SIFIRLAYIN")
                return
            if self._durum["armed"]:
                self._log("BILGI: IDA zaten ARM durumda.")
                return

        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            self._set(
                requested_arm_state=True,
                arm_change_pending=True,
                decision_log="ARM KOMUTU GITTI | Heartbeat onayi bekleniyor...",
            )
            self._log("ARM KOMUTU GONDERILDI -> HEARTBEAT onayi bekleniyor")

    def disarm_yap(self):
        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0):
            self._set(
                requested_arm_state=False,
                arm_change_pending=True,
                hiz=0.0,
                decision_log="DISARM KOMUTU GITTI | Heartbeat onayi bekleniyor...",
            )
            self._log("DISARM KOMUTU GONDERILDI -> HEARTBEAT onayi bekleniyor")

    def mod_ayarla(self, mod_id):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if not self.connection:
            self._log("HATA: Mod degisimi icin baglanti yok.")
            return

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mod_id,
        )
        self._set(
            requested_mode=mod_id,
            mode_change_pending=True,
            decision_log=f"MOD DEGISIMI: {mod_name} | Heartbeat onayi bekleniyor...",
        )
        self._log(f"MOD KOMUTU GONDERILDI: {mod_name} ({mod_id})")

    def gorev_baslat(self, gorev_adi):
        with self._lock:
            armed = self._durum["armed"]

        if not armed:
            self._log("HATA: OTONOMA GECMEDEN ONCE ARACI ARM EDIN")
            return

        self.mod_ayarla(10)
        self._set(
            active_mission=gorev_adi,
            decision_log=(
                f"GOREV BASLATILDI: {gorev_adi} | "
                "AUTO moduna gecis komutu gonderildi..."
            ),
        )
        self._log(f"GOREV BASLATILDI: {gorev_adi}")

    def acil_durum(self):
        self.disarm_yap()
        self._set(mod="EMERGENCY", decision_log="!!! ACIL DURUM DURDURMASI AKTIF !!!")
        self._log("!!! ACIL DURUM DURDURMASI !!!")

    def durum_al(self):
        return self._snapshot()

    def update_battery(self, battery_data):
        with self._lock:
            self._durum["battery"].update(battery_data)
        self._emit_durum()
