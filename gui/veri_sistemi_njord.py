import time
import math
import copy
from threading import Thread, Lock
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage
from pymavlink import mavutil

import importlib.util
import sys

# ─────────────────────────────────────────────
#  ArduRover Mod Haritası (Pixhawk → İsim)
# ─────────────────────────────────────────────
ARDUROVER_MODS = {
    0:  "MANUAL",
    1:  "ACRO",
    2:  "LEARNING",
    3:  "STEERING",
    4:  "HOLD",
    5:  "LOITER",
    6:  "FOLLOW",
    7:  "SIMPLE",
    8:  "DOCK",
    9:  "CIRCLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}

# Bağlantı kayıp eşiği (saniye)
HEARTBEAT_TIMEOUT = 5.0

# ─────────────────────────────────────────────
#  grap_video yükleyici  (Belge 1'den)
# ─────────────────────────────────────────────
def _get_grap_video_path():
    """
    Repo içinde birkaç olası konuma bakar:
    Fetch Data / fetch_data / Fetch_Data
    """
    from pathlib import Path
    repo_root = Path(__file__).resolve().parent.parent
    candidates = [
        repo_root / "Fetch Data" / "grap_video.py",
        repo_root / "fetch_data" / "grap_video.py",
        repo_root / "Fetch_Data" / "grap_video.py",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


try:
    import importlib as _importlib
    _client_side = _importlib.import_module("zed.task1.client_side")
    grap_video = getattr(_client_side, "grap_video", _client_side)
except Exception:
    grap_video = None
    _gv_path = _get_grap_video_path()
    if _gv_path:
        _spec = importlib.util.spec_from_file_location("grap_video", str(_gv_path))
        if _spec:
            _module = importlib.util.module_from_spec(_spec)
            sys.modules["grap_video"] = _module
            _spec.loader.exec_module(_module)   # type: ignore
            grap_video = _module
    if grap_video is None:
        raise ImportError(
            "Cannot import grap_video. "
            "Tried package 'zed.task1.client_side' and local grap_video.py paths."
        )


# ─────────────────────────────────────────────
#  Veri Sistemi Sınıfı
# ─────────────────────────────────────────────


class NjordVeriSistemi(QObject):
    """
    Pixhawk MAVLink telemetrisi + ZED2i kamera yönetimi.

    Sinyal / İş Parçacığı Mimarisi
    ───────────────────────────────
    • veri_guncelle    → arayüze kopyalanmış durum sözlüğü
    • log_sinyali      → zaman damgalı log satırı
    • kamera_sinyali   → QImage kare  (ZED2i stream)
    • baglanti_kesildi → heartbeat timeout → UI ikaz
    """

    veri_guncelle    = pyqtSignal(dict)
    log_sinyali      = pyqtSignal(str)
    kamera_sinyali   = pyqtSignal(QImage)       # QImage  (grap_video uyumlu)
    baglanti_kesildi = pyqtSignal()             # Link-lost sinyali

    # ──────────────────────────────
    #  Başlangıç
    # ──────────────────────────────
    def __init__(self):
        super().__init__()

        self.connection  = None
        self._lock       = Lock()
        self._aktif      = True
        self._last_hb    = 0.0

        # Kamera bayrakları (Belge 1 tarzı — duplicate start koruması)
        self._camera_started = False
        self._camera_running = False

        # Durum sözlüğü — Belge 2 genişletilmiş yapısı
        self._durum = {
            "baglanti": False,
            "armed":    False,
            "mod":      "UNKNOWN",
            "mod_id":   -1,
            "hiz":      0.0,
            "yaw":      0.0,
            "roll":     0.0,
            "pitch":    0.0,
            "lat":      0.0,
            "lon":      0.0,
            "gps":      0,
            "gps_uydu": 0,
            "mesafe":   0.0,
            "decision_log": "Sistem Hazır. Bağlantı bekleniyor...",
            "active_mission": None,
            "link_ok":  False,

            # ── Pending bayrakları (Belge 2) ──────────────────
            "arm_change_pending":   False,
            "requested_arm_state":  False,
            "mode_change_pending":  False,
            "requested_mode":       -1,

            # ── Pil — nested dict (Belge 2) ──────────────────
            "battery": {
                "total_voltage": 0.0,
                "current":       0.0,
                "percentage":    0,
            },
        }

    # ──────────────────────────────
    #  Yardımcı metodlar
    # ──────────────────────────────
    def _set(self, **kwargs):
        """Lock altında birden fazla anahtarı günceller ve UI'a emit eder."""
        with self._lock:
            self._durum.update(kwargs)
        self._emit_durum()

    def _snapshot(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._durum)

    def _emit_durum(self):
        """Flat uyumluluk alanlarını ekleyerek durumu yayar."""
        with self._lock:
            kopya = copy.deepcopy(self._durum)
        # Eski kod uyumluluğu: pil değerlerini üst seviyeye de koy
        battery = kopya.get("battery") or {}
        kopya["voltaj"] = battery.get("total_voltage", 0.0)
        kopya["akim"] = battery.get("current", 0.0)
        kopya["pil_yuzde"] = battery.get("percentage", 0)
        self.veri_guncelle.emit(kopya)

    def _log(self, mesaj: str):
        print(f"[NJORD] {mesaj}")
        self.log_sinyali.emit(mesaj)

    # ──────────────────────────────
    #  Bağlantı Kurma / Kesme
    # ──────────────────────────────
    def baglanti_kur(self, tip: str, baud: str, port: str):
        """Arayüzden gelen parametrelerle Pixhawk + Kamera bağlantılarını başlatır."""
        self._log(f"BAĞLANTI BAŞLATILIYOR: {tip} → {port}")
        try:
            if tip == "UDP":
                address = f"udp:{port}" if ":" in port else f"udp:127.0.0.1:{port}"
                self.connection = mavutil.mavlink_connection(address)
            elif tip == "TCP":
                self.connection = mavutil.mavlink_connection(f"tcp:{port}")
            else:
                self.connection = mavutil.mavlink_connection(port, baud=int(baud))

            self._log("Pixhawk heartbeat bekleniyor…")

            # İş parçacıkları
            Thread(target=self._dinleme_dongusu, daemon=True, name="MAVLink").start()
            Thread(target=self._guvenlik_dongusu, daemon=True, name="Watchdog").start()

            self._set(baglanti=True,
                      decision_log="Bağlantı başarılı. Heartbeat bekleniyor...")
            self._log("Bağlantı başarılı.")

            # Kamerayı da başlat
            self._kamera_baslat()

        except Exception as exc:
            self._log(f"BAĞLANTI HATASI: {exc}")

    def baglanti_kes(self):
        """Tüm iş parçacıklarını durdurur ve bağlantıyı kapatır."""
        self._aktif          = False
        self._camera_running = False
        self._camera_started = False
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None
        self._set(baglanti=False, link_ok=False, mod="UNKNOWN", mod_id=-1,
                  armed=False, decision_log="BAĞLANTI KESİLDİ")
        self._log("BAĞLANTI KAPATILDI")

    def kapat(self):
        """Uygulama kapanışında çağrılır."""
        self.baglanti_kes()

    # ──────────────────────────────
    #  ZED2i Kamera  (Belge 1)
    # ──────────────────────────────
    def kamera_oto_baslat(self):
        """GUI açılır açılmaz Jetson kamerasına bağlanmayı dener."""
        self._kamera_baslat()

    def _kamera_baslat(self):
        """Duplicate start korumalı kamera thread başlatıcı."""
        if self._camera_started:
            return
        self._camera_started = True
        self._camera_running = True
        Thread(target=self._kamera_dongusu, daemon=True, name="Camera").start()

    def _kamera_dongusu(self):
        """Jetson üzerindeki ZED2i stream'ini alıp QImage olarak fırlatır."""
        try:
            grap_video.start(
                frame_callback=self.kamera_sinyali.emit,
                log_callback=self._log,
                stop_callback=lambda: self._camera_running,
            )
        except Exception as exc:
            self._log(f"KAMERA THREAD HATASI: {exc}")
        finally:
            self._log("KAMERA ÇEVRIMDIŞI")

    # ──────────────────────────────
    #  MAVLink Dinleme Döngüsü  (Belge 2)
    # ──────────────────────────────
    def _dinleme_dongusu(self):
        """Pixhawk tamponunu asenkron boşaltır."""
        while self._aktif and self.connection:
            try:
                msg = self.connection.recv_match(blocking=True, timeout=0.1)
                if msg:
                    self._islenmis_mesaj(msg)
            except Exception:
                pass

    # ──────────────────────────────
    #  Watchdog / Güvenlik Döngüsü  (Belge 2)
    # ──────────────────────────────
    def _guvenlik_dongusu(self):
        """Heartbeat zaman aşımını ve recovery'yi ayrı thread'de izler."""
        while self._aktif:
            time.sleep(1)
            with self._lock:
                gecen_sure   = time.time() - self._last_hb
                baglanti_var = self._durum["baglanti"]

            if baglanti_var and gecen_sure > HEARTBEAT_TIMEOUT:
                self._log("!!! UYARI: HEARTBEAT KAYBEDİLDİ !!!")
                self._set(
                    baglanti=False,
                    link_ok=False,
                    decision_log=f"TELEMETRİ KAYBI! {HEARTBEAT_TIMEOUT:.0f}s heartbeat yok.",
                )
                self.baglanti_kesildi.emit()

            elif not baglanti_var and gecen_sure <= HEARTBEAT_TIMEOUT and self._last_hb > 0:
                self._log("Heartbeat yeniden alındı. Bağlantı stabil.")
                self._set(baglanti=True, link_ok=True)

    def _request_stream(self, stream_id: int, rate: int):
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            stream_id, rate, 1,
        )

    # ──────────────────────────────
    #  Mesaj İşleyici
    # ──────────────────────────────
    def _islenmis_mesaj(self, msg):
        msg_type = msg.get_type()

        # ── HEARTBEAT → mod + armed + pending onaylar (Belge 2) ──
        if msg_type == "HEARTBEAT":
            self._last_hb = time.time()
            armed    = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mod_id   = int(msg.custom_mode)
            mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")

            with self._lock:
                # ARM/DISARM onay kontrolü
                if self._durum.get("arm_change_pending"):
                    if self._durum.get("requested_arm_state") == armed:
                        self._durum["arm_change_pending"] = False
                        durum_str = "ARMED (AKTİF)" if armed else "DISARMED (KİLİTLİ)"
                        self._durum["decision_log"] = f"ONAYLANDI: {durum_str}"
                        self._log(f"BAŞARILI: İDA ŞU AN {durum_str}")

                # Mod değişim onay kontrolü
                if self._durum.get("mode_change_pending"):
                    if self._durum.get("requested_mode") == mod_id:
                        self._durum["mode_change_pending"] = False
                        self._durum["decision_log"] = f"ONAYLANDI: {mod_name} MODU AKTİF"
                        self._log(f"BAŞARILI: MOD {mod_name} OLARAK DEĞİŞTİRİLDİ")

            self._set(armed=armed, mod_id=mod_id, mod=mod_name,
                      baglanti=True, link_ok=True)

        elif msg_type == "VFR_HUD":
            self._set(yaw=msg.heading, hiz=msg.groundspeed)

        elif msg_type == "ATTITUDE":
            self._set(
                roll=math.degrees(msg.roll),
                pitch=math.degrees(msg.pitch),
            )

        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            self._set(mesafe=msg.wp_dist)

        elif msg_type == "GLOBAL_POSITION_INT":
            self._set(lat=msg.lat / 1e7, lon=msg.lon / 1e7)

        elif msg_type == "GPS_RAW_INT":
            self._set(
                gps=msg.fix_type,
                gps_uydu=msg.satellites_visible,
            )

        elif msg_type == "SYS_STATUS":
            with self._lock:
                self._durum["battery"]["total_voltage"] = msg.voltage_battery / 1000.0
                self._durum["battery"]["current"]       = msg.current_battery / 100.0
                self._durum["battery"]["percentage"]    = msg.battery_remaining
            self._emit_durum()

    # ──────────────────────────────
    #  Komut Metodları
    # ──────────────────────────────
    def _komut_gonder(self, komut_id, p1=0.0, p2=0.0, p3=0.0,
                      p4=0.0, p5=0.0, p6=0.0, p7=0.0) -> bool:
        if not self.connection:
            self._log("HATA: Bağlantı yok, komut gönderilemedi.")
            return False
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            komut_id, 0,
            p1, p2, p3, p4, p5, p6, p7,
        )
        return True

    def arm_yap(self):
        with self._lock:
            if self._durum["mod"] == "EMERGENCY":
                self._log("HATA: ARM OLMADAN ÖNCE ACİL DURUMU SIFIRLAYIN")
                return
            if self._durum["armed"]:
                self._log("BİLGİ: İDA zaten silahlı (Armed)")
                return
        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            self._set(requested_arm_state=True, arm_change_pending=True,
                      decision_log="ARM KOMUTU GİTTİ | Onay bekleniyor...")
            self._log("ARM KOMUTU GÖNDERİLDİ → HEARTBEAT onayı bekleniyor")

    def disarm_yap(self):
        if self._komut_gonder(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0):
            self._set(requested_arm_state=False, arm_change_pending=True,
                      hiz=0.0,
                      decision_log="DISARM KOMUTU GİTTİ | Onay bekleniyor...")
            self._log("DISARM KOMUTU GÖNDERİLDİ → HEARTBEAT onayı bekleniyor")

    def mod_ayarla(self, mod_id: int):
        mod_name = ARDUROVER_MODS.get(mod_id, f"MODE_{mod_id}")
        if not self.connection:
            self._log("HATA: Mod değişimi için bağlantı yok")
            return
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mod_id,
        )
        self._set(requested_mode=mod_id, mode_change_pending=True,
                  decision_log=f"MOD DEĞİŞİMİ: {mod_name} | Onay bekleniyor...")
        self._log(f"MOD KOMUTU GÖNDERİLDİ: {mod_name} ({mod_id})")

    def gorev_baslat(self, gorev_adi: str):
        with self._lock:
            armed = self._durum["armed"]
        if not armed:
            self._log("HATA: OTONOMA GEÇMEDEN ÖNCE ARACI ARM EDİN")
            return
        self.mod_ayarla(10)  # AUTO
        self._set(active_mission=gorev_adi,
                  decision_log=f"GÖREV BAŞLATILDI: {gorev_adi} | AUTO moduna geçiş komutu gönderildi...")
        self._log(f"GÖREV BAŞLATILDI: {gorev_adi}")

    def acil_durum(self):
        self.disarm_yap()
        self._set(mod="EMERGENCY",
                  decision_log="!!! ACİL DURUM DURDURMASI AKTİF !!!")
        self._log("!!! ACİL DURUM DURDURMASI !!!")

    # ──────────────────────────────
    #  Durum Okuma (thread-safe)
    # ──────────────────────────────
    def durum_al(self) -> dict:
        """Mevcut durumun derin kopyasını döndürür (harici sorgulama için)."""
        return self._snapshot()

    def update_battery(self, battery_data: dict):
        with self._lock:
            self._durum["battery"].update(battery_data)
        self._emit_durum()