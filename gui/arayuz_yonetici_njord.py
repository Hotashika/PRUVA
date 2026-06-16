import os
from PyQt5.QtWidgets import QMainWindow, QDialog, QFileDialog, QTableWidgetItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

try:
    from gui.veri_sistemi_njord import NjordVeriSistemi
except ImportError:
    from veri_sistemi_njord import NjordVeriSistemi

# UI dosyalarının tam yolu
KLASOR_YOLU = os.path.dirname(os.path.abspath(__file__))
UI_KLASOR = os.path.join(KLASOR_YOLU, 'ui')

ALIGN_CENTER = getattr(getattr(Qt, "AlignmentFlag", None), "AlignCenter", None)
if ALIGN_CENTER is None:
    ALIGN_CENTER = getattr(Qt, "AlignCenter", 0)


# ─────────────────────────────────────────────
#  Port Ekranı
# ─────────────────────────────────────────────
class PortEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, 'port.ui'), self)
        self.veri_sistemi = veri_sistemi
        self.pushButton_2.clicked.connect(self.close)
        self.buttonBox.accepted.connect(self.onayla)

    def onayla(self):
        self.veri_sistemi.baglanti_kur(
            self.comboBox.currentText(),
            self.comboBox_2.currentText(),
            self.lineEdit.text(),
        )


# ─────────────────────────────────────────────
#  Harita Ekranı
# ─────────────────────────────────────────────
class HaritaEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, 'map.ui'), self)
        self.veri_sistemi = veri_sistemi
        self.veri_sistemi.veri_guncelle.connect(self.guncelle)
        self.pushButton_4.clicked.connect(self.close)

    def guncelle(self, d):
        self.pushButton.setText(f"GPS: {d['gps']}")
        self.pushButton_2.setText(f"SATS: {d['gps_uydu']}")
        self.lcdNumber.display(d['mesafe'])   # Belge 1 düzeltmesi: hız değil mesafe


# ─────────────────────────────────────────────
#  Görev Planı Ekranı  (Belge 1'den)
# ─────────────────────────────────────────────
class GorevPlaniEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, 'planning.ui'), self)
        self.veri_sistemi = veri_sistemi
        self.tableWidget.setRowCount(0)
        self.pushButton_4.clicked.connect(self.close)
        self.pushButton.clicked.connect(self.dosya_sec)
        self.pushButton_2.clicked.connect(self.yukle)

    def dosya_sec(self):
        yol, _ = QFileDialog.getOpenFileName(self, "Open File", "", "TXT (*.txt)")
        if yol:
            self.pushButton.setText(os.path.basename(yol))
            self.veri_sistemi.log_sinyali.emit(f"LOADED: {os.path.basename(yol)}")

    def yukle(self):
        pts = [("WP_01", "63.44", "10.40"), ("WP_02", "63.45", "10.41")]
        self.tableWidget.setRowCount(len(pts))
        for i, (m, lat, lon) in enumerate(pts):
            self.tableWidget.setItem(i, 0, QTableWidgetItem(m))
            self.tableWidget.setItem(i, 1, QTableWidgetItem(lat))
            self.tableWidget.setItem(i, 2, QTableWidgetItem(lon))
        self.veri_sistemi.log_sinyali.emit("WAYPOINTS UPLOADED TO VEHICLE")


# ─────────────────────────────────────────────
#  Ana Ekran
# ─────────────────────────────────────────────
class NjordAnaEkran(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, 'njord.ui'), self)
        self.sistem = NjordVeriSistemi()

        # ── Sinyal bağlantıları ──────────────────────────────
        self.sistem.veri_guncelle.connect(self.tazele)
        self.sistem.log_sinyali.connect(self.log_ekle)
        self.sistem.kamera_sinyali.connect(self.kamera_goster)   # QImage

        # ── UI olayları ──────────────────────────────────────
        self.pushButton.clicked.connect(self.port_ac)
        self.pushButton_3.clicked.connect(self.harita_ac)
        self.pushButton_2.clicked.connect(self.plan_ac)

        self.pushButton_4.clicked.connect(self.sistem.arm_yap)
        self.pushButton_8.clicked.connect(self.sistem.disarm_yap)
        self.pushButton_7.clicked.connect(self.sistem.acil_durum)
        self.pushButton_6.clicked.connect(self.icra)

        # ── Kamera label başlangıç stili (Belge 1) ──────────
        self.label_7.setText("LINKING CAMERA STREAM...")
        self.label_7.setStyleSheet(
            "background-color: #2c3e50; color: #f39c12; "
            "font-weight: bold; border: 2px dashed #f39c12;"
        )
        self.label_7.setAlignment(ALIGN_CENTER)

        # ── Log alanı temizle ────────────────────────────────
        self.label_2.setText("")
        self.label_3.setText("")

        self._camera_auto_started = False

    # ──────────────────────────────
    #  Kamera — Belge 1 sistemi
    # ──────────────────────────────
    def showEvent(self, event):
        """Pencere açılır açılmaz ZED2i stream'ini otomatik başlatır."""
        super().showEvent(event)
        if not self._camera_auto_started:
            self._camera_auto_started = True
            self.sistem.kamera_oto_baslat()

    def kamera_goster(self, image):
        """QImage alır, label boyutuna orantılı ölçekler, ortalar."""
        if image is None:
            return

        pixmap = QPixmap.fromImage(image)
        self.label_7.setText("")
        self.label_7.setStyleSheet("background-color: #000000;")

        hedef = self.label_7.size()
        if hedef.width() > 0 and hedef.height() > 0:
            pixmap = pixmap.scaled(
                hedef,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )

        self.label_7.setPixmap(pixmap)
        self.label_7.setAlignment(ALIGN_CENTER)

    # ──────────────────────────────
    #  Dialog açıcılar
    # ──────────────────────────────
    def port_ac(self):
        PortEkrani(self.sistem).exec_()

    def harita_ac(self):
        HaritaEkrani(self.sistem).show()

    def plan_ac(self):
        GorevPlaniEkrani(self.sistem).show()

    # ──────────────────────────────
    #  Telemetri — Belge 2 (renk feedback dahil)
    # ──────────────────────────────
    def tazele(self, d):
        self.pushButton_10.setText(f"YAW: {d['yaw']:.1f}°")
        self.pushButton_5.setText(f"ROLL: {d['roll']:.1f}°")
        self.pushButton_9.setText(f"PITCH: {d['pitch']:.1f}°")

        self.lcdNumber_3.display(d['hiz'])
        self.pushButton_11.setText(str(d['lat']))
        self.pushButton_12.setText(str(d['lon']))
        self.progressBar.setValue(int(d.get('pil_yuzde', 0)))
        self.lcdNumber.display(d.get('akim', 0))
        self.lcdNumber_2.display(d.get('voltaj', 0))

        self.textEdit.setPlainText(d['decision_log'])

        # ── Buton renk yönetimi (Belge 2) ───────────────────
        bekliyor   = "background-color: #7f8c8d; color: white; font-weight: bold;"
        arm_aktif  = "background-color: #2ecc71; color: white; font-weight: bold;"
        disarm_red = "background-color: #e74c3c; color: white; font-weight: bold;"
        pasif      = ""

        if d.get("arm_change_pending"):
            if d.get("requested_arm_state"):
                self.pushButton_4.setStyleSheet(bekliyor)
                self.pushButton_4.setText("ARM BEKLENİYOR...")
                self.pushButton_8.setStyleSheet(pasif)
            else:
                self.pushButton_8.setStyleSheet(bekliyor)
                self.pushButton_8.setText("DISARM BEKLENİYOR...")
                self.pushButton_4.setStyleSheet(pasif)
        else:
            if d.get("armed"):
                self.pushButton_4.setStyleSheet(arm_aktif)
                self.pushButton_4.setText("ARMED")
                self.pushButton_8.setStyleSheet(pasif)
                self.pushButton_8.setText("DISARM")
            else:
                self.pushButton_8.setStyleSheet(disarm_red)
                self.pushButton_8.setText("DISARMED")
                self.pushButton_4.setStyleSheet(pasif)
                self.pushButton_4.setText("ARM")

        bekliyor_turuncu = "background-color: #e67e22; color: white; font-weight: bold;"
        onaylandi_mavi   = "background-color: #3498db; color: white; font-weight: bold;"

        if d.get("mode_change_pending") and d.get("requested_mode") == 10:
            self.pushButton_6.setStyleSheet(bekliyor_turuncu)
            self.pushButton_6.setText("OTONOM GEÇİŞİ BEKLENİYOR...")
        elif d.get("mod_id") == 10:
            self.pushButton_6.setStyleSheet(onaylandi_mavi)
            self.pushButton_6.setText("OTONOM AKTİF")
        else:
            self.pushButton_6.setStyleSheet(pasif)
            self.pushButton_6.setText("GÖREVİ İCRA ET")

    # ──────────────────────────────
    #  Log — Belge 2 (Türkçe keyword'ler dahil)
    # ──────────────────────────────
    def log_ekle(self, m):
        self.label_3.setText(self.label_2.text())
        self.label_2.setText(f">> {m}")

        if "!!!" in m or "ERROR" in m or "HATA" in m:
            self.label_2.setStyleSheet("color: #e74c3c; font-weight: bold;")
        elif "COMPLETED" in m or "SUCCESS" in m or "BAŞARILI" in m or "ONAYLANDI" in m:
            self.label_2.setStyleSheet("color: #2ecc71; font-weight: bold;")
        else:
            self.label_2.setStyleSheet("color: #3498db; font-weight: bold;")

    # ──────────────────────────────
    #  Görev İcrası
    # ──────────────────────────────
    def icra(self):
        s = "M1"
        if self.radioButton_2.isChecked(): s = "M2"
        elif self.radioButton_3.isChecked(): s = "M3"
        elif self.radioButton_4.isChecked(): s = "M4"
        self.sistem.gorev_baslat(s)

    # ──────────────────────────────
    #  Kapatma
    # ──────────────────────────────
    def closeEvent(self, event):
        """Pencere kapanırken arka plan thread'lerini güvenli kapatır."""
        self.sistem.kapat()
        event.accept()