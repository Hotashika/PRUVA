import os
import time

from PyQt5 import uic
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QBrush, QFont, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QDialog, QFileDialog, QMainWindow, QTableWidgetItem

try:
    from gui.veri_sistemi_njord import NjordVeriSistemi
except ImportError:
    from veri_sistemi_njord import NjordVeriSistemi


KLASOR_YOLU = os.path.dirname(os.path.abspath(__file__))
UI_KLASOR = os.path.join(KLASOR_YOLU, "ui")

ALIGN_CENTER = getattr(getattr(Qt, "AlignmentFlag", None), "AlignCenter", None)
if ALIGN_CENTER is None:
    ALIGN_CENTER = getattr(Qt, "AlignCenter", 0)

KEEP_ASPECT = getattr(getattr(Qt, "AspectRatioMode", None), "KeepAspectRatio", None)
if KEEP_ASPECT is None:
    KEEP_ASPECT = getattr(Qt, "KeepAspectRatio", 1)

SMOOTH = getattr(getattr(Qt, "TransformationMode", None), "SmoothTransformation", None)
if SMOOTH is None:
    SMOOTH = getattr(Qt, "SmoothTransformation", 1)


def _patch_pyqt5_uic_enums():
    # Some .ui files were saved with enum names that PyQt5's uic does not know.
    # Adding the aliases here keeps direct imports and main_njord.py launches consistent.
    aliases = {
        "Dec": 1,
        "QLCDNumber::Mode::Dec": QtWidgets.QLCDNumber.Dec,
        "Flat": 0,
        "QDialogButtonBox::StandardButton::Cancel": QtWidgets.QDialogButtonBox.Cancel,
        "QDialogButtonBox::StandardButton::Ok": QtWidgets.QDialogButtonBox.Ok,
    }
    for name, value in aliases.items():
        if not hasattr(QtCore.Qt, name):
            setattr(QtCore.Qt, name, value)


_patch_pyqt5_uic_enums()


class PortEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "port.ui"), self)
        self.veri_sistemi = veri_sistemi
        self.pushButton_2.clicked.connect(self.close)
        self.buttonBox.accepted.connect(self.onayla)

    def onayla(self):
        self.veri_sistemi.baglanti_kur(
            self.comboBox.currentText(),
            self.comboBox_2.currentText(),
            self.lineEdit.text(),
        )


class HaritaCizimKatmani(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._vehicle = None
        self._waypoints = []
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)

    def set_vehicle(self, vehicle):
        self._vehicle = vehicle
        self.update()

    def set_waypoints(self, waypoints):
        self._waypoints = waypoints or []
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        self._arka_plan_efekti_ciz(painter)

        points = self._valid_points()
        if not points:
            self._bos_durum_ciz(painter)
            return

        bounds = self._bounds(points)
        waypoint_pixels = []
        for wp in self._waypoints:
            if self._valid_coordinate(wp):
                waypoint_pixels.append((wp, self._to_pixel(wp["lat"], wp["lon"], bounds)))

        if len(waypoint_pixels) > 1:
            self._rota_ciz(painter, [p for _, p in waypoint_pixels])

        for index, (wp, point) in enumerate(waypoint_pixels, start=1):
            self._waypoint_ciz(painter, point, wp.get("name") or f"WP{index}", index)

        if self._vehicle and self._valid_coordinate(self._vehicle):
            point = self._to_pixel(self._vehicle["lat"], self._vehicle["lon"], bounds)
            self._arac_ciz(painter, point, self._vehicle.get("yaw", 0.0))

    def _valid_points(self):
        points = [wp for wp in self._waypoints if self._valid_coordinate(wp)]
        if self._vehicle and self._valid_coordinate(self._vehicle):
            points.append(self._vehicle)
        return points

    def _valid_coordinate(self, point):
        try:
            lat = float(point.get("lat"))
            lon = float(point.get("lon"))
        except (TypeError, ValueError, AttributeError):
            return False
        return abs(lat) > 0.000001 or abs(lon) > 0.000001

    def _bounds(self, points):
        lats = [float(point["lat"]) for point in points]
        lons = [float(point["lon"]) for point in points]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        lat_span = max(max_lat - min_lat, 0.0004)
        lon_span = max(max_lon - min_lon, 0.0004)
        lat_pad = lat_span * 0.22
        lon_pad = lon_span * 0.22

        return (
            min_lat - lat_pad,
            max_lat + lat_pad,
            min_lon - lon_pad,
            max_lon + lon_pad,
        )

    def _to_pixel(self, lat, lon, bounds):
        min_lat, max_lat, min_lon, max_lon = bounds
        w = max(self.width(), 1)
        h = max(self.height(), 1)
        margin = 34
        x_ratio = (float(lon) - min_lon) / max(max_lon - min_lon, 0.000001)
        y_ratio = (max_lat - float(lat)) / max(max_lat - min_lat, 0.000001)
        x = margin + x_ratio * max(w - 2 * margin, 1)
        y = margin + y_ratio * max(h - 2 * margin, 1)
        return QPointF(x, y)

    def _arka_plan_efekti_ciz(self, painter):
        painter.fillRect(self.rect(), QColor(7, 29, 39, 42))
        painter.setPen(QPen(QColor(255, 255, 255, 38), 1))
        step = 58
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)

        painter.setPen(QPen(QColor(255, 255, 255, 95), 1))
        painter.drawText(14, 24, "LIVE MISSION TRACK")

    def _bos_durum_ciz(self, painter):
        painter.setPen(QPen(QColor(255, 255, 255, 180), 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(self.rect(), Qt.AlignCenter, "Waiting for GPS and mission waypoints")

    def _rota_ciz(self, painter, points):
        painter.setPen(QPen(QColor(255, 193, 7, 225), 3))
        for start, end in zip(points, points[1:]):
            painter.drawLine(start, end)

    def _waypoint_ciz(self, painter, point, name, index):
        painter.setPen(QPen(QColor(16, 96, 62), 2))
        painter.setBrush(QBrush(QColor(46, 204, 113, 235)))
        painter.drawEllipse(point, 8, 8)

        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(point.x() + 11), int(point.y() - 8), f"{name or 'WP'}")

        painter.setPen(QPen(QColor(10, 54, 39), 1))
        painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
        painter.drawText(int(point.x() - 4), int(point.y() + 4), str(index))

    def _arac_ciz(self, painter, point, yaw):
        painter.setPen(QPen(QColor(3, 70, 112), 3))
        painter.setBrush(QBrush(QColor(0, 210, 255, 245)))
        painter.drawEllipse(point, 11, 11)

        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(QPointF(point.x(), point.y() - 15), QPointF(point.x(), point.y() + 15))
        painter.drawLine(QPointF(point.x() - 15, point.y()), QPointF(point.x() + 15, point.y()))


class HaritaEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "map.ui"), self)
        self.veri_sistemi = veri_sistemi
        self._son_arac_konumu = None
        self._waypoints = self.veri_sistemi.gorev_noktalarini_al()
        self.veri_sistemi.veri_guncelle.connect(self.guncelle)
        self.veri_sistemi.waypoint_guncelle.connect(self.waypointleri_guncelle)
        self.pushButton_4.clicked.connect(self.close)
        self._harita_veri_panelini_hazirla()
        self._waypoint_panelini_yaz()

    def guncelle(self, d):
        self.pushButton.setText(f"GPS: {d.get('gps', 0)}")
        self.pushButton_2.setText(f"SATS: {d.get('gps_uydu', 0)}")
        self.lcdNumber.display(d.get("mesafe", 0.0))
        self._son_arac_konumu = {
            "lat": d.get("lat", 0.0),
            "lon": d.get("lon", 0.0),
            "speed": d.get("hiz", 0.0),
            "yaw": d.get("yaw", 0.0),
        }
        self.haritaKatmani.set_vehicle(self._son_arac_konumu)

    def waypointleri_guncelle(self, waypoints):
        self._waypoints = waypoints
        self.haritaKatmani.set_waypoints(self._waypoints)
        self._waypoint_panelini_yaz()

    def _harita_veri_panelini_hazirla(self):
        self._statik_harita_noktalarini_gizle()

        self.haritaKatmani = HaritaCizimKatmani(self.groupBox)
        self.haritaKatmani.setGeometry(self.label.geometry())
        self.haritaKatmani.raise_()
        self.haritaKatmani.set_waypoints(self._waypoints)

        self.waypointInfo = QtWidgets.QTextEdit(self.groupBox)
        self.waypointInfo.setGeometry(220, 585, 190, 70)
        self.waypointInfo.setReadOnly(True)
        self.waypointInfo.setStyleSheet(
            "background-color: rgba(255,255,255,230); color: #111; "
            "border: 1px solid #27ae60; font-size: 10px;"
        )

    def _statik_harita_noktalarini_gizle(self):
        for ad in (
            "pushButton_5",
            "pushButton_6",
            "pushButton_7",
            "pushButton_8",
            "pushButton_9",
            "label_2",
            "label_3",
            "label_4",
            "label_5",
        ):
            widget = getattr(self, ad, None)
            if widget is not None:
                widget.hide()

    def _waypoint_panelini_yaz(self):
        if not hasattr(self, "waypointInfo"):
            return
        if not self._waypoints:
            self.waypointInfo.setPlainText("WAYPOINTS\nNo uploaded mission points")
            return

        lines = ["WAYPOINTS"]
        for wp in self._waypoints[:6]:
            lines.append(f"{wp['name']}: {wp['lat']:.6f}, {wp['lon']:.6f}")
        if len(self._waypoints) > 6:
            lines.append(f"... +{len(self._waypoints) - 6} more")
        self.waypointInfo.setPlainText("\n".join(lines))


class GorevPlaniEkrani(QDialog):
    def __init__(self, veri_sistemi):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "planning.ui"), self)
        self.veri_sistemi = veri_sistemi
        self.secili_txt_yolu = None
        self.tableWidget.setRowCount(0)
        self.pushButton_4.clicked.connect(self.close)
        self.pushButton.clicked.connect(self.dosya_sec)
        self.pushButton_2.clicked.connect(self.yukle)

    def dosya_sec(self):
        yol, _ = QFileDialog.getOpenFileName(
            self,
            "Open File",
            "",
            "TXT (*.txt)",
        )
        if yol:
            self.secili_txt_yolu = yol
            self.pushButton.setText(os.path.basename(yol))
            self.veri_sistemi.log_sinyali.emit(f"MISSION TXT SELECTED: {os.path.basename(yol)}")

    def yukle(self):
        if not self.secili_txt_yolu:
            self.veri_sistemi.log_sinyali.emit("ERROR: Select a mission TXT file first.")
            return

        try:
            response = self.veri_sistemi.gorev_txt_yukle(self.secili_txt_yolu)
        except Exception as exc:
            self.veri_sistemi.log_sinyali.emit(f"ERROR: Mission TXT upload failed: {exc}")
            return

        waypoints = self._waypoints_from_response(response)
        if waypoints:
            self._tabloyu_doldur(waypoints)
            self.veri_sistemi.gorev_noktalarini_guncelle(waypoints)
        else:
            self.tableWidget.setRowCount(0)
            self.veri_sistemi.gorev_noktalarini_guncelle([])

        mission_id = response.get("mission_id") or response.get("mission_name") or "backend"
        self.veri_sistemi.log_sinyali.emit(f"MISSION TXT SYNCED: {mission_id}")

    def _waypoints_from_response(self, response):
        raw_waypoints = (
            response.get("waypoints")
            or response.get("parsed_waypoints")
            or response.get("mission_items")
            or []
        )

        waypoints = []
        for index, item in enumerate(raw_waypoints, start=1):
            if isinstance(item, dict):
                name = item.get("name") or item.get("id") or item.get("label") or f"WP_{index:02d}"
                lat = item.get("lat", item.get("latitude"))
                lon = item.get("lon", item.get("lng", item.get("longitude")))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                name = f"WP_{index:02d}"
                lat = item[0]
                lon = item[1]
            else:
                continue

            if lat is None or lon is None:
                continue
            waypoints.append((str(name), str(lat), str(lon)))

        return waypoints

    def _tabloyu_doldur(self, waypoints):
        self.tableWidget.setRowCount(len(waypoints))

        for i, (m, lat, lon) in enumerate(waypoints):
            self.tableWidget.setItem(i, 0, QTableWidgetItem(m))
            self.tableWidget.setItem(i, 1, QTableWidgetItem(lat))
            self.tableWidget.setItem(i, 2, QTableWidgetItem(lon))


class NjordAnaEkran(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(os.path.join(UI_KLASOR, "njord.ui"), self)
        self.setFixedSize(self.size())

        self.sistem = NjordVeriSistemi()
        self.harita_pencere = None
        self.plan_pencere = None
        self._ui_hazir = False
        self._last_camera_frame_time = None

        self.sistem.veri_guncelle.connect(self.tazele)
        self.sistem.log_sinyali.connect(self.log_ekle)
        self.sistem.kamera_sinyali.connect(self.kamera_goster)

        self.pushButton.clicked.connect(self.port_ac)
        self.pushButton_3.clicked.connect(self.harita_ac)
        self.pushButton_2.clicked.connect(self.plan_ac)

        self.pushButton_4.clicked.connect(self.sistem.arm_yap)
        self.pushButton_8.clicked.connect(self.sistem.disarm_yap)
        self.pushButton_7.clicked.connect(self.sistem.acil_durum)
        self.pushButton_6.clicked.connect(self.icra)
        self.comboBox_2.currentTextChanged.connect(self.mod_secildi)

        self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for Jetson video")

        self._log_etiketleri = [
            self.label_2,
            self.label_3,
            self.label_9,
            self.label_10,
        ]
        self._log_gecmisi = []
        for etiket in self._log_etiketleri:
            etiket.setText("")
            etiket.setStyleSheet("")

        self._ek_veri_widgetlarini_hazirla()
        self._komut_stillerini_hazirla()
        self._kamera_watchdog_timer = QtCore.QTimer(self)
        self._kamera_watchdog_timer.timeout.connect(self._kamera_watchdog_kontrol)
        self._kamera_watchdog_timer.start(1000)

        self._arm_butonlarini_sabitle()
        self._camera_auto_started = False
        self._ui_hazir = True

    def showEvent(self, event):
        super().showEvent(event)
        if not self._camera_auto_started:
            self._camera_auto_started = True
            self.sistem.kamera_oto_baslat()

    def kamera_goster(self, image):
        if image is None:
            self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for video signal")
            return

        self._last_camera_frame_time = time.time()
        pixmap = QPixmap.fromImage(image)
        self.label_7.setText("")
        self.label_7.setStyleSheet("background-color: #000000;")

        hedef = self.label_7.size()
        if hedef.width() > 0 and hedef.height() > 0:
            pixmap = pixmap.scaled(hedef, KEEP_ASPECT, SMOOTH)

        self.label_7.setPixmap(pixmap)
        self.label_7.setAlignment(ALIGN_CENTER)

    def _kamera_watchdog_kontrol(self):
        if self._last_camera_frame_time is None:
            return
        if time.time() - self._last_camera_frame_time > 3.0:
            self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for video signal")
            self._last_camera_frame_time = None

    def _kamera_placeholder_goster(self, mesaj):
        self.label_7.clear()
        self.label_7.setText(mesaj)
        self.label_7.setStyleSheet(
            "background-color: #2c3e50; color: #f39c12; "
            "font-weight: bold; border: 2px dashed #f39c12;"
        )
        self.label_7.setAlignment(ALIGN_CENTER)

    def port_ac(self):
        pencere = PortEkrani(self.sistem)
        pencere.exec_()

    def harita_ac(self):
        self.harita_pencere = HaritaEkrani(self.sistem)
        self.harita_pencere.show()

    def plan_ac(self):
        self.plan_pencere = GorevPlaniEkrani(self.sistem)
        self.plan_pencere.show()

    def _ek_veri_widgetlarini_hazirla(self):
        self._cell_etiketleri = [
            self.label,
            self.label_13,
            self.label_14,
            self.label_15,
            self.label_16,
            self.label_17,
        ]
        for index, etiket in enumerate(self._cell_etiketleri, start=1):
            etiket.setText(f"CELL {index}: 0.00 V")

        self.textEdit.setPlainText(
            "Distance to next WP: 0.0 m\n"
            "COLREG rule: --\n"
            "Decision: --\n"
            "Reason: --"
        )

    def _komut_stillerini_hazirla(self):
        self._komut_buton_stili = (
            "QPushButton { background-color: #34495e; color: white; "
            "border-radius: 12px; font-weight: bold; padding: 8px; "
            "border: 1px solid #2c3e50; }"
            "QPushButton:hover { background-color: #2c3e50; }"
            "QPushButton:pressed { background-color: #1a252f; }"
        )
        self._mode_combo_stili = (
            "QComboBox { background-color: #34495e; color: white; "
            "border-radius: 12px; font-weight: bold; padding: 8px 30px 8px 12px; "
            "border: 1px solid #2c3e50; }"
            "QComboBox::drop-down { subcontrol-origin: padding; "
            "subcontrol-position: top right; width: 28px; border-left: 1px solid #2c3e50; }"
            "QComboBox::down-arrow { image: none; width: 0; height: 0; "
            "border-left: 6px solid transparent; border-right: 6px solid transparent; "
            "border-top: 8px solid white; margin-top: 2px; }"
            "QComboBox QAbstractItemView { background-color: #1a1a1a; color: #ecf0f1; "
            "selection-background-color: #3498db; selection-color: white; outline: none; }"
        )
        self.pushButton_6.setStyleSheet(self._komut_buton_stili)
        self.comboBox_2.setStyleSheet(self._mode_combo_stili)

    def _decision_metni(self, d):
        mesafe = d.get("distance_to_next_wp", d.get("mesafe", 0.0))
        colreg = d.get("colreg_rule", d.get("colreg", "--"))
        decision = d.get("maneuver", d.get("decision", "--"))
        reason = d.get("decision_reason", d.get("colreg_reason", "--"))
        return (
            f"Distance to next WP: {float(mesafe or 0.0):.1f} m\n"
            f"COLREG rule: {colreg or '--'}\n"
            f"Decision: {decision or '--'}\n"
            f"Reason: {reason or '--'}"
        )

    def _batarya_guncelle(self, d):
        battery = d.get("battery", {})
        percent = int(d.get("pil_yuzde", battery.get("percentage", 0)) or 0)
        percent = max(0, min(percent, 100))
        voltage = float(d.get("voltaj", battery.get("total_voltage", 0.0)) or 0.0)
        current = float(d.get("akim", battery.get("current", 0.0)) or 0.0)
        cells = battery.get("cell_voltages", d.get("cell_voltages", [])) or []

        self.progressBar.setValue(percent)
        self.lcdNumber.display(current)
        self.lcdNumber_2.display(voltage)

        for index, etiket in enumerate(self._cell_etiketleri):
            value = cells[index] if index < len(cells) else 0.0
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = 0.0
            etiket.setText(f"CELL {index + 1}: {value:.2f} V")

    def tazele(self, d):
        self.pushButton_10.setText(f"YAW: {d.get('yaw', 0.0):.1f}°")
        self.pushButton_5.setText(f"ROLL: {d.get('roll', 0.0):.1f}°")
        self.pushButton_9.setText(f"PITCH: {d.get('pitch', 0.0):.1f}°")

        self.lcdNumber_3.display(d.get("hiz", 0.0))
        self.pushButton_11.setText(str(d.get("lat", 0.0)))
        self.pushButton_12.setText(str(d.get("lon", 0.0)))
        self._batarya_guncelle(d)
        self.textEdit.setPlainText(self._decision_metni(d))

        if hasattr(self, "pushButton_wifi"):
            if d.get("wifi_aktif"):
                self.pushButton_wifi.setStyleSheet(
                    "background-color: #2ecc71; color: white; "
                    "font-weight: bold; border-radius: 5px;"
                )
                self.pushButton_wifi.setText(
                    f"WI-FI ACTIVE\nIP: {d.get('jetson_ip', '')}"
                )
            else:
                self.pushButton_wifi.setStyleSheet(
                    "background-color: #e74c3c; color: white; "
                    "font-weight: bold; border-radius: 5px;"
                )
                self.pushButton_wifi.setText("WI-FI LOST\nSearching Jetson")

        bekliyor = "background-color: #7f8c8d; color: white; font-weight: bold;"

        if d.get("arm_change_pending"):
            if d.get("requested_arm_state"):
                self.pushButton_4.setStyleSheet(bekliyor)
                self.pushButton_4.setText("ARM PENDING...")
                self._disarmed_butonunu_sabitle()
            else:
                self.pushButton_8.setStyleSheet(bekliyor)
                self.pushButton_8.setText("DISARM PENDING...")
                self._armed_butonunu_sabitle()
        else:
            self._arm_butonlarini_sabitle()

        bekliyor_turuncu = (
            "background-color: #e67e22; color: white; font-weight: bold; "
            "border-radius: 12px; padding: 8px; border: 1px solid #d35400;"
        )
        onaylandi_mavi = (
            "background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 12px; padding: 8px; border: 1px solid #2980b9;"
        )

        if d.get("mode_change_pending") and d.get("requested_mode") == 10:
            self.pushButton_6.setStyleSheet(bekliyor_turuncu)
            self.pushButton_6.setText("AUTO TRANSITION PENDING...")
        elif d.get("mod_id") == 10:
            self.pushButton_6.setStyleSheet(onaylandi_mavi)
            self.pushButton_6.setText("AUTO ACTIVE")
        else:
            self.pushButton_6.setStyleSheet(self._komut_buton_stili)
            self.pushButton_6.setText("EXECUTE MISSION")

        self._mode_combo_durumunu_guncelle(d, bekliyor_turuncu, onaylandi_mavi)

    def mod_secildi(self, mod_adi):
        if not self._ui_hazir or not mod_adi:
            return
        self.sistem.mod_ayarla_ad(mod_adi)

    def _mode_combo_durumunu_guncelle(self, d, bekliyor_stil, aktif_stil):
        secili_mod = self.comboBox_2.currentText()
        aktif_mod = d.get("mod", "")

        if d.get("mode_change_pending"):
            self.comboBox_2.setStyleSheet(bekliyor_stil)
        elif aktif_mod == secili_mod:
            self.comboBox_2.setStyleSheet(aktif_stil)
        else:
            self.comboBox_2.setStyleSheet(self._mode_combo_stili)

    def _armed_butonunu_sabitle(self):
        self.pushButton_4.setStyleSheet(
            "background-color: #2ecc71; color: white; "
            "font-weight: bold; border-radius: 10px; "
            "border: 2px solid #27ae60;"
        )
        self.pushButton_4.setText("ARMED")

    def _disarmed_butonunu_sabitle(self):
        self.pushButton_8.setStyleSheet(
            "background-color: #e74c3c; color: white; "
            "font-weight: bold; border-radius: 6px; "
            "border: 2px solid #c0392b;"
        )
        self.pushButton_8.setText("DISARMED")

    def _arm_butonlarini_sabitle(self):
        self._armed_butonunu_sabitle()
        self._disarmed_butonunu_sabitle()

    def log_ekle(self, m):
        if "!!!" in m or "ERROR" in m or "FAIL" in m:
            stil = "color: #e74c3c; font-weight: bold;"
        elif (
            "COMPLETED" in m
            or "SUCCESS" in m
            or "CONFIRMED" in m
        ):
            stil = "color: #2ecc71; font-weight: bold;"
        else:
            stil = "color: #3498db; font-weight: bold;"

        self._log_gecmisi.insert(0, (f">> {m}", stil))
        self._log_gecmisi = self._log_gecmisi[: len(self._log_etiketleri)]

        for i, etiket in enumerate(self._log_etiketleri):
            if i < len(self._log_gecmisi):
                metin, satir_stili = self._log_gecmisi[i]
                etiket.setText(metin)
                etiket.setStyleSheet(satir_stili)
            else:
                etiket.setText("")
                etiket.setStyleSheet("")

    def icra(self):
        gorev = "M1"
        if self.radioButton_2.isChecked():
            gorev = "M2"
        elif self.radioButton_3.isChecked():
            gorev = "M3"
        elif self.radioButton_4.isChecked():
            gorev = "M4"
        self.sistem.gorev_baslat(gorev)

    def closeEvent(self, event):
        self.sistem.kapat()
        event.accept()
