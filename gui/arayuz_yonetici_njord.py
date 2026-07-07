import os
import time

from PyQt5 import uic
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QBrush, QFont, QPainter, QPen, QPixmap
from PyQt5.QtGui import QPolygonF
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

MAP_IMAGE_FILE = "ecdat_map.png"
MAP_IMAGE_SIZE = (592, 832)
MAP_CALIBRATION_POINTS = [
    {"name": "WP1", "lat": 37.9524548, "lon": 32.5009435, "pixel": (260, 92)},
    {"name": "WP2", "lat": 37.9524210, "lon": 32.5015175, "pixel": (475, 98)},
    {"name": "WP3", "lat": 37.9514904, "lon": 32.5013566, "pixel": (421, 482)},
    {"name": "WP4", "lat": 37.9510082, "lon": 32.5012600, "pixel": (376, 704)},
    {"name": "WP5", "lat": 37.9510589, "lon": 32.5006807, "pixel": (190, 694)},
    {"name": "WP6", "lat": 37.9516681, "lon": 32.5006807, "pixel": (194, 374)},
]
DEFAULT_BATTERY_CAPACITY_WH = 0.0


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
        self._trail = []
        self._affine_x, self._affine_y = self._kalibrasyon_hazirla()
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)

    def set_vehicle(self, vehicle):
        self._vehicle = vehicle
        if self._valid_coordinate(vehicle):
            son = self._trail[-1] if self._trail else None
            if not son or abs(float(son["lat"]) - float(vehicle["lat"])) > 0.000002 or abs(float(son["lon"]) - float(vehicle["lon"])) > 0.000002:
                self._trail.append({"lat": float(vehicle["lat"]), "lon": float(vehicle["lon"])})
                self._trail = self._trail[-120:]
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

        waypoint_pixels = []
        for wp in self._waypoints:
            if self._valid_coordinate(wp):
                waypoint_pixels.append((wp, self._to_pixel(wp["lat"], wp["lon"])))

        if len(waypoint_pixels) > 1:
            self._rota_ciz(painter, [p for _, p in waypoint_pixels])

        trail_pixels = [self._to_pixel(p["lat"], p["lon"]) for p in self._trail]
        if len(trail_pixels) > 1:
            self._trail_ciz(painter, trail_pixels)

        for index, (wp, point) in enumerate(waypoint_pixels, start=1):
            self._waypoint_ciz(painter, point, wp.get("name") or f"WP{index}", index)

        if self._vehicle and self._valid_coordinate(self._vehicle):
            point = self._to_pixel(self._vehicle["lat"], self._vehicle["lon"])
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

    def _to_pixel(self, lat, lon):
        native_x = self._affine_x[0] * float(lat) + self._affine_x[1] * float(lon) + self._affine_x[2]
        native_y = self._affine_y[0] * float(lat) + self._affine_y[1] * float(lon) + self._affine_y[2]
        scale_x = max(self.width(), 1) / MAP_IMAGE_SIZE[0]
        scale_y = max(self.height(), 1) / MAP_IMAGE_SIZE[1]
        return QPointF(native_x * scale_x, native_y * scale_y)

    def _kalibrasyon_hazirla(self):
        rows = [(p["lat"], p["lon"], 1.0) for p in MAP_CALIBRATION_POINTS]
        xs = [p["pixel"][0] for p in MAP_CALIBRATION_POINTS]
        ys = [p["pixel"][1] for p in MAP_CALIBRATION_POINTS]
        return self._least_squares_3(rows, xs), self._least_squares_3(rows, ys)

    def _least_squares_3(self, rows, values):
        normal = [[0.0 for _ in range(3)] for _ in range(3)]
        rhs = [0.0 for _ in range(3)]
        for row, value in zip(rows, values):
            for i in range(3):
                rhs[i] += row[i] * value
                for j in range(3):
                    normal[i][j] += row[i] * row[j]
        return self._solve_3x3(normal, rhs)

    def _solve_3x3(self, matrix, rhs):
        a = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
        for col in range(3):
            pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
            a[col], a[pivot] = a[pivot], a[col]
            div = a[col][col] or 1.0
            for j in range(col, 4):
                a[col][j] /= div
            for r in range(3):
                if r == col:
                    continue
                factor = a[r][col]
                for j in range(col, 4):
                    a[r][j] -= factor * a[col][j]
        return [a[i][3] for i in range(3)]

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

    def _trail_ciz(self, painter, points):
        painter.setPen(QPen(QColor(0, 210, 255, 185), 2))
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
        try:
            yaw_deg = float(yaw)
        except (TypeError, ValueError):
            yaw_deg = 0.0

        painter.save()
        painter.translate(point)
        painter.rotate(yaw_deg)

        govde = QPolygonF(
            [
                QPointF(0, -20),
                QPointF(12, 12),
                QPointF(0, 7),
                QPointF(-12, 12),
            ]
        )
        painter.setPen(QPen(QColor(3, 70, 112), 3))
        painter.setBrush(QBrush(QColor(0, 210, 255, 235)))
        painter.drawPolygon(govde)

        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(QPointF(0, -17), QPointF(0, 7))
        painter.restore()

        painter.setPen(QPen(QColor(255, 255, 255, 225), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(point, 18, 18)

        painter.setPen(QPen(QColor(0, 210, 255), 1))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(point.x() + 20), int(point.y() + 5), f"{yaw_deg:.0f}°")


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
        uic.loadUi(os.path.join(UI_KLASOR, "njord_redesign.ui"), self)
        self._yeni_ui_adlarini_esle()
        self._yeni_ui_layout_duzelt()
        self.setMinimumSize(1280, 720)

        self.sistem = NjordVeriSistemi()
        self.harita_pencere = None
        self.plan_pencere = None
        self._ui_hazir = False
        self._last_camera_frame_time = None
        self._mode_combo_son_stil = None
        self._vessel_status_label = None
        self._cog_label = None

        self.sistem.veri_guncelle.connect(self.tazele)
        self.sistem.log_sinyali.connect(self.log_ekle)
        self.sistem.kamera_sinyali.connect(self.kamera_goster)
        self.sistem.waypoint_guncelle.connect(self._ana_harita_waypoint_guncelle)

        self.pushButton.clicked.connect(self.port_ac)
        self.pushButton_2.clicked.connect(self.plan_ac)

        self.pushButton_4.clicked.connect(self.sistem.arm_yap)
        self.pushButton_8.clicked.connect(self.sistem.disarm_yap)
        self.pushButton_7.clicked.connect(self.sistem.acil_durum)
        self.pushButton_6.clicked.connect(self.icra)
        self.comboBox_2.currentTextChanged.connect(self.mod_secildi)

        self._kamera_placeholder_goster("NO CAMERA SIGNAL\nWaiting for Jetson video")

        self._log_etiketleri = [
            getattr(self, "label_2", None),
            getattr(self, "label_3", None),
            getattr(self, "label_9", None),
            getattr(self, "label_10", None),
        ]
        self._log_etiketleri = [etiket for etiket in self._log_etiketleri if etiket is not None]
        self._log_gecmisi = []
        for etiket in self._log_etiketleri:
            etiket.setText("")
            etiket.setStyleSheet("")

        self._ek_veri_widgetlarini_hazirla()
        self._ana_haritayi_hazirla()
        self._komut_stillerini_hazirla()
        self._kamera_watchdog_timer = QtCore.QTimer(self)
        self._kamera_watchdog_timer.timeout.connect(self._kamera_watchdog_kontrol)
        self._kamera_watchdog_timer.start(1000)

        self._arm_butonlarini_sabitle()
        self._camera_auto_started = False
        self._ui_hazir = True
        QtCore.QTimer.singleShot(0, self.showMaximized)

    def _yeni_ui_adlarini_esle(self):
        eslemeler = {
            "pushButton": "BTNCONNECT",
            "pushButton_2": "BTNFILE",
            "pushButton_4": "BTNARM",
            "pushButton_5": "BTNROLL",
            "pushButton_6": "BTNMISSION",
            "pushButton_7": "BTNSTOP",
            "pushButton_8": "BTNDISARM",
            "pushButton_9": "BTNPITCH",
            "pushButton_10": "BTNYAW",
            "pushButton_11": "BTNLAT",
            "pushButton_12": "BTNLONG",
            "pushButton_wifi": "BTNWIFI",
            "comboBox_2": "CMBMODE",
            "radioButton": "RM1",
            "radioButton_2": "RM2",
            "radioButton_3": "RM3",
            "radioButton_4": "RM4",
            "label_7": "LCAMERA",
            "textEdit": "TXTCOLREG",
            "progressBar": "BATTERBAR",
            "lcdNumber": "LCDCURRENT",
            "lcdNumber_2": "LCDVOLT",
            "lcdNumber_3": "LCDSPEED",
            "label": "LC1",
            "label_13": "LC2",
            "label_14": "LC3",
            "label_15": "LC4",
            "label_16": "LC5",
            "label_17": "LC6",
            "label_2": "LLOG1",
            "label_3": "LLOG2",
            "label_9": "LLOG3",
            "label_10": "LLOG4",
            "label_8": "LLOGIC",
            "label_7_mainmap": "LMAINMAP",
            "textEdit_status": "TXTSTATUSLOG",
        }
        for eski_ad, yeni_ad in eslemeler.items():
            if hasattr(self, yeni_ad):
                setattr(self, eski_ad, getattr(self, yeni_ad))

    def _layouttan_cikar(self, widget):
        parent = widget.parentWidget()
        if parent is not None and parent.layout() is not None:
            parent.layout().removeWidget(widget)

    def _gorsel_ust_duzeni_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("topVisualPanel", "groupBox_4", "groupBox_mapMain", "groupBox_8")):
            return
        top_layout = self.topVisualPanel.layout()
        if top_layout is None:
            return

        if not hasattr(self, "topRightVisualPanel"):
            self.topRightVisualPanel = QtWidgets.QWidget(self.topVisualPanel)
            right_visual_layout = QtWidgets.QVBoxLayout(self.topRightVisualPanel)
            right_visual_layout.setContentsMargins(0, 0, 0, 0)
            right_visual_layout.setSpacing(8)
        else:
            right_visual_layout = self.topRightVisualPanel.layout()

        self._layouttan_cikar(self.groupBox_mapMain)
        self._layouttan_cikar(self.groupBox_8)

        if top_layout.indexOf(self.topRightVisualPanel) < 0:
            top_layout.addWidget(self.topRightVisualPanel)

        if right_visual_layout.indexOf(self.groupBox_mapMain) < 0:
            right_visual_layout.addWidget(self.groupBox_mapMain)
        if right_visual_layout.indexOf(self.groupBox_8) < 0:
            right_visual_layout.addWidget(self.groupBox_8)

        right_visual_layout.setStretchFactor(self.groupBox_mapMain, 4)
        right_visual_layout.setStretchFactor(self.groupBox_8, 1)
        self.topRightVisualPanel.setMinimumWidth(520)
        self.topRightVisualPanel.setMaximumWidth(780)
        self.groupBox_mapMain.setMinimumHeight(350)
        self.groupBox_8.setMinimumHeight(115)

    def _sol_gorsel_sag_bilgi_duzeni_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("centralwidget", "topVisualPanel", "bottomInfoPanel")):
            return
        if hasattr(self, "mainSplitPanel"):
            return

        old_layout = self.centralwidget.layout()
        if old_layout is None:
            return

        old_layout.removeWidget(self.topVisualPanel)
        old_layout.removeWidget(self.bottomInfoPanel)

        self.mainSplitPanel = QtWidgets.QWidget(self.centralwidget)
        split_layout = QtWidgets.QHBoxLayout(self.mainSplitPanel)
        split_layout.setContentsMargins(0, 0, 0, 0)
        split_layout.setSpacing(10)

        old_layout.addWidget(self.mainSplitPanel)
        split_layout.addWidget(self.topVisualPanel)
        split_layout.addWidget(self.bottomInfoPanel)
        split_layout.setStretchFactor(self.topVisualPanel, 42)
        split_layout.setStretchFactor(self.bottomInfoPanel, 58)

        visual_layout = self.topVisualPanel.layout()
        if visual_layout is None:
            visual_layout = QtWidgets.QVBoxLayout(self.topVisualPanel)
        visual_layout.setDirection(QtWidgets.QBoxLayout.TopToBottom)
        visual_layout.setContentsMargins(0, 0, 0, 0)
        visual_layout.setSpacing(10)

        self._layouttan_cikar(self.groupBox_4)
        self._layouttan_cikar(self.groupBox_mapMain)
        if visual_layout.indexOf(self.groupBox_4) < 0:
            visual_layout.addWidget(self.groupBox_4)
        if visual_layout.indexOf(self.groupBox_mapMain) < 0:
            visual_layout.addWidget(self.groupBox_mapMain)
        visual_layout.setStretchFactor(self.groupBox_4, 45)
        visual_layout.setStretchFactor(self.groupBox_mapMain, 55)

        if hasattr(self, "topRightVisualPanel"):
            self.topRightVisualPanel.hide()

        info_layout = self.bottomInfoPanel.layout()
        if info_layout is not None and hasattr(self, "groupBox_8"):
            self._layouttan_cikar(self.groupBox_8)
            if self.rightPanel.layout() is not None and self.rightPanel.layout().indexOf(self.groupBox_8) < 0:
                self.rightPanel.layout().insertWidget(0, self.groupBox_8)

        self.topVisualPanel.setMinimumWidth(520)
        self.bottomInfoPanel.setMinimumWidth(680)
        self.LCAMERA.setMinimumSize(400, 290)
        self.LMAINMAP.setMinimumSize(400, 340)
        self.groupBox_4.setMinimumHeight(310)
        self.groupBox_mapMain.setMinimumHeight(360)

    def _safety_panelini_sola_tasi(self):
        if not all(hasattr(self, ad) for ad in ("leftPanel", "groupBox_bottom")):
            return
        left_layout = self.leftPanel.layout()
        if left_layout is None:
            return

        self._layouttan_cikar(self.groupBox_bottom)
        if left_layout.indexOf(self.groupBox_bottom) < 0:
            left_layout.addWidget(self.groupBox_bottom)
        self.groupBox_bottom.setMaximumHeight(150)
        self.groupBox_bottom.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

    def _algoritma_gorsel_panelini_hazirla(self):
        if not hasattr(self, "groupBox_algorithm"):
            self.groupBox_algorithm = QtWidgets.QGroupBox("ALGORITHM / DETECTION", self.rightPanel)
            layout = QtWidgets.QVBoxLayout(self.groupBox_algorithm)
            layout.setContentsMargins(10, 20, 10, 10)
            layout.setSpacing(4)
            self.LALGORITHM = QtWidgets.QLabel("Waiting for detection / planning data", self.groupBox_algorithm)
            self.LALGORITHM.setAlignment(ALIGN_CENTER)
            self.LALGORITHM.setScaledContents(True)
            self.LALGORITHM.setMinimumHeight(130)
            self.LALGORITHM.setStyleSheet(
                "QLabel { background-color: #eaf2f8; color: #0b2239; "
                "border: 1px solid #7fb3d5; font-size: 11pt; font-weight: bold; }"
            )
            layout.addWidget(self.LALGORITHM)

        gorsel_yolu = os.path.join(KLASOR_YOLU, "images", "colreg.jpeg")
        if os.path.exists(gorsel_yolu):
            pixmap = QPixmap(gorsel_yolu)
            if not pixmap.isNull():
                self.LALGORITHM.setPixmap(pixmap)

        self.groupBox_algorithm.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.LALGORITHM.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def _sag_karar_panelini_duzenle(self):
        if not all(hasattr(self, ad) for ad in ("rightPanel", "groupBox_8", "groupBox_5")):
            return
        right_layout = self.rightPanel.layout()
        if right_layout is None:
            return

        self._algoritma_gorsel_panelini_hazirla()

        for widget in (self.groupBox_algorithm, self.groupBox_8, self.groupBox_5):
            self._layouttan_cikar(widget)

        right_layout.insertWidget(0, self.groupBox_algorithm)
        right_layout.insertWidget(1, self.groupBox_8)
        right_layout.insertWidget(2, self.groupBox_5)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        for widget in (self.groupBox_algorithm, self.groupBox_8, self.groupBox_5):
            widget.setMinimumHeight(150)
            widget.setMaximumHeight(16777215)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        right_layout.setStretchFactor(self.groupBox_algorithm, 1)
        right_layout.setStretchFactor(self.groupBox_8, 1)
        right_layout.setStretchFactor(self.groupBox_5, 1)

        if hasattr(self, "TXTSTATUSLOG"):
            self.TXTSTATUSLOG.setMinimumHeight(120)
            self.TXTSTATUSLOG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTSTATUSLOG.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            self.TXTSTATUSLOG.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)

        if hasattr(self, "groupBox_8") and hasattr(self, "LLOGIC") and hasattr(self, "TXTCOLREG"):
            decision_layout = self.groupBox_8.layout()
            if isinstance(decision_layout, QtWidgets.QGridLayout):
                self._layouttan_cikar(self.LLOGIC)
                if hasattr(self, "label_12"):
                    self._layouttan_cikar(self.label_12)
                self._layouttan_cikar(self.TXTCOLREG)
                decision_layout.addWidget(self.LLOGIC, 0, 0, 1, 2)
                if hasattr(self, "label_12"):
                    decision_layout.addWidget(self.label_12, 1, 0, 1, 2, ALIGN_CENTER)
                decision_layout.addWidget(self.TXTCOLREG, 2, 0, 1, 2)
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 1)
                decision_layout.setRowStretch(0, 0)
                decision_layout.setRowStretch(1, 0)
                decision_layout.setRowStretch(2, 1)

            self.LLOGIC.setMinimumHeight(34)
            self.LLOGIC.setMaximumHeight(54)
            self.LLOGIC.setWordWrap(True)
            self.LLOGIC.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            self.TXTCOLREG.setMinimumHeight(110)
            self.TXTCOLREG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def _batarya_paneli_tek_sutun_yap(self):
        if not hasattr(self, "groupBox_3"):
            return
        layout = self.groupBox_3.layout()
        if not isinstance(layout, QtWidgets.QGridLayout):
            return

        layout.setContentsMargins(10, 20, 10, 10)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        cell_widgets = [
            getattr(self, "label", None),
            getattr(self, "label_13", None),
            getattr(self, "label_14", None),
            getattr(self, "label_15", None),
            getattr(self, "label_16", None),
            getattr(self, "label_17", None),
        ]
        for widget in cell_widgets:
            if widget is not None:
                widget.setMinimumHeight(24)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
                widget.setWordWrap(False)

        if hasattr(self, "progressBar"):
            layout.addWidget(self.progressBar, 0, 0, 1, 2)

        for row, widget in enumerate((w for w in cell_widgets if w is not None), start=1):
            layout.addWidget(widget, row, 0, 1, 2)

        total_current_label = getattr(self, "label_18", None)
        total_voltage_label = getattr(self, "label_20", None)
        current_lcd = getattr(self, "lcdNumber", None)
        voltage_lcd = getattr(self, "lcdNumber_2", None)

        current_row = 7
        if total_current_label is not None:
            layout.addWidget(total_current_label, current_row, 0, 1, 2)
            total_current_label.setMinimumHeight(24)
            total_current_label.setWordWrap(False)
        if current_lcd is not None:
            layout.addWidget(current_lcd, current_row + 1, 0, 1, 2)
            current_lcd.setMinimumHeight(46)
            current_lcd.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        voltage_row = current_row + 2
        if total_voltage_label is not None:
            layout.addWidget(total_voltage_label, voltage_row, 0, 1, 2)
            total_voltage_label.setMinimumHeight(24)
            total_voltage_label.setWordWrap(False)
        if voltage_lcd is not None:
            layout.addWidget(voltage_lcd, voltage_row + 1, 0, 1, 2)
            voltage_lcd.setMinimumHeight(46)
            voltage_lcd.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        if not hasattr(self, "LBATTERYPOWER"):
            self.LBATTERYPOWER = QtWidgets.QLabel("POWER: 0.0 W", self.groupBox_3)
        if not hasattr(self, "LBATTERYWH"):
            self.LBATTERYWH = QtWidgets.QLabel("ENERGY LEFT: 0.0 Wh", self.groupBox_3)

        energy_style = "font-size: 12pt; font-weight: bold; color: #0b2239;"
        for widget in (self.LBATTERYPOWER, self.LBATTERYWH):
            widget.setStyleSheet(energy_style)
            widget.setMinimumHeight(26)
            widget.setWordWrap(False)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        layout.addWidget(self.LBATTERYPOWER, voltage_row + 2, 0, 1, 2)
        layout.addWidget(self.LBATTERYWH, voltage_row + 3, 0, 1, 2)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

    def _layout_ogelerini_temizle(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            child_layout = item.layout()
            if widget is not None:
                layout.removeWidget(widget)
            elif child_layout is not None:
                self._layout_ogelerini_temizle(child_layout)

    def _sistem_status_dikey_yap(self):
        if not hasattr(self, "groupBox") or self.groupBox.layout() is None:
            return
        layout = self.groupBox.layout()
        self._layout_ogelerini_temizle(layout)
        layout.setContentsMargins(14, 26, 14, 16)
        layout.setSpacing(14)

        for widget in (
            getattr(self, "pushButton", None),
            self._vessel_status_label,
            getattr(self, "pushButton_4", None),
            getattr(self, "pushButton_8", None),
            getattr(self, "comboBox_2", None),
        ):
            if widget is None:
                continue
            layout.addWidget(widget)
            widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            if widget is self._vessel_status_label:
                layout.addSpacing(12)

        if hasattr(self, "pushButton"):
            self.pushButton.setMinimumHeight(54)
        if self._vessel_status_label is not None:
            self._vessel_status_label.setMinimumHeight(62)
            self._vessel_status_label.setMaximumHeight(86)
        for buton in (getattr(self, "pushButton_4", None), getattr(self, "pushButton_8", None)):
            if buton is not None:
                buton.setMinimumHeight(50)
        if hasattr(self, "comboBox_2"):
            self.comboBox_2.setMinimumHeight(50)

    def _imu_paneli_dikey_okunur_yap(self):
        if not hasattr(self, "groupBox_2"):
            return
        layout = self.groupBox_2.layout()
        if not isinstance(layout, QtWidgets.QGridLayout):
            return

        layout.setContentsMargins(12, 22, 12, 12)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        roll = getattr(self, "pushButton_5", None)
        pitch = getattr(self, "pushButton_9", None)
        yaw = getattr(self, "pushButton_10", None)
        lat_title = getattr(self, "label_lat_title", None)
        lon_title = getattr(self, "label_lon_title", None)
        speed_title = getattr(self, "label_speed_title", None)
        lat_value = getattr(self, "pushButton_11", None)
        lon_value = getattr(self, "pushButton_12", None)
        speed_value = getattr(self, "lcdNumber_3", None)

        for row, widget in enumerate((roll, pitch, yaw)):
            if widget is not None:
                layout.addWidget(widget, row, 0, 1, 2)
                widget.setMinimumHeight(38)
                widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        pairs = (
            (lat_title, lat_value),
            (lon_title, lon_value),
            (speed_title, speed_value),
        )
        start_row = 3
        for offset, (title, value) in enumerate(pairs):
            row = start_row + offset
            if title is not None:
                layout.addWidget(title, row, 0)
                title.setMinimumHeight(34)
                title.setStyleSheet("font-size: 10pt; font-weight: bold; color: #0b2239;")
            if value is not None:
                layout.addWidget(value, row, 1)
                value.setMinimumHeight(38)
                value.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        if self._cog_label is not None:
            layout.addWidget(self._cog_label, start_row + len(pairs), 0, 1, 2, ALIGN_CENTER)
            self._cog_label.setMinimumSize(150, 36)
            self._cog_label.setMaximumHeight(42)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 2)

    def _mission_secimi_dikey_yap(self):
        if not hasattr(self, "groupBox_7") or self.groupBox_7.layout() is None:
            return
        layout = self.groupBox_7.layout()
        self._layout_ogelerini_temizle(layout)
        layout.setContentsMargins(14, 20, 14, 14)
        layout.setSpacing(8)
        if isinstance(layout, QtWidgets.QBoxLayout):
            layout.setDirection(QtWidgets.QBoxLayout.TopToBottom)

        for radio in (self.radioButton, self.radioButton_2, self.radioButton_3, self.radioButton_4):
            layout.addWidget(radio)
            radio.setMinimumHeight(34)
            radio.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    def _yeni_ui_layout_duzelt(self):
        if hasattr(self, "topVisualPanel") and hasattr(self, "bottomInfoPanel"):
            self._safety_panelini_sola_tasi()
            self._sol_gorsel_sag_bilgi_duzeni_duzenle()
            self._sag_karar_panelini_duzenle()
            self._batarya_paneli_tek_sutun_yap()
            self._mission_secimi_dikey_yap()

            main_layout = getattr(self, "verticalLayout_main", None)
            if main_layout is not None:
                main_layout.setStretchFactor(getattr(self, "mainSplitPanel", self.topVisualPanel), 1)

            if self.topVisualPanel.layout() is not None:
                self.topVisualPanel.layout().setStretchFactor(self.groupBox_4, 45)
                self.topVisualPanel.layout().setStretchFactor(self.groupBox_mapMain, 55)

            self.topVisualPanel.setMinimumHeight(0)
            self.groupBox_4.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.groupBox_mapMain.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.LCAMERA.setMinimumSize(400, 290)
            self.LMAINMAP.setMinimumSize(400, 340)

        if hasattr(self, "leftPanel") and hasattr(self, "middlePanel") and hasattr(self, "rightPanel"):
            panel_layout = getattr(self, "horizontalLayout", None)
            if panel_layout is not None:
                panel_layout.setStretch(0, 30)
                panel_layout.setStretch(1, 30)
                panel_layout.setStretch(2, 40)
                panel_layout.setStretchFactor(self.leftPanel, 30)
                panel_layout.setStretchFactor(self.middlePanel, 30)
                panel_layout.setStretchFactor(self.rightPanel, 40)

            self.leftPanel.setMinimumWidth(330)
            self.middlePanel.setMinimumWidth(330)
            self.rightPanel.setMinimumWidth(430)
            for panel in (self.leftPanel, self.middlePanel, self.rightPanel):
                panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        elif hasattr(self, "leftPanel") and hasattr(self, "rightPanel"):
            panel_layout = getattr(self, "horizontalLayout", None)
            if panel_layout is not None:
                panel_layout.setStretch(0, 35)
                panel_layout.setStretch(1, 65)
                panel_layout.setStretchFactor(self.leftPanel, 35)
                panel_layout.setStretchFactor(self.rightPanel, 65)

            self.leftPanel.setMinimumWidth(420)
            self.rightPanel.setMinimumWidth(760)
            for panel in (self.leftPanel, self.rightPanel):
                panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        if (
            hasattr(self, "groupBox_4")
            and hasattr(self, "LCAMERA")
            and hasattr(self, "groupBox_8")
            and self.groupBox_8.parentWidget() == self.groupBox_4
        ):
            right_layout = getattr(self, "rightPanel", None)
            if right_layout is not None and self.rightPanel.layout() is not None and hasattr(self, "groupBox_5"):
                self.rightPanel.layout().setStretchFactor(self.groupBox_4, 4)
                self.rightPanel.layout().setStretchFactor(self.groupBox_5, 1)

            layout = self.groupBox_4.layout()
            if layout is None:
                layout = QtWidgets.QVBoxLayout(self.groupBox_4)
                layout.setContentsMargins(10, 24, 10, 10)
                layout.setSpacing(10)
                layout.addWidget(self.LCAMERA, 4)
                layout.addWidget(self.groupBox_8, 4)
            else:
                layout.setStretchFactor(self.LCAMERA, 4)
                layout.setStretchFactor(self.groupBox_8, 4)

            self.LCAMERA.setMinimumSize(320, 180)
            self.LCAMERA.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.groupBox_8.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        if hasattr(self, "groupBox_8") and hasattr(self, "LLOGIC") and hasattr(self, "TXTCOLREG"):
            decision_layout = self.groupBox_8.layout()
            if decision_layout is None:
                decision_layout = QtWidgets.QGridLayout(self.groupBox_8)
                decision_layout.setContentsMargins(10, 24, 10, 10)
                decision_layout.setSpacing(6)
                decision_layout.addWidget(self.LLOGIC, 0, 0, 2, 1)
                if hasattr(self, "label_12"):
                    decision_layout.addWidget(self.label_12, 0, 1)
                decision_layout.addWidget(self.TXTCOLREG, 1, 1)
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 2)
                decision_layout.setRowStretch(1, 1)
            else:
                decision_layout.setColumnStretch(0, 1)
                decision_layout.setColumnStretch(1, 2)
                decision_layout.setRowStretch(1, 1)

            self.LLOGIC.setMinimumSize(170, 150)
            self.LLOGIC.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTCOLREG.setMinimumSize(300, 150)
            self.TXTCOLREG.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            self.TXTCOLREG.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            self.TXTCOLREG.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.TXTCOLREG.setLineWrapMode(QtWidgets.QTextEdit.WidgetWidth)
            if hasattr(self, "label_12"):
                self.label_12.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self._sag_karar_panelini_duzenle()

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

    def plan_ac(self):
        self.plan_pencere = GorevPlaniEkrani(self.sistem)
        self.plan_pencere.show()

    def _ana_haritayi_hazirla(self):
        self._ana_harita_katmani = None
        if not hasattr(self, "LMAINMAP"):
            return

        map_path = os.path.join(KLASOR_YOLU, "images", MAP_IMAGE_FILE)
        if os.path.exists(map_path):
            self.LMAINMAP.setPixmap(QPixmap(map_path))
            self.LMAINMAP.setScaledContents(True)
        self.LMAINMAP.setText("")
        self.LMAINMAP.setStyleSheet("background-color: #1b2a34; border: 1px solid #7fb3d5;")

        self._ana_harita_katmani = HaritaCizimKatmani(self.LMAINMAP)
        self._ana_harita_katmani.setGeometry(self.LMAINMAP.rect())
        self._ana_harita_katmani.raise_()
        self._ana_harita_katmani.set_waypoints(self.sistem.gorev_noktalarini_al())

    def _ana_harita_waypoint_guncelle(self, waypoints):
        if self._ana_harita_katmani is not None:
            self._ana_harita_katmani.set_waypoints(waypoints)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if getattr(self, "_ana_harita_katmani", None) is not None and hasattr(self, "LMAINMAP"):
            self._ana_harita_katmani.setGeometry(self.LMAINMAP.rect())

    def _ek_veri_widgetlarini_hazirla(self):
        self._vessel_status_label = getattr(self, "LVESSELSTATUS", None)
        if self._vessel_status_label is None:
            self._vessel_status_label = QtWidgets.QLabel("DISCONNECTED - No Telemetry", self.groupBox)
            if self.groupBox.layout() is not None:
                self.groupBox.layout().addWidget(self._vessel_status_label)
        self._vessel_status_label.setAlignment(ALIGN_CENTER)
        self._vessel_status_label.setMinimumHeight(38)
        self._vessel_status_label.setMaximumHeight(58)
        self._vessel_status_label.setWordWrap(True)
        self._vessel_status_label.setStyleSheet(self._vessel_status_stili("#2980b9"))
        self._sistem_status_dikey_yap()

        self._cog_label = getattr(self, "LCOG", None)
        if self._cog_label is None:
            self._cog_label = QtWidgets.QLabel("COG: 0.0°", self.groupBox_2)
        if self.groupBox_2.layout() is not None:
            self.groupBox_2.layout().addWidget(self._cog_label, 3, 1, 1, 1, ALIGN_CENTER)
        self._cog_label.setAlignment(ALIGN_CENTER)
        self._cog_label.setMinimumSize(100, 28)
        self._cog_label.setMaximumHeight(34)
        self._cog_label.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self._cog_label.setStyleSheet(
            "QLabel { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 10pt; font-weight: bold; padding: 3px; }"
        )
        self._imu_paneli_dikey_okunur_yap()

        map_status_stili = (
            "QLabel { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 10pt; font-weight: bold; padding: 3px; }"
        )
        for ad in ("LMAPGPS", "LMAPSATS", "LMAPDIST"):
            widget = getattr(self, ad, None)
            if widget is not None:
                widget.setStyleSheet(map_status_stili)

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
            etiket.setStyleSheet("font-size: 12pt; font-weight: bold; color: #0b2239;")

        for etiket in (self.label_18, self.label_20):
            etiket.setStyleSheet("font-size: 11pt; font-weight: bold; color: #0b2239;")

        self.textEdit.setPlainText(
            "Active COLREG rule: --\n\n"
            "Situation: --\n\n"
            "Decision: --\n\n"
            "Reason: --\n\n"
            "Collision risk: --\n\n"
            "Target vessel distance: --"
        )
        self.label_8.setText("Mission state: Waiting for telemetry")

    def _vessel_status_stili(self, renk):
        return (
            f"QLabel {{ background-color: {renk}; color: white; border-radius: 8px; "
            "font-size: 11pt; font-weight: bold; padding: 6px; }}"
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
        self._veri_gostergesi_stili = (
            "QPushButton { background-color: #f8fcff; color: #0b2239; "
            "border: 2px solid #00a8e8; border-radius: 6px; "
            "font-size: 11pt; font-weight: bold; padding: 6px; }"
        )
        self._lcd_stili = (
            "QLCDNumber { color: #00D2FF; border: 2px solid #00a8e8; "
            "border-radius: 5px; background-color: #f8fcff; }"
        )
        self._batarya_stili = (
            "QProgressBar { border: 1px solid #7fb3d5; border-radius: 4px; "
            "background-color: #eef6fb; text-align: center; font-weight: bold; }"
            "QProgressBar::chunk { background-color: #3498db; border-radius: 3px; }"
        )
        self._acil_stop_stili = (
            "QPushButton { background-color: #c0392b; color: white; "
            "border-radius: 10px; font-weight: bold; padding: 8px; "
            "border: 2px solid #922b21; }"
            "QPushButton:hover { background-color: #e74c3c; }"
            "QPushButton:pressed { background-color: #922b21; }"
        )

        for buton in (
            self.pushButton,
            self.pushButton_2,
            self.pushButton_6,
        ):
            buton.setStyleSheet(self._komut_buton_stili)
        for buton in (
            self.pushButton_5,
            self.pushButton_9,
            self.pushButton_10,
            self.pushButton_11,
            self.pushButton_12,
        ):
            buton.setStyleSheet(self._veri_gostergesi_stili)

        for lcd in (self.lcdNumber, self.lcdNumber_2, self.lcdNumber_3):
            lcd.setStyleSheet(self._lcd_stili)

        for buton in (self.pushButton, self.pushButton_2):
            buton.setMinimumHeight(48)
            buton.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self.textEdit.setStyleSheet(
            "QTextEdit { font-size: 11pt; font-weight: bold; color: #0b2239; "
            "background-color: #ffffff; border: 1px solid #7fb3d5; padding: 3px; }"
        )

        for etiket in self._log_etiketleri:
            etiket.setMinimumHeight(28)
            etiket.setStyleSheet("font-size: 12pt; font-weight: bold; color: #1f618d;")

        self.progressBar.setStyleSheet(self._batarya_stili)
        self.pushButton_7.setStyleSheet(self._acil_stop_stili)
        self.pushButton_6.setStyleSheet(self._komut_buton_stili)
        self.comboBox_2.setStyleSheet(self._mode_combo_stili)

        if hasattr(self, "groupBox_7"):
            self.groupBox_7.setMaximumHeight(205)
            self.groupBox_7.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        radio_stili = (
            "QRadioButton { font-size: 12pt; font-weight: bold; color: #0b2239; "
            "spacing: 10px; padding: 6px; }"
            "QRadioButton::indicator { width: 20px; height: 20px; }"
            "QRadioButton::indicator:unchecked { border: 2px solid #5dade2; "
            "border-radius: 10px; background-color: #ffffff; }"
            "QRadioButton::indicator:checked { border: 2px solid #1f618d; "
            "border-radius: 10px; background-color: #3498db; }"
        )
        for radio in (self.radioButton, self.radioButton_2, self.radioButton_3, self.radioButton_4):
            radio.setMinimumHeight(36)
            radio.setStyleSheet(radio_stili)
            radio.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        if hasattr(self, "groupBox_6"):
            self.groupBox_6.setMaximumHeight(380)
            self.groupBox_6.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox"):
            self.groupBox.setMaximumHeight(360)
            self.groupBox.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox_2"):
            self.groupBox_2.setMaximumHeight(360)
            self.groupBox_2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "groupBox_3"):
            self.groupBox_3.setMaximumHeight(500)
            self.groupBox_3.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)

        if hasattr(self, "middlePanel") and self.middlePanel.layout() is not None:
            orta_layout = self.middlePanel.layout()
            if hasattr(self, "groupBox_6"):
                orta_layout.setStretchFactor(self.groupBox_6, 0)
            if hasattr(self, "pushButton_wifi"):
                orta_layout.setStretchFactor(self.pushButton_wifi, 0)
            if hasattr(self, "pushButton_7"):
                orta_layout.setStretchFactor(self.pushButton_7, 0)

        for buton in (self.pushButton_6, self.pushButton_wifi, self.pushButton_7):
            buton.setMinimumHeight(58)
            buton.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self._sistem_status_dikey_yap()
        self._imu_paneli_dikey_okunur_yap()
        self._mission_secimi_dikey_yap()

    def _decision_metni(self, d):
        colreg = d.get("active_colreg_rule") or d.get("colreg_rule") or d.get("colreg") or "--"
        situation = (
            d.get("colreg_situation")
            or d.get("situation")
            or d.get("encounter_type")
            or d.get("encounter")
            or "--"
        )
        decision = d.get("maneuver") or d.get("decision") or d.get("action") or "--"
        reason = d.get("decision_reason") or d.get("colreg_reason") or d.get("reason") or "--"
        collision_risk = d.get("collision_risk") or d.get("risk") or d.get("risk_level") or "--"
        target_distance = (
            d.get("target_vessel_distance")
            or d.get("target_distance")
            or d.get("obstacle_distance")
            or d.get("distance_to_target")
        )
        if target_distance is None:
            target_distance_text = "--"
        else:
            try:
                target_distance_text = f"{float(target_distance):.1f} m"
            except (TypeError, ValueError):
                target_distance_text = str(target_distance)

        return (
            f"Active COLREG rule: {colreg or '--'}\n\n"
            f"Situation: {situation or '--'}\n\n"
            f"Decision: {decision or '--'}\n\n"
            f"Reason: {reason or '--'}\n\n"
            f"Collision risk: {collision_risk or '--'}\n\n"
            f"Target vessel distance: {target_distance_text}"
        )

        mesafe = d.get("distance_to_next_wp", d.get("mesafe", 0.0))
        colreg = d.get("colreg_rule", d.get("colreg", "--"))
        decision = d.get("maneuver", d.get("decision", "--"))
        reason = d.get("decision_reason", d.get("colreg_reason", "--"))
        mission = d.get("active_mission") or "--"
        status, _renk = self._vessel_status_bilgisi(d)
        compact_status = status
        for prefix in ("🔵 ", "🟢 ", "🟡 ", "🔴 "):
            compact_status = compact_status.replace(prefix, "")
        compact_status = compact_status.split(" - ", 1)[0]
        return (
            f"Mission: {mission}\n\n"
            f"Status: {compact_status}\n\n"
            f"Distance to next WP: {float(mesafe or 0.0):.1f} m\n\n"
            f"COLREG rule: {colreg or '--'}\n\n"
            f"Decision: {decision or '--'}\n\n"
            f"Reason: {reason or '--'}"
        )

    def _vessel_status_bilgisi(self, d):
        mod = str(d.get("mod", "UNKNOWN") or "UNKNOWN").upper()
        link_ok = bool(d.get("link_ok"))
        armed = bool(d.get("armed"))
        mission_active = bool(d.get("active_mission"))
        system_status = str(d.get("system_status", "") or "").upper()

        if (
            mod == "EMERGENCY"
            or "FAILSAFE" in system_status
            or "CRITICAL" in system_status
            or "EMERGENCY" in system_status
        ):
            return "OUT OF CONTROL - Communication Lost", "#c0392b"
        if not d.get("baglanti"):
            return "DISCONNECTED - No Telemetry", "#7f8c8d"
        if d.get("baglanti") and not link_ok:
            return "OUT OF CONTROL - Communication Lost", "#c0392b"
        if not link_ok:
            return "STANDBY - Waiting for Mission", "#2980b9"
        if mod in ("AUTO", "GUIDED"):
            detail = "Executing Mission" if mission_active else "Autonomous Mode Active"
            return f"AUTONOMOUS - {detail}", "#27ae60"
        if mod in ("MANUAL", "HOLD", "STEERING", "LEARNING", "ACRO", "LOITER"):
            if armed:
                return "REMOTE CONTROL - Operator Driving", "#f1c40f"
            return "STANDBY - Waiting for Mission", "#2980b9"
        if armed:
            return f"REMOTE CONTROL - {mod}", "#f1c40f"
        return "STANDBY - Waiting for Mission", "#2980b9"

    def _vessel_status_guncelle(self, d):
        if self._vessel_status_label is None:
            return
        status, renk = self._vessel_status_bilgisi(d)
        self._vessel_status_label.setText(status)
        self._vessel_status_label.setStyleSheet(self._vessel_status_stili(renk))

    def _batarya_guncelle(self, d):
        battery = d.get("battery", {})
        percent = int(d.get("pil_yuzde", battery.get("percentage", 0)) or 0)
        percent = max(0, min(percent, 100))
        voltage = float(d.get("voltaj", battery.get("total_voltage", 0.0)) or 0.0)
        current = float(d.get("akim", battery.get("current", 0.0)) or 0.0)
        cells = battery.get("cell_voltages", d.get("cell_voltages", [])) or []
        power_w = float(battery.get("power_w", d.get("power_w", voltage * current)) or 0.0)
        remaining_wh = battery.get("remaining_wh", d.get("remaining_wh", battery.get("energy_wh")))
        if remaining_wh is None:
            capacity_wh = float(battery.get("capacity_wh", d.get("capacity_wh", DEFAULT_BATTERY_CAPACITY_WH)) or 0.0)
            remaining_wh = capacity_wh * percent / 100.0
        try:
            remaining_wh = float(remaining_wh)
        except (TypeError, ValueError):
            remaining_wh = 0.0

        self.progressBar.setValue(percent)
        self.lcdNumber.display(current)
        self.lcdNumber_2.display(voltage)
        if hasattr(self, "LBATTERYPOWER"):
            self.LBATTERYPOWER.setText(f"POWER: {power_w:.1f} W")
        if hasattr(self, "LBATTERYWH"):
            self.LBATTERYWH.setText(f"ENERGY LEFT: {remaining_wh:.1f} Wh")

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
        if self._cog_label is not None:
            self._cog_label.setText(f"COG: {float(d.get('cog', 0.0) or 0.0):.1f}°")

        self.lcdNumber_3.display(d.get("hiz", 0.0))
        self.pushButton_11.setText(str(d.get("lat", 0.0)))
        self.pushButton_12.setText(str(d.get("lon", 0.0)))
        if getattr(self, "_ana_harita_katmani", None) is not None:
            self._ana_harita_katmani.set_vehicle(
                {
                    "lat": d.get("lat", 0.0),
                    "lon": d.get("lon", 0.0),
                    "yaw": d.get("yaw", 0.0),
                    "speed": d.get("hiz", 0.0),
                }
            )
        if hasattr(self, "LMAPGPS"):
            self.LMAPGPS.setText(f"GPS: {d.get('gps', 0)}")
        if hasattr(self, "LMAPSATS"):
            self.LMAPSATS.setText(f"SATS: {d.get('gps_uydu', 0)}")
        if hasattr(self, "LMAPDIST"):
            self.LMAPDIST.setText(f"NEXT WP: {float(d.get('mesafe', 0.0) or 0.0):.1f} m")
        self._batarya_guncelle(d)
        self.textEdit.setPlainText(self._decision_metni(d))
        self._vessel_status_guncelle(d)
        self.label_8.setText(d.get("decision_log", "Waiting for mission data..."))

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
        if d.get("mode_change_pending"):
            yeni_stil = bekliyor_stil
        else:
            yeni_stil = self._mode_combo_stili

        if self._mode_combo_son_stil != yeni_stil:
            self.comboBox_2.setStyleSheet(yeni_stil)
            self._mode_combo_son_stil = yeni_stil

    def _armed_butonunu_sabitle(self):
        self.pushButton_4.setMinimumSize(180, 50)
        self.pushButton_4.setStyleSheet(
            "background-color: #2ecc71; color: white; "
            "font-weight: bold; border-radius: 10px; "
            "border: 2px solid #27ae60; padding: 8px 18px;"
        )
        self.pushButton_4.setText("ARMED")

    def _disarmed_butonunu_sabitle(self):
        self.pushButton_8.setMinimumSize(180, 50)
        self.pushButton_8.setStyleSheet(
            "background-color: #e74c3c; color: white; "
            "font-weight: bold; border-radius: 6px; "
            "border: 2px solid #c0392b; padding: 8px 18px;"
        )
        self.pushButton_8.setText("DISARMED")

    def _arm_butonlarini_sabitle(self):
        self._armed_butonunu_sabitle()
        self._disarmed_butonunu_sabitle()

    def log_ekle(self, m):
        temel_log_stili = "font-size: 12pt; font-weight: bold;"
        if "!!!" in m or "ERROR" in m or "FAIL" in m:
            stil = temel_log_stili + " color: #e74c3c;"
        elif (
            "COMPLETED" in m
            or "SUCCESS" in m
            or "CONFIRMED" in m
        ):
            stil = temel_log_stili + " color: #2ecc71;"
        else:
            stil = temel_log_stili + " color: #3498db;"

        self._log_gecmisi.insert(0, (f">> {m}", stil))
        log_limit = 80 if hasattr(self, "TXTSTATUSLOG") else len(self._log_etiketleri)
        self._log_gecmisi = self._log_gecmisi[:log_limit]

        if hasattr(self, "TXTSTATUSLOG"):
            html_lines = []
            for metin, satir_stili in self._log_gecmisi:
                color = "#3498db"
                if "#e74c3c" in satir_stili:
                    color = "#e74c3c"
                elif "#2ecc71" in satir_stili:
                    color = "#2ecc71"
                safe_text = (
                    metin.replace("&", "&amp;")
                    .replace("<", "&lt;")
                    .replace(">", "&gt;")
                )
                html_lines.append(
                    f"<div style='font-size:11pt;font-weight:700;color:{color};white-space:nowrap;'>{safe_text}</div>"
                )
            self.TXTSTATUSLOG.setHtml("".join(html_lines))
            return

        for i, etiket in enumerate(self._log_etiketleri):
            if i < len(self._log_gecmisi):
                metin, satir_stili = self._log_gecmisi[i]
                etiket.setText(metin)
                etiket.setStyleSheet(satir_stili)
            else:
                etiket.setText("")
                etiket.setStyleSheet("")

    def icra(self):
        if self.radioButton.isChecked():
            gorev = "M1"
        elif self.radioButton_2.isChecked():
            gorev = "M2"
        elif self.radioButton_3.isChecked():
            gorev = "M3"
        elif self.radioButton_4.isChecked():
            gorev = "M4"
        else:
            self.sistem.log_sinyali.emit("ERROR: Select a mission before EXECUTE. Vehicle remains in HOLD.")
            self.sistem.mod_ayarla_ad("HOLD")
            return
        self.sistem.gorev_baslat(gorev)

    def closeEvent(self, event):
        self.sistem.kapat()
        event.accept()
