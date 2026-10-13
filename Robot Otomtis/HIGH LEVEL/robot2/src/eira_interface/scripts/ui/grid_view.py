from PySide6.QtWidgets import QWidget, QPushButton, QGraphicsOpacityEffect
from PySide6.QtGui import QPainter, QColor, QFont, QPen
from PySide6.QtCore import Qt, Signal, QRectF, QPointF
from .constants import THEMES, KFS_LIMITS, sw, sh, sf

class MatchGrid(QWidget):
    grid_clicked = Signal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_blue = True
        self.grid_data = {i: 0 for i in range(1, 13)}
        self.kfs_counts = {1: 0, 2: 0, 3: 0}
        self.is_locked = False
        self.theme_name = "dark"
        self._qcolors = {}
        self.cell_buttons = {}
        self.plan_path_nodes = []
        self.plan_picked_kfs = []
        self.plan_visual_path = []
        self.plan_visible = False
        
        self._row_map = {10:0, 11:0, 12:0, 7:1, 8:1, 9:1, 4:2, 5:2, 6:2, 1:3, 2:3, 3:3}
        self._col_blue = {10:0, 11:1, 12:2, 7:0, 8:1, 9:2, 4:0, 5:1, 6:2, 1:0, 2:1, 3:2}
        self._col_red = {12:0, 11:1, 10:2, 9:0, 8:1, 7:2, 6:0, 5:1, 4:2, 3:0, 2:1, 1:2}
        
        # self._font_grid_large = QFont("Arial", sf(24), QFont.Bold)
        self._font_grid_large = QFont("Arial", sf(48), QFont.Bold)

        self._font_grid_small = QFont("Arial", sf(12), QFont.Bold)
        self._font_kfs = QFont("Arial", sf(18), QFont.Black)
        self._create_cell_buttons()
        self.set_theme(self.theme_name)

    def _create_cell_buttons(self):
        for gid in range(1, 13):
            self.cell_buttons[gid] = {}

            for obj_type in self._allowed_objects(gid):
                btn = QPushButton(self)
                btn.setCursor(Qt.PointingHandCursor)
                btn.clicked.connect(
                    lambda _, cell_id=gid, obj=obj_type: self.grid_clicked.emit(
                        cell_id,
                        obj,
                    )
                )

                self.cell_buttons[gid][obj_type] = btn

    def _allowed_objects(self, gid):
        objects = [1, 2, 3]

        if gid in [1, 2, 3]:
            objects.remove(3)

        if gid in [5, 8]:
            objects.remove(1)

        return objects

    def set_theme(self, theme_key):
        self.theme_name = theme_key
        self._qcolors = {k: QColor(v) for k, v in THEMES[theme_key].items()}
        self._refresh_button_styles()
        self.update()

    def set_kfs_counts(self, counts):
        self.kfs_counts = dict(counts)
        self._refresh_button_styles()
        self._layout_cell_buttons()
        self.update()

    def set_locked(self, is_locked):
        self.is_locked = is_locked
        self._layout_cell_buttons()

    def set_team(self, team_id):
        self.is_blue = team_id == 0
        self._layout_cell_buttons()
        self.update()

    def set_kfs_retrieval_plan(self, path_nodes, picked_kfs, visual_path, visible):
        self.plan_path_nodes = list(path_nodes)
        self.plan_picked_kfs = list(picked_kfs)
        self.plan_visual_path = list(visual_path)
        self.plan_visible = bool(visible)
        self.update()

    def _get_pos(self, gid):
        return self._row_map[gid], self._col_blue[gid] if self.is_blue else self._col_red[gid]

    def _cell_rect(self, gid):
        w, h = self.width(), self.height()

        pad_x, pad_y = sw(15), sh(15)
        gap_x, gap_y = sw(12), sh(12)

        cell_w = (w - 2 * pad_x - 2 * gap_x) / 3
        cell_h = (h - 2 * pad_y - 3 * gap_y) / 4

        r, c = self._get_pos(gid)
        x = pad_x + c * (cell_w + gap_x)
        y = pad_y + r * (cell_h + gap_y)

        return QRectF(x, y, cell_w, cell_h)

    def _is_limit_reached(self, obj_type):
        return self.kfs_counts.get(obj_type, 0) >= KFS_LIMITS[obj_type]

    def _available_objects(self, gid):
        return [
            obj_type
            for obj_type in self._allowed_objects(gid)
            if not self._is_limit_reached(obj_type)
        ]

    def _refresh_button_styles(self):
        theme = THEMES[self.theme_name]
        labels = {1: "R1", 2: "R2", 3: "FAKE"}
        colors = {1: "kfs1", 2: "kfs2", 3: "fake"}

        for buttons in self.cell_buttons.values():
            for obj_type, btn in buttons.items():
                opacity = QGraphicsOpacityEffect(btn)
                opacity.setOpacity(1.0)
                btn.setGraphicsEffect(opacity)

                left = max(
                    0,
                    KFS_LIMITS[obj_type] - self.kfs_counts.get(obj_type, 0),
                )
                fg_color = (
                    "black"
                    if obj_type in [1, 2] and self.theme_name == "light"
                    else "white"
                )

                # btn.setText(f"{labels[obj_type]}\n{left}/{KFS_LIMITS[obj_type]}")
                btn.setText(f"{labels[obj_type]}")

                # btn.setStyleSheet(
                #     f"QPushButton {{ "
                #     f"background-color: {theme[colors[obj_type]]}; "
                #     f"color: {fg_color}; "
                #     f"font-size: {sf(10)}px; "
                #     f"font-weight: bold; "
                #     f"border: {sw(2)}px solid {theme['border']}; "
                #     f"border-radius: {sw(4)}px; "
                #     f"padding: 0px; "
                #     f"}} "
                #     f"QPushButton:hover {{ "
                #     f"border-color: {theme['accent']}; "
                #     f"}}"
                # )

                btn.setStyleSheet(
                    f"QPushButton {{ "
                    f"background-color: rgba(15, 23, 42, 0.20); "
                    f"color: white; "
                    f"font-size: {sf(22)}px; "
                    f"font-weight: bold; "
                    f"border: {sw(4)}px solid {theme[colors[obj_type]]}; "
                    f"border-radius: {sw(7)}px; "
                    f"padding: 0px; "
                    f"}} "
                    f"QPushButton:hover {{ "
                    f"background-color: rgba(15, 23, 42, 0.30); "
                    f"border-color: white; "
                    f"}}"
                )

    def _layout_cell_buttons(self):
        margin = sw(8)
        gap = sw(5)

        for gid, buttons in self.cell_buttons.items():
            is_filled = self.grid_data.get(gid, 0) > 0
            visible_objects = [] if is_filled else self._available_objects(gid)
            rect = self._cell_rect(gid)
            button_count = len(visible_objects)

            if button_count == 0:
                for btn in buttons.values():
                    btn.setVisible(False)
                continue

            total_gap = gap * (button_count - 1)
            button_w = max(
                1,
                int((rect.width() - (2 * margin) - total_gap) / button_count),
            )
            button_h = max(1, int(rect.height() - (2 * margin)))
            x = int(rect.x() + margin)
            y = int(rect.y() + margin)

            for index, obj_type in enumerate(visible_objects):
                btn = buttons[obj_type]
                btn.setGeometry(x + index * (button_w + gap), y, button_w, button_h)
                btn.setDisabled(self.is_locked)
                btn.setVisible(True)

            for obj_type, btn in buttons.items():
                if obj_type not in visible_objects:
                    btn.setVisible(False)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._layout_cell_buttons()

    def paintEvent(self, event):
        self._layout_cell_buttons()

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        bg_color = self._qcolors["blue_zone"] if self.is_blue else self._qcolors["red_zone"]
        painter.fillRect(self.rect(), bg_color)

        w, h = self.width(), self.height()
        
        pad_x, pad_y = sw(15), sh(15)
        gap_x, gap_y = sw(12), sh(12)
        
        cell_w = (w - 2 * pad_x - 2 * gap_x) / 3
        cell_h = (h - 2 * pad_y - 3 * gap_y) / 4

        for gid in range(1, 13):
            rect = self._cell_rect(gid)
            
            c_key = "grid_dark" if gid in [2, 4, 10, 12] else ("grid_light" if gid in [6, 8] else "grid_med")
            painter.setBrush(self._qcolors[c_key])
            painter.setPen(QPen(Qt.white, sf(3)))
            painter.drawRect(rect)
            
            obj = self.grid_data.get(gid, 0)
            if obj > 0:
                painter.setPen(QPen(Qt.white))
                painter.setFont(self._font_grid_small)
                painter.drawText(rect.adjusted(sw(8), sh(8), 0, 0), Qt.AlignLeft | Qt.AlignTop, str(gid))

                k_w = cell_w * 0.8
                k_h = cell_h * 0.55
                cx = rect.center().x()
                cy = rect.center().y() + (cell_h * 0.1)
                k_rect = QRectF(cx - k_w / 2, cy - k_h / 2, k_w, k_h)

                cmap = {1: "kfs1", 2: "kfs2", 3: "fake"}
                painter.setBrush(self._qcolors[cmap[obj]])
                painter.setPen(QPen(Qt.black if self.theme_name == "light" else Qt.white, sf(3)))
                painter.drawRect(k_rect)

                painter.setPen(QPen(Qt.black))
                painter.setFont(self._font_kfs)
                kfs_names = {1: "R1", 2: "R2", 3: "FAKE"}
                painter.drawText(k_rect, Qt.AlignCenter, kfs_names[obj])
            # else:
            #     painter.setPen(QPen(Qt.white))
            #     painter.setFont(self._font_grid_small)
            #     painter.drawText(rect.adjusted(sw(8), sh(6), 0, 0), Qt.AlignLeft | Qt.AlignTop, str(gid))

            else:
                watermark_color = QColor(255, 255, 255, 200)  # putih transparan
                painter.setPen(QPen(watermark_color))
                painter.setFont(self._font_grid_large)
                painter.drawText(rect, Qt.AlignCenter, str(gid))

        if self.plan_visible:
            self._draw_kfs_retrieval_plan(painter)

    def _draw_kfs_retrieval_plan(self, painter):
        path_points = self._plan_path_points()
        picked_kfs = [
            int(node)
            for node in self.plan_picked_kfs
            if 1 <= int(node) <= 12
        ]

        if not path_points:
            return

        path_color = QColor(236, 72, 153, 235)
        point_color = QColor(255, 182, 219, 245)
        picked_fill = QColor(131, 24, 67, 210)
        picked_border = QColor(255, 205, 235, 255)

        painter.save()

        if len(path_points) > 1:
            painter.setPen(
                QPen(
                    path_color,
                    sf(4),
                    Qt.SolidLine,
                    Qt.RoundCap,
                    Qt.RoundJoin,
                )
            )
            for start, end in zip(path_points, path_points[1:]):
                painter.drawLine(start, end)

        painter.setBrush(point_color)
        painter.setPen(QPen(path_color, sf(2)))
        point_radius = sh(6)

        for point in path_points:
            painter.drawEllipse(point, point_radius, point_radius)

        circle_radius = sh(18)
        painter.setFont(QFont("Arial", sf(16), QFont.Bold))

        for index, node in enumerate(picked_kfs):
            center = self._cell_rect(node).center()

            painter.setBrush(picked_fill)
            painter.setPen(QPen(picked_border, sf(4)))
            painter.drawEllipse(center, circle_radius, circle_radius)

            painter.setPen(QPen(Qt.white))
            painter.drawText(
                QRectF(
                    center.x() - circle_radius,
                    center.y() - circle_radius,
                    circle_radius * 2,
                    circle_radius * 2,
                ),
                Qt.AlignCenter,
                str(index + 1),
            )

        painter.restore()

    def _plan_path_points(self):
        if self.plan_visual_path:
            points = []

            for item in self.plan_visual_path:
                if len(item) != 3:
                    continue

                zone, node, anchor_node = [int(value) for value in item]
                point = self._visual_path_point(zone, node, anchor_node)

                if point is not None:
                    points.append(point)

            return points

        return [
            self._cell_rect(int(node)).center()
            for node in self.plan_path_nodes
            if 1 <= int(node) <= 12
        ]

    def _visual_path_point(self, zone, node, anchor_node):
        if not 1 <= int(anchor_node) <= 12:
            if not 1 <= int(node) <= 12:
                return None

            anchor_node = node

        rect = self._cell_rect(int(anchor_node))

        if int(zone) == 0:
            return QPointF(
                rect.center().x(),
                min(self.height() - sh(6), rect.bottom() + sh(8)),
            )

        if int(zone) == 2:
            return QPointF(
                rect.center().x(),
                max(sh(6), rect.top() - sh(8)),
            )

        return rect.center()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            w, h = self.width(), self.height()
            pad_x, pad_y = sw(15), sh(15)
            gap_x, gap_y = sw(12), sh(12)
            
            cell_w = (w - 2 * pad_x - 2 * gap_x) / 3
            cell_h = (h - 2 * pad_y - 3 * gap_y) / 4

            rx, ry = event.position().x() - pad_x, event.position().y() - pad_y
            if 0 <= rx < (cell_w * 3 + 2 * gap_x) and 0 <= ry < (cell_h * 4 + 3 * gap_y):
                c = int(rx / (cell_w + gap_x))
                r = int(ry / (cell_h + gap_y))
                for gid in range(1, 13):
                    gr, gc = self._get_pos(gid)
                    if gr == r and gc == c:
                        current_obj = self.grid_data.get(gid, 0)

                        if current_obj > 0:
                            self.grid_clicked.emit(gid, current_obj)

                        return
