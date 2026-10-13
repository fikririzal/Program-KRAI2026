from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Signal, Qt, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont

from .constants import sw, sh, sf, THEMES


class TicTacToeGrid(QWidget):
    cell_clicked = Signal(int)

    def __init__(self):
        super().__init__()

        self.is_blue = True
        self.theme_name = "dark"

        # format internal:
        # id_ttt -> step
        # contoh: {5: 1, 9: 2}
        # self.ttt_data = {}

        self.ttt_data = {
            5: 1,
            8: 2,
        }
        
        self.setMinimumHeight(sh(260))
        self.setMouseTracking(True)

    def set_theme(self, theme_name):
        self.theme_name = theme_name
        self.update()

    def set_team(self, team_id):
        self.is_blue = team_id == 0
        self.update()

    def reset(self):
        self.ttt_data = {
            5: 1,
            8: 2,
        }
        self.update()

    def get_matrix_steps(self):
        """
        Output 2 dimensi:
        [[step, id_ttt], [1, 5], [2, 9]]
        """
        items = []

        for id_ttt, step in self.ttt_data.items():
            items.append([step, id_ttt])

        items.sort(key=lambda x: x[0])
        return items

    def _visible_ids(self):
        if self.is_blue:
            return [
                [7, 8, 9],
                [4, 5, 6],
                [1, 2, 3],
            ]

        # tim merah mirror vertikal dari biru
        return [
            [9, 8, 7],
            [6, 5, 4],
            [3, 2, 1],
        ]

    def _cell_rect(self, row, col):
        margin_x = sw(18)
        margin_y = sh(18)
        gap = sw(8)

        area_w = self.width() - margin_x * 2
        area_h = self.height() - margin_y * 2

        cell_w = (area_w - gap * 2) / 3
        cell_h = (area_h - gap * 2) / 3

        x = margin_x + col * (cell_w + gap)
        y = margin_y + row * (cell_h + gap)

        return QRectF(x, y, cell_w, cell_h)

    def _id_from_pos(self, pos):
        ids = self._visible_ids()

        for row in range(3):
            for col in range(3):
                rect = self._cell_rect(row, col)

                if rect.contains(pos):
                    return ids[row][col]

        return None

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return

        id_ttt = self._id_from_pos(event.position())

        if id_ttt is None:
            return

        self.cell_clicked.emit(id_ttt)

    def paintEvent(self, event):
        theme = THEMES[self.theme_name]

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        ids = self._visible_ids()

        for row in range(3):
            for col in range(3):
                id_ttt = ids[row][col]
                rect = self._cell_rect(row, col)

                selected = id_ttt in self.ttt_data

                if selected:
                    if id_ttt in [1, 2, 3]:
                        bg_color = QColor("#FACC15")      # kuning
                        border_color = QColor("#FDE68A")
                        text_color = QColor("#111827")
                    elif self.is_blue:
                        bg_color = QColor("#2563EB")      # biru
                        border_color = QColor("#60A5FA")
                        text_color = QColor("#FFFFFF")
                    else:
                        bg_color = QColor("#A83E3E")      # merah
                        border_color = QColor("#F87171")
                        text_color = QColor("#FFFFFF")


                    text_color = QColor("#FFFFFF")
                else:
                    bg_color = QColor(theme["panel"])
                    border_color = QColor(theme["border"])
                    text_color = QColor(theme["log_text"])

                painter.setBrush(bg_color)
                painter.setPen(QPen(border_color, sw(2)))
                painter.drawRoundedRect(rect, sw(8), sw(8))

                painter.setPen(text_color)

                font = QFont()
                font.setBold(True)
                font.setPointSize(sf(18))
                painter.setFont(font)

                painter.drawText(rect, Qt.AlignCenter, str(id_ttt))

                if selected:
                    step = self.ttt_data[id_ttt]

                    step_rect = QRectF(
                        rect.left() + sw(6),
                        rect.top() + sh(6),
                        sw(36),
                        sh(26),
                    )

                    painter.setBrush(QColor("#111827"))
                    painter.setPen(Qt.NoPen)
                    painter.drawRoundedRect(step_rect, sw(5), sw(5))

                    painter.setPen(QColor("#FFFFFF"))

                    step_font = QFont()
                    step_font.setBold(True)
                    step_font.setPointSize(sf(10))
                    painter.setFont(step_font)

                    painter.drawText(step_rect, Qt.AlignCenter, f"S{step}")