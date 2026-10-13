from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QGridLayout,
    QGroupBox,
)
from PySide6.QtCore import Signal, Qt

from .constants import (
    THEMES,
    STRATEGY_OPTIONS,
    KFS_RETRIEVAL_OPTIONS,
    sw,
    sh,
    sf,
)


class ControlPanel(QWidget):
    team_changed = Signal(int)
    obj_selected = Signal(object)
    ready_clicked = Signal(dict)
    cancel_ready_clicked = Signal()
    restart_clicked = Signal()
    theme_toggled = Signal(str)
    fullscreen_toggled = Signal()
    ttc_strategy_changed = Signal(str)
    option_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.team = 0
        self.global_counts = {1: 0, 2: 0, 3: 0}
        self.selected_obj = None

        self.theme_name = "dark"
        self.is_ready_state = False

        self.labels = []
        self.option_buttons = {
            "strategy": {},
            "kfs_retrieval": {},
        }
        self.option_values = {
            "spearhead": "2",
            "strategy": STRATEGY_OPTIONS[0],
            "kfs_retrieval": KFS_RETRIEVAL_OPTIONS[0],
        }

        self.tactic_button_height = sh(48)
        self.tactic_spacing = sh(12)

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(sh(10))
        layout.setContentsMargins(0, 0, 0, 0)

        top_layout = QHBoxLayout()
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(sw(8))

        self.btn_fullscreen = QPushButton("⛶")
        self.btn_fullscreen.setFixedSize(sh(40), sh(40))
        self.btn_fullscreen.setCursor(Qt.PointingHandCursor)
        self.btn_fullscreen.clicked.connect(self.fullscreen_toggled.emit)
        top_layout.addWidget(self.btn_fullscreen)

        self.btn_theme = QPushButton("☀ Switch to Light Mode")
        self.btn_theme.setFixedHeight(sh(40))
        self.btn_theme.setCursor(Qt.PointingHandCursor)
        self.btn_theme.clicked.connect(self._toggle_theme)
        top_layout.addWidget(self.btn_theme, 1)

        layout.addLayout(top_layout)

        self.box_team = QGroupBox("TEAM SELECTION")
        team_layout = QHBoxLayout(self.box_team)
        team_layout.setContentsMargins(sw(12), sh(18), sw(12), sh(12))

        self.btn_blue = QPushButton("BLUE")
        self.btn_red = QPushButton("RED")

        for btn, team_value in [
            (self.btn_blue, 0),
            (self.btn_red, 1),
        ]:
            btn.setFixedHeight(sh(45))
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda _, t=team_value: self._set_team(t))
            team_layout.addWidget(btn)

        layout.addWidget(self.box_team)

        self.box_tactics = QGroupBox("TACTICS CONFIGURATION")
        tactics_layout = QVBoxLayout(self.box_tactics)
        tactics_layout.setContentsMargins(sw(12), sh(22), sw(12), sh(16))
        tactics_layout.setSpacing(self.tactic_spacing)

        self._create_button_group(
            "Tic Tac Toe Strategy:",
            STRATEGY_OPTIONS,
            tactics_layout,
            "strategy",
            columns=2,
        )

        self._create_button_group(
            "KFS Retrieval:",
            KFS_RETRIEVAL_OPTIONS,
            tactics_layout,
            "kfs_retrieval",
            columns=3,
        )

        layout.addWidget(self.box_tactics)

        layout.addStretch()

        self.btn_restart = QPushButton("RESTART MATCH")
        self.btn_restart.setFixedHeight(sh(45))
        self.btn_restart.setCursor(Qt.PointingHandCursor)
        self.btn_restart.clicked.connect(self.restart_clicked.emit)
        layout.addWidget(self.btn_restart)

    def _create_button_group(
        self,
        label_text,
        items,
        parent_layout,
        group_key,
        columns,
    ):
        label = QLabel(label_text)
        self.labels.append(label)
        parent_layout.addWidget(label)

        group_widget = QWidget()
        group_layout = QGridLayout(group_widget)
        group_layout.setContentsMargins(0, 0, 0, 0)
        group_layout.setSpacing(sh(8))

        for idx, item in enumerate(items):
            btn = QPushButton(self._option_button_text(group_key, item))
            btn.setFixedHeight(self.tactic_button_height)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda _, key=group_key, value=item: self._set_option(
                    key,
                    value,
                )
            )

            row = idx // columns
            col = idx % columns

            group_layout.addWidget(btn, row, col)
            self.option_buttons[group_key][item] = btn

        parent_layout.addWidget(group_widget)

    def _option_button_text(self, group_key, value):
        text = str(value)

        if group_key == "strategy" and text == "TIC TAC TOE CUSTOM":
            return "TTC CUSTOM"

        if group_key == "kfs_retrieval":
            return text

        return text

    def _toggle_theme(self):
        new_theme = "light" if self.theme_name == "dark" else "dark"

        self.btn_theme.setText(
            "🌓 Switch to Dark Mode"
            if new_theme == "light"
            else "☀ Switch to Light Mode"
        )

        self.theme_toggled.emit(new_theme)

    def apply_theme(self, theme_key):
        self.theme_name = theme_key
        theme = THEMES[theme_key]

        group_box_style = (
            f"QGroupBox {{ "
            f"font-size: {sf(13)}px; "
            f"font-weight: bold; "
            f"border: {sw(2)}px solid {theme['border']}; "
            f"border-radius: {sw(6)}px; "
            f"margin-top: {sh(12)}px; "
            f"background: {theme['panel']}; "
            f"}} "
            f"QGroupBox::title {{ "
            f"subcontrol-origin: margin; "
            f"subcontrol-position: top left; "
            f"left: {sw(10)}px; "
            f"top: 0px; "
            f"color: {theme['box_title']}; "
            f"padding: 0 {sw(5)}px; "
            f"}}"
        )

        self.box_team.setStyleSheet(group_box_style)
        self.box_tactics.setStyleSheet(group_box_style)

        for label in self.labels:
            label.setStyleSheet(
                f"font-size: {sf(13)}px; "
                f"font-weight: bold; "
                f"color: {theme['text']}; "
                f"margin-top: {sh(2)}px; "
                f"margin-bottom: 0px;"
            )

        self.btn_theme.setStyleSheet(
            f"QPushButton {{ "
            f"font-size: {sf(13)}px; "
            f"font-weight: bold; "
            f"background: {theme['bg']}; "
            f"color: {theme['text']}; "
            f"border: {sw(2)}px solid {theme['border']}; "
            f"border-radius: {sw(6)}px; "
            f"}}"
        )

        self.btn_fullscreen.setStyleSheet(
            f"QPushButton {{ "
            f"font-size: {sf(18)}px; "
            f"font-weight: bold; "
            f"background: {theme['bg']}; "
            f"color: {theme['text']}; "
            f"border: {sw(2)}px solid {theme['border']}; "
            f"border-radius: {sw(6)}px; "
            f"}}"
        )

        self.btn_restart.setStyleSheet(
            f"QPushButton {{ "
            f"background-color: {theme['panel']}; "
            f"color: {theme['text']}; "
            f"font-weight: bold; "
            f"font-size: {sf(14)}px; "
            f"border: {sw(2)}px solid {theme['border']}; "
            f"border-radius: {sw(6)}px; "
            f"}} "
            f"QPushButton:hover {{ "
            f"background-color: {theme['btn_hover']}; "
            f"}}"
        )

        self._refresh_team_ui()
        self._refresh_option_ui()
        self.update_counter(0)

    def _set_option(self, group_key, value):
        self.option_values[group_key] = value
        self._refresh_option_ui()

        if group_key == "strategy":
            self.ttc_strategy_changed.emit(str(value))

        self.option_changed.emit()

    def _refresh_option_ui(self):
        theme = THEMES[self.theme_name]

        for group_key, buttons in self.option_buttons.items():
            selected = self.option_values[group_key]

            for value, btn in buttons.items():
                if value == selected:
                    btn.setStyleSheet(
                        f"QPushButton {{ "
                        f"background-color: {theme['accent']}; "
                        f"color: white; "
                        f"font-weight: bold; "
                        f"font-size: {sf(12)}px; "
                        f"border: {sw(4)}px solid {theme['accent']}; "
                        f"border-radius: {sw(6)}px; "
                        f"padding: 0 {sw(4)}px; "
                        f"}}"
                    )
                else:
                    btn.setStyleSheet(
                        f"QPushButton {{ "
                        f"background-color: {theme['panel']}; "
                        f"color: {theme['text']}; "
                        f"font-weight: bold; "
                        f"font-size: {sf(12)}px; "
                        f"border: {sw(2)}px solid {theme['border']}; "
                        f"border-radius: {sw(6)}px; "
                        f"padding: 0 {sw(4)}px; "
                        f"}} "
                        f"QPushButton:hover {{ "
                        f"background-color: {theme['btn_hover']}; "
                        f"}}"
                    )

    def _refresh_team_ui(self):
        theme = THEMES[self.theme_name]

        active_style = (
            f"color: white; "
            f"font-weight: bold; "
            f"font-size: {sf(14)}px; "
            f"border: none; "
            f"border-radius: {sw(6)}px;"
        )

        inactive_style = (
            f"background-color: {theme['panel']}; "
            f"color: {theme['text']}; "
            f"font-weight: bold; "
            f"font-size: {sf(14)}px; "
            f"border: {sw(2)}px solid {theme['unselected']}; "
            f"border-radius: {sw(6)}px;"
        )

        self.btn_blue.setStyleSheet(
            f"background-color: {theme['blue_team']}; {active_style}"
            if self.team == 0
            else inactive_style
        )

        self.btn_red.setStyleSheet(
            f"background-color: {theme['red_team']}; {active_style}"
            if self.team == 1
            else inactive_style
        )

    def _select_obj(self, obj_type):
        self.selected_obj = obj_type

        if obj_type is not None:
            self.obj_selected.emit(obj_type)


    def force_select_kfs_r2(self):
        self._select_obj(2)

    def update_counter(self, obj_type, inc=True):
        if obj_type != 0:
            self.global_counts[obj_type] += 1 if inc else -1

    def set_gui_state(self, state_code):
        is_locked = state_code != 0
        self.is_ready_state = state_code == 1

        self.btn_blue.setDisabled(is_locked)
        self.btn_red.setDisabled(is_locked)

        for buttons in self.option_buttons.values():
            for btn in buttons.values():
                btn.setDisabled(is_locked)

        self.btn_restart.setDisabled(state_code == 2)

    def _set_team(self, team):
        self.team = team
        self._refresh_team_ui()
        self.team_changed.emit(team)
        self.option_changed.emit()

    def reset_ui(self):
        self.global_counts = {1: 0, 2: 0, 3: 0}

        self.option_values["spearhead"] = "2"
        self._set_option("strategy", STRATEGY_OPTIONS[0])
        self._set_option("kfs_retrieval", KFS_RETRIEVAL_OPTIONS[0])

        self._set_team(0)
        self._select_obj(None)
        self.update_counter(0)
