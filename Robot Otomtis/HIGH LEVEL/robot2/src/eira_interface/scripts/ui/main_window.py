from PySide6.QtWidgets import (
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QMessageBox,
    QSplitter,
    QGroupBox,
    QStackedWidget,
    QGridLayout,
)
from PySide6.QtCore import Qt, Signal
from eira_msgs.msg import KfsRetrievalPlan

from .control_panel import ControlPanel
from .grid_view import MatchGrid
from .ttt_view import TicTacToeGrid
from .constants import (
    WINDOW_WIDTH,
    WINDOW_HEIGHT,
    WINDOW_TITLE,
    APP_MARGIN,
    LEFT_PANEL_WIDTH,
    KFS_LIMITS,
    THEMES,
    RETRY_OPTIONS,
    DEFAULT_RETRY_STAGE,
    SPEARHEAD_OPTIONS,
    sw,
    sh,
    sf,
)


class ConfigApp(QMainWindow):
    kfs_plan_received = Signal(list, list, list, bool)

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.match_state = 0

        # Default: tidak ada retry dipilih.
        # Ini akan dikirim sebagai retry_stage="".
        self.selected_retry_target = DEFAULT_RETRY_STAGE

        self.is_ttt_custom_mode = False
        self.start_requested = False
        self.theme_name = "dark"
        self._last_kfs_plan = None

        self.setWindowTitle(WINDOW_TITLE)
        self.setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self._build_ui()
        self.kfs_plan_received.connect(self.grid.set_kfs_retrieval_plan)
        self.kfs_plan_sub = self.node.create_subscription(
            KfsRetrievalPlan,
            "/intelligent/kfs_retrieval_plan",
            self._on_kfs_retrieval_plan,
            10,
        )
        self.apply_theme(self.theme_name)

        self._log(
            "Sistem Siap. Lakukan konfigurasi taktik dan KFS.",
            "SYS",
        )

    def _build_ui(self):
        main_splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(main_splitter)
        main_splitter.setContentsMargins(
            APP_MARGIN,
            APP_MARGIN,
            APP_MARGIN,
            APP_MARGIN,
        )
        main_splitter.setHandleWidth(sw(15))
        main_splitter.setStyleSheet(
            "QSplitter::handle { background: transparent; }"
        )

        self.ctrl = ControlPanel()
        self.ctrl.setFixedWidth(LEFT_PANEL_WIDTH)

        self.ctrl.theme_toggled.connect(self.apply_theme)
        self.ctrl.fullscreen_toggled.connect(self._toggle_fullscreen)
        self.ctrl.team_changed.connect(self._on_team_change)
        self.ctrl.obj_selected.connect(
            lambda ot: self._log(f"KFS Selected: {ot}", "CFG")
        )
        self.ctrl.restart_clicked.connect(self._on_restart)
        self.ctrl.ttc_strategy_changed.connect(self._on_ttc_strategy_change)
        self.ctrl.option_changed.connect(self._publish_current_gui_state)

        right_splitter = QSplitter(Qt.Vertical)
        right_splitter.setHandleWidth(sh(15))
        right_splitter.setStyleSheet(
            "QSplitter::handle { background: transparent; }"
        )

        self.spearhead_box = QGroupBox("SPEARHEAD")
        spearhead_layout = QHBoxLayout(self.spearhead_box)
        spearhead_layout.setContentsMargins(sw(10), sh(18), sw(10), sh(10))
        spearhead_layout.setSpacing(sw(8))

        self.spearhead_buttons = {}

        for option in SPEARHEAD_OPTIONS:
            btn = QPushButton(option)
            btn.setFixedHeight(sh(88))
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda _, value=option: self._set_spearhead(value)
            )

            spearhead_layout.addWidget(btn)
            self.spearhead_buttons[option] = btn

        self.grid_box = QGroupBox("MEIHUA FOREST (MF)")
        grid_layout = QVBoxLayout(self.grid_box)
        grid_layout.setContentsMargins(sw(10), sh(20), sw(10), sh(10))

        self.grid_stack = QStackedWidget()

        self.grid = MatchGrid()
        self.grid.grid_clicked.connect(self._on_grid_click)
        self.grid.set_kfs_counts(self.ctrl.global_counts)

        self.ttt_grid = TicTacToeGrid()
        self.ttt_grid.cell_clicked.connect(self._on_ttt_click)

        self.grid_stack.addWidget(self.grid)
        self.grid_stack.addWidget(self.ttt_grid)

        grid_layout.addWidget(self.grid_stack)

        self.retry_box = QGroupBox("RETRY")
        retry_layout = QGridLayout(self.retry_box)
        retry_layout.setContentsMargins(sw(10), sh(18), sw(10), sh(10))
        retry_layout.setSpacing(sw(8))

        self.retry_buttons = {}

        for idx, retry in enumerate(RETRY_OPTIONS):
            retry_key = str(retry.get("key", "") or "").strip()
            retry_text = str(retry.get("label", retry_key) or retry_key).strip()

            # Tidak bikin tombol INITIAL START.
            # Normal start terjadi otomatis kalau tidak ada retry dipilih.
            if retry_key == "":
                continue

            # btn = QPushButton(f"RETRY {retry_text}")
            btn = QPushButton(retry_text)
            btn.setFixedHeight(sh(50))
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda _, key=retry_key: self._on_retry_click(key)
            )

            row = idx // 3
            col = idx % 3

            retry_layout.addWidget(btn, row, col)
            retry_layout.setColumnStretch(col, 1)

            self.retry_buttons[retry_key] = btn

        self.match_control_box = QGroupBox("MATCH CONTROL")
        match_control_layout = QHBoxLayout(self.match_control_box)
        match_control_layout.setContentsMargins(sw(10), sh(18), sw(10), sh(10))
        match_control_layout.setSpacing(sw(14))

        self.btn_ready = QPushButton("READY TO START")
        self.btn_ready.setFixedHeight(sh(68))
        self.btn_ready.setCursor(Qt.PointingHandCursor)
        self.btn_ready.clicked.connect(self._on_ready_button_click)

        self.btn_start = QPushButton("START")
        self.btn_start.setFixedHeight(sh(68))
        self.btn_start.setCursor(Qt.PointingHandCursor)
        self.btn_start.pressed.connect(self._on_start_pressed)
        self.btn_start.released.connect(self._on_start_released)

        match_control_layout.addWidget(self.btn_ready)
        match_control_layout.addWidget(self.btn_start)

        ctrl_layout = self.ctrl.layout()
        ctrl_layout.insertWidget(ctrl_layout.count() - 2, self.retry_box)
        ctrl_layout.addWidget(self.match_control_box)

        right_splitter.addWidget(self.spearhead_box)
        right_splitter.addWidget(self.grid_box)

        right_splitter.setSizes(
            [
                int(WINDOW_HEIGHT * 0.18),
                int(WINDOW_HEIGHT * 0.82),
            ]
        )

        main_splitter.addWidget(self.ctrl)
        main_splitter.addWidget(right_splitter)

        main_splitter.setSizes(
            [
                LEFT_PANEL_WIDTH,
                WINDOW_WIDTH - LEFT_PANEL_WIDTH - sw(15),
            ]
        )

    def apply_theme(self, theme_key):
        self.theme_name = theme_key
        t = THEMES[theme_key]

        self.setStyleSheet(
            f"QMainWindow {{ background-color: {t['bg']}; }}"
        )

        box_style = (
            f"QGroupBox {{ "
            f"font-size: {sf(14)}px; "
            f"font-weight: bold; "
            f"border: {sw(2)}px solid {t['border']}; "
            f"border-radius: {sw(6)}px; "
            f"margin-top: {sh(12)}px; "
            f"background: {t['panel']}; "
            f"}} "
            f"QGroupBox::title {{ "
            f"subcontrol-origin: margin; "
            f"subcontrol-position: top left; "
            f"left: {sw(10)}px; "
            f"top: 0px; "
            f"color: {t['box_title']}; "
            f"padding: 0 {sw(5)}px; "
            f"}}"
        )

        self.spearhead_box.setStyleSheet(box_style)
        self.grid_box.setStyleSheet(box_style)
        self.retry_box.setStyleSheet(box_style)
        self.match_control_box.setStyleSheet(box_style)

        self._refresh_spearhead_ui()
        self._refresh_retry_ui()
        self._update_match_control_ui()

        self.ctrl.apply_theme(theme_key)
        self.grid.set_theme(theme_key)
        self.ttt_grid.set_theme(theme_key)

    def _refresh_retry_ui(self):
        red_75 = "rgba(220, 38, 38, 0.20)"
        red_100 = "rgba(220, 38, 38, 1.0)"
        red_hover = "rgba(220, 38, 38, 0.90)"

        label_by_key = {
            str(item.get("key", "") or ""): str(
                item.get("label", item.get("key", ""))
            )
            for item in RETRY_OPTIONS
        }

        for retry_key, btn in self.retry_buttons.items():
            retry_label = label_by_key.get(retry_key, retry_key).upper()
            btn.setText(retry_label)

            # Bagian stylesheet tetap seperti sebelumnya.
            if retry_key == self.selected_retry_target:
                btn.setStyleSheet(
                    f"""
                    QPushButton {{
                        background-color: {red_100};
                        color: white;
                        font-weight: bold;
                        font-size: {sf(12)}px;
                        border: {sw(2)}px solid {red_100};
                        border-radius: {sw(6)}px;
                    }}
                    """
                )
            else:
                btn.setStyleSheet(
                    f"""
                    QPushButton {{
                        background-color: {red_75};
                        color: white;
                        font-weight: bold;
                        font-size: {sf(12)}px;
                        border: {sw(2)}px solid {red_75};
                        border-radius: {sw(6)}px;
                    }}
                    QPushButton:hover {{
                        background-color: {red_75};
                        border-color: {red_75};
                    }}
                    QPushButton:pressed {{
                        background-color: {red_100};
                        border-color: {red_100};
                    }}
                    """
                )

            # Tambahkan bagian ini.
            if self.match_state == 1:
                # Saat READY hanya EMERGENCY yang aktif.
                btn.setEnabled(retry_key == "emergency")
            elif self.match_state == 2:
                # Saat RUNNING semuanya mati.
                btn.setEnabled(False)
            else:
                # Saat INITIAL semuanya aktif.
                btn.setEnabled(True)

    def _set_spearhead(self, spearhead_target):
        if self.match_state != 0:
            return self._log(
                "Spearhead hanya bisa dipilih sebelum READY TO START.",
                "WARN",
            )

        self.ctrl.option_values["spearhead"] = str(spearhead_target)
        self._refresh_spearhead_ui()
        self._publish_current_gui_state()

        self._log(
            f"Spearhead Target: {spearhead_target}",
            "CFG",
        )

    def _refresh_spearhead_ui(self):
        theme = THEMES[self.theme_name]
        selected = str(self.ctrl.option_values["spearhead"])
        is_locked = self.match_state != 0

        for value, btn in self.spearhead_buttons.items():
            btn.setDisabled(is_locked)

            if value == selected:
                btn.setStyleSheet(
                    f"QPushButton {{ "
                    f"background-color: {theme['accent']}; "
                    f"color: white; "
                    f"font-weight: bold; "
                    f"font-size: {sf(14)}px; "
                    f"border: {sw(4)}px solid {theme['accent']}; "
                    f"border-radius: {sw(6)}px; "
                    f"}}"
                )
            else:
                btn.setStyleSheet(
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

    def _on_ready_button_click(self):
        if self.match_state == 1:
            self._on_cancel_ready()
        elif self.match_state == 0:
            self._on_ready()

    def _on_start_pressed(self):
        if self.match_state != 1:
            return self._log(
                "START hanya bisa dikirim setelah READY TO START.",
                "WARN",
            )

        self.start_requested = True
        self.node.update_gui_config(
            self._current_config_data(ready=True, start=True)
        )
        self._update_match_control_ui()

        self._log(
            "START pressed: true dikirim ke /gui_data.",
            "SYS",
        )

    def _on_start_released(self):
        if self.match_state != 1:
            return

        self.start_requested = False
        self.node.update_gui_config(
            self._current_config_data(ready=True, start=False)
        )
        self._update_match_control_ui()

        self._log(
            "START released: false dikirim ke /gui_data.",
            "SYS",
        )

    def _update_match_control_ui(self):
        theme = THEMES[self.theme_name]

        ready_active = self.match_state == 1
        self.btn_ready.setDisabled(self.match_state > 1)
        self.btn_ready.setText(
            "CANCEL READY"
            if ready_active
            else "READY TO START"
        )

        ready_color = (
            theme["state_cancel"]
            if ready_active
            else theme["state_run"]
        )

        self.btn_ready.setStyleSheet(
            f"QPushButton {{ "
            f"background-color: {ready_color}; "
            f"color: white; "
            f"font-weight: bold; "
            f"font-size: {sf(17)}px; "
            f"border-radius: {sw(6)}px; "
            f"}}"
        )

        start_enabled = self.match_state == 1
        self.btn_start.setDisabled(not start_enabled)

        start_color = (
            theme["accent"]
            if self.start_requested
            else theme["panel"]
        )

        start_text_color = (
            "white"
            if self.start_requested
            else theme["text"]
        )

        self.btn_start.setStyleSheet(
            f"QPushButton {{ "
            f"background-color: {start_color}; "
            f"color: {start_text_color}; "
            f"font-weight: bold; "
            f"font-size: {sf(17)}px; "
            f"border: {sw(2)}px solid {theme['accent']}; "
            f"border-radius: {sw(6)}px; "
            f"}} "
            f"QPushButton:hover {{ "
            f"background-color: {start_color}; "
            f"}}"
        )

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.setFixedSize(WINDOW_WIDTH, WINDOW_HEIGHT)
        else:
            self.setMinimumSize(0, 0)
            self.setMaximumSize(16777215, 16777215)
            self.showFullScreen()

    # def _on_retry_click(self, retry_target):
    #     if self.match_state != 0:
    #         return self._log(
    #             "Retry hanya bisa dipilih sebelum READY TO START.",
    #             "WARN",
    #         )

    #     if self.selected_retry_target == retry_target:
    #         self.selected_retry_target = DEFAULT_RETRY_STAGE
    #         self._log(
    #             "Retry selection cleared. Mode: INITIAL START.",
    #             "SYS",
    #         )
    #     else:
    #         self.selected_retry_target = retry_target
    #         self._log(
    #             f"Retry selected: {retry_target.upper()}",
    #             "SYS",
    #         )

    #     self._refresh_retry_ui()
    #     self._publish_current_gui_state()


    # def _on_retry_click(self, retry_target):
    #     if self.match_state != 0:
    #         return self._log(
    #             "Retry hanya bisa dipilih sebelum READY TO START.",
    #             "WARN",
    #         )

    #     if self.selected_retry_target == retry_target:
    #         self.selected_retry_target = ""
    #         self._log(
    #             "Retry selection cleared. Mode: INITIAL START.",
    #             "SYS",
    #         )
    #     else:
    #         self.selected_retry_target = retry_target
    #         self._log(
    #             f"Retry selected: {retry_target.upper()}",
    #             "SYS",
    #         )

    #     self._refresh_retry_ui()
    #     self._publish_current_gui_state()

    def _on_retry_click(self, retry_target):
        # Saat RUNNING, semua tombol retry dikunci.
        if self.match_state == 2:
            return self._log(
                "Retry tidak dapat diubah saat robot RUNNING.",
                "WARN",
            )

        # Saat READY, hanya EMERGENCY yang boleh ditekan.
        if self.match_state == 1:
            if retry_target != "emergency":
                return self._log(
                    "Saat READY, hanya EMERGENCY yang dapat dipilih.",
                    "WARN",
                )

            self.match_state = 0
            self.start_requested = False
            self.selected_retry_target = "emergency"

            self.ctrl.set_gui_state(0)
            self.grid.set_locked(False)

            self._refresh_retry_ui()
            self._refresh_spearhead_ui()
            self._update_match_control_ui()

            # Karena match_state sudah 0:
            # ready=False dan start=False.
            self._publish_current_gui_state()

            self._log(
                "EMERGENCY dipilih. READY dibatalkan.",
                "WARN",
            )
            return

        # Di state INITIAL, semua retry bisa dipilih atau dibatalkan.
        if self.selected_retry_target == retry_target:
            self.selected_retry_target = ""

            self._log(
                "Retry selection cleared. Mode: INITIAL START.",
                "SYS",
            )
        else:
            self.selected_retry_target = retry_target

            self._log(
                f"Retry selected: {retry_target.upper()}",
                "SYS",
            )

        self._refresh_retry_ui()
        self._publish_current_gui_state()

    def _on_grid_click(self, gid, sel_obj):
        if self.match_state != 0:
            return self._log(
                "Terkunci. Hanya dapat diedit di state INITIAL.",
                "WARN",
            )

        old_val = self.grid.grid_data[gid]

        if old_val == sel_obj:
            new_val = 0
        else:
            new_val = sel_obj

        if new_val > 0 and old_val != new_val:
            if self.ctrl.global_counts[new_val] >= KFS_LIMITS[new_val]:
                return self._log(
                    "Limit maksimal tercapai!",
                    "WARN",
                )

        if new_val == 3 and gid in [1, 2, 3]:
            return self._log(
                "KFS Fake dilarang di baris R1!",
                "WARN",
            )

        if new_val == 1 and gid in [5, 8]:
            return self._log(
                "KFS R1 dilarang di zona ini!",
                "WARN",
            )

        self.grid.grid_data[gid] = new_val
        self.grid.update()

        if old_val > 0:
            self.ctrl.update_counter(old_val, inc=False)

        if new_val > 0:
            self.ctrl.update_counter(new_val, inc=True)

        self.grid.set_kfs_counts(self.ctrl.global_counts)

        if new_val == 0:
            self._log(f"Grid {gid}: Cleared", "PLACE")
        else:
            self._log(f"Grid {gid}: KFS {new_val}", "PLACE")

        self._sync_kfs_plan_overlay()
        self._publish_current_gui_state()

    def _on_team_change(self, team_id):
        self.grid.set_team(team_id)

        self.ttt_grid.set_team(team_id)

        self._log(
            f"Team: {'BLUE' if team_id == 0 else 'RED'}",
            "CFG",
        )

    def _on_ready(self, config_data=None):
        self.start_requested = False
        self.match_state = 1
        self.ctrl.set_gui_state(1)
        self.grid.set_locked(True)
        self._refresh_spearhead_ui()
        self._update_match_control_ui()

        config_data = self._current_config_data(ready=True, start=False)

        if self.is_ttt_custom_mode:
            if self.selected_retry_target:
                self._log(
                    f"Ready! TTC CUSTOM + {self.selected_retry_target.upper()} dikirim.",
                    "SYS",
                )
            else:
                self._log(
                    f"Ready! TTC CUSTOM dikirim: {config_data['ttt_custom']}",
                    "SYS",
                )
        else:
            if self.selected_retry_target:
                self._log(
                    f"Ready! {self.selected_retry_target.upper()} dikirim ke topic.",
                    "SYS",
                )
            else:
                self._log(
                    "Ready! Config locked. Mode: INITIAL START. Menunggu tombol fisik START.",
                    "SYS",
                )

        self.node.send_config_to_strategy(config_data)

    def _on_cancel_ready(self):
        config_data = self._current_config_data(ready=False, start=False)
        self.node.send_config_to_strategy(config_data)

        self.match_state = 0
        self.ctrl.set_gui_state(0)
        self.grid.set_locked(False)
        self.start_requested = False
        self._refresh_spearhead_ui()
        self._update_match_control_ui()

        self._log(
            "Dibatalkan! Config unlocked. Silakan edit kembali.",
            "WARN",
        )

    def _on_restart(self):
        if self.match_state == 2:
            return self._log(
                "TIDAK BISA RESTART. Robot sedang RUNNING!",
                "WARN",
            )

        reply = QMessageBox.question(
            self,
            "Reset",
            "Clear all data and return to INITIAL?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self.match_state = 0
            self.start_requested = False

            self.ctrl.reset_ui()
            self.ctrl.set_gui_state(0)
            self.grid.set_locked(False)

            self.grid.grid_data = {
                i: 0
                for i in range(1, 13)
            }

            self.selected_retry_target = DEFAULT_RETRY_STAGE
            self.is_ttt_custom_mode = False

            self.ttt_grid.reset()
            self.grid_stack.setCurrentWidget(self.grid)
            self.grid_box.setTitle("MEIHUA FOREST (MF)")

            self._refresh_retry_ui()
            self._refresh_spearhead_ui()
            self._update_match_control_ui()
            self.grid.set_kfs_counts(self.ctrl.global_counts)
            self.grid.update()
            self._sync_kfs_plan_overlay()

            self._log(
                "Reset complete. State: INITIAL",
                "SYS",
            )

            self._publish_current_gui_state()

    def update_state_from_ros(self, new_state):
        self.match_state = new_state
        self.ctrl.set_gui_state(new_state)
        self.grid.set_locked(new_state != 0)
        self._refresh_spearhead_ui()
        self._update_match_control_ui()

        self._log(
            f"Hardware Sync: State {new_state}",
            "SYS",
        )

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self,
            "Exit",
            "Close EIRA Configuration?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

    def _log(self, msg, level="INFO"):
        logger = self.node.get_logger()

        if level == "WARN":
            logger.warn(msg)
        else:
            logger.info(f"[{level}] {msg}")

    def _on_kfs_retrieval_plan(self, msg):
        path_nodes = list(getattr(msg, "path_nodes", []))
        picked_kfs = list(getattr(msg, "picked_kfs", []))
        visual_path = [
            (
                int(point.zone),
                int(point.node),
                int(point.anchor_node),
            )
            for point in getattr(msg, "visual_path", [])
        ]
        plan = (
            tuple(path_nodes),
            tuple(picked_kfs),
            tuple(visual_path),
        )

        if plan == self._last_kfs_plan:
            return

        self._last_kfs_plan = plan
        self._sync_kfs_plan_overlay()

    def _has_kfs_in_forest(self):
        return any(value != 0 for value in self.grid.grid_data.values())

    def _sync_kfs_plan_overlay(self):
        if self._last_kfs_plan is None:
            return

        path_nodes, picked_kfs, visual_path = self._last_kfs_plan
        visible = self._has_kfs_in_forest()
        self.kfs_plan_received.emit(
            list(path_nodes) if visible else [],
            list(picked_kfs) if visible else [],
            list(visual_path) if visible else [],
            visible,
        )

    def _current_config_data(self, ready=None, start=None):
        config_data = {
            "spearhead": int(self.ctrl.option_values["spearhead"]),
            "strategy": str(self.ctrl.option_values["strategy"]),
            "kfs_retrieval": str(self.ctrl.option_values["kfs_retrieval"]),
            "team": int(self.ctrl.team),
            "retry_stage": self.selected_retry_target,
            "ready": self.match_state == 1 if ready is None else bool(ready),
            "start": self.start_requested if start is None else bool(start),
        }

        config_data["forest"] = [
            self.grid.grid_data[i]
            for i in range(1, 13)
        ]
        config_data["ttt_custom"] = self.ttt_grid.get_matrix_steps()

        return config_data

    def _publish_current_gui_state(self):
        if not hasattr(self, "ctrl") or not hasattr(self, "grid"):
            return

        self.node.update_gui_config(self._current_config_data())

    def _on_ttc_strategy_change(self, strategy_text):
        if self.match_state != 0:
            return

        strategy = strategy_text.strip().upper()
        self.is_ttt_custom_mode = "CUSTOM" in strategy

        if self.is_ttt_custom_mode:
            self.grid_box.setTitle("TIC TAC TOE CUSTOM")
            self.grid_stack.setCurrentWidget(self.ttt_grid)

            self.ctrl.force_select_kfs_r2()

            self._log(
                "TTC CUSTOM aktif. Auto selected: KFS R2.",
                "SYS",
            )
        else:
            self.grid_box.setTitle("MEIHUA FOREST (MF)")
            self.grid_stack.setCurrentWidget(self.grid)

            self._log(
                "TTC CUSTOM nonaktif. Kembali ke Meihua Forest.",
                "SYS",
            )

        self._publish_current_gui_state()

    def _on_ttt_click(self, id_ttt):
        if self.match_state != 0:
            return self._log(
                "Terkunci. Tic Tac Toe hanya dapat diedit di state INITIAL.",
                "WARN",
            )

        if not self.is_ttt_custom_mode:
            return

        # mode CUSTOM otomatis R2
        if self.ctrl.selected_obj != 2:
            self.ctrl.force_select_kfs_r2()

        # KFS R2 hanya boleh di 4,5,6,7,8,9
        # if id_ttt in [1, 2, 3]:
        #     return self._log(
        #         "KFS R2 tidak boleh ditempatkan di Tic Tac Toe 1, 2, 3.",
        #         "WARN",
        #     )

        # klik cell yang sudah dipilih = hapus
        if id_ttt in self.ttt_grid.ttt_data:
            removed_step = self.ttt_grid.ttt_data[id_ttt]
            del self.ttt_grid.ttt_data[id_ttt]

            # rapikan ulang step setelah delete
            ordered = sorted(
                self.ttt_grid.ttt_data.items(),
                key=lambda item: item[1],
            )

            self.ttt_grid.ttt_data = {
                cell_id: idx + 1
                for idx, (cell_id, old_step) in enumerate(ordered)
            }

            self.ttt_grid.update()
            self._publish_current_gui_state()

            return self._log(
                f"TTT {id_ttt}: removed dari step {removed_step}",
                "PLACE",
            )

        next_step = len(self.ttt_grid.ttt_data) + 1
        self.ttt_grid.ttt_data[id_ttt] = next_step
        self.ttt_grid.update()
        self._publish_current_gui_state()

        self._log(
            f"TTT step {next_step}: KFS R2 placed at {id_ttt}",
            "PLACE",
        )
