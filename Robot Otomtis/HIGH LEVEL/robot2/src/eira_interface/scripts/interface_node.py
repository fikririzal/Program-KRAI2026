import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import sys
import threading
import signal
from copy import deepcopy
from PySide6.QtWidgets import QApplication
from eira_interface.ui.main_window import ConfigApp
from eira_msgs.msg import GuiData


PUBLISH_PERIOD_SEC = 0.1

DEFAULT_CONFIG = {
    "team": 0,
    "spearhead": 2,
    "kfs_retrieval": "Automatic",
    "forest": [0 for _ in range(12)],
    # "ttt_custom": [],
    "ttt_custom": [
        [1, 5],
        [2, 8],
    ],
    "retry_stage": "retry_spearhead",
    "ready": False,
    "start": False,
}

KFS_RETRIEVAL_TO_CODE = {
    "2": 0,
    "3": 1,
    "4": 2,
}

RETRY_TO_INDEX = {
    "retry_spearhead": 0,
    "retry_forest": 1,
    "retry_arena": 2,
    "retry_1": 3,
    "retry_2": 4,
    "emergency": 5,
}


class InterfaceNode(Node):
    def __init__(self):
        super().__init__('eira_interface')
        self.gui = None
        self.gui_data_pub = self.create_publisher(GuiData, '/gui_data', 10)
        self._config_lock = threading.Lock()
        self._current_config = deepcopy(DEFAULT_CONFIG)
        self._publish_timer = self.create_timer(
            PUBLISH_PERIOD_SEC,
            self.publish_current_gui_data,
        )

        self.get_logger().info(
            "GUI publishing active: /gui_data [eira_msgs/msg/GuiData] @ 10 Hz"
        )
        self.publish_current_gui_data()

    def send_config_to_strategy(self, config_dict):
        self.update_gui_config(config_dict)

    def update_gui_config(self, config_dict, publish_now=True):
        with self._config_lock:
            self._current_config.update(deepcopy(config_dict))
            current_config = deepcopy(self._current_config)

        if publish_now:
            self.publish_gui_config(current_config)

    def publish_current_gui_data(self):
        with self._config_lock:
            current_config = deepcopy(self._current_config)

        self.publish_gui_config(current_config)

    def publish_gui_config(self, config_dict):
        msg = GuiData()
        msg.stamp = self.get_clock().now().to_msg()
        msg.team = int(config_dict.get('team', 0))
        msg.spearhead = int(config_dict.get('spearhead', 0))
        msg.forest = self._normalize_forest(config_dict.get('forest', []))
        msg.kfsret = self._kfs_retrieval_code(
            config_dict.get('kfs_retrieval', "Automatic")
        )
        msg.tictactoe = self._normalize_tictactoe(
            config_dict.get('ttt_custom', [])
        )
        msg.retry = self._retry_array(config_dict.get('retry_stage', ""))
        msg.ready = bool(config_dict.get('ready', False))
        msg.start = bool(config_dict.get('start', False))

        self.gui_data_pub.publish(msg)

    def _normalize_forest(self, forest):
        values = [int(value) for value in list(forest)]

        if len(values) < 12:
            values.extend([0] * (12 - len(values)))

        return values[:12]

    def _kfs_retrieval_code(self, value):
        key = str(value or "0").strip()
        return KFS_RETRIEVAL_TO_CODE.get(key, 0)

    def _normalize_tictactoe(self, ttt_custom):
        values = []

        for item in list(ttt_custom):
            if isinstance(item, (list, tuple)):
                if len(item) >= 2:
                    values.append(int(item[1]))
            else:
                values.append(int(item))

        return values

    def _retry_array(self, retry_stage):
        retry_stage = str(retry_stage or "").strip()

        retry = [0, 0, 0, 0, 0, 0]
        idx = RETRY_TO_INDEX.get(retry_stage)

        if idx is not None:
            retry[idx] = 1

        return retry

    def state_callback(self, msg):
        if self.gui:
            self.gui.update_state_from_ros(msg.match_state)


def spin_ros_node(node):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    
    ros_node = InterfaceNode()
    gui = ConfigApp(ros_node)
    ros_node.gui = gui
    
    gui.show()
    
    spin_thread = threading.Thread(target=spin_ros_node, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    exit_code = app.exec()
    
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
