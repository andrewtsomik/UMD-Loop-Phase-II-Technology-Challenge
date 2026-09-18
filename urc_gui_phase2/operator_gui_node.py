"""Entry point: the operator GUI, with rclpy driven from a Qt QTimer.

Everything runs on the Qt main thread. A QTimer periodically calls the
executor's spin_once(timeout_sec=0), which runs any ROS callbacks that are
ready and returns immediately. Because callbacks execute on the same thread
as the widgets, they can update the GUI directly -- no worker thread, no
cross-thread signals, no locks.

Trade-off (defend this honestly): a callback that blocks freezes the GUI, and
ROS latency is bounded by the timer period. Callbacks must therefore stay
short. The timer also keeps ticking while a modal dialog is open, because
QDialog.exec_() runs a nested Qt event loop.
"""

import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

SPIN_PERIOD_MS = 10
# spin_once() runs at most one ready callback per call, so drain a bounded
# number per tick. The bound stops a message flood from starving Qt painting.
MAX_SPINS_PER_TICK = 10


class OperatorGuiNode(Node):
    """ROS side of the GUI. Holds no widgets.

    TODO: subscriptions/publishers once Phase 2 topics are decided
    (e.g. sensor_msgs/NavSatFix for rover GNSS, nav_msgs/Odometry for pose).
    """

    def __init__(self):
        super().__init__('operator_gui_node')


class OperatorWindow(QMainWindow):
    def __init__(self, node: OperatorGuiNode):
        super().__init__()
        self._node = node
        self.setWindowTitle('URC Phase 2 - Navigation GUI (skeleton)')
        self.resize(1200, 800)
        # TODO: map widget (pyqtlet2, offline tiles), mission editor panel.
        self.setCentralWidget(QLabel('Phase 2 skeleton - nothing wired yet'))


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)

    node = OperatorGuiNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros():
        for _ in range(MAX_SPINS_PER_TICK):
            executor.spin_once(timeout_sec=0)

    timer = QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(SPIN_PERIOD_MS)

    window = OperatorWindow(node)
    window.show()

    exit_code = app.exec_()

    # No spin thread to join: stop the timer, then tear ROS down in order.
    timer.stop()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
