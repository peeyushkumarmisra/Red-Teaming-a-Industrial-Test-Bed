# =============================================================================
#  shutdown.py - quiet, clean shutdown of the nodes
# -----------------------------------------------------------------------------
#  Ctrl-C can interrupt rclpy while it converts a message, which surfaces as a
#  RuntimeError instead of KeyboardInterrupt (B22). Such an error is ignored only if
#  ROS is already shutting down; a real error while running is still raised.
# =============================================================================

import rclpy
from rclpy.executors import ExternalShutdownException


def spin_until_shutdown(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
