import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml

from ids_bringup.monitor import EFFORT_LIMITS, RunMonitor
from ids_interfaces.msg import IdsStatus, JointMeasurement, JointReference


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class RunSupervisor(Node):

    def __init__(self):
        super().__init__('run_supervisor')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('summary_file', '')
        self.declare_parameter('start_timeout', 300.0)
        self.declare_parameter('progress_period', 30.0)
        self.declare_parameter('namespaces', ['ids', 'ids_thesis'])
        self.declare_parameter('effort_limits', list(EFFORT_LIMITS))
        self.declare_parameter('saturation_warning', 0.01)
        namespaces = list(self.get_parameter('namespaces').value)
        self.monitor = RunMonitor(self.get_parameter('duration').value, namespaces,
                                  self.get_parameter('effort_limits').value,
                                  self.get_parameter('saturation_warning').value)
        self.status = None
        self._summary_file = self.get_parameter('summary_file').value
        self._start_timeout = self.get_parameter('start_timeout').value
        self._period = self.get_parameter('progress_period').value
        self._next_progress = self._period
        self._started = time.monotonic()
        self.create_subscription(JointReference, '/plc/reference', self._on_reference, 100)
        self.create_subscription(JointMeasurement, '/robot/measurement',
                                 lambda msg: self.monitor.on_measurement(msg.effort_applied), 100)
        for ns in namespaces:
            self.create_subscription(IdsStatus, f'/{ns}/status',
                                     lambda msg, ns=ns: self._on_status(ns, msg), 100)
        self.get_logger().info(f'run lasts {self.monitor.duration:.0f} s after the first '
                               'reference')

    def _on_reference(self, msg):
        if self.status is not None:
            return
        done = self.monitor.on_reference(stamp_of(msg))
        if self.monitor.elapsed >= self._next_progress:
            self._next_progress += self._period
            self.get_logger().info(self.monitor.progress())
        if done:
            self.finish('complete')

    def _on_status(self, namespace, msg):
        self.monitor.on_status(namespace, stamp_of(msg), msg.mode, msg.alarm)

    def check_timeout(self):
        if (self.status is None and not self.monitor.started
                and time.monotonic() - self._started > self._start_timeout):
            self.finish('no_reference')

    def finish(self, status):
        self.status = status
        summary = self.monitor.summary(status)
        if self._summary_file:
            with open(self._summary_file, 'w') as f:
                yaml.safe_dump(summary, f, sort_keys=False)
        for warning in summary['warnings']:
            self.get_logger().warn(warning)
        self.get_logger().info(f'run {status}: {self.monitor.progress()}')


def main(args=None):
    rclpy.init(args=args)
    node = RunSupervisor()
    try:
        while rclpy.ok() and node.status is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_timeout()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if node.status is None:
        node.finish('interrupted')
    code = 0 if node.status == 'complete' else 1
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code