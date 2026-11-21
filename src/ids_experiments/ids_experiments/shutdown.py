import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM)


def init(args=None, context=None, domain_id=None):
    kwargs = {'args': args, 'context': context,
              'signal_handler_options': SignalHandlerOptions.NO}
    if domain_id is not None:
        kwargs['domain_id'] = int(domain_id)
    rclpy.init(**kwargs)


class StopFlag:

    def __init__(self, signals=STOP_SIGNALS, install=signal.signal):
        self._event = threading.Event()
        for s in signals:
            install(s, self._handle)

    def _handle(self, signum, frame):
        self._event.set()

    def set(self):
        self._event.set()

    def is_set(self):
        return self._event.is_set()


def _spin(executor, node, stop, period, errors):
    try:
        while not stop.is_set() and node.context.ok():
            executor.spin_once(timeout_sec=period)
    except ExternalShutdownException:
        pass
    except RuntimeError as e:
        if node.context.ok() and not stop.is_set():
            errors.append(e)
    finally:
        stop.set()


def spin_until_shutdown(*nodes, stop=None, period=0.1, executor_type=SingleThreadedExecutor):
    stop = stop or StopFlag()
    executors = []
    for node in nodes:
        executor = executor_type(context=node.context)
        executor.add_node(node)
        executors.append(executor)
    errors = []
    threads = [threading.Thread(target=_spin, args=(e, n, stop, period, errors), daemon=True)
               for e, n in zip(executors[1:], nodes[1:])]
    try:
        for t in threads:
            t.start()
        _spin(executors[0], nodes[0], stop, period, errors)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        for executor, node in zip(executors, nodes):
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
        for node in nodes:
            rclpy.try_shutdown(context=node.context)
    if errors:
        raise errors[0]