import signal
import threading

import pytest

from ids_experiments import shutdown


class FakeContext:
    def __init__(self):
        self.alive = True

    def ok(self):
        return self.alive


class FakeNode:
    def __init__(self, context=None):
        self.context = context or FakeContext()
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def executor_factory(behaviour, log):
    class FakeExecutor:
        def __init__(self, context):
            self.context = context
            self.node = None
            self.spins = 0

        def add_node(self, node):
            self.node = node

        def remove_node(self, node):
            log.append(('remove', node))

        def shutdown(self, timeout_sec=None):
            log.append(('shutdown', self.node))

        def spin_once(self, timeout_sec=None):
            self.spins += 1
            behaviour(self)

    return FakeExecutor


@pytest.fixture
def ros(monkeypatch):
    state = {'shutdowns': [], 'inits': []}
    monkeypatch.setattr(shutdown.rclpy, 'try_shutdown',
                        lambda context=None: state['shutdowns'].append(context))
    monkeypatch.setattr(shutdown.rclpy, 'init', lambda **kw: state['inits'].append(kw))
    return state


def no_signals(signum, handler):
    pass


def test_init_turns_rclpy_signal_handling_off(ros):
    shutdown.init(args=['x'], domain_id=20)
    kw = ros['inits'][0]
    assert kw['signal_handler_options'] == shutdown.SignalHandlerOptions.NO
    assert kw['domain_id'] == 20 and kw['args'] == ['x']
    shutdown.init()
    assert 'domain_id' not in ros['inits'][1]


def test_stop_flag_installs_handlers_for_sigint_and_sigterm():
    installed = {}
    stop = shutdown.StopFlag(install=lambda s, h: installed.update({s: h}))
    assert set(installed) == {signal.SIGINT, signal.SIGTERM}
    assert not stop.is_set()
    installed[signal.SIGINT](signal.SIGINT, None)
    assert stop.is_set()


def test_a_stop_request_ends_the_loop_without_an_exception(ros):
    log = []
    stop = shutdown.StopFlag(install=no_signals)

    def behaviour(executor):
        if executor.spins == 3:
            stop.set()

    node = FakeNode()
    shutdown.spin_until_shutdown(node, stop=stop, executor_type=executor_factory(behaviour, log))
    assert node.destroyed and ros['shutdowns'] == [node.context]
    assert ('remove', node) in log and ('shutdown', node) in log


def test_an_error_after_shutdown_ends_quietly(ros):
    stop = shutdown.StopFlag(install=no_signals)
    node = FakeNode()

    def behaviour(executor):
        node.context.alive = False
        raise RuntimeError('Unable to convert call argument')

    shutdown.spin_until_shutdown(node, stop=stop, executor_type=executor_factory(behaviour, []))
    assert node.destroyed


def test_an_error_while_running_is_raised_after_cleanup(ros):
    stop = shutdown.StopFlag(install=no_signals)

    def behaviour(executor):
        raise RuntimeError('real bug')

    node = FakeNode()
    with pytest.raises(RuntimeError, match='real bug'):
        shutdown.spin_until_shutdown(node, stop=stop,
                                     executor_type=executor_factory(behaviour, []))
    assert node.destroyed and ros['shutdowns'] == [node.context]


def test_external_shutdown_ends_quietly(ros):
    stop = shutdown.StopFlag(install=no_signals)

    def behaviour(executor):
        raise shutdown.ExternalShutdownException()

    node = FakeNode()
    shutdown.spin_until_shutdown(node, stop=stop, executor_type=executor_factory(behaviour, []))
    assert node.destroyed


def test_two_contexts_spin_in_parallel_and_both_shut_down(ros):
    stop = shutdown.StopFlag(install=no_signals)
    seen = set()
    lock = threading.Lock()

    def behaviour(executor):
        with lock:
            seen.add(id(executor.node))
            if len(seen) == 2 and executor.spins > 2:
                stop.set()

    cell, monitor = FakeNode(), FakeNode()
    shutdown.spin_until_shutdown(cell, monitor, stop=stop, period=0.001,
                                 executor_type=executor_factory(behaviour, []))
    assert cell.destroyed and monitor.destroyed
    assert ros['shutdowns'] == [cell.context, monitor.context]