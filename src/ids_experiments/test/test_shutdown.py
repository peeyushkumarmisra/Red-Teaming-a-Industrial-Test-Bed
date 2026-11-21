import pytest

from ids_experiments import shutdown


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


@pytest.fixture
def ros(monkeypatch):
    state = {'ok': True, 'shutdowns': 0, 'raise': None}

    def spin(node):
        if state['raise'] is not None:
            raise state['raise']

    def do_shutdown():
        state['shutdowns'] += 1
        state['ok'] = False

    monkeypatch.setattr(shutdown.rclpy, 'spin', spin)
    monkeypatch.setattr(shutdown.rclpy, 'ok', lambda: state['ok'])
    monkeypatch.setattr(shutdown.rclpy, 'shutdown', do_shutdown)
    return state


def test_an_error_while_the_context_is_already_shut_down_ends_quietly(ros):
    ros['raise'] = RuntimeError('Unable to convert call argument')
    ros['ok'] = False
    node = FakeNode()
    shutdown.spin_until_shutdown(node)
    assert node.destroyed and ros['shutdowns'] == 0


def test_an_error_while_running_is_not_hidden(ros):
    ros['raise'] = RuntimeError('real bug')
    node = FakeNode()
    with pytest.raises(RuntimeError, match='real bug'):
        shutdown.spin_until_shutdown(node)
    assert node.destroyed and ros['shutdowns'] == 1


@pytest.mark.parametrize('stop', [KeyboardInterrupt(), shutdown.ExternalShutdownException()])
def test_interrupts_destroy_the_node_and_shut_down_once(ros, stop):
    ros['raise'] = stop
    node = FakeNode()
    shutdown.spin_until_shutdown(node)
    assert node.destroyed and ros['shutdowns'] == 1