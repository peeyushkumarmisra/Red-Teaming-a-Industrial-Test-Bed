import numpy as np

MARK = 'rogue'
HOME = (0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0)
JOINT_NAMES = tuple(f'joint_a{i}' for i in range(1, 8))


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def offset_position(position, attack):
    q = np.array(position, dtype=float)
    for j, m in zip(attack.joints, attack.joint_magnitudes()):
        q[j] += m
    return q


def blind_position(home, attacks):
    return offset_position(home, attacks[0]) if attacks else np.array(home, dtype=float)


def spoofed_state(schedule, t, position):
    attack = schedule.current(t)
    return None if attack is None else offset_position(position, attack)