import os
import tempfile
import warnings

import ikpy.chain
import numpy as np

JOINTS = [f'joint_a{i}' for i in range(1, 8)]
DOWN = np.array([0.0, 0.0, -1.0])


class IkError(RuntimeError):
    pass


class Kinematics:

    def __init__(self, urdf, position_tolerance=1e-3, axis_tolerance=1e-2):
        fd, path = tempfile.mkstemp(suffix='.urdf')
        try:
            with os.fdopen(fd, 'w') as f:
                f.write(urdf)
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                chain = ikpy.chain.Chain.from_urdf_file(path, base_elements=['world'])
        finally:
            os.remove(path)
        names = [link.name for link in chain.links]
        missing = [j for j in JOINTS if j not in names]
        if missing:
            raise IkError(f'joints missing from the kinematic chain: {missing}')
        self._index = [names.index(j) for j in JOINTS]
        mask = [True if i in self._index else 0.0 for i in range(len(chain.links))]
        self._chain = ikpy.chain.Chain(chain.links, active_links_mask=mask, name='iiwa')
        self._size = len(chain.links)
        self._position_tolerance = position_tolerance
        self._axis_tolerance = axis_tolerance

    def _full(self, q):
        full = np.zeros(self._size)
        full[self._index] = q
        return full

    def forward(self, q):
        return self._chain.forward_kinematics(self._full(q))

    def inverse(self, position, seed):
        target = np.asarray(position, dtype=float)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            full = self._chain.inverse_kinematics(target_position=target,
                                                  target_orientation=DOWN,
                                                  orientation_mode='Z',
                                                  initial_position=self._full(seed))
        q = np.asarray(full)[self._index]
        frame = self.forward(q)
        position_error = float(np.linalg.norm(frame[:3, 3] - target))
        axis_error = float(np.linalg.norm(frame[:3, 2] - DOWN))
        if position_error > self._position_tolerance or axis_error > self._axis_tolerance:
            raise IkError(f'IK for {target.tolist()} missed by {position_error:.4f} m, '
                          f'tool axis error {axis_error:.4f}')
        return q

    def chain(self, targets, seed):
        solutions = {}
        q = np.asarray(seed, dtype=float)
        for name, position in targets:
            q = self.inverse(position, q)
            solutions[name] = q
        return solutions


def station_targets(prefix, xy, hover_height, grab_height, drop_height):
    x, y = xy
    return [(f'{prefix}_hover', (x, y, hover_height)),
            (f'{prefix}_grab', (x, y, grab_height)),
            (f'{prefix}_drop', (x, y, drop_height))]