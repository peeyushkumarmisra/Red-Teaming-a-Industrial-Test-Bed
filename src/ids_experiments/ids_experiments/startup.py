class StartupGate:

    def __init__(self, settle_time, detach_period=0.1, wait_for_gripper=True):
        self._settle_time = settle_time
        self._detach_period = detach_period
        self._detached = not wait_for_gripper
        self._next_detach = None
        self._ready_at = None

    @property
    def detached(self):
        return self._detached

    def step(self, t, inputs_ready, gripper_state=None):
        if not inputs_ready or t <= 0.0:
            return False, False
        publish = False
        if not self._detached:
            if gripper_state == 'detached':
                self._detached = True
            elif self._next_detach is None or t >= self._next_detach - 1e-9:
                self._next_detach = t + self._detach_period
                publish = True
        if self._detached and self._ready_at is None:
            self._ready_at = t + self._settle_time
        return publish, self._ready_at is not None and t >= self._ready_at - 1e-9