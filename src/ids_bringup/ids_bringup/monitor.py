# ==================================================================================================
# monitor.py - this monitors runs
# --------------------------------------------------------------------------------------------------
# ROS-free bookkeeping for the run supervisor: run time from the first reference, status counts per
# IDS instance, first alarm, and torque saturation. A run with the arm at its torque limits in more
# than 1 % of windows is flagged as not nominal and must be excluded from evaluation (B21).
# ==================================================================================================

# Torque limits of the iiwa 14 (N*m), same as the controller's.
EFFORT_LIMITS = (320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0)
MODES = {0: 'waiting', 1: 'calibrating', 2: 'armed', 3: 'fault'}


class RunMonitor:

    def __init__(self, duration, namespaces=('ids', 'ids_thesis'), effort_limits = EFFORT_LIMITS, 
                 saturation_warning=0.01):
        self.duration = float(duration)
        self.first_reference = None
        self.last_reference = None
        self.reference = 0
        self.effort_limits = [float(x) for x in effort_limits]
        self.saturation_warning = float(saturation_warning)
        self.measurements = 0
        self.saturated = 0
        self.ids = {ns: {'status_messages': 0, 'alarm_samples': 0, 'first_alarm': None,
                         'modes': {}} for ns in namespaces}

    @property
    def started(self):
        return self.first_reference is not None

    @property
    def elapsed(self):
        return self.last_reference - self.first_reference if self.started else 0.0

    @property
    def done(self):
        return self.started and self.elapsed >= self.duration - 1e-9

    def on_reference(self, stamp):
        if self.first_reference is None:
            self.first_reference = stamp
        self.last_reference = max(stamp. self.last_reference or stamp)
        self.reference += 1
        return self.done

    # Count window in which any joint torque sits at its limit.
    def on_measurement(self, efforts):
        if not self.started:
            return
        self.measurements += 1
        if any(abs(e) >= limit * (1.0 - 1e-9) for e, limit in zip(efforts, self.effort_limits)):
            self.saturated += 1

    @property
    def torque_saturation(self):
        return self.saturated / self.measurements if self.measurements else None

    # Per IDS instance: messages, modes, alarm samples, first alarm time
    def on_status(self, namespace, stamp, mode, alarm):
        s = self.ids[namespace]
        s['status_messages'] += 1
        name = MODES.get(mode, str(mode))
        s['modes'][name] = s['modes'].get(name,0) + 1
        if alarm:
            s['alarm_samples'] += 1
            if s['first_alarm'] is None and self.started:
                s['first_alarm'] = round(stamp - self.first_reference, 3)

    def progress(self):
        parts = [f't={self.elapsed:.0f}/{self.duration:.0f} s', f'references {self.references}']
        for ns, s in self.ids.items():
            parts.append(f'{ns}: {s["status_messages"]} status, {s["alarm_samples"]} alarm')
        return ', '.join(parts)

    # Plain-language hints that point at the missing piece.
    def warnings(self):
        out = []
        if not self.started:
            out.append('no /plc/reference received: check the planner and the gripper start-up')
        elif self.measurements == 0:
            out.append('no /robot/measurement received: is the tracking controller active?')
        elif self.torque_saturation > self.saturation_warning:
            out.append(f'torque at an effort limit in {100.0 * self.torque_saturation:.1f}% of '
                       'measurement windows: the arm is not tracking (blocked, at a joint limit '
                       'or in contact), so this run is not nominal')
        for ns, s in self.ids.items():
            if s['status_messages'] == 0:
                out.append(f'no /{ns}/status received: is the IDS running on the monitor network?')
            elif s['modes'].get('fault'):
                out.append(f'/{ns} reported FAULT: missing calibration or fingerprint mismatch')
        return out

    # Contents of summary.yaml.
    def summary(self, status):
        return {
            'status': status,
            'duration': self.duration,
            'elapsed': round(self.elapsed, 3),
            'first_reference': self.first_reference,
            'references': self.references,
            'measurements': self.measurements,
            'torque_saturation': (None if self.torque_saturation is None
                                  else round(self.torque_saturation, 4)),
            'ids': self.ids,
            'warnings': self.warnings(),
        }