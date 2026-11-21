import os
import time

import yaml


def security_context(env=os.environ):
    enabled = env.get('ROS_SECURITY_ENABLE', '').lower() == 'true'
    return {'security_enabled': enabled,
            'strategy': env.get('ROS_SECURITY_STRATEGY', '') if enabled else '',
            'enclave': env.get('ROS_SECURITY_ENCLAVE_OVERRIDE', '') if enabled else '',
            'domain': int(env.get('ROS_DOMAIN_ID', '0') or 0)}


class NodeReport:

    def __init__(self, path, role, info=None, period=1.0, clock=time.monotonic, env=os.environ):
        self.path = path
        self.role = role
        self.info = dict(info or {})
        self.counts = {}
        self.matched = {}
        self._period = period
        self._clock = clock
        self._started = clock()
        self._next = self._started
        self._security = security_context(env)

    def count(self, key, n=1):
        self.counts[key] = self.counts.get(key, 0) + n

    def observe(self, key, value):
        value = int(value)
        seen = self.matched.setdefault(key, {'max': value, 'last': value})
        seen['max'] = max(seen['max'], value)
        seen['last'] = value

    def snapshot(self, final=False):
        return {'role': self.role, 'final': final, **self._security,
                'elapsed_s': round(self._clock() - self._started, 3), **self.info,
                'counts': dict(sorted(self.counts.items())),
                'matched': {k: dict(v) for k, v in sorted(self.matched.items())}}

    def maybe_write(self):
        if self._clock() >= self._next:
            self._next = self._clock() + self._period
            self.write()

    def write(self, final=False):
        if not self.path:
            return None
        data = self.snapshot(final)
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = f'{self.path}.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(tmp, self.path)
        return data