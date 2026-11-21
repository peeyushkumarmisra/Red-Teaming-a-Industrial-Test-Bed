# =============================================================================
#  metrics.py - detection, latency, attribution and false alarms (C18)
# -----------------------------------------------------------------------------
#  Definitions (docs/v1.4.md):
#  - An attack is detected if an alarm onset falls in [start, end + grace]; latency =
#    onset - start (0 if an alarm was already active at the start).
#  - Attributed: the primary joint at that onset is an attacked joint.
#  - Before attack end: the onset came while the attack was still active, not from the
#    transient when it stopped.
#  - False alarm: an onset after the detector warm-up and outside every
#    [start, end + tail]; the rate is per hour of that observed time.
# =============================================================================

from dataclasses import dataclass, field
import math

import numpy as np

from ids_evaluation import layout


@dataclass(frozen=True)
class Event:
    event_id: int
    kind: str
    joints: tuple
    magnitude: float
    start: float
    end: float


@dataclass(frozen=True)
class Episode:
    start: float
    end: float
    primary: int


@dataclass
class EventResult:
    event: Event
    detected: bool
    latency: float
    attributed: bool
    already_alarmed: bool
    primary: int
    before_end: bool = False


@dataclass
class InstanceResult:
    instance: str
    events: list = field(default_factory=list)
    false_alarms: list = field(default_factory=list)
    observed_hours: float = 0.0
    armed_windows: int = 0
    alarm_windows: int = 0
    detection_start: float = math.nan

    @property
    def false_alarms_per_hour(self):
        if self.observed_hours <= 0.0:
            return math.nan
        return len(self.false_alarms) / self.observed_hours

    @property
    def alarm_fraction(self):
        return self.alarm_windows / self.armed_windows if self.armed_windows else math.nan


# Attack events from the truth changes; an event still active at the end of the run stays open.
def events_from_truth(attack, start=0.0):
    rows = np.asarray(attack, dtype=float).reshape(-1, layout.ATTACK_COLUMNS)
    events = []
    current = None
    for t, active, event_id, bits, magnitude, kind in rows:
        if current is not None and (active < 0.5 or int(event_id) != current[0]):
            events.append(Event(current[0], current[1], current[2], current[3], current[4],
                                t - start))
            current = None
        if active > 0.5 and current is None:
            current = (int(event_id), layout.kind_name(kind), layout.joints_of(bits),
                       float(magnitude), t - start)
    if current is not None:
        events.append(Event(current[0], current[1], current[2], current[3], current[4],
                            math.inf))
    return events


# Alarm episodes: onset, end and primary joint.
def episodes(t, alarm, primary):
    t = np.asarray(t, dtype=float)
    a = np.asarray(alarm) > 0.5
    if not a.any():
        return []
    edges = np.diff(np.r_[0, a.astype(int), 0])
    starts = np.nonzero(edges == 1)[0]
    ends = np.nonzero(edges == -1)[0] - 1
    step = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    return [Episode(float(t[s]), float(t[e]) + step, int(primary[s]))
            for s, e in zip(starts, ends)]


def _in_windows(x, windows):
    return any(a <= x <= b for a, b in windows)


def _overlap(a, b, windows):
    total = 0.0
    for lo, hi in windows:
        total += max(0.0, min(b, hi) - max(a, lo))
    return total


# Score one detector on one run. Only armed windows count; detection starts warmup seconds
# after arming, as in the live IDS.
def evaluate_instance(name, t, status, events, warmup=3.99, grace=1.0, tail=30.0, horizon=None):
    t = np.asarray(t, dtype=float)
    status = layout.pad_status(status)
    armed = status[:, layout.MODE] == layout.ARMED
    result = InstanceResult(name)
    if not armed.any():
        return result
    ta = t[armed]
    sa = status[armed]
    result.armed_windows = int(armed.sum())
    result.alarm_windows = int((sa[:, layout.ALARM] > 0.5).sum())
    result.detection_start = float(ta[0] + warmup)
    eps = episodes(ta, sa[:, layout.ALARM], sa[:, layout.PRIMARY])
    end = float(ta[-1]) if horizon is None else float(horizon)
    excluded = []
    for ev in events:
        ev_end = min(ev.end, end)
        excluded.append((ev.start, ev_end + tail))
        ongoing = [e for e in eps if e.start < ev.start <= e.end]
        if ongoing:
            p = ongoing[0].primary
            result.events.append(EventResult(ev, True, 0.0, p in ev.joints, True, p, True))
            continue
        hit = [e for e in eps if ev.start <= e.start <= ev_end + grace]
        if hit:
            p = hit[0].primary
            result.events.append(EventResult(ev, True, hit[0].start - ev.start, p in ev.joints,
                                             False, p, bool(hit[0].start < ev_end)))
        else:
            result.events.append(EventResult(ev, False, math.nan, False, False, -1))
    result.false_alarms = [e for e in eps if e.start >= result.detection_start
                           and not _in_windows(e.start, excluded)]
    observed = max(0.0, end - result.detection_start)
    observed -= _overlap(result.detection_start, end, excluded)
    result.observed_hours = observed / 3600.0
    return result


# Pool events by detector, kind, joints and magnitude.
def per_joint_table(results):
    groups = {}
    for run, r in results:
        for e in r.events:
            key = (r.instance, e.event.kind, layout.joint_label(e.event.joints),
                   e.event.magnitude)
            groups.setdefault(key, []).append(e)
    rows = []
    for (instance, kind, joints, magnitude), items in sorted(groups.items()):
        lat = [e.latency for e in items if e.detected]
        rows.append({
            'instance': instance, 'kind': kind, 'joints': joints, 'magnitude': magnitude,
            'events': len(items),
            'detected': sum(e.detected for e in items),
            'before_end': sum(e.before_end for e in items),
            'attributed': sum(e.attributed for e in items),
            'median_latency': float(np.median(lat)) if lat else math.nan,
            'max_latency': float(np.max(lat)) if lat else math.nan,
        })
    return rows


# Pool false alarms and observed hours by detector.
def false_alarm_table(results):
    pooled = {}
    for run, r in results:
        p = pooled.setdefault(r.instance, [0, 0.0, 0, 0])
        p[0] += len(r.false_alarms)
        p[1] += r.observed_hours
        p[2] += r.alarm_windows
        p[3] += r.armed_windows
    return [{'instance': k, 'false_alarms': v[0], 'hours': v[1],
             'per_hour': v[0] / v[1] if v[1] > 0 else math.nan,
             'alarm_fraction': v[2] / v[3] if v[3] else math.nan}
            for k, v in sorted(pooled.items())]
