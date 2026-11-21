# =============================================================================
#  scenario.py - the scenario file format
# -----------------------------------------------------------------------------
#  A scenario says what one run does: name, seed, friction profile, duration, planner
#  settings and the attacks. Invalid scenarios are rejected with a clear message
#  before any simulation starts (bad joint, overlapping attacks, attack after the end).
# =============================================================================

from dataclasses import dataclass, field
import math

import yaml

from ids_experiments.attacks import Attack, KINDS

# Must match the profiles in iiwa_sim/config/friction.yaml.
FRICTION_PROFILES = ('none', 'nominal', 'wear', 'thesis')
PLANNER_DEFAULTS = {'cycles': 0, 'payload_mass': 5.0}


@dataclass(frozen=True)
# A validated scenario.
class Scenario:
    name: str
    seed: int
    friction_profile: str
    duration: float
    planner: dict = field(default_factory=dict)
    attacks: tuple = ()


def _number(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{what} must be a finite number')
    return float(value)


# Check every field; times are seconds after the first trusted reference.
def parse_scenario(data):
    if not isinstance(data, dict):
        raise ValueError('scenario must be a mapping')
    unknown = set(data) - {'name', 'seed', 'friction_profile', 'duration', 'planner', 'attacks'}
    if unknown:
        raise ValueError(f'unknown scenario keys: {sorted(unknown)}')
    name = data.get('name')
    if not isinstance(name, str) or not name:
        raise ValueError('name must be a non-empty string')
    seed = data.get('seed', 1)
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('seed must be a non-negative integer')
    profile = data.get('friction_profile', 'wear')
    if profile not in FRICTION_PROFILES:
        raise ValueError(f'friction_profile must be one of {FRICTION_PROFILES}')
    duration = _number(data.get('duration'), 'duration')
    if duration <= 0.0:
        raise ValueError('duration must be positive')
    planner = dict(PLANNER_DEFAULTS)
    planner_data = data.get('planner') or {}
    unknown = set(planner_data) - set(PLANNER_DEFAULTS)
    if unknown:
        raise ValueError(f'unknown planner keys: {sorted(unknown)}')
    planner.update(planner_data)
    if isinstance(planner['cycles'], bool) or not isinstance(planner['cycles'], int) \
            or planner['cycles'] < 0:
        raise ValueError('planner.cycles must be a non-negative integer')
    planner['payload_mass'] = _number(planner['payload_mass'], 'planner.payload_mass')
    if planner['payload_mass'] <= 0.0:
        raise ValueError('planner.payload_mass must be positive')
    attacks = []
    for i, a in enumerate(data.get('attacks') or []):
        if not isinstance(a, dict):
            raise ValueError(f'attack {i} must be a mapping')
        unknown = set(a) - {'id', 'kind', 'joint', 'magnitude', 'start', 'duration'}
        if unknown:
            raise ValueError(f'unknown keys in attack {i}: {sorted(unknown)}')
        kind = a.get('kind')
        if kind not in KINDS:
            raise ValueError(f'attack {i}: kind must be one of {KINDS}')
        joint = a.get('joint')
        if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 7:
            raise ValueError(f'attack {i}: joint must be an integer from 1 to 7')
        start = _number(a.get('start'), f'attack {i}: start')
        length = _number(a.get('duration'), f'attack {i}: duration')
        if start < 0.0 or length <= 0.0 or start + length > duration:
            raise ValueError(f'attack {i} must lie inside the scenario duration')
        event_id = a.get('id', i + 1)
        if isinstance(event_id, bool) or not isinstance(event_id, int) or event_id < 1:
            raise ValueError(f'attack {i}: id must be a positive integer')
        attacks.append(Attack(event_id, kind, joint - 1,
                              _number(a.get('magnitude'), f'attack {i}: magnitude'),
                              start, length))
    attacks.sort(key=lambda x: x.start)
    for first, second in zip(attacks, attacks[1:]):
        if second.start < first.end:
            raise ValueError(f'attacks {first.event_id} and {second.event_id} overlap')
    if len({a.event_id for a in attacks}) != len(attacks):
        raise ValueError('attack ids must be unique')
    return Scenario(name, seed, profile, duration, planner, tuple(attacks))


# Read and validate a scenario YAML file.
def load_scenario(path):
    with open(path) as f:
        return parse_scenario(yaml.safe_load(f))
