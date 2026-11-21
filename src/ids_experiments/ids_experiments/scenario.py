
from dataclasses import dataclass, field
import math

import yaml

from ids_attacks.library import make_attack

FRICTION_PROFILES = ('none', 'nominal', 'wear', 'thesis')
PLANNER_DEFAULTS = {'cycles': 0, 'payload_mass': 5.0}
SCENARIO_KEYS = {'name', 'seed', 'friction_profile', 'duration', 'planner', 'attacks',
                 'cover_torque', 'attacker', 'halt_on_alarm'}
ROLES = ('none', 'mitm', 'rogue', 'spoof')
CREDENTIALS = ('stolen', 'none', 'forged')
ATTACKER_KEYS = {'role', 'credentials'}
ATTACK_KEYS = {'id', 'kind', 'joint', 'joints', 'magnitude', 'magnitudes', 'start', 'duration',
               'frequency', 'lag'}


@dataclass(frozen=True)
class Scenario:
    name: str
    seed: int
    friction_profile: str
    duration: float
    planner: dict = field(default_factory=dict)
    attacks: tuple = ()
    cover_torque: bool = False
    attacker_role: str = 'none'
    credentials: str = 'stolen'
    halt_on_alarm: bool = False


def _number(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{what} must be a finite number')
    return float(value)


def _joints(a, i):
    if 'joint' in a and 'joints' in a:
        raise ValueError(f'attack {i}: give joint or joints, not both')
    joints = a.get('joints', [a.get('joint')] if 'joint' in a else None)
    if not isinstance(joints, list) or not joints or any(
            isinstance(j, bool) or not isinstance(j, int) or not 1 <= j <= 7 for j in joints):
        raise ValueError(f'attack {i}: joint(s) must be integers from 1 to 7')
    return tuple(j - 1 for j in joints)


def _attack(a, i, seed, duration):
    if not isinstance(a, dict):
        raise ValueError(f'attack {i} must be a mapping')
    unknown = set(a) - ATTACK_KEYS
    if unknown:
        raise ValueError(f'unknown keys in attack {i}: {sorted(unknown)}')
    magnitudes = a.get('magnitudes') or ()
    if not isinstance(magnitudes, (list, tuple)):
        raise ValueError(f'attack {i}: magnitudes must be a list')
    magnitude = a.get('magnitude')
    if magnitude is None and not magnitudes:
        if a.get('kind') not in ('freeze', 'replay', 'delay'):
            raise ValueError(f'attack {i}: magnitude is required')
        magnitude = 0.0
    joints = _joints(a, i)
    try:
        attack = make_attack(a.get('id', i + 1), a.get('kind'), joints, magnitude,
                             a.get('start'), a.get('duration'), a.get('frequency', 0.0),
                             a.get('lag', 0.0), tuple(magnitudes), seed)
    except (TypeError, ValueError) as e:
        raise ValueError(f'attack {i}: {e}') from None
    if attack.end > duration:
        raise ValueError(f'attack {i} must lie inside the scenario duration')
    return attack


def _attacker(data, has_attacks, cover):
    spec = data.get('attacker')
    if spec is None:
        spec = {}
    if not isinstance(spec, dict):
        raise ValueError('attacker must be a mapping with role and credentials')
    unknown = set(spec) - ATTACKER_KEYS
    if unknown:
        raise ValueError(f'unknown attacker keys: {sorted(unknown)}')
    role = spec.get('role', 'mitm' if has_attacks else 'none')
    if role not in ROLES:
        raise ValueError(f'attacker.role must be one of {ROLES}')
    credentials = spec.get('credentials', 'none' if role == 'rogue' else 'stolen')
    if credentials not in CREDENTIALS:
        raise ValueError(f'attacker.credentials must be one of {CREDENTIALS}')
    if role == 'none' and has_attacks:
        raise ValueError('attacks need an attacker role other than none')
    if role != 'none' and not has_attacks:
        raise ValueError(f'attacker role {role} needs at least one attack')
    if cover and role != 'mitm':
        raise ValueError('cover_torque needs the mitm role')
    return role, credentials


def parse_scenario(data):
    if not isinstance(data, dict):
        raise ValueError('scenario must be a mapping')
    unknown = set(data) - SCENARIO_KEYS
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
    cover = data.get('cover_torque', False)
    if not isinstance(cover, bool):
        raise ValueError('cover_torque must be true or false')
    attacks = [_attack(a, i, seed, duration) for i, a in enumerate(data.get('attacks') or [])]
    attacks.sort(key=lambda x: x.start)
    for first, second in zip(attacks, attacks[1:]):
        if second.start < first.end:
            raise ValueError(f'attacks {first.event_id} and {second.event_id} overlap')
    if len({a.event_id for a in attacks}) != len(attacks):
        raise ValueError('attack ids must be unique')
    role, credentials = _attacker(data, bool(attacks), cover)
    halt = data.get('halt_on_alarm', False)
    if not isinstance(halt, bool):
        raise ValueError('halt_on_alarm must be true or false')
    return Scenario(name, seed, profile, duration, planner, tuple(attacks), cover, role,
                    credentials, halt)


def load_scenario(path):
    with open(path) as f:
        return parse_scenario(yaml.safe_load(f))