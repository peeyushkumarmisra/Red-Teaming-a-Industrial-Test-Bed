
from dataclasses import dataclass
import fnmatch

import yaml

from ids_experiments.scenario import parse_scenario

TOP_KEYS = {'defaults', 'scenarios', 'families', 'sweeps', 'campaign'}
DEFAULT_KEYS = {'friction_profile', 'planner', 'seeds'}
SCENARIO_KEYS = {'friction_profile', 'duration', 'planner', 'seeds', 'attacks', 'cover_torque',
                 'attacker', 'halt_on_alarm'}
FAMILY_KEYS = {'friction_profile', 'duration', 'planner', 'seeds', 'joints', 'attack',
               'cover_torque', 'attacker', 'halt_on_alarm'}
OPTIONAL_KEYS = ('cover_torque', 'attacker', 'halt_on_alarm')
SWEEP_KEYS = FAMILY_KEYS | {'magnitudes'}


@dataclass(frozen=True)
class Campaign:
    entries: dict
    order: tuple


def _mapping(value, what):
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f'{what} must be a mapping')
    return value


def _keys(value, allowed, what):
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f'unknown keys in {what}: {sorted(unknown)}')


def _seeds(value, what):
    if not isinstance(value, list) or not value or any(
            isinstance(s, bool) or not isinstance(s, int) or s < 0 for s in value):
        raise ValueError(f'{what} must be a non-empty list of non-negative integers')
    return tuple(value)


def _entry(name, defaults, spec, attacks):
    planner = dict(_mapping(defaults.get('planner'), 'defaults.planner'))
    planner.update(_mapping(spec.get('planner'), f'{name}.planner'))
    entry = {
        'name': name,
        'friction_profile': spec.get('friction_profile', defaults.get('friction_profile', 'wear')),
        'duration': spec.get('duration'),
        'planner': planner,
        'attacks': attacks,
        'seeds': _seeds(spec.get('seeds', defaults.get('seeds', [1])), f'{name}.seeds'),
        'cover_torque': spec.get('cover_torque', False),
        'attacker': spec.get('attacker'),
        'halt_on_alarm': spec.get('halt_on_alarm', False),
    }
    try:
        scenario_for(entry, entry['seeds'][0])
    except ValueError as e:
        raise ValueError(f'{name}: {e}') from None
    return entry


def scenario_for(entry, seed):
    data = {k: entry[k] for k in ('name', 'friction_profile', 'duration', 'planner', 'attacks')}
    for key in OPTIONAL_KEYS:
        if entry.get(key):
            data[key] = dict(entry[key]) if isinstance(entry[key], dict) else entry[key]
    data['seed'] = seed
    data['attacks'] = [dict(a) for a in entry['attacks']]
    data['planner'] = dict(entry['planner'])
    parse_scenario(data)
    return data


def parse_campaign(data):
    data = _mapping(data, 'campaign file')
    _keys(data, TOP_KEYS, 'campaign file')
    defaults = _mapping(data.get('defaults'), 'defaults')
    _keys(defaults, DEFAULT_KEYS, 'defaults')
    entries = {}
    families = {}
    for name, spec in _mapping(data.get('scenarios'), 'scenarios').items():
        spec = _mapping(spec, f'scenario {name}')
        _keys(spec, SCENARIO_KEYS, f'scenario {name}')
        entries[name] = _entry(name, defaults, spec, list(spec.get('attacks') or []))
    for family, spec in _mapping(data.get('families'), 'families').items():
        spec = _mapping(spec, f'family {family}')
        _keys(spec, FAMILY_KEYS, f'family {family}')
        joints = spec.get('joints')
        if not isinstance(joints, list) or not joints:
            raise ValueError(f'family {family}: joints must be a non-empty list')
        attack = _mapping(spec.get('attack'), f'family {family}.attack')
        families[family] = []
        for joint in joints:
            name = f'{family}_a{joint}'
            if name in entries:
                raise ValueError(f'duplicate scenario name {name}')
            entries[name] = _entry(name, defaults, spec, [dict(attack, id=1, joint=joint)])
            families[family].append(name)
    for sweep, spec in _mapping(data.get('sweeps'), 'sweeps').items():
        spec = _mapping(spec, f'sweep {sweep}')
        _keys(spec, SWEEP_KEYS, f'sweep {sweep}')
        joints = spec.get('joints')
        magnitudes = spec.get('magnitudes')
        if not isinstance(joints, list) or not joints:
            raise ValueError(f'sweep {sweep}: joints must be a non-empty list')
        if not isinstance(magnitudes, list) or not magnitudes:
            raise ValueError(f'sweep {sweep}: magnitudes must be a non-empty list')
        attack = _mapping(spec.get('attack'), f'sweep {sweep}.attack')
        families[sweep] = []
        for joint in joints:
            for magnitude in magnitudes:
                name = f'{sweep}_a{joint}_m{magnitude:g}'
                if name in entries:
                    raise ValueError(f'duplicate scenario name {name}')
                entries[name] = _entry(name, defaults, spec,
                                       [dict(attack, id=1, joint=joint, magnitude=magnitude)])
                families[sweep].append(name)
    order = []
    for item in data.get('campaign') or []:
        if item in families:
            order.extend(families[item])
        elif item in entries:
            order.append(item)
        else:
            raise ValueError(f'campaign entry {item} is neither a scenario, a family nor a sweep')
    return Campaign(entries, tuple(order))


def load_campaign(path):
    with open(path) as f:
        return parse_campaign(yaml.safe_load(f))


def resolve(campaign, name, seed):
    if name not in campaign.entries:
        raise ValueError(f'unknown scenario {name}; known: {", ".join(sorted(campaign.entries))}')
    return scenario_for(campaign.entries[name], int(seed))


def plan(campaign, only=None, seeds=None):
    if only:
        candidates = list(campaign.order) + sorted(set(campaign.entries) - set(campaign.order))
        names = [n for n in candidates if any(fnmatch.fnmatchcase(n, p) for p in only)]
        if not names:
            raise ValueError(f'no scenario matches {only}')
    else:
        names = list(campaign.order)
    return [(name, seed) for name in names
            for seed in (seeds or campaign.entries[name]['seeds'])]