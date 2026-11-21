from dataclasses import dataclass
import fnmatch
import os
import re

import yaml

ACTIONS = ('publish', 'subscribe')
STORES = ('keystore', 'loot')
AUTHORITIES = ('cell', 'forged')
DOMAIN_KEYS = {'publish', 'subscribe', 'deny_publish'}
ENCLAVE_KEYS = {'store', 'ca', 'domains'}
TOP_KEYS = {'authority', 'domains', 'infrastructure', 'enclaves'}
AUTHORITY_KEYS = {'common_name', 'forged_common_name', 'validity_days'}
SEGMENT = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


@dataclass(frozen=True)
class DomainGrant:
    name: str
    domain_id: int
    publish: tuple = ()
    subscribe: tuple = ()
    deny_publish: tuple = ()


@dataclass(frozen=True)
class Enclave:
    path: str
    store: str
    ca: str
    grants: tuple

    @property
    def relative(self):
        return self.path.lstrip('/')

    def grant(self, domain_id):
        return next((g for g in self.grants if g.domain_id == domain_id), None)


@dataclass(frozen=True)
class Policy:
    common_name: str
    forged_common_name: str
    validity_days: int
    domains: dict
    infrastructure: dict
    enclaves: tuple

    @property
    def domain_ids(self):
        return tuple(sorted(set(self.domains.values())))

    def enclave(self, path):
        for e in self.enclaves:
            if e.path == path:
                return e
        raise KeyError(f'no enclave {path} in the policy')

    def in_store(self, store):
        return tuple(e for e in self.enclaves if e.store == store)


def dds_topic(name):
    return 'rt' + name if name.startswith('/') else name


def _strings(value, what):
    if value is None:
        return ()
    if not isinstance(value, list) or any(not isinstance(v, str) or not v for v in value):
        raise ValueError(f'{what} must be a list of non-empty strings')
    return tuple(value)


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


def validate_enclave_path(path):
    if not isinstance(path, str) or not path.startswith('/') or path == '/':
        raise ValueError(f'enclave {path!r} must be an absolute path such as /cell/robot')
    for segment in path[1:].split('/'):
        if not SEGMENT.match(segment):
            raise ValueError(f'enclave {path}: segment {segment!r} must start with a letter '
                             'and hold only letters, digits and underscores')
    return path


def parse_policy(data):
    data = _mapping(data, 'policy')
    _keys(data, TOP_KEYS, 'policy')
    authority = _mapping(data.get('authority'), 'authority')
    _keys(authority, AUTHORITY_KEYS, 'authority')
    common_name = authority.get('common_name', 'thesis-cell-ca')
    forged = authority.get('forged_common_name', 'rogue-ca')
    days = authority.get('validity_days', 3650)
    if isinstance(days, bool) or not isinstance(days, int) or days <= 0:
        raise ValueError('authority.validity_days must be a positive integer')
    if not common_name or not forged or common_name == forged:
        raise ValueError('authority names must be set and differ')
    domains = _mapping(data.get('domains'), 'domains')
    if not domains:
        raise ValueError('domains must name at least one DDS domain')
    for name, domain_id in domains.items():
        if isinstance(domain_id, bool) or not isinstance(domain_id, int) \
                or not 0 <= domain_id <= 232:
            raise ValueError(f'domain {name} must be an integer from 0 to 232')
    if len(set(domains.values())) != len(domains):
        raise ValueError('domain ids must be unique')
    infra = _mapping(data.get('infrastructure'), 'infrastructure')
    _keys(infra, set(ACTIONS), 'infrastructure')
    infrastructure = {a: _strings(infra.get(a), f'infrastructure.{a}') for a in ACTIONS}
    enclaves = []
    for path, spec in _mapping(data.get('enclaves'), 'enclaves').items():
        validate_enclave_path(path)
        spec = _mapping(spec, f'enclave {path}')
        _keys(spec, ENCLAVE_KEYS, f'enclave {path}')
        store = spec.get('store', 'keystore')
        if store not in STORES:
            raise ValueError(f'enclave {path}: store must be one of {STORES}')
        ca = spec.get('ca', 'cell')
        if ca not in AUTHORITIES:
            raise ValueError(f'enclave {path}: ca must be one of {AUTHORITIES}')
        if ca == 'forged' and store != 'loot':
            raise ValueError(f'enclave {path}: a forged identity belongs in the loot store')
        grants = []
        for name, rules in _mapping(spec.get('domains'), f'enclave {path}.domains').items():
            if name not in domains:
                raise ValueError(f'enclave {path}: unknown domain {name}')
            rules = _mapping(rules, f'enclave {path}.domains.{name}')
            _keys(rules, DOMAIN_KEYS, f'enclave {path}.domains.{name}')
            grants.append(DomainGrant(name, domains[name], **{
                k: _strings(rules.get(k), f'enclave {path}.domains.{name}.{k}')
                for k in DOMAIN_KEYS}))
        if not grants:
            raise ValueError(f'enclave {path} must be granted at least one domain')
        enclaves.append(Enclave(path, store, ca, tuple(grants)))
    if not enclaves:
        raise ValueError('the policy must define at least one enclave')
    for a, b in ((x, y) for i, x in enumerate(enclaves) for y in enclaves[i + 1:]):
        if a.path.startswith(b.path + '/') or b.path.startswith(a.path + '/'):
            raise ValueError(f'enclaves {a.path} and {b.path} are nested')
    return Policy(common_name, forged, days, dict(domains), infrastructure, tuple(enclaves))


def default_policy_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('ids_security'), 'config',
                            'cell_policy.yaml')
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config',
                        'cell_policy.yaml')


def load_policy(path=None):
    with open(path or default_policy_path()) as f:
        return parse_policy(yaml.safe_load(f))


@dataclass(frozen=True)
class Rule:
    kind: str
    domains: tuple
    publish: tuple = ()
    subscribe: tuple = ()


def rules(policy, enclave):
    out = []
    for g in enclave.grants:
        out += [Rule('allow_rule', (g.domain_id,), subscribe=g.subscribe),
                Rule('deny_rule', (g.domain_id,), publish=g.deny_publish),
                Rule('allow_rule', (g.domain_id,), publish=g.publish)]
    out.append(Rule('allow_rule', tuple(sorted(g.domain_id for g in enclave.grants)),
                    policy.infrastructure['publish'], policy.infrastructure['subscribe']))
    return [r for r in out if r.publish or r.subscribe]


def _matches(patterns, topic):
    return any(fnmatch.fnmatchcase(topic, dds_topic(p)) for p in patterns)


def _first_rule(ordered, domain_id, name, actions):
    for rule in ordered:
        if domain_id in rule.domains and _matches(
                sum((getattr(rule, a) for a in actions), ()), name):
            return rule.kind == 'allow_rule'
    return False


def allowed(policy, enclave, domain_id, action, topic):
    if action not in ACTIONS:
        raise ValueError(f'action must be one of {ACTIONS}')
    ordered = rules(policy, enclave)
    name = dds_topic(topic)
    return (_first_rule(ordered, domain_id, name, ACTIONS)
            and _first_rule(ordered, domain_id, name, (action,)))