import glob
import json
import math
import os

import numpy as np
import yaml

from ids_evaluation import layout

NODE_REPORTS = ('mitm', 'rogue', 'spoof')
PUBLISHED = ('forwarded', 'injected_tampered', 'injected_blind', 'spoofed')
FIELDS = ('run', 'attacker', 'credentials', 'security', 'received', 'published', 'peers',
          'rogue_samples', 'v2_alarms', 'outcome')


def reports_dir(run_dir):
    run_dir = os.path.abspath(os.path.normpath(run_dir))
    return os.path.join(os.path.dirname(os.path.dirname(run_dir)), 'cell', 'reports',
                        os.path.basename(run_dir))


def read_reports(run_dir):
    out = {}
    for path in sorted(glob.glob(os.path.join(reports_dir(run_dir), '*.yaml'))):
        name = os.path.splitext(os.path.basename(path))[0]
        if name == 'attacker_scenario':
            continue
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError):
            continue
        if isinstance(data, dict):
            out[name] = data
    return out


def _json(export, key):
    try:
        value = json.loads(str(export.get(key, '{}')))
    except ValueError:
        return {}
    return value if isinstance(value, dict) else {}


def attacker_of(scenario):
    spec = scenario.get('attacker') or {}
    role = spec.get('role', 'mitm' if scenario.get('attacks') else 'none')
    return role, spec.get('credentials', 'none' if role == 'rogue' else 'stolen')


def alarm_onsets(export, name='v2'):
    status = export.get(name)
    if status is None or not len(status):
        return 0
    armed = status[:, layout.MODE] == layout.ARMED
    alarm = (status[:, layout.ALARM] > 0.5) & armed
    return int(np.count_nonzero(alarm & ~np.r_[False, alarm[:-1]]))


def _max_matched(report, suffix):
    values = [v.get('max', 0) for k, v in (report.get('matched') or {}).items()
              if k.endswith(suffix)]
    return max(values) if values else 0


def outcome(role, credentials, report, received, peers, rogue):
    if role == 'none':
        return 'no attacker'
    if report is None:
        return 'no node report'
    if credentials in ('none', 'forged'):
        clean = received == 0 and peers == 0 and (rogue == 0 or math.isnan(rogue))
        return 'blocked by SROS2' if clean else 'NOT blocked'
    return 'passed SROS2' if received > 0 or peers > 0 else 'did not connect'


def security_row(export):
    scenario = _json(export, 'scenario')
    role, credentials = attacker_of(scenario)
    reports = _json(export, 'security')
    report = reports.get(role) if role in NODE_REPORTS else None
    counts = (report or {}).get('counts') or {}
    received = int(counts.get('received_trusted', 0))
    published = int(sum(counts.get(k, 0) for k in PUBLISHED))
    peers = int(_max_matched(report or {}, '_subscribers'))
    foreign = np.asarray(export.get('foreign', np.full(3, np.nan)), dtype=float)
    rogue = float(np.nansum(foreign)) if np.isfinite(foreign).any() else math.nan
    if report is None:
        mode = '-'
    elif report.get('security_enabled'):
        mode = f"on, {report.get('enclave') or 'no enclave'}"
    else:
        mode = 'off'
    return {'run': str(export['run']), 'attacker': role, 'credentials': credentials,
            'security': mode, 'received': received if report else None,
            'published': published if report else None, 'peers': peers if report else None,
            'rogue_samples': rogue, 'v2_alarms': alarm_onsets(export),
            'outcome': outcome(role, credentials, report, received, peers, rogue)}


def wanted(export):
    role, _ = attacker_of(_json(export, 'scenario'))
    return role != 'none' or bool(_json(export, 'security'))