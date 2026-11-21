# =============================================================================
#  pipeline.py - the evaluation steps put together
# -----------------------------------------------------------------------------
#  Loads runs (cached exports), extracts the live IDS series, scores them, and builds
#  the baseline series: residual baselines on the recorded v2 disturbance, the
#  momentum observer, and three ids_replay variants (v2 settings, the thesis
#  settings, and v2 with commanded-torque input).
#  Training uses the first warm-up + duration + 1 s of a nominal run; that part of
#  the run is not scored, so baselines are judged on data they have not seen.
# =============================================================================

from dataclasses import dataclass
import glob
import json
import math
import os

import numpy as np

from ids_evaluation import bag, baselines, layout, metrics, replay

WARMUP = 3.99
DEFAULT_CALIBRATION_DIR = '/ws/data/calibration'
# Replay variants: parameter file and --set overrides.
REPLAYS = {
    'v2_replay': ('ids.yaml', ()),
    'thesis_replay': ('ids_thesis.yaml', ()),
    'tau_cmd_replay': ('ids.yaml', ('input_source=commanded_torque',)),
}
SOURCES = {
    'v2': 'live /ids/status', 'thesis': 'live /ids_thesis/status',
    'v2_replay': 'ids_replay, ids.yaml', 'thesis_replay': 'ids_replay, ids_thesis.yaml',
    'tau_cmd_replay': 'ids_replay, ids.yaml + input_source=commanded_torque',
    'static': 'static threshold on standardised v2 disturbance',
    'ewma': 'single EWMA (lambda 0.2) on standardised v2 disturbance',
    'momentum': 'generalised-momentum observer (K 5) on measured torque',
}


@dataclass
class Series:
    name: str
    t: np.ndarray
    status: np.ndarray
    limit: float
    slow_limit: float = math.nan


def run_name(export):
    return str(export['run'])


def start_of(export):
    return float(export['start'])


def is_run_dir(path):
    return os.path.isdir(os.path.join(path, 'bag')) and \
        os.path.isfile(os.path.join(path, 'summary.yaml'))


def expand(paths):
    out = []
    for p in paths:
        if os.path.isdir(p) and not is_run_dir(p):
            out += sorted(d for d in glob.glob(os.path.join(p, '*')) if is_run_dir(d))
        else:
            out.append(p)
    return out


# Re-export only when the bag is newer than the cached export.
def load_run(path, calibration_dir=DEFAULT_CALIBRATION_DIR, force=False):
    if path.endswith('.npz'):
        return bag.load_export(path)
    cache = os.path.join(path, 'export.npz')
    bag_dir = os.path.join(path, 'bag')
    fresh = os.path.isfile(cache) and all(
        os.path.getmtime(cache) >= os.path.getmtime(f)
        for f in glob.glob(os.path.join(bag_dir, '*')))
    if fresh and not force:
        return bag.load_export(cache)
    export = bag.export_run(path, calibration_dir=calibration_dir)
    bag.save_export(cache, export)
    return bag.normalise(export)


# Scenario of a v1.2 run from its name and the campaign file.
def legacy_scenarios(runs, campaign_path):
    if not campaign_path or not os.path.isfile(campaign_path):
        return {}
    try:
        from ids_bringup.campaign import load_campaign, resolve
    except ImportError:
        return {}
    campaign = load_campaign(campaign_path)
    out = {}
    for run in runs:
        name, _, seed = run.rpartition('_s')
        if name in campaign.entries and seed.isdigit():
            out[run] = resolve(campaign, name, int(seed))
    return out


def load_legacy(path, campaign_path=None, scenarios=None):
    with np.load(path, allow_pickle=False) as data:
        runs = sorted({k.split('__', 1)[0] for k in data.files})
    if scenarios is None and campaign_path:
        scenarios = legacy_scenarios(runs, campaign_path)
    split = bag.split_legacy_export(path, scenarios)
    return [split[r] for r in runs]


def live_series(export):
    limits = np.asarray(export['limits'], dtype=float)
    out = []
    for i, name in enumerate(layout.INSTANCES):
        status = export[name]
        if not len(status):
            continue
        limit = limits[2 * i]
        if math.isnan(limit):
            col = status[:, layout.LIMIT]
            limit = float(np.nanmax(col)) if np.isfinite(col).any() else math.nan
        out.append(Series(name, export[f'{name}_t'] - start_of(export), status, limit,
                          limits[2 * i + 1]))
    return out


# Mark the training part as not armed, so it is not scored.
def held_out(series, after):
    if after is None:
        return series
    status = series.status.copy()
    status[series.t < after, layout.MODE] = 0.0
    return Series(series.name, series.t, status, series.limit, series.slow_limit)


def evaluate(export, series, grace=1.0, tail=30.0, after=None):
    events = metrics.events_from_truth(export['attack'], start_of(export))
    results = []
    for s in series:
        s = held_out(s, after)
        results.append(metrics.evaluate_instance(s.name, s.t, s.status, events, WARMUP, grace,
                                                 tail))
    return events, results


def crop(export, end):
    out = dict(export)
    stop = start_of(export) + end
    for key in ('reference', 'measurement'):
        out[key] = export[key][export[key][:, 0] <= stop]
    return out


def training_end(duration, warmup=2.0):
    return warmup + duration + 1.0


def armed_disturbance(export):
    status = export['v2']
    armed = status[:, layout.MODE] == layout.ARMED
    return export['v2_t'][armed] - start_of(export), status[armed][:, layout.DISTURBANCE]


# Standardise with the training windows, then apply the static and EWMA statistics.
def residual_baselines(train, tests, train_end, rules=baselines.DetectorRules()):
    t, d = armed_disturbance(train)
    keep = (t >= t[0] + WARMUP) & (t <= train_end) if len(t) else np.zeros(0, dtype=bool)
    if keep.sum() < 100:
        raise ValueError('training run has fewer than 100 armed v2 windows')
    scale = baselines.Standardiser.fit(d[keep])
    statistics = {'static': baselines.static_statistic, 'ewma': baselines.ewma_statistic}
    limits = {name: baselines.lowest_quiet_limit(f(scale(d))[keep], rules)
              for name, f in statistics.items()}
    out = {}
    for export in tests:
        tt, dd = armed_disturbance(export)
        runs = []
        for name, f in statistics.items():
            s = f(scale(dd))
            alarm, primary = baselines.alarm_series(s, limits[name], rules)
            status = baselines.status_from_alarms(alarm, primary, s, limits[name])
            runs.append(Series(name, tt, status, limits[name]))
        out[run_name(export)] = runs
    return out, limits


def momentum_residual(observer, export):
    t, q, v, tau = baselines.measurement_arrays(export['measurement'])
    mass = baselines.context_mass(export['reference'], t)
    return t - start_of(export), observer.residual(t, q, v, tau, mass)


def momentum_baseline(urdf, train, tests, train_end, rules=baselines.DetectorRules()):
    observer = baselines.MomentumObserver(urdf)
    t, r = momentum_residual(observer, train)
    keep = (t >= 2.0) & (t <= train_end)
    if keep.sum() < 100:
        raise ValueError('training run has fewer than 100 measurement samples')
    scale = baselines.Standardiser.fit(r[keep])
    limit = baselines.lowest_quiet_limit(np.abs(scale(r))[keep], rules)
    out = {}
    for export in tests:
        tt, rr = momentum_residual(observer, export)
        s = np.abs(scale(rr))
        alarm, primary = baselines.alarm_series(s, limit, rules)
        out[run_name(export)] = Series('momentum', tt, baselines.status_from_alarms(
            alarm, primary, s, limit), limit)
    return out, limit


# Calibrate each variant on the training part, then arm it on every test run.
def replay_baselines(urdf, train, tests, workdir, duration=120.0, variants=REPLAYS,
                     binary=None):
    os.makedirs(workdir, exist_ok=True)
    cropped = crop(train, training_end(duration))
    out = {run_name(e): [] for e in tests}
    calibrations = {}
    for name, (params_file, overrides) in variants.items():
        params = replay.package_params(params_file)
        cal = os.path.join(workdir, f'{name}_calibration.yaml')
        replay.run_replay(cropped, urdf, params, calibrate=cal, duration=duration,
                          overrides=overrides, binary=binary)
        calibrations[name] = replay.read_limits(cal)
        for export in tests:
            windows, limit, _ = replay.run_replay(export, urdf, params, calibration=cal,
                                                  overrides=overrides, binary=binary)
            t, status = replay.replay_status(windows, limit)
            out[run_name(export)].append(Series(name, t - start_of(export), status, limit,
                                                calibrations[name][1]))
    return out, calibrations


# How closely the replayed disturbance matches the live one, window by window: a check of the
# replay itself.
def replay_fidelity(live, replayed, tolerance=0.002, agree=1e-3):
    a = live.status[:, layout.MODE] == layout.ARMED
    b = replayed.status[:, layout.MODE] == layout.ARMED
    ta, tb = live.t[a], replayed.t[b]
    if not len(ta) or not len(tb):
        return None
    idx = np.clip(np.searchsorted(tb, ta), 0, len(tb) - 1)
    prev = np.clip(idx - 1, 0, len(tb) - 1)
    idx = np.where(np.abs(tb[prev] - ta) < np.abs(tb[idx] - ta), prev, idx)
    match = np.abs(tb[idx] - ta) <= tolerance
    if not match.any():
        return None
    da = live.status[a][match][:, layout.DISTURBANCE]
    db = replayed.status[b][idx[match]][:, layout.DISTURBANCE]
    rms = np.sqrt(np.mean(da ** 2, axis=0))
    diff = np.sqrt(np.mean((da - db) ** 2, axis=0))
    corr = [float(np.corrcoef(da[:, j], db[:, j])[0, 1]) if da[:, j].std() > 0 else math.nan
            for j in range(layout.DOF)]
    close = np.abs(da - db).max(axis=1) <= agree
    return {'matched': int(match.sum()), 'live': int(a.sum()), 'replay': int(b.sum()),
            'agree_fraction': float(close.mean()),
            'relative_rms': (diff / np.maximum(rms, 1e-12)).tolist(), 'correlation': corr}


# Runs with torque at an effort limit are not nominal and are excluded unless asked for.
def saturated(export):
    try:
        summary = json.loads(str(export['summary']))
    except ValueError:
        return False
    return any('effort limit' in str(w) for w in summary.get('warnings') or [])


def keep_runs(exports, include_saturated=False):
    drop = [not include_saturated and saturated(e) for e in exports]
    return ([e for e, d in zip(exports, drop) if not d],
            [run_name(e) for e, d in zip(exports, drop) if d])


def scenario_of(export):
    try:
        return json.loads(str(export['scenario']))
    except ValueError:
        return {}
