# =============================================================================
#  report.py - Markdown and CSV report
# -----------------------------------------------------------------------------
#  report.md with the detectors, every attack event, a table per detector and attack,
#  false alarms per run and pooled, extra sections and the figures; plus events.csv,
#  runs.csv and per_joint.csv for further analysis.
# =============================================================================

import csv
import math
import os

from ids_evaluation import layout, metrics

EVENT_FIELDS = ('run', 'instance', 'event_id', 'kind', 'joints', 'magnitude', 'start', 'end',
                'detected', 'before_end', 'latency', 'attributed', 'already_alarmed',
                'primary')
RUN_FIELDS = ('run', 'instance', 'armed_windows', 'alarm_windows', 'alarm_fraction',
              'false_alarms', 'observed_hours', 'false_alarms_per_hour')


def fmt(value, digits=3):
    if value is None:
        return '-'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        if math.isnan(value):
            return '-'
        if math.isinf(value):
            return 'open'
        return f'{value:.{digits}f}'
    return str(value)


def event_rows(results):
    rows = []
    for run, r in results:
        for e in r.events:
            ev = e.event
            rows.append({'run': run, 'instance': r.instance, 'event_id': ev.event_id,
                         'kind': ev.kind, 'joints': layout.joint_label(ev.joints),
                         'magnitude': ev.magnitude, 'start': ev.start, 'end': ev.end,
                         'detected': e.detected, 'before_end': e.before_end,
                         'latency': e.latency,
                         'attributed': e.attributed, 'already_alarmed': e.already_alarmed,
                         'primary': layout.joint_label((e.primary,)) if e.primary >= 0
                         else '-'})
    return rows


def run_rows(results):
    return [{'run': run, 'instance': r.instance, 'armed_windows': r.armed_windows,
             'alarm_windows': r.alarm_windows, 'alarm_fraction': r.alarm_fraction,
             'false_alarms': len(r.false_alarms), 'observed_hours': r.observed_hours,
             'false_alarms_per_hour': r.false_alarms_per_hour} for run, r in results]


def write_csv(path, rows, fields):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(fields))
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k) for k in fields})


def table(rows, fields, headers=None):
    headers = headers or fields
    lines = ['| ' + ' | '.join(headers) + ' |', '|' + '---|' * len(fields)]
    for row in rows:
        lines.append('| ' + ' | '.join(fmt(row.get(k)) for k in fields) + ' |')
    return '\n'.join(lines)


def write_report(out_dir, title, results, sources, notes=(), figures=(), extra=()):
    os.makedirs(out_dir, exist_ok=True)
    events = event_rows(results)
    runs = run_rows(results)
    joints = metrics.per_joint_table(results)
    pooled = metrics.false_alarm_table(results)
    write_csv(os.path.join(out_dir, 'events.csv'), events, EVENT_FIELDS)
    write_csv(os.path.join(out_dir, 'runs.csv'), runs, RUN_FIELDS)
    if joints:
        write_csv(os.path.join(out_dir, 'per_joint.csv'), joints, list(joints[0]))
    parts = [f'# {title}', '', 'All values below are measured from the listed runs. They hold '
             'only for these runs, seeds and scenarios.', '']
    parts += [f'- {n}' for n in notes]
    parts += ['', '## Detectors', '', table(
        [{'instance': k, 'source': v} for k, v in sources.items()], ('instance', 'source')),
        '', '## Attack events', '']
    parts.append(table(events, ('run', 'instance', 'kind', 'joints', 'magnitude', 'start',
                                'detected', 'before_end', 'latency', 'attributed', 'primary'),
                       ('run', 'detector', 'kind', 'joints', 'magnitude', 'start (s)',
                        'detected', 'before attack end', 'latency (s)', 'right joint',
                        'primary'))
                 if events else 'No attack events in these runs.')
    parts += ['', '## Per detector and attack', '']
    parts.append(table(joints, ('instance', 'kind', 'joints', 'magnitude', 'events',
                                'detected', 'before_end', 'attributed', 'median_latency',
                                'max_latency'),
                       ('detector', 'kind', 'joints', 'magnitude', 'events', 'detected',
                        'before attack end', 'right joint', 'median latency (s)',
                        'max latency (s)'))
                 if joints else 'No attack events in these runs.')
    parts += ['', '## False alarms', '', table(
        runs, ('run', 'instance', 'false_alarms', 'observed_hours', 'false_alarms_per_hour',
               'alarm_fraction'),
        ('run', 'detector', 'false alarms', 'observed (h)', 'per hour', 'alarm fraction')),
        '', 'Pooled over runs:', '', table(
            pooled, ('instance', 'false_alarms', 'hours', 'per_hour', 'alarm_fraction'),
            ('detector', 'false alarms', 'observed (h)', 'per hour', 'alarm fraction'))]
    for heading, body in extra:
        parts += ['', f'## {heading}', '', body]
    if figures:
        parts += ['', '## Figures', '']
        parts += [f'![{os.path.basename(f)}]({os.path.relpath(f, out_dir)})' for f in figures]
    parts += ['', 'Files: events.csv, runs.csv' + (', per_joint.csv' if joints else '') + '.',
              '']
    path = os.path.join(out_dir, 'report.md')
    with open(path, 'w') as f:
        f.write('\n'.join(parts))
    return path
