# =============================================================================
#  plots.py - figures for the report
# -----------------------------------------------------------------------------
#  Statistic / limit over time for each detector (log scale; the dashed line is the
#  limit, red ticks show alarms, the grey band the attack), and the v2 friction
#  estimate against the simulator's friction truth.
# =============================================================================

import math

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ids_evaluation import layout  # noqa: E402

WARMUP = 3.99
# Configured wear schedule (iiwa_sim friction.yaml), used as truth only when a run has no
# /sim/friction_truth.
WEAR = {'delay': 30.0,
        'initial': np.array([4.0, 4.0, 2.0, 2.0, 0.4, 0.2, 0.1]),
        'rate': np.array([0.004, 0.004, 0.002, 0.002, 0.0004, 0.0002, 0.0001]),
        'max': np.array([8.0, 8.0, 4.0, 4.0, 0.8, 0.4, 0.2])}

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
GRID = '#e4e3df'
BAND = '#ecebe7'
S1, S2, S3 = '#2a78d6', '#eb6834', '#1baf7a'
CRITICAL = '#d03b3b'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'font.size': 9, 'axes.titlesize': 10,
    'axes.titleweight': 'bold', 'axes.titlecolor': INK, 'legend.frameon': False,
    'lines.linewidth': 1.4,
})


class Panel:

    def __init__(self, label, t, status, limit, slow_limit=None):
        self.label = label
        self.t = np.asarray(t, dtype=float)
        self.status = layout.pad_status(status)
        self.limit = float(limit)
        self.slow_limit = None if slow_limit is None or math.isnan(slow_limit) \
            else float(slow_limit)


def wear_truth(t_abs):
    age = np.maximum(0.0, np.asarray(t_abs)[:, None] - WEAR['delay'])
    grown = WEAR['initial'] + WEAR['rate'] * age
    return np.clip(grown, 0.0, np.maximum(WEAR['max'], WEAR['initial']))


# Show the maximum per bin, so short peaks stay visible in long runs.
def binned_max(x, y, width):
    if width is None or len(x) < 2:
        return x, y
    k = np.floor((x - x[0]) / width).astype(int)
    starts = np.r_[0, np.nonzero(np.diff(k))[0] + 1]
    return x[starts] + width / 2, np.maximum.reduceat(y, starts, axis=0)


def intervals(t, flag):
    f = np.asarray(flag, dtype=bool)
    if not f.any():
        return []
    edges = np.diff(np.r_[0, f.astype(int), 0])
    starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0] - 1
    step = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    return [(t[a], t[b] + step) for a, b in zip(starts, ends)]


def _decorate(ax, events, x, alarm, on, xlim):
    for ev in events:
        end = ev.end if math.isfinite(ev.end) else x[-1]
        ax.axvspan(ev.start, end, color=BAND, zorder=0, lw=0)
        if xlim is None or xlim[0] <= ev.start <= xlim[1]:
            ax.text(ev.start, 1.0, f' attack {ev.event_id}: {ev.kind} '
                    f'{layout.joint_label(ev.joints)}', transform=ax.get_xaxis_transform(),
                    va='top', ha='left', color=INK2, fontsize=8)
    spans = intervals(x, alarm)
    for a, b in spans:
        ax.axvspan(a, max(b, a + 0.15), ymin=0.0, ymax=0.035, color=CRITICAL, lw=0, zorder=3)
    note = (f"alarm ({len(spans)} onset{'s' if len(spans) != 1 else ''}, red ticks at the base)"
            if spans else 'no alarm')
    ax.text(1.0, 1.0, note, transform=ax.transAxes, va='bottom', ha='right',
            color=CRITICAL if spans else INK2, fontsize=8)
    ax.axhline(1.0, color=INK, lw=1.0, ls=(0, (4, 3)), zorder=4)
    ax.text(0.005, 1.0, 'limit', transform=ax.get_yaxis_transform(), ha='left', va='bottom',
            color=INK, fontsize=8, zorder=4)
    if xlim is None or xlim[0] <= on <= xlim[1]:
        ax.axvline(on, color=INK2, lw=0.8, ls=(0, (1, 2)), zorder=2)
        ax.text(on, 0.06, ' detection on', transform=ax.get_xaxis_transform(), color=INK2,
                fontsize=7.5, ha='left', va='bottom')


def statistics_figure(panels, events, start, title, path, joint=None, xlim=None, binw=0.5):
    fig, axes = plt.subplots(len(panels), 1, figsize=(9, 2.8 * len(panels)), sharex=True,
                             constrained_layout=True, squeeze=False)
    for ax, p in zip(axes[:, 0], panels):
        armed = p.status[:, layout.MODE] == layout.ARMED
        x = p.t[armed] - start
        s = p.status[armed]
        if not len(x):
            ax.set_title(f'{p.label}: never armed', loc='left')
            continue
        fast = s[:, layout.FAST] / p.limit
        slow = s[:, layout.SLOW] / p.slow_limit if p.slow_limit else None
        sel = np.ones(len(x), dtype=bool) if xlim is None else (x >= xlim[0]) & (x <= xlim[1])
        x, fast, alarm = x[sel], fast[sel], s[sel, layout.ALARM] > 0.5
        slow = slow[sel] if slow is not None else None
        series = []
        if joint is None:
            series.append((fast.max(1), S1, 'fast channel, max over a1-a7'))
            if slow is not None:
                series.append((slow.max(1), S2, 'slow channel, max over a1-a7'))
        else:
            both = np.maximum(fast, slow) if slow is not None else fast
            series.append((np.delete(both, joint, axis=1).max(1), S3,
                           'other joints, max of their channels'))
            series.append((fast[:, joint], S1, f'a{joint + 1} fast channel'))
            if slow is not None:
                series.append((slow[:, joint], S2, f'a{joint + 1} slow channel'))
        lows = []
        for y, color, label in series:
            xb, yb = binned_max(x, y, binw)
            ax.plot(xb, yb, color=color, label=label, lw=1.0 if color == S3 else 1.4)
            lows.append(np.nanmin(np.maximum(yb, 1e-3)) if len(yb) else 1.0)
        _decorate(ax, events, x, alarm, p.t[armed][0] - start + WARMUP, xlim)
        ax.set_yscale('log')
        ax.set_ylabel('statistic / limit')
        ax.set_ylim(bottom=max(0.02, min(lows) * 0.8))
        ax.set_title(p.label, loc='left')
    axes[0, 0].legend(loc='upper left', bbox_to_anchor=(0.0, -0.02), ncol=3, fontsize=8)
    axes[-1, 0].set_xlabel('time after the first PLC reference (s)')
    if xlim is not None:
        axes[-1, 0].set_xlim(*xlim)
    fig.suptitle(title, x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.savefig(path, dpi=150)
    plt.close(fig)


# Returns estimate/truth - 1 over the last 10 s, per joint.
def friction_figure(t, status, start, truth_t, truth, title, path):
    s = layout.pad_status(status)
    armed = (s[:, layout.MODE] == layout.ARMED) & np.any(s[:, layout.FRICTION] != 0.0, axis=1)
    t_abs = np.asarray(t, dtype=float)[armed]
    est = s[armed][:, layout.FRICTION]
    if truth_t is not None and len(truth_t):
        idx = np.clip(np.searchsorted(truth_t, t_abs), 0, len(truth_t) - 1)
        b = np.asarray(truth)[idx]
        truth_label = 'truth (/sim/friction_truth)'
    else:
        b = wear_truth(t_abs)
        truth_label = 'truth (configured wear schedule)'
    x = t_abs - start
    fig, axes = plt.subplots(2, 4, figsize=(10, 5.2), sharex=True, constrained_layout=True)
    for j, ax in enumerate(axes.flat[:7]):
        ax.plot(x, est[:, j], color=S1, label='v2 EKF estimate', zorder=2)
        ax.plot(x, b[:, j], color=INK, lw=1.2, ls=(0, (4, 3)), label=truth_label, zorder=3)
        ax.set_title(f'a{j + 1}', loc='left')
        if j % 4 == 0:
            ax.set_ylabel('viscous friction (N*m*s/rad)')
    legend_ax = axes.flat[7]
    legend_ax.axis('off')
    handles, labels = axes.flat[0].get_legend_handles_labels()
    legend_ax.legend(handles, labels, loc='center left', fontsize=9)
    for ax in axes[1]:
        ax.set_xlabel('time after the first PLC reference (s)')
    fig.suptitle(title, x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    last = x >= x[-1] - 10.0 if len(x) else np.zeros(0, dtype=bool)
    return [float(est[last, j].mean() / b[last, j].mean() - 1.0) if last.any() else math.nan
            for j in range(layout.DOF)]
