import math
import struct

import numpy as np

HEADER = struct.Struct('<QQ')


def pack(seq, sent_ns, size):
    head = HEADER.pack(seq, sent_ns)
    return head + bytes(max(0, size - len(head)))


def unpack(data):
    return HEADER.unpack_from(bytes(data[:HEADER.size]))


def summarise(rtt_ns, sent, cpu_seconds=math.nan, wall_seconds=math.nan):
    rtt = np.asarray(rtt_ns, dtype=float) * 1e-6
    out = {'sent': int(sent), 'received': int(rtt.size),
           'lost': int(max(0, sent - rtt.size)),
           'cpu_percent': (100.0 * cpu_seconds / wall_seconds
                           if wall_seconds and math.isfinite(wall_seconds) and wall_seconds > 0
                           else math.nan)}
    if rtt.size:
        q = np.percentile(rtt, [50.0, 95.0, 99.0])
        out.update({'rtt_median_ms': float(q[0]), 'rtt_p95_ms': float(q[1]),
                    'rtt_p99_ms': float(q[2]), 'rtt_max_ms': float(rtt.max()),
                    'rtt_mean_ms': float(rtt.mean())})
    else:
        out.update({k: math.nan for k in ('rtt_median_ms', 'rtt_p95_ms', 'rtt_p99_ms',
                                          'rtt_max_ms', 'rtt_mean_ms')})
    return out


def overhead(secure, plain):
    out = {}
    for key in ('rtt_median_ms', 'rtt_p95_ms', 'rtt_p99_ms', 'rtt_max_ms', 'cpu_percent'):
        a, b = secure.get(key, math.nan), plain.get(key, math.nan)
        out[key] = {'sros2': a, 'plain': b, 'difference': a - b,
                    'ratio': a / b if b and math.isfinite(b) and b > 0 else math.nan}
    return out


def report_lines(secure, plain, size, rate):
    rows = overhead(secure, plain)
    lines = ['# SROS2 overhead (probe round trip, plc -> robot -> plc)', '',
             f'- payload {size} bytes, {rate:g} Hz; sros2: domain 30 with the cell enclaves; '
             'plain: domain 31, security off',
             f"- sent / received: sros2 {secure['sent']} / {secure['received']}, "
             f"plain {plain['sent']} / {plain['received']}", '',
             '| quantity | SROS2 | plain | difference | ratio |', '|---|---|---|---|---|']
    labels = {'rtt_median_ms': 'round trip median (ms)', 'rtt_p95_ms': 'round trip p95 (ms)',
              'rtt_p99_ms': 'round trip p99 (ms)', 'rtt_max_ms': 'round trip max (ms)',
              'cpu_percent': 'ping process CPU (%)'}
    for key, label in labels.items():
        r = rows[key]
        lines.append(f"| {label} | {r['sros2']:.3f} | {r['plain']:.3f} | "
                     f"{r['difference']:+.3f} | {r['ratio']:.2f} |")
    return lines