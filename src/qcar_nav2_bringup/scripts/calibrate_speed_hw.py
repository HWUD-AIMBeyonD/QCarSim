#!/usr/bin/env python3
"""Calibrate the sim's throttle -> speed gain from hardware OptiTrack runs.

The real car drives faster than /cmd_vel asks for: the hardware interface
maps v to a PWM duty (throttle = v / max_speed * 0.2) and the motor's duty ->
speed curve is not the inverse of that. This measures the ratio
    measured speed / commanded speed
over every moving sample of every hardware bag and turns it into the
qcar_hw_emulator parameter:
    throttle_to_speed = (max_speed / 0.2) * ratio

Measured speed = OptiTrack displacement over a centred --window, sampled at
10 Hz. Commanded speed = last /cmd_vel (zero after the hardware's 1 s
timeout), shifted by the response lag that best correlates the two in each
bag. The ratio is sum(measured) / sum(commanded) over samples where the
command is > 0.02 m/s, i.e. it matches distance driven per distance commanded.

Usage:
    python3 calibrate_speed_hw.py [benchmarks_hw] [--max-speed 0.5]
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compute_kpis_hw as kpi  # noqa: E402

CMD_TIMEOUT = 1.0   # s, qcar_hardware_interface.py
MOVING = 0.02       # m/s


def commanded_at(cmd, ts):
    idx = np.searchsorted(cmd[:, 0], ts) - 1
    v = np.where(idx >= 0, cmd[np.clip(idx, 0, None), 1], 0.0)
    stale = ts - cmd[np.clip(idx, 0, None), 0] > CMD_TIMEOUT
    return np.where(stale, 0.0, v)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('root', nargs='?', default='benchmarks_hw')
    p.add_argument('--max-speed', type=float, default=0.5, help='hardware interface max_speed')
    p.add_argument('--window', type=float, default=0.5, help='s, speed differencing window')
    args = p.parse_args()

    all_cmd, all_spd = [], []
    for bag in sorted(glob.glob(os.path.join(args.root, '*', 'rosbags', '*'))):
        _, odoms, cmd_vels, _ = kpi.read_bag_sqlite(bag, '/plan', '/odom_opti', '/cmd_vel')
        if not odoms or not any(abs(v) > MOVING for _, v, _ in cmd_vels):
            continue
        odom = np.array(odoms, float)
        odom[:, 0] /= 1e9
        cmd = np.array(cmd_vels, float)
        cmd[:, 0] /= 1e9

        t, w = odom[:, 0], args.window / 2
        ts = np.arange(t[0] + w, t[-1] - w, 0.1)
        dx = np.interp(ts + w, t, odom[:, 1]) - np.interp(ts - w, t, odom[:, 1])
        dy = np.interp(ts + w, t, odom[:, 2]) - np.interp(ts - w, t, odom[:, 2])
        spd = np.hypot(dx, dy) / args.window

        best = None
        for lag in np.arange(0.0, 1.01, 0.05):
            cv = commanded_at(cmd, ts - lag)
            m = cv > MOVING
            r = np.corrcoef(cv[m], spd[m])[0, 1] if m.sum() > 10 else -1
            if best is None or r > best[0]:
                best = (r, lag, cv)
        r, lag, cv = best
        m = cv > MOVING
        print('{:<28} lag {:.2f} s  r {:.2f}  measured/commanded {:.2f}'.format(
            os.path.basename(bag), lag, r, spd[m].sum() / cv[m].sum()))
        all_cmd.append(cv[m])
        all_spd.append(spd[m])

    n_bags = len(all_cmd)
    all_cmd = np.concatenate(all_cmd)
    all_spd = np.concatenate(all_spd)
    ratio = all_spd.sum() / all_cmd.sum()
    print('\nmeasured / commanded speed: {:.3f}  ({} samples, {} bags)'.format(
        ratio, len(all_cmd), n_bags))
    print('throttle_to_speed = {:.2f}  (ideal, car drives at the commanded speed: {:.2f})'.format(
        args.max_speed / 0.2 * ratio, args.max_speed / 0.2))


if __name__ == '__main__':
    main()
