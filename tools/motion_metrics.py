#!/usr/bin/env python3
"""Summarize raw wheel feedback; never infer success from body average alone."""
import argparse
import json
import math
import statistics
from pathlib import Path


def analyze(record):
    frames = record['frames']
    commands = [f for f in frames if f['id'] == 0x7f and
                f['data']['request_id'] == 0x10 and f['data']['result'] == 0]
    if not commands:
        raise ValueError('no accepted body commands')
    start, end = commands[0]['time'], commands[-1]['time'] + 20000
    samples = [f for f in frames if f['id'] == 0x20 and start <= f['time'] < end]
    steady = [f for f in samples if start + 700000 <= f['time'] < end - 100000]
    if not steady:
        raise ValueError('insufficient steady-window samples')
    result = {'duration_s': (end-start)/1e6, 'wheels': [],
              'all_samples_valid': all(f['data']['valid'] == 15 for f in samples)}
    for i, wheel in enumerate(('C/LF', 'D/LR', 'B/RF', 'A/RR')):
        onset = next((samples[k]['time'] for k in range(len(samples)-2)
                      if samples[k+2]['time']-samples[k]['time'] <= 20000 and
                      all(abs(samples[j]['data']['omega'][i]) > .4 for j in (k,k+1,k+2))), None)
        speed = [f['data']['omega'][i] for f in steady]
        # Integrate count differences over actual telemetry intervals, avoiding
        # aliasing from sampling the controller's latest 2ms speed at 100Hz.
        interval_speed = []
        for previous_sample, sample in zip(steady, steady[1:]):
            elapsed = sample['time'] - previous_sample['time']
            if elapsed <= 0:
                raise ValueError('non-increasing wheel sample timestamps')
            interval_speed.append((sample['data']['count'][i] -
                                   previous_sample['data']['count'][i]) *
                                  (2 * math.pi / 20000) * 1e6 / elapsed)
        if not interval_speed:
            raise ValueError('insufficient count intervals')
        longest = current = 0
        previous = steady[0]['time']
        for f in steady:
            step = f['time'] - previous
            current = current + step if abs(f['data']['omega'][i]) < .25 else 0
            longest = max(longest, current)
            previous = f['time']
        result['wheels'].append({'wheel': wheel,
            'sustained_start_ms': None if onset is None else (onset-start)/1000,
            'mean_rad_s': statistics.mean(speed), 'std_rad_s': statistics.pstdev(speed),
            'count_interval_mean_rad_s': statistics.mean(interval_speed),
            'count_interval_std_rad_s': statistics.pstdev(interval_speed),
            'longest_near_zero_ms': longest/1000,
            'peak_rad_s': max(abs(f['data']['omega'][i]) for f in samples)})
    result['body_mean'] = record.get('measured_body_mean')
    result['host_test_passed'] = record.get('passed', False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path, nargs='+')
    args = parser.parse_args()
    for path in args.records:
        print(json.dumps({'file': str(path), **analyze(json.loads(path.read_text()))}, indent=2))

if __name__ == '__main__':
    main()
