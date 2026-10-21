#!/usr/bin/env python3
"""Record real IMU attitude via an already-running OpenOCD Debug session.

Normal recording only halts/resumes the CPU. --fault-check additionally changes
the Debug injection flag. Never builds/flashes, changes calibration, or simulates
physical movement. Device ticks define recording duration.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import hardware_validate_imu as safety

ROOT = Path(__file__).resolve().parents[1]
INTS = {
    'sequence': 'navbot_imu_snapshot.sequence',
    'timestamp_ms': 'navbot_imu_snapshot.timestamp_ms',
    'imu_valid': 'navbot_imu_snapshot.valid',
    'period_min_us': 'navbot_imu_snapshot.period_min_us',
    'period_max_us': 'navbot_imu_snapshot.period_max_us',
    'execution_max_us': 'navbot_imu_snapshot.execution_max_us',
    'deadline_misses': 'navbot_imu_snapshot.deadline_misses',
}
for field in ('valid', 'calibrated', 'calibration_state', 'calibration_samples',
              'calibration_resets', 'calibration_rejections', 'calibration_runs', 'updates', 'heading_epoch',
              'input_errors', 'dt_rejects', 'accel_rejects', 'numeric_errors'):
    INTS[field] = 'navbot_attitude_snapshot.' + field
for i in range(4):
    INTS[f'stack_free_{i}'] = f'navbot_stack_free_words[{i}]'
INTS['oled_frames'] = 'navbot_oled_frames'
INTS['oled_max_us'] = 'navbot_oled_max_us'
FLOATS = {field: 'navbot_attitude_snapshot.' + field for field in
          ('roll_deg', 'pitch_deg', 'yaw_deg', 'dt_s', 'accel_norm_g', 'calibration_temperature_c')}
for field in ('gyro_bias_rad_s', 'gyro_std_rad_s', 'gyro_corrected_rad_s', 'q'):
    for i in range(4 if field == 'q' else 3):
        FLOATS[f'{field}_{i}'] = f'navbot_attitude_snapshot.{field}[{i}]'
FLOATS['temperature_c'] = 'navbot_imu_snapshot.temperature_c'
for field in ('accel_m_s2', 'gyro_rad_s'):
    for i in range(3):
        FLOATS[f'{field}_{i}'] = f'navbot_imu_snapshot.{field}[{i}]'


def capture(directory, name):
    registers = [f'*(unsigned int*)0x{a:x}' for a in safety.REGISTERS]
    registers.append('(*(unsigned int*)0x40010044)&0x8000')
    floats = 'printf "FLOAT ' + ' '.join('%g' for _ in FLOATS) + r'\n", ' + ', '.join(FLOATS.values())
    commands = ['target remote localhost:3333', safety.printf_command('SAFE', registers),
                safety.printf_command('INT', list(INTS.values())), floats,
                'monitor resume', 'disconnect']
    args = ['arm-none-eabi-gdb', '-q', '-batch', 'build/Debug/navbot_body.elf']
    for command in commands:
        args.extend(['-ex', command])
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=15)
    (directory / f'{name}.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    lines = result.stdout.splitlines()
    data = {}
    for marker, keys, convert in (('SAFE', registers, int), ('INT', INTS, int), ('FLOAT', FLOATS, float)):
        values = next(line for line in lines if line.startswith(marker + ' ')).split()[1:]
        assert len(values) == len(keys), (marker, values)
        if marker == 'SAFE':
            assert all(int(v) == 0 for v in values), values
        else:
            data.update(zip(keys, map(convert, values)))
    print(json.dumps(data), flush=True)
    return data


def record(directory, seconds):
    samples = []
    try:
        deadline = time.monotonic() + 120
        while True:
            data = capture(directory, f'wait-{len(samples):03d}')
            samples.append(data)
            if data['valid'] and data['calibrated']:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Calibration/attitude not ready within 120 seconds')
            time.sleep(1)
        start = data['timestamp_ms']
        epoch = data['heading_epoch']
        window = [data]
        while ((data['timestamp_ms'] - start) & 0xFFFFFFFF) < seconds * 1000:
            time.sleep(1)
            data = capture(directory, f'sample-{len(window):03d}')
            assert data['valid'] and data['heading_epoch'] == epoch, data
            assert data['deadline_misses'] == 0 and data['numeric_errors'] == 0, data
            window.append(data)
            samples.append(data)
        summary = {'elapsed_ms': (window[-1]['timestamp_ms'] - start) & 0xFFFFFFFF,
                   'sequence_delta': window[-1]['sequence'] - window[0]['sequence']}
        for axis in ('roll_deg', 'pitch_deg', 'yaw_deg'):
            # Unwrap each short step so crossing +/-180 does not look like drift.
            unwrapped = [window[0][axis]]
            for before, after in zip(window, window[1:]):
                step = (after[axis] - before[axis] + 180) % 360 - 180
                unwrapped.append(unwrapped[-1] + step)
            summary[axis] = {'start': unwrapped[0], 'end': unwrapped[-1],
                             'net': unwrapped[-1] - unwrapped[0],
                             'span': max(unwrapped) - min(unwrapped)}
        (directory / 'summary.json').write_text(json.dumps(summary, indent=2))
        print('SUMMARY ' + json.dumps(summary), flush=True)
    finally:
        (directory / 'samples.json').write_text(json.dumps(samples, indent=2))


def set_fault(value):
    commands = ['target remote localhost:3333', f'set variable c30d_imu_test_fault={value}',
                'monitor resume', 'disconnect']
    args = ['arm-none-eabi-gdb', '-q', '-batch', 'build/Debug/navbot_body.elf']
    for command in commands:
        args.extend(['-ex', command])
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=15)
    result.check_returncode()


def fault_check(directory):
    samples = []
    try:
        before = capture(directory, 'before')
        samples.append(before)
        assert before['valid'] and before['calibrated'], before
        set_fault(1)  # Real NACK at unpopulated 0x69; not simulated physical motion.
        time.sleep(0.15)
        failed = capture(directory, 'invalid')
        samples.append(failed)
        assert not failed['imu_valid'] and not failed['valid'], failed
        set_fault(0)
        time.sleep(1.5)
        warming = capture(directory, 'recalibrating')
        samples.append(warming)
        assert warming['imu_valid'] and not warming['valid'] and not warming['calibrated'], warming
        deadline = time.monotonic() + 60
        while True:
            time.sleep(1)
            recovered = capture(directory, f'recovery-wait-{len(samples):02d}')
            samples.append(recovered)
            if recovered['valid'] and recovered['calibrated']:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Recovery recalibration did not finish: keep board still')
        assert recovered['heading_epoch'] == before['heading_epoch'] + 1, recovered
        assert recovered['calibration_runs'] == before['calibration_runs'] + 1, recovered
        assert recovered['input_errors'] > before['input_errors'], recovered
        print('PASS: invalid on stream fault, mandatory recalibration, new heading epoch after recovery', flush=True)
    finally:
        try:
            set_fault(0)
        finally:
            (directory / 'samples.json').write_text(json.dumps(samples, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--name', default='static-drift')
    parser.add_argument('--fault-check', action='store_true', help='Inject real address NACK and check attitude invalidation/recovery')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600 or not args.name.replace('-', '').isalnum():
        parser.error('seconds must be 1..600 and name must be alphanumeric/hyphen')
    directory = ROOT / 'build/Debug/attitude-validation' / args.name
    directory.mkdir(parents=True, exist_ok=True)
    if args.fault_check:
        fault_check(directory)
    else:
        record(directory, args.seconds)


if __name__ == '__main__':
    main()
