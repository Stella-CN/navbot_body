#!/usr/bin/env python3
"""Bench-test already-flashed Debug firmware through OpenOCD on port 3333.

Halts/resumes the target and injects diagnostic faults. Never builds or flashes.
Keep motors disconnected. See README for the limits of simulated fault tests.
"""
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'build/Debug/imu-validation'
FIELDS = (
    'sequence', 'timestamp_ms', 'valid', 'health', 'nack_count', 'timeout_count',
    'stale_count', 'recovery_count', 'deadline_misses', 'period_min_us',
    'period_max_us', 'execution_max_us',
)
# CCER, all eight CCRs, CFSR/HFSR, and TIM1 main output enable must be zero.
REGISTERS = (
    0x40010020, 0x40014020, 0x40014420, 0x40014820,
    0x40010034, 0x40010038, 0x4001003C, 0x40010040,
    0x40014034, 0x40014038, 0x40014434, 0x40014834,
    0xE000ED28, 0xE000ED2C,
)


def printf_command(marker, expressions):
    format_string = marker + ' ' + ' '.join('%u' for _ in expressions) + r'\n'
    return 'printf "' + format_string + '", ' + ', '.join(expressions)


def snapshot(label, results, fault=None):
    commands = ['target remote localhost:3333']
    if fault is not None:
        commands.append(f'set variable c30d_imu_test_fault={fault}')
    safe = [f'*(unsigned int*)0x{address:x}' for address in REGISTERS]
    safe.append('(*(unsigned int*)0x40010044)&0x8000')
    commands.extend([
        printf_command('SAFE', safe),
        printf_command('SNAP', ['navbot_imu_snapshot.' + field for field in FIELDS]),
        'p navbot_imu_snapshot', 'p navbot_oled_frames', 'p navbot_oled_max_us',
        'p navbot_stack_free_words', 'monitor resume', 'disconnect',
    ])
    args = ['arm-none-eabi-gdb', '-q', '-batch', 'build/Debug/navbot_body.elf']
    for command in commands:
        args.extend(['-ex', command])
    process = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, timeout=20)
    (OUTPUT / f'{label}.log').write_text(process.stdout + process.stderr)
    process.check_returncode()
    lines = process.stdout.splitlines()
    safe_line = next(line for line in lines if line.startswith('SAFE '))
    assert all(int(value) == 0 for value in safe_line[5:].split()), safe_line
    sample_line = next(line for line in lines if line.startswith('SNAP '))
    result = dict(zip(FIELDS, map(int, sample_line[5:].split())))
    result['label'] = label
    results.append(result)
    print(json.dumps(result), flush=True)
    return result


def validate(results):
    baseline = snapshot('baseline', results, fault=0)
    time.sleep(1)
    updating = snapshot('updating', results)
    assert updating['sequence'] > baseline['sequence'] + 80 and updating['valid'] == 1
    for flag, name, counter, health in (
        (1, 'nack', 'nack_count', 2),
        (2, 'timeout', 'timeout_count', 2),
        (4, 'stale', 'stale_count', 5),
    ):
        before = snapshot(name + '-inject', results, fault=flag)
        time.sleep(0.15)
        failed = snapshot(name + '-fault', results)
        assert failed['valid'] == 0 and failed['health'] == health, failed
        assert failed[counter] > before[counter], failed
        snapshot(name + '-clear', results, fault=0)
        time.sleep(1.5)
        recovered = snapshot(name + '-recovered', results)
        assert recovered['valid'] == 1, recovered
        assert recovered['recovery_count'] > before['recovery_count'], recovered
    start = snapshot('soak-start', results)
    time.sleep(30)
    end = snapshot('soak-end', results)
    assert end['valid'] == 1 and end['sequence'] > start['sequence'] + 2900, end
    assert end['deadline_misses'] == 0, end


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    results = []
    try:
        validate(results)
        print('PASS: updates, NACK/timeout/stale invalidation, recovery, 30-second soak', flush=True)
    finally:
        # Clearing injections is also attempted on assertion/command failures.
        try:
            snapshot('cleanup', results, fault=0)
        finally:
            (OUTPUT / 'results.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
