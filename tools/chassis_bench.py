#!/usr/bin/env python3
"""Explicit hardware test: requires flashed Debug + OpenOCD + disconnected motors.

Injects Debug IMU NACK, floods USART3, resets MCU, and asserts motor registers.
No flash writes. All injected fault flags are cleared in finally.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path
from chassis_serial import Client, encode
from hardware_validate_imu import REGISTERS

ROOT = Path(__file__).resolve().parents[1]

def gdb(commands, output):
    args = ['arm-none-eabi-gdb', '-q', '-batch', 'build/Debug/navbot_body.elf',
            '-ex', 'target remote localhost:3333']
    for command in commands + ['monitor resume', 'disconnect']:
        args += ['-ex', command]
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=15)
    output.write_text(result.stdout + result.stderr)
    result.check_returncode()
    if any(x in result.stderr for x in ('No symbol', 'Cannot access memory', 'not available')):
        raise RuntimeError(result.stderr)
    return result.stdout

def wait_frame(client, predicate, seconds=3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        frame = client.receive()
        if frame and predicate(frame):
            return frame
    raise TimeoutError('expected serial state did not arrive')

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--output', type=Path, default=ROOT / 'build/Debug/serial-fault-validation')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(passed=False)
    client = Client(args.port)
    try:
        boot = client.request(1)['data']['boot_id']
        wait_frame(client, lambda f: f['id'] == 0x22 and f['data']['valid'], 15)
        # Continuous full-rate UART traffic: parser remains bounded; no motion path.
        for _ in range(16):
            client.serial.write(b'\x5a\xa5\x01\x55\xff\xff' * 80)
            while client.serial.in_waiting:
                client.receive()
        report['flood_stop_ack'] = client.request(0x12, expected=0x7f)['data']
        assert report['flood_stop_ack']['result'] == 0
        gdb(['set variable c30d_imu_test_fault=1'], args.output / 'nack-inject.log')
        report['invalid_imu'] = wait_frame(client, lambda f: f['id'] == 0x21 and not f['data']['valid'])['data']
        report['invalid_attitude'] = wait_frame(client, lambda f: f['id'] == 0x22 and not f['data']['valid'])['data']
        gdb(['set variable c30d_imu_test_fault=0'], args.output / 'nack-clear.log')
        report['recovered_imu'] = wait_frame(client, lambda f: f['id'] == 0x21 and f['data']['valid'], 4)['data']
        report['recovered_attitude'] = wait_frame(client, lambda f: f['id'] == 0x22 and f['data']['valid'], 15)['data']
        status = wait_frame(client, lambda f: f['id'] == 0x23)['data']
        assert status['imu_recoveries'] >= 1 and status['heading_epoch'] >= 2, status
        report['recovered_status'] = status
        gdb(['monitor reset halt'], args.output / 'reset.log')
        time.sleep(0.1)
        client.serial.reset_input_buffer()
        client.pending.clear()
        client.parser.buffer.clear()
        client.last_sequence = None
        # A reboot requires HELLO before any status/command replies resume.
        try:
            client.request(0x12, expected=0x7f)
        except TimeoutError:
            report['old_session_after_reset_silent'] = True
        else:
            raise AssertionError('reboot retained session')
        new_boot = client.request(1)['data']['boot_id']
        assert new_boot != boot
        report['boot_before'] = boot
        report['boot_after'] = new_boot
        report['reset_stop_ack'] = client.request(0x12, expected=0x7f)['data']
        wait_frame(client, lambda f: f['id'] == 0x22 and f['data']['valid'], 15)
        addresses = list(REGISTERS)
        expr = [f'*(unsigned int*)0x{a:x}' for a in addresses] + ['(*(unsigned int*)0x40010044)&0x8000']
        commands = ['printf "SAFE ' + ' '.join(['%u'] * len(expr)) + r'\n", ' + ', '.join(expr),
                    'p navbot_stack_free_words', 'p navbot_imu_snapshot',
                    'p huart3.ErrorCode', 'p rx_overflows', 'p uart_errors', 'p tx_errors']
        log = gdb(commands, args.output / 'safety.log')
        safe_line = next(line for line in log.splitlines() if line.startswith('SAFE '))
        assert all(int(v) == 0 for v in safe_line[5:].split()), safe_line
        report['motor_and_fault_registers_zero'] = True
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        try:
            gdb(['set variable c30d_imu_test_fault=0'], args.output / 'cleanup.log')
        finally:
            client.close()
            (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
