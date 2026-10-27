#!/usr/bin/env python3
"""C30D bench telemetry client; Python 3 + pyserial 3.5. No motor enable API."""
import argparse
import binascii
import collections
import json
import math
import secrets
import struct
import time
from pathlib import Path

HEADER = struct.Struct('<2sBBHIIQ')
STATUS = ('motion_enabled', 'wheel_valid', 'imu_health', 'imu_error', 'imu_sequence',
          'imu_timestamp_ms', 'calibration_state', 'heading_epoch', 'crc_errors',
          'length_errors', 'version_errors', 'parse_timeouts', 'rx_overflows',
          'uart_errors', 'tx_drops', 'response_drops', 'rejected', 'accepted',
          'imu_recoveries', 'tx_errors')

def encode(msg_id, payload=b'', seq=0, session=1, timestamp_us=0):
    if len(payload) > 256:
        raise ValueError('payload exceeds 256 bytes')
    data = HEADER.pack(b'\x5a\xa5', 1, msg_id, len(payload), seq, session, timestamp_us) + payload
    return data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff))

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0
        self.started = None

    def feed(self, data):
        # Call with bounded reads, at least every 20ms when idle.
        self.buffer.extend(data)
        frames = []
        now = time.monotonic()
        while self.buffer:
            if self.buffer[:1] != b'\x5a' or (len(self.buffer) > 1 and self.buffer[:2] != b'\x5a\xa5'):
                del self.buffer[0]
                self.started = None
                continue
            if self.started is None:
                self.started = now
            if len(self.buffer) < 6:
                if now - self.started >= 0.1:
                    del self.buffer[0]
                    self.errors += 1
                    self.started = None
                    continue
                break
            length = int.from_bytes(self.buffer[4:6], 'little')
            if self.buffer[2] != 1 or length > 256:
                del self.buffer[0]
                self.errors += 1
                self.started = None
                continue
            total = length + 24
            if len(self.buffer) < total:
                if now - self.started >= 0.1:
                    del self.buffer[0]
                    self.errors += 1
                    self.started = None
                    continue
                break
            # Validate a complete candidate before applying an idle timeout.
            # OS scheduling delays do not prove a gap on the UART wire.
            packet = self.buffer[:total]
            if binascii.crc_hqx(packet[2:-2], 0xffff) != int.from_bytes(packet[-2:], 'little'):
                del self.buffer[0]
                self.errors += 1
                self.started = None
                continue
            _, _, msg_id, _, seq, session, timestamp = HEADER.unpack_from(packet)
            frames.append(dict(id=msg_id, seq=seq, session=session, timestamp_us=timestamp,
                               payload=bytes(packet[22:-2])))
            del self.buffer[:total]
            self.started = None
        return frames

def decode(frame):
    p = frame['payload']
    msg_id = frame['id']
    if msg_id == 1 and len(p) == 36:
        fields = struct.unpack('<IQIIIHHHHBBBB', p)
        keys = ('request_seq', 'boot_id', 'firmware', 'caps', 'clock_resolution_us',
                'imu_hz', 'attitude_hz', 'wheel_hz', 'status_hz', 'imu_id',
                'motor_present_mask', 'axes', 'sample_time_kind')
        return dict(zip(keys, fields))
    if msg_id == 2 and len(p) == 28:
        return dict(zip(('request_seq', 't1', 't2', 't3'), struct.unpack('<IQQQ', p)))
    if msg_id == 0x7f and len(p) == 6:
        return dict(zip(('request_seq', 'request_id', 'result'), struct.unpack('<IBB', p)))
    if msg_id == 0x20 and len(p) == 52:
        fields = struct.unpack('<4q4fI', p)
        return dict(count=fields[:4], omega=fields[4:8], valid=fields[8])
    if msg_id == 0x21 and len(p) == 28:
        fields = struct.unpack('<I6f', p)
        return dict(valid=fields[0], gyro=fields[1:4], accel=fields[4:7])
    if msg_id == 0x22 and len(p) == 20:
        fields = struct.unpack('<I4f', p)
        return dict(valid=fields[0], q_xyzw=fields[1:])
    if msg_id == 0x23 and len(p) == 80:
        return dict(zip(STATUS, struct.unpack('<20I', p)))
    raise ValueError(f'unexpected frame id/length: {msg_id:#x}/{len(p)}')

class Client:
    def __init__(self, port, session=None):
        import serial
        self.serial = serial.Serial(port, 460800, timeout=0.02, write_timeout=0.5,
                                    exclusive=True)
        self.parser = Parser()
        self.session = session or secrets.randbelow(0xffffffff) + 1
        self.seq = 0
        self.pending = collections.deque(maxlen=1024)
        self.gaps = 0
        self.last_sequence = None

    def close(self):
        self.serial.close()

    def send(self, msg_id, payload=b'', **overrides):
        seq = overrides.pop('seq', self.seq)
        self.seq = (seq + 1) & 0xffffffff
        packet = encode(msg_id, payload, seq, overrides.pop('session', self.session),
                        time.monotonic_ns() // 1000)
        if overrides:
            raise ValueError(overrides)
        self.serial.write(packet)
        return seq

    def receive(self):
        if not self.pending:
            for f in self.parser.feed(self.serial.read(2048)):
                if f['session'] != self.session:
                    continue
                if self.last_sequence is not None:
                    self.gaps += (f['seq'] - self.last_sequence - 1) & 0xffffffff
                self.last_sequence = f['seq']
                f['data'] = decode(f)
                f['host_us'] = time.monotonic_ns() // 1000
                self.pending.append(f)
        return self.pending.popleft() if self.pending else None

    def wait_acknowledgements(self, sequences, result=0):
        remaining = set(sequences)
        end = time.monotonic() + 2
        while remaining and time.monotonic() < end:
            f = self.receive()
            if f and f['id'] == 0x7f and f['data']['request_seq'] in remaining:
                assert f['data']['result'] == result, f
                remaining.remove(f['data']['request_seq'])
        if remaining:
            raise TimeoutError(f'missing ACKs: {sorted(remaining)}')

    def request(self, msg_id, payload=b'', expected=None, **overrides):
        seq = self.send(msg_id, payload, **overrides)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            f = self.receive()
            if f and f['id'] == (expected if expected is not None else msg_id) and f['data'].get('request_seq') == seq:
                return f
        raise TimeoutError(f'no response for {msg_id:#x}, seq={seq}')

def exercise(client):
    hello = client.request(1)['data']
    assert hello['motor_present_mask'] == 0 and hello['imu_id'] == 0xea, hello
    # Frame noise, overlength header, wrong version, corrupted CRC, truncated frame.
    bad = bytearray(encode(0x12, seq=client.seq, session=client.session))
    bad[-1] ^= 1
    client.serial.write(b'noise' + b'\x5a\xa5\x01\x55\xff\xff' +
                        b'\x5a\xa5\x02\x55\x00\x00' + bad)
    client.serial.write(encode(0x12, seq=client.seq, session=client.session)[:8])
    time.sleep(0.04)
    assert client.request(0x12, expected=0x7f)['data']['result'] == 0
    for msg in (0x10, 0x11, 0x13, 0x31):
        assert client.request(msg, expected=0x7f)['data']['result'] == 4
    assert client.request(0x55, expected=0x7f)['data']['result'] == 1
    assert client.request(2, b'bad', expected=0x7f)['data']['result'] == 2
    old_seq = client.seq - 1
    assert client.request(0x12, expected=0x7f, seq=old_seq)['data']['result'] == 3
    # Wrong session must be silent and must not change the active session.
    client.send(0x12, session=(client.session ^ 0xffffffff) or 1)
    assert client.request(0x12, expected=0x7f)['data']['result'] == 0
    # Deliberate fragmentation and concatenation through actual UART.
    seq = client.seq
    packet = encode(0x12, seq=seq, session=client.session)
    for chunk in (packet[:1], packet[1:7], packet[7:]):
        client.serial.write(chunk)
        time.sleep(0.002)
    client.seq += 1
    client.serial.write(encode(0x12, seq=client.seq, session=client.session) +
                        encode(0x12, seq=client.seq + 1, session=client.session))
    client.seq += 2
    client.wait_acknowledgements((seq, seq + 1, seq + 2))
    # Maximum-length valid frame includes embedded SOF bytes.
    assert client.request(0x55, b'\x5a\xa5' * 128, expected=0x7f)['data']['result'] == 1
    assert client.request(0x12, expected=0x7f)['data']['result'] == 0
    # New session, including sequence wrap, must retain motor inhibition.
    client.session = (client.session % 0xffffffff) + 1
    client.pending.clear()
    client.last_sequence = None
    client.seq = 0xfffffffe
    assert client.request(1)['data']['boot_id'] == hello['boot_id']
    assert client.request(0x12, expected=0x7f)['data']['result'] == 0
    assert client.request(0x12, expected=0x7f)['data']['result'] == 0
    return hello

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, help='explicit USART3 USB serial path; never auto-select DAPLink')
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--test', action='store_true', help='inject malformed frames and exercise bench commands')
    ap.add_argument('--output', type=Path, default=Path('build/Debug/serial-validation'))
    args = ap.parse_args()
    if args.seconds < 2:
        ap.error('--seconds must be at least 2')
    args.output.mkdir(parents=True, exist_ok=True)
    client = Client(args.port)
    report = dict(port=args.port, baud=460800, duration_s=args.seconds, passed=False)
    counts = collections.Counter()
    samples = collections.Counter()
    previous = {}
    latest = {}
    latencies = []
    try:
        report['hello'] = exercise(client) if args.test else client.request(1)['data']
        # Warmup/calibration before measurement, while draining incoming data.
        limit = time.monotonic() + 15
        while time.monotonic() < limit:
            f = client.receive()
            if f and f['id'] == 0x22 and f['data']['valid']:
                break
        else:
            raise AssertionError('attitude not ready within 15s; keep board still')
        baseline_gaps = client.gaps
        baseline_errors = client.parser.errors
        report['startup_resync_errors'] = baseline_errors
        start = time.monotonic()
        sync_due = start
        with (args.output / 'frames.jsonl').open('w') as log:
            while time.monotonic() - start < args.seconds:
                if time.monotonic() >= sync_due:
                    client.send(2, struct.pack('<Q', time.monotonic_ns() // 1000))
                    sync_due += 1
                f = client.receive()
                if not f:
                    continue
                log.write(json.dumps({k: v for k, v in f.items() if k != 'payload'}) + '\n')
                msg = f['id']
                counts[msg] += 1
                d = f['data']
                latest[msg] = d
                if msg in (0x20, 0x21, 0x22):
                    assert f['timestamp_us'] >= previous.get(msg, 0), f
                    if f['timestamp_us'] > previous.get(msg, 0):
                        samples[msg] += 1
                    previous[msg] = f['timestamp_us']
                if msg == 0x20:
                    assert d['valid'] == 0 and all(v == 0 for v in d['count']), d
                if msg == 0x21:
                    assert d['valid'] == 1 and all(math.isfinite(x) for x in d['gyro'] + d['accel']), d
                if msg == 0x22:
                    assert d['valid'] == 1 and abs(sum(x*x for x in d['q_xyzw']) - 1) < 0.01, d
                if msg == 0x23:
                    assert d['motion_enabled'] == 0 and d['wheel_valid'] == 0, d
                    assert all(d[k] == 0 for k in ('rx_overflows', 'uart_errors', 'tx_drops', 'response_drops', 'tx_errors')), d
                if msg == 2:
                    rtt = f['host_us'] - d['t1'] - (d['t3'] - d['t2'])
                    latencies.append(rtt)
            elapsed = time.monotonic() - start
        rates = {hex(msg): counts[msg] / elapsed for msg in (0x20, 0x21, 0x22, 0x23)}
        report.update(rates_hz=rates, counts=dict(counts), unique_samples=dict(samples),
                      host_parse_errors=client.parser.errors - baseline_errors,
                      sequence_gaps=client.gaps - baseline_gaps)
        for msg, nominal in ((0x20, 100), (0x21, 100), (0x22, 50), (0x23, 10)):
            assert nominal * 0.9 <= rates[hex(msg)] <= nominal * 1.1, rates
        assert samples[0x21] >= args.seconds * 90, samples
        assert client.gaps == baseline_gaps and client.parser.errors == baseline_errors, (client.gaps - baseline_gaps, client.parser.errors - baseline_errors)
        assert latencies, 'no time-sync responses'
        if args.test:
            status = latest[0x23]
            assert all(status[k] > 0 for k in ('crc_errors', 'length_errors', 'version_errors', 'parse_timeouts', 'rejected')), status
        measurement_errors = client.parser.errors - baseline_errors
        measurement_gaps = client.gaps - baseline_gaps
        boot = report['hello']['boot_id']
        client.close()
        client = Client(args.port)
        assert client.request(1)['data']['boot_id'] == boot
        report.update(passed=True, rates_hz=rates, unique_samples=dict(samples),
                      counts=dict(counts), status=latest.get(0x23), imu=latest.get(0x21),
                      attitude=latest.get(0x22), rtt_us_min=min(latencies),
                      rtt_us_max=max(latencies), host_parse_errors=measurement_errors, sequence_gaps=measurement_gaps,
                      close_reopen_passed=True)
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        client.close()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
