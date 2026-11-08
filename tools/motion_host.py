#!/usr/bin/env python3
"""ROS-independent callback bridge and bounded on-stand motion test.

Wheel order LF(C), LR(D), RF(B), RR(A). Unconfigured wheel signs are electrical.
A future ROS subscriber calls VelocityBridge.on_twist; it must retain upstream age.
No automatic re-arm/retry of a stale command. Geometry is nominal, not calibrated.
"""
import argparse
import json
import math
import struct
import time
from pathlib import Path
from chassis_serial import Client


def body_velocity(omega, radius, track):
    """Logical wheel rad/s -> body m/s and rad/s; nominal geometry only."""
    if len(omega) != 4 or not all(math.isfinite(v) for v in (*omega, radius, track)) or radius <= 0 or track <= 0:
        raise ValueError('invalid wheel feedback or geometry')
    left = radius * (omega[0] + omega[1]) / 2
    right = radius * (omega[2] + omega[3]) / 2
    return {'vx': (left + right) / 2, 'wz': (right - left) / track}


class VelocityBridge:
    def __init__(self, client, clock=time.monotonic):
        self.client, self.clock = client, clock
        self.message = None
        self.stopped = False

    def on_twist(self, vx, wz, received_at=None):
        if not all(math.isfinite(x) for x in (vx, wz)) or abs(vx) > .3 or abs(wz) > 1.5:
            raise ValueError('invalid body velocity')
        self.message = (vx, wz, self.clock() if received_at is None else received_at)

    def poll(self):
        if self.stopped:
            return
        if self.message is None or not 0 <= self.clock() - self.message[2] < .15:
            self.client.send(0x12)
            self.stopped = True
            return
        # Remaining upstream lifetime, not a fresh lease for an old ROS message.
        remaining = min(200, int((.2 - (self.clock() - self.message[2])) * 1000))
        self.client.send(0x10, struct.pack('<ffH', *self.message[:2], remaining))


def run(args):
    c = Client(args.port)
    frames = []
    report = {'mode': args.mode, 'passed': False}

    def receive():
        f = c.receive()
        if f:
            if f['id'] == 0x20 and args.mode == 'body' and f['data']['valid'] == 15:
                f['data']['body_velocity'] = body_velocity(f['data']['omega'], args.radius, args.track)
            frames.append({'id': f['id'], 'time': f['timestamp_us'], 'data': f['data']})
            if f['id'] == 0x7f and f['data']['result']:
                raise RuntimeError(f['data'])
        return f

    def drain(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            receive()

    def management(msg_id, payload=b''):
        seq = c.send(msg_id, payload)
        ack = done = False
        until = time.monotonic() + 1
        while time.monotonic() < until:
            f = receive()
            if not f:
                continue
            if f['id'] == 0x7f and f['data']['request_seq'] == seq:
                ack = True
            if f['id'] == 0x24 and f['data']['request_seq'] == seq:
                if f['data']['result']:
                    raise RuntimeError(f['data'])
                done = True
            if ack and done:
                return
        raise TimeoutError(f'management {msg_id:#x} not applied')

    try:
        report['hello'] = c.request(1)['data']
        if report['hello']['firmware'] >> 8 != 0x101:
            raise RuntimeError('motion firmware 1.1 required')
        management(0x12)
        if args.mode == 'body':
            if args.signs is None:
                raise ValueError('body test requires observed forward signs LF LR RF RR')
            management(0x31, struct.pack('<ff4b', args.radius, args.track, *args.signs))
        drain(.1)
        report['startup_parse_errors'] = c.parser.errors
        base_errors, base_gaps = c.parser.errors, c.gaps
        management(0x11, b'\1')
        start = time.monotonic()
        report['motion_start_host'] = start
        bridge = VelocityBridge(c)
        next_send = start
        while time.monotonic() - start < args.seconds:
            now = time.monotonic()
            if now >= next_send:
                if args.mode == 'body':
                    # Simulated ROS callback supplies a NEW source sample each 20ms.
                    bridge.on_twist(args.vx, args.wz, now)
                    bridge.poll()
                else:
                    values = [0.] * 4
                    values[args.wheel] = args.omega
                    c.send(0x14, struct.pack('<4fH', *values, 200))
                next_send = now + .02
            receive()
        if args.timeout:
            # No STOP/heartbeat renewal: MCU must expire the last velocity command.
            drain(.6)
            controls = [f['data'] for f in frames if f['id'] == 0x24]
            if not controls or controls[-1]['state'] != 2 or any(controls[-1]['duty']):
                raise AssertionError('MCU command timeout did not disable outputs')
            report['timeout_verified'] = True
        management(0x12)
        drain(.8)
        wheels = [f for f in frames if f['id'] == 0x20]
        controls = [f['data'] for f in frames if f['id'] == 0x24]
        report.update(parser_errors=c.parser.errors-base_errors, gaps=c.gaps-base_gaps,
                      peak_omega=[max(abs(f['data']['omega'][i]) for f in wheels) for i in range(4)],
                      final_omega=wheels[-1]['data']['omega'], final_control=controls[-1])
        if report['parser_errors'] or report['gaps']:
            raise AssertionError('serial parse error or sequence gap')
        if controls[-1]['state'] != 0 or any(controls[-1]['duty']):
            raise AssertionError('STOP did not disable outputs')
        if any(abs(v) > .5 for v in report['final_omega']):
            raise AssertionError('wheel not settled')
        active = [args.wheel] if args.mode == 'wheel' else list(range(4))
        if any(report['peak_omega'][i] < 1 for i in active):
            raise AssertionError('no encoder motion')
        if any(abs(v)>200 for f in controls for v in f['duty']):
            raise AssertionError('duty exceeded bench limit')
        if args.mode == 'body':
            active_frames = [f for f in frames if f['id'] == 0x24 and f['data']['state'] == 1
                             and any(abs(v) > 0 for v in f['data']['target'])]
            if not active_frames:
                raise AssertionError('body command never applied')
            first = active_frames[0]['time']
            settled = [f['data']['body_velocity'] for f in wheels
                       if first + 700000 <= f['time'] <= first + int(args.seconds * 1e6) - 100000
                       and 'body_velocity' in f['data']]
            if not settled:
                raise AssertionError('no settled body samples')
            report['requested_body'] = {'vx': args.vx, 'wz': args.wz}
            report['measured_body_mean'] = {k: sum(d[k] for d in settled)/len(settled) for k in ('vx','wz')}
            report['geometry'] = {'radius': args.radius, 'track': args.track, 'signs': args.signs}
            # Loose bench acceptance while tuning; not a calibrated accuracy claim.
            if abs(report['measured_body_mean']['vx']-args.vx) > .04 or abs(report['measured_body_mean']['wz']-args.wz) > .2:
                raise AssertionError('body speed outside initial bench tolerance')
        report['passed'] = True
    finally:
        try:
            c.request(0x12, expected=0x7f)
        finally:
            c.close()
            report['frames'] = frames
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='frames'}, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--mode', choices=['wheel', 'body'], default='wheel')
    ap.add_argument('--wheel', type=int, choices=range(4), default=0)
    ap.add_argument('--omega', type=float, default=6)
    ap.add_argument('--vx', type=float, default=.15)
    ap.add_argument('--wz', type=float, default=0)
    ap.add_argument('--radius', type=float, default=.0425)
    ap.add_argument('--track', type=float, default=.228)
    ap.add_argument('--signs', type=int, nargs=4, choices=[-1,1])
    ap.add_argument('--seconds', type=float, default=1.5)
    ap.add_argument('--timeout', action='store_true')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if not .3 <= args.seconds <= 5 or not math.isfinite(args.omega) or abs(args.omega)>12:
        ap.error('test duration .3..5 s, wheel speed within ±12 rad/s')
    run(args)

if __name__ == '__main__':
    main()
