import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from chassis_serial import Parser, encode, decode

class ProtocolTests(unittest.TestCase):
    def test_golden(self):
        golden = bytes.fromhex('5aa5011200007856341201efcdab0807060504030201948e')
        self.assertEqual(encode(0x12, seq=0x12345678, session=0xabcdef01,
                                timestamp_us=0x0102030405060708), golden)
        p = Parser()
        frames = []
        for b in golden:
            frames += p.feed(bytes([b]))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]['timestamp_us'], 0x0102030405060708)

    def test_all_lengths_and_noise(self):
        p = Parser()
        for length in range(257):
            payload = bytes(i % 256 for i in range(length))
            packet = encode(0x55, payload)
            frames = p.feed(b'noise' + packet + packet)
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[0]['payload'], payload)
        with self.assertRaises(ValueError):
            encode(1, bytes(257))

    def test_delayed_read_of_complete_frame(self):
        p = Parser()
        packet = encode(0x12)
        self.assertEqual(p.feed(packet[:10]), [])
        p.started -= 1
        frames = p.feed(packet[10:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(p.errors, 0)

    def test_errors(self):
        p = Parser()
        packet = bytearray(encode(0x12))
        packet[-1] ^= 1
        frames = p.feed(b'\x5a\xa5\x01\x00\xff\xff' + b'\x5a\xa5\x02\x00\x00\x00' + packet + encode(0x12))
        self.assertEqual(len(frames), 1)
        self.assertEqual(p.errors, 3)
        p.feed(encode(1)[:8])
        p.started -= 0.2
        self.assertEqual(len(p.feed(encode(0x12))), 1)
        with self.assertRaises(ValueError):
            decode(dict(id=0x21, payload=b'bad'))

if __name__ == '__main__':
    unittest.main()
