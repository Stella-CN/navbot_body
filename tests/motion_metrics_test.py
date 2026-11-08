"""Count-derived metrics use actual sample times rather than nominal rate."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion_metrics import analyze

class MetricsTest(unittest.TestCase):
    def record(self):
        frames = [{'id': 127, 'time': t, 'data': {'request_id': 16, 'result': 0}}
                  for t in (0, 1000000)]
        for t in (700000, 710000, 730000, 740000):
            frames.append({'id': 32, 'time': t, 'data': {
                'count': [t//100, -t//100, t//100, -t//100],
                'omega': [3, -3, 3, -3], 'valid': 15}})
        return {'frames': frames, 'passed': True}

    def test_irregular_intervals_and_direction(self):
        result = analyze(self.record())
        for i, wheel in enumerate(result['wheels']):
            self.assertAlmostEqual(wheel['count_interval_mean_rad_s'],
                                   math.pi * (1 if i % 2 == 0 else -1))
            self.assertAlmostEqual(wheel['count_interval_std_rad_s'], 0)

    def test_duplicate_timestamps_rejected(self):
        record = self.record()
        record['frames'][-1]['time'] = record['frames'][-2]['time']
        with self.assertRaisesRegex(ValueError, 'timestamps'):
            analyze(record)

if __name__ == '__main__':
    unittest.main()
