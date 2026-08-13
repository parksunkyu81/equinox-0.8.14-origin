import json
import os
import tempfile
import unittest

from tools.equinox_sim.perception_diagnostics import (
  FrameDeltaTracker,
  PathWobbleTracker,
  RotatingJsonlWriter,
  sample_path_at_distances,
  sample_plan_points,
)


class TestPerceptionDiagnostics(unittest.TestCase):
  def test_path_sampling_interpolates_and_clamps(self):
    result = sample_path_at_distances(
      [0.0, 10.0, 30.0], [0.0, 1.0, 3.0], [5.0, 20.0, 50.0])
    self.assertEqual(result, [0.5, 2.0, 3.0])

  def test_plan_sampling_uses_stable_indices(self):
    self.assertEqual(sample_plan_points(range(17)), [0.0, 4.0, 8.0, 12.0, 16.0])
    self.assertEqual(sample_plan_points([1.0, 2.0]), [1.0, 2.0, 2.0, 2.0, 2.0])

  def test_frame_delta_does_not_count_downsampling_as_drop(self):
    tracker = FrameDeltaTracker()
    self.assertEqual(tracker.update(100), 0)
    self.assertEqual(tracker.update(104), 4)
    self.assertEqual(tracker.update(108), 4)
    self.assertEqual(tracker.update(2), 0)

  def test_wobble_tracker_counts_direction_changes(self):
    tracker = PathWobbleTracker(window_s=2.0)
    for index, value in enumerate([0.0, 0.10, -0.10, 0.10, -0.10]):
      result = tracker.update(index * 0.2, [0.0, 0.0, value, value, value])
    self.assertGreaterEqual(result["directionFlips2s"], 3)
    self.assertAlmostEqual(result["range2s"][2], 0.2)

  def test_rotating_writer_is_bounded_and_contains_no_image_field(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      writer = RotatingJsonlWriter(tmpdir, max_file_bytes=300, max_files=2)
      for index in range(20):
        writer.write({"type": "sample", "index": index, "values": list(range(20))})
      writer.close()

      paths = sorted(os.path.join(tmpdir, name) for name in os.listdir(tmpdir))
      self.assertLessEqual(len(paths), 2)
      for path in paths:
        with open(path, encoding="utf-8") as file:
          records = [json.loads(line) for line in file]
        self.assertEqual(records[0]["type"], "metadata")
        self.assertFalse(records[0]["containsVideo"])
        self.assertNotIn("image", json.dumps(records))


if __name__ == "__main__":
  unittest.main()
