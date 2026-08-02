import unittest

from tools.equinox_sim.vehicle import EquinoxVehicle, pedal_accel_multiplier


class TestEquinoxVehicle(unittest.TestCase):
  def test_recovery_pedal_accelerates_vehicle(self):
    vehicle = EquinoxVehicle(initial_speed_kph=80.0)
    for _ in range(300):
      vehicle.step(0.01, 0.060)

    self.assertGreater(vehicle.speed_mps * 3.6, 82.0)
    self.assertGreater(vehicle.accel_mps2, 0.25)

  def test_released_pedal_coasts(self):
    vehicle = EquinoxVehicle(initial_speed_kph=80.0)
    for _ in range(300):
      vehicle.step(0.01, 0.0)

    self.assertLess(vehicle.speed_mps * 3.6, 80.0)

  def test_brake_overrides_pedal(self):
    vehicle = EquinoxVehicle(initial_speed_kph=80.0)
    for _ in range(100):
      vehicle.step(0.01, 0.20, brake_pressed=True)

    self.assertLess(vehicle.accel_mps2, -2.0)
    self.assertLess(vehicle.speed_mps * 3.6, 75.0)

  def test_multiplier_matches_gm_controller_map(self):
    self.assertAlmostEqual(pedal_accel_multiplier(0.0), 0.186)
    self.assertAlmostEqual(pedal_accel_multiplier(60.0 / 3.6), 0.170)
    self.assertAlmostEqual(pedal_accel_multiplier(100.0 / 3.6), 0.184)


if __name__ == '__main__':
  unittest.main()

