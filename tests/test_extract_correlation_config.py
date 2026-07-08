import json
import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import extract_correlation_config  # noqa: E402


class ExtractCorrelationConfigTest(unittest.TestCase):
    def test_pack_power_advance_estimates_lagged_pack_power(self):
        time = np.arange(0.0, 8.0, 0.01)
        delay = 0.06
        demand = (
            80.0 * np.exp(-0.5 * ((time - 2.0) / 0.25) ** 2)
            + 55.0 * np.exp(-0.5 * ((time - 5.2) / 0.35) ** 2)
        )
        lagged_demand = np.interp(time - delay, time, demand, left=0.0, right=0.0)
        voltage = np.full(time.shape, 300.0)
        pack_power_kw = -0.25 * lagged_demand
        data = {
            "time_s": time,
            "motor_torque_command_nm": -demand,
            "pack_voltage_v": voltage,
            "pack_current_a": pack_power_kw * 1000.0 / voltage,
        }

        estimate = extract_correlation_config.estimate_pack_power_advance(
            data, max_advance_s=0.2, step_s=0.005
        )

        self.assertEqual(estimate["status"], "ok")
        self.assertAlmostEqual(estimate["advanceS"], delay, delta=0.006)

    def test_gps_advance_estimates_lagged_gps_course(self):
        time = np.arange(0.0, 10.0, 0.01)
        delay = 0.12
        yaw_rate = 0.35 * np.sin(2.0 * np.pi * 0.35 * time) + 0.18 * np.sin(
            2.0 * np.pi * 0.75 * time
        )
        heading = np.cumsum(yaw_rate) * np.median(np.diff(time))
        lagged_heading = np.interp(time - delay, time, heading, left=heading[0], right=heading[-1])
        gps_course = np.pi / 2.0 - lagged_heading
        data = {
            "time_s": time,
            "yaw_rate_radps": yaw_rate,
            "gps_course_rad": gps_course,
        }

        estimate = extract_correlation_config.estimate_gps_advance(
            data, max_advance_s=0.3, step_s=0.01, smoothing_s=0.05
        )

        self.assertEqual(estimate["status"], "ok")
        self.assertAlmostEqual(estimate["advanceS"], delay, delta=0.011)

    def test_writes_correlation_config_json(self):
        time = np.arange(0.0, 6.0, 0.01)
        delay = 0.05
        demand = 70.0 * np.exp(-0.5 * ((time - 2.0) / 0.25) ** 2)
        lagged_demand = np.interp(time - delay, time, demand, left=0.0, right=0.0)
        yaw_rate = 0.3 * np.sin(2.0 * np.pi * 0.4 * time)
        heading = np.cumsum(yaw_rate) * np.median(np.diff(time))
        lagged_heading = np.interp(time - delay, time, heading, left=heading[0], right=heading[-1])
        voltage = np.full(time.shape, 300.0)

        data = {
            "time_s": time,
            "motor_torque_command_nm": -demand,
            "pack_voltage_v": voltage,
            "pack_current_a": -0.2 * lagged_demand * 1000.0 / voltage,
            "yaw_rate_radps": yaw_rate,
            "gps_course_rad": np.pi / 2.0 - lagged_heading,
        }
        pack = extract_correlation_config.estimate_pack_power_advance(
            data, max_advance_s=0.2, step_s=0.005
        )
        gps = extract_correlation_config.estimate_gps_advance(
            data, max_advance_s=0.2, step_s=0.01, smoothing_s=0.05
        )
        config = extract_correlation_config.build_config(Path("lap_replay.csv"), pack, gps)
        loaded = json.loads(json.dumps(config))

        self.assertEqual(loaded["schema"], "lts.correlation.config.v1")
        self.assertAlmostEqual(loaded["offsets"]["PackPowerAdvanceS"], delay, delta=0.006)
        self.assertAlmostEqual(loaded["offsets"]["GpsAdvanceS"], delay, delta=0.011)
        self.assertEqual(
            loaded["runCorrelationOptions"]["PackPowerAdvanceS"],
            loaded["offsets"]["PackPowerAdvanceS"],
        )
        self.assertEqual(
            loaded["plotCorrelationPositionOverlayOptions"]["RawTimeOffsetS"],
            loaded["offsets"]["GpsAdvanceS"],
        )


if __name__ == "__main__":
    unittest.main()
