import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "external" / "MotecLogGenerator" / "ldparser"))

import extract_motec_lap  # noqa: E402
from ldparser import decode_string  # noqa: E402


class FakeChannel:
    def __init__(self, name, unit, freq, values):
        self.name = name
        self.unit = unit
        self.freq = freq
        self.data = np.asarray(values, dtype=float)
        self.data_len = len(self.data)


class FakeData:
    def __init__(self, channels):
        self.channs = channels


class ExtractMotecLapTest(unittest.TestCase):
    def test_ldparser_decode_string_ignores_binary_after_null(self):
        raw = (
            b"Mo\x00\x00\x01\x00\x00\x0e\x00\x05\x96\x9c"
            b"Mo\x00\x00\x01\x00\x00\t\x00\x00\x0f\xdb"
        )

        self.assertEqual(decode_string(raw), "Mo")

    def test_ldparser_decode_string_accepts_cp1252_names(self):
        self.assertEqual(decode_string(b"Brake \x96 Front\x00\x00"), "Brake \u2013 Front")

    def test_brake_ratio_derives_from_front_and_rear_pressure(self):
        data = FakeData(
            [
                FakeChannel("Brake Pressure Front", "bar", 10, [0, 50, 120]),
                FakeChannel("Brake Pressure Rear", "bar", 10, [0, 20, 80]),
            ]
        )
        channel_map = extract_motec_lap.load_channel_map(
            REPO_ROOT / "config" / "motec" / "default_channel_map.json"
        )

        signal = extract_motec_lap.extract_raw_signal(
            data,
            "brake_ratio",
            channel_map["channels"]["brake_ratio"],
        )

        self.assertEqual(signal["source"], "derived")
        np.testing.assert_allclose(signal["values"], [0.0, 0.5, 1.0])

    def test_source_specific_steering_scale_applies_only_to_matching_source(self):
        spec = {
            "required": True,
            "sources": [
                {
                    "label": "sim_road_wheel",
                    "names": ["Steer Raw"],
                    "source_unit_scale": {"rad": 1.0},
                },
                {
                    "label": "steering_sensor",
                    "names": ["Steering.Angle"],
                    "scale": 0.25,
                    "source_unit_scale": {"deg": np.pi / 180.0},
                },
            ],
        }

        sim_data = FakeData([FakeChannel("Steer Raw", "rad", 10, [0.2, 0.4])])
        real_data = FakeData([FakeChannel("Steering.Angle", "deg", 10, [20.0, 40.0])])

        sim_signal = extract_motec_lap.extract_raw_signal(sim_data, "steer_rad", spec)
        real_signal = extract_motec_lap.extract_raw_signal(real_data, "steer_rad", spec)

        self.assertEqual(sim_signal["source_label"], "sim_road_wheel")
        np.testing.assert_allclose(sim_signal["values"], [0.2, 0.4])
        self.assertEqual(real_signal["source_label"], "steering_sensor")
        np.testing.assert_allclose(real_signal["values"], np.deg2rad([5.0, 10.0]))

    def test_optional_gps_course_and_accel_channels_are_extracted(self):
        data = FakeData(
            [
                FakeChannel("GPS.Sensor.True Course", "deg", 10, [0.0, 90.0]),
                FakeChannel("GPS.Sensor.Latitude", "deg", 10, [1.0, 1.1]),
                FakeChannel("GPS.Sensor.Longitude", "deg", 10, [2.0, 2.1]),
                FakeChannel("G Sensor.Front.Yaw Rate", "deg/s", 10, [10.0, 20.0]),
                FakeChannel("G Sensor.Front.Acceleration.Late", "G", 10, [0.1, 0.2]),
                FakeChannel("G Sensor.Front.Acceleration.Long G", "G", 10, [-0.1, 0.3]),
            ]
        )
        channel_map = extract_motec_lap.load_channel_map(
            REPO_ROOT / "config" / "motec" / "default_channel_map.json"
        )

        course = extract_motec_lap.extract_raw_signal(
            data,
            "gps_course_rad",
            channel_map["channels"]["gps_course_rad"],
        )
        yaw_rate = extract_motec_lap.extract_raw_signal(
            data,
            "yaw_rate_radps",
            channel_map["channels"]["yaw_rate_radps"],
        )
        lat_accel = extract_motec_lap.extract_raw_signal(
            data,
            "lat_accel_g",
            channel_map["channels"]["lat_accel_g"],
        )
        long_accel = extract_motec_lap.extract_raw_signal(
            data,
            "long_accel_g",
            channel_map["channels"]["long_accel_g"],
        )

        np.testing.assert_allclose(course["values"], [0.0, np.pi / 2.0])
        np.testing.assert_allclose(yaw_rate["values"], np.deg2rad([10.0, 20.0]))
        np.testing.assert_allclose(lat_accel["values"], [0.1, 0.2])
        np.testing.assert_allclose(long_accel["values"], [-0.1, 0.3])


if __name__ == "__main__":
    unittest.main()
