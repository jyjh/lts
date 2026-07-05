import sys
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "external" / "MotecLogGenerator" / "ldparser"))

import extract_motec_lap  # noqa: E402
from ldparser import decode_string, read_ldx_beacons, write_ldx_beacons  # noqa: E402


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

    def test_ldparser_writes_ldx_beacons_readable_by_parser(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ldx_file = Path(tmp_dir) / "generated.ldx"

            write_ldx_beacons(str(ldx_file), [2500000, 1000000])

            self.assertEqual(
                read_ldx_beacons(str(ldx_file)),
                [Decimal("1000000"), Decimal("2500000")],
            )

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
        self.assertEqual(signal["normalization"], "peak_combined_pressure")
        self.assertEqual(signal["combine"], "sum")
        self.assertEqual(signal["peak_combined_pressure_bar"], 200.0)
        np.testing.assert_allclose(signal["values"], [0.0, 0.35, 1.0])

    def test_front_and_rear_brake_pressure_are_export_columns(self):
        data = FakeData(
            [
                FakeChannel("Brake Pressure Front", "kPa", 10, [0, 250, 500]),
                FakeChannel("Brake Pressure Rear", "bar", 10, [0, 3, 6]),
            ]
        )
        channel_map = extract_motec_lap.load_channel_map(
            REPO_ROOT / "config" / "motec" / "default_channel_map.json"
        )

        front = extract_motec_lap.extract_raw_signal(
            data,
            "brake_pressure_front_bar",
            channel_map["channels"]["brake_pressure_front_bar"],
        )
        rear = extract_motec_lap.extract_raw_signal(
            data,
            "brake_pressure_rear_bar",
            channel_map["channels"]["brake_pressure_rear_bar"],
        )

        self.assertIn("brake_pressure_front_bar", extract_motec_lap.REPLAY_COLUMNS)
        self.assertIn("brake_pressure_rear_bar", extract_motec_lap.REPLAY_COLUMNS)
        np.testing.assert_allclose(front["values"], [0.0, 2.5, 5.0])
        np.testing.assert_allclose(rear["values"], [0.0, 3.0, 6.0])

    def test_brake_ratio_peak_scales_combined_pressure_trace(self):
        spec = {
            "required": True,
            "clamp": [0.0, 1.0],
            "derive": {
                "method": "brake_pressure",
                "combine": "sum",
                "front": {
                    "names": ["Brake Pressure Front"],
                    "source_unit_scale": {"bar": 1.0},
                },
                "rear": {
                    "names": ["Brake Pressure Rear"],
                    "source_unit_scale": {"bar": 1.0},
                },
            },
        }
        data = FakeData(
            [
                FakeChannel("Brake Pressure Front", "bar", 10, [0, 30, 60]),
                FakeChannel("Brake Pressure Rear", "bar", 10, [0, 10, 40]),
            ]
        )

        signal = extract_motec_lap.extract_raw_signal(data, "brake_ratio", spec)

        np.testing.assert_allclose(signal["values"], [0.0, 0.4, 1.0])

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
                FakeChannel("Body Vx", "km/h", 10, [36.0, 72.0]),
                FakeChannel("Body Vy", "m/s", 10, [0.5, 0.6]),
                FakeChannel("Body Slip", "deg", 10, [1.0, 2.0]),
                FakeChannel("G Sensor.Front.Yaw Rate", "deg/s", 10, [10.0, 20.0]),
                FakeChannel("G Sensor.Front.Acceleration.Late", "G", 10, [0.1, 0.2]),
                FakeChannel("G Sensor.Rear.Acceleration.Later", "m/s/s", 10, [4.903325, 9.80665]),
                FakeChannel("G Sensor G Front Longitudinal", "m/s/s", 10, [-0.980665, 2.941995]),
                FakeChannel("G Sensor G Rear Longitudinal", "m/s/s", 10, [-1.96133, 3.92266]),
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
        vx = extract_motec_lap.extract_raw_signal(
            data,
            "vx_mps",
            channel_map["channels"]["vx_mps"],
        )
        vy = extract_motec_lap.extract_raw_signal(
            data,
            "vy_mps",
            channel_map["channels"]["vy_mps"],
        )
        body_slip = extract_motec_lap.extract_raw_signal(
            data,
            "body_slip_rad",
            channel_map["channels"]["body_slip_rad"],
        )
        lat_accel = extract_motec_lap.extract_raw_signal(
            data,
            "lat_accel_g",
            channel_map["channels"]["lat_accel_g"],
        )
        front_lat_accel = extract_motec_lap.extract_raw_signal(
            data,
            "front_lat_accel_g",
            channel_map["channels"]["front_lat_accel_g"],
        )
        rear_lat_accel = extract_motec_lap.extract_raw_signal(
            data,
            "rear_lat_accel_g",
            channel_map["channels"]["rear_lat_accel_g"],
        )
        long_accel = extract_motec_lap.extract_raw_signal(
            data,
            "long_accel_g",
            channel_map["channels"]["long_accel_g"],
        )
        front_long_accel = extract_motec_lap.extract_raw_signal(
            data,
            "front_long_accel_g",
            channel_map["channels"]["front_long_accel_g"],
        )
        rear_long_accel = extract_motec_lap.extract_raw_signal(
            data,
            "rear_long_accel_g",
            channel_map["channels"]["rear_long_accel_g"],
        )

        self.assertIn("front_lat_accel_g", extract_motec_lap.REPLAY_COLUMNS)
        self.assertIn("rear_lat_accel_g", extract_motec_lap.REPLAY_COLUMNS)
        self.assertIn("front_long_accel_g", extract_motec_lap.REPLAY_COLUMNS)
        self.assertIn("rear_long_accel_g", extract_motec_lap.REPLAY_COLUMNS)
        np.testing.assert_allclose(course["values"], [0.0, np.pi / 2.0])
        np.testing.assert_allclose(yaw_rate["values"], np.deg2rad([10.0, 20.0]))
        np.testing.assert_allclose(vx["values"], [10.0, 20.0])
        np.testing.assert_allclose(vy["values"], [0.5, 0.6])
        np.testing.assert_allclose(body_slip["values"], np.deg2rad([1.0, 2.0]))
        np.testing.assert_allclose(lat_accel["values"], [0.1, 0.2])
        np.testing.assert_allclose(front_lat_accel["values"], [0.1, 0.2])
        np.testing.assert_allclose(rear_lat_accel["values"], [0.5, 1.0])
        np.testing.assert_allclose(front_long_accel["values"], [-0.1, 0.3])
        np.testing.assert_allclose(rear_long_accel["values"], [-0.2, 0.4])
        np.testing.assert_allclose(long_accel["values"], [-0.1, 0.3])


if __name__ == "__main__":
    unittest.main()
