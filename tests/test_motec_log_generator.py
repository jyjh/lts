import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "external" / "MotecLogGenerator"))

from data_log import DataLog  # noqa: E402


def test_resample_preserves_endpoint_count_and_requested_frequency():
    rows = [
        ["Time", "Throttle (%)"],
        ["0", "0"],
        ["0.001", "10"],
        ["0.002", "20"],
    ]

    data_log = DataLog()
    data_log.from_csv_log([",".join(row) + "\n" for row in rows])
    data_log.resample(1000)

    channel = data_log.channels["Throttle"]
    assert channel.sample_count() == 3
    assert channel.timestamps.tolist() == [0, 0.001, 0.002]
    assert channel.sample_values(float).tolist() == [0, 10, 20]
    assert channel.avg_frequency() == 1000


def test_csv_headers_keep_units_without_affecting_channel_name():
    rows = [
        ["Time", "Replay Throttle Input (%)"],
        ["0", "12.5"],
    ]

    data_log = DataLog()
    data_log.from_csv_log([",".join(row) + "\n" for row in rows])

    assert "Replay Throttle Input" in data_log.channels
    assert data_log.channels["Replay Throttle Input"].units == "%"
