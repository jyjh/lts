import pytest

import extract_motec_lap  # noqa: E402


def test_public_laps_to_ldparser_keeps_lap_optional():
    assert extract_motec_lap.public_laps_to_ldparser(None) is None
    assert extract_motec_lap.public_laps_to_ldparser("") is None


def test_public_laps_to_ldparser_converts_to_zero_based_parser_range():
    assert extract_motec_lap.public_laps_to_ldparser("1") == "0"
    assert extract_motec_lap.public_laps_to_ldparser("4-5") == "3-4"
    assert extract_motec_lap.public_laps_to_ldparser("4:5") == "3-4"


@pytest.mark.parametrize("laps", ["0", "3-2", "-1", ""])
def test_public_laps_to_ldparser_rejects_invalid_public_ranges(laps):
    if laps == "":
        assert extract_motec_lap.public_laps_to_ldparser(laps) is None
        return

    with pytest.raises(ValueError):
        extract_motec_lap.public_laps_to_ldparser(laps)
