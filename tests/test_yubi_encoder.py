"""Standard YUBI firmware interpretation and the canonical four-bar geometry."""

import math

import pytest

from sx_embodiments.yubi import YubiEncoder, YubiEncoderError, encoder_envelope_rad, jaw_gap_m


def test_standard_yubi_wrap_and_geometry() -> None:
    encoder = YubiEncoder()
    assert encoder.parse("L003,0.75000000") == pytest.approx(0.75)
    assert encoder.parse("0.125") == pytest.approx(0.125)
    assert encoder.parse("Magnet not detected") is None
    assert encoder.parse("L003,6.143593") == pytest.approx(2.0 * math.pi - 6.143593)
    assert encoder.parse("L003,-0.004602") == pytest.approx(0.004602)
    assert encoder.parse("L003,0.876") == pytest.approx(0.876)
    assert jaw_gap_m(0.0) == 0.0
    assert jaw_gap_m(0.785398) == pytest.approx(0.100083349, abs=1e-9)
    assert jaw_gap_m(0.876) == jaw_gap_m(0.785398)
    assert encoder_envelope_rad() == pytest.approx((0.0, 0.785398 + math.radians(10)))


@pytest.mark.parametrize("line", ["791534", "-7.0", "nan", "inf"])
def test_encoder_refuses_outside_wire_range(line: str) -> None:
    with pytest.raises(YubiEncoderError, match="AS5601 range"):
        YubiEncoder().parse(line)


def test_encoder_refuses_outside_jaw_envelope() -> None:
    with pytest.raises(YubiEncoderError, match="YUBI limit"):
        YubiEncoder().parse("1.5")


def test_encoder_identity_is_stable_per_connection() -> None:
    encoder = YubiEncoder()
    assert encoder.parse("0.1") == pytest.approx(0.1)
    assert encoder.parse("LEFT-001,0.2") == pytest.approx(0.2)
    assert encoder.parse("LEFT-001,0.3") == pytest.approx(0.3)
    with pytest.raises(YubiEncoderError, match="changed"):
        encoder.parse("RIGHT-001,0.4")
    with pytest.raises(YubiEncoderError, match="empty device ID"):
        YubiEncoder().parse(",0.1")
    assert YubiEncoder().parse("RIGHT-001,0.4") == pytest.approx(0.4)
