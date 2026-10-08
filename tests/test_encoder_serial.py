"""UART framing does not turn a timeout fragment into an encoder sample."""

import pytest

from sx_embodiments.encoder_serial import EncoderLineDecoder, EncoderReadError


def test_synchronization_and_timeout_fragments_require_newline() -> None:
    decoder = EncoderLineDecoder()
    assert decoder.feed(b"startup par") is None
    assert decoder.feed(b"") is None
    assert decoder.feed(b"tial\n") is None
    assert decoder.feed(b"LEFT-001,0.") is None
    assert decoder.feed(b"") is None
    assert decoder.feed(b"75\n") == "LEFT-001,0.75"
    assert decoder.feed(b"LEFT-001,0.5\r\n") == "LEFT-001,0.5"


@pytest.mark.parametrize("bad", [b"\xff\n", b"x" * 4097, b"a\nb\n"])
def test_unusable_uart_data_is_refused(bad: bytes) -> None:
    decoder = EncoderLineDecoder()
    decoder.feed(b"startup\n")
    with pytest.raises(EncoderReadError):
        decoder.feed(bad)


def test_fragment_limit_applies_across_timeouts() -> None:
    decoder = EncoderLineDecoder()
    decoder.feed(b"x" * 4090)
    with pytest.raises(EncoderReadError, match="4096"):
        decoder.feed(b"more data")
