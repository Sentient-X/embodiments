"""UART framing does not turn a timeout fragment into an encoder sample."""

import pytest

from sx_embodiments.encoder_serial import EncoderLineDecoder, EncoderReadError, open_encoder_lines


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


def test_failed_close_does_not_replace_the_read_failure(monkeypatch, tmp_path) -> None:
    serial = pytest.importorskip("serial")

    class Port:
        def __init__(self, *_: object, **__: object) -> None:
            pass

        def reset_input_buffer(self) -> None:
            pass

        def readline(self, _: int) -> bytes:
            raise serial.SerialException("unplugged")

        def close(self) -> None:
            raise OSError("close failed")

    monkeypatch.setattr(serial, "Serial", Port)
    with pytest.raises(EncoderReadError, match="read failed: unplugged"):
        with open_encoder_lines(tmp_path / "tty", baud=115200, timeout_s=0.1) as lines:
            next(lines)
    with pytest.raises(EncoderReadError, match="close failed"):
        with open_encoder_lines(tmp_path / "tty", baud=115200, timeout_s=0.1):
            pass
