"""Bounded UART line framing shared by the YUBI and Piper capture encoders."""

import math
import time
from collections.abc import Generator, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

MAX_ENCODER_LINE_BYTES = 4096


class EncoderReadError(Exception):
    """An encoder is unavailable or emits unusable serial framing."""


class EncoderLineDecoder:
    """Reassemble timed-out readline fragments and discard the initial partial frame.

    A UART can be opened midway through a line. Synchronization is established
    only at the first newline, even if reaching it takes multiple reads. Each
    subsequent newline terminates one complete sample or diagnostic line.
    """

    def __init__(self) -> None:
        self._pending = bytearray()
        self._synchronized = False

    def feed(self, fragment: bytes) -> str | None:
        """Consume one bounded serial.readline result; incomplete lines yield nothing."""
        if b"\n" in fragment[:-1]:
            raise EncoderReadError("encoder readline returned multiple lines")
        if len(self._pending) + len(fragment) > MAX_ENCODER_LINE_BYTES:
            raise EncoderReadError("encoder line exceeds 4096 bytes")
        self._pending.extend(fragment)
        if not self._pending.endswith(b"\n"):
            return None
        complete = bytes(self._pending)
        self._pending.clear()
        if not self._synchronized:
            self._synchronized = True
            return None
        try:
            return complete.decode("utf-8").strip()
        except UnicodeDecodeError as error:
            raise EncoderReadError("encoder line is not valid UTF-8") from error


@dataclass(frozen=True, slots=True)
class EncoderLine:
    """One complete UART line stamped with host monotonic time at receipt."""

    text: str
    received_monotonic_ns: int


@contextmanager
def open_encoder_lines(
    device: Path, *, baud: int, timeout_s: float
) -> Generator[Iterator[EncoderLine | None]]:
    """Open one encoder UART; timeouts yield no sample so owners can check stop/freshness."""
    if type(baud) is not int or baud <= 0:
        raise EncoderReadError("encoder baud must be a positive integer")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise EncoderReadError("encoder timeout must be finite and positive")
    try:
        import serial
    except ImportError as error:
        raise EncoderReadError("encoder capture requires sx-embodiments[capture]") from error
    try:
        port = serial.Serial(str(device), baud, timeout=timeout_s)
    except (serial.SerialException, OSError) as error:
        raise EncoderReadError(f"encoder serial open failed: {error}") from error
    try:
        try:
            port.reset_input_buffer()
        except (serial.SerialException, OSError) as error:
            raise EncoderReadError(f"encoder serial reset failed: {error}") from error
        decoder = EncoderLineDecoder()

        def lines() -> Iterator[EncoderLine | None]:
            while True:
                try:
                    fragment = port.readline(MAX_ENCODER_LINE_BYTES + 1)
                except (serial.SerialException, OSError) as error:
                    raise EncoderReadError(f"encoder serial read failed: {error}") from error
                received = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                text = decoder.feed(fragment)
                yield None if text is None else EncoderLine(text, received)

        yield lines()
    finally:
        try:
            port.close()
        except (serial.SerialException, OSError) as error:
            raise EncoderReadError(f"encoder serial close failed: {error}") from error
