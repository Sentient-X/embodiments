"""The device driver parses actual HTTP responses without a headset."""

import json
import threading
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from sx_embodiments.quest_capture import QuestLink, QuestUnavailableError


@pytest.fixture
def clock_endpoint() -> Iterator[tuple[QuestLink, dict[str, object]]]:
    exchange: dict[str, object] = {
        "response": {
            "quest_recv_mono_ns": 100,
            "quest_send_mono_ns": 110,
            "quest_utc_ns": 200,
        }
    }

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:
            exchange["path"] = self.path
            exchange["request"] = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            payload = json.dumps(exchange["response"]).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format: str, *args: object) -> None:
            pass

    with HTTPServer(("127.0.0.1", 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield QuestLink(local_port=server.server_port), exchange
        finally:
            server.shutdown()
            thread.join(timeout=2)
            assert not thread.is_alive()


def test_clock_reply_preserves_device_times_and_request_audit(
    clock_endpoint: tuple[QuestLink, dict[str, object]],
) -> None:
    link, exchange = clock_endpoint
    reply = link.timesync_ping(3, 400)
    assert exchange["path"] == "/api/timesync/ping"
    assert exchange["request"] == {
        "ping_id": 3,
        "host_utc_ns": 400,
        "host_send_utc_ns": 400,
    }
    assert (reply.receive_monotonic_ns, reply.send_monotonic_ns, reply.utc_ns) == (100, 110, 200)


@pytest.mark.parametrize("invalid", [True, "100", 100.5, None, -1, 0])
def test_clock_reply_refuses_non_integer_or_nonpositive_device_time(
    clock_endpoint: tuple[QuestLink, dict[str, object]], invalid: object
) -> None:
    link, exchange = clock_endpoint
    exchange["response"] = {
        "quest_recv_mono_ns": invalid,
        "quest_send_mono_ns": 110,
        "quest_utc_ns": 200,
    }
    with pytest.raises(QuestUnavailableError, match="positive integer"):
        link.timesync_ping(3, 400)


def test_clock_reply_refuses_non_object_response(
    clock_endpoint: tuple[QuestLink, dict[str, object]],
) -> None:
    link, exchange = clock_endpoint
    exchange["response"] = [100, 110, 200]
    with pytest.raises(QuestUnavailableError, match="non-object"):
        link.timesync_ping(3, 400)


def test_clock_reply_refuses_reversed_device_times(
    clock_endpoint: tuple[QuestLink, dict[str, object]],
) -> None:
    link, exchange = clock_endpoint
    exchange["response"] = {
        "quest_recv_mono_ns": 120,
        "quest_send_mono_ns": 110,
        "quest_utc_ns": 200,
    }
    with pytest.raises(QuestUnavailableError, match="reverses"):
        link.timesync_ping(3, 400)
