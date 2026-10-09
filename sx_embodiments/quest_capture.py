"""OpenQuestCapture device control over USB/adb-forwarded HTTP.

Owns the headset protocol and APK identity. Recording files are pulled with
adb because the HTTP recording route omits pose CSVs. Clock-fit acceptance
and episode orchestration belong to the capture consumer.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from sx_embodiments.quest import (
    QuestApkBuildId,
    QuestApkBuildIdError,
    QuestCaptureApp,
    QuestCaptureAppError,
)

QUEST_PACKAGE = "com.samusynth.OpenQuestCapture"
QUEST_PORT = 8080
QUEST_FILES_DIR = f"/sdcard/Android/data/{QUEST_PACKAGE}/files"
HTTP_TIMEOUT_S = 5.0
ADB_TIMEOUT_S = 30.0
PULL_TIMEOUT_S = 600.0


class QuestUnavailableError(Exception):
    """The headset, the adb link, or the capture app is not reachable."""


class QuestArmTimeoutError(QuestUnavailableError):
    """The absolute managed-arm deadline expired."""


class QuestApkIdentityError(QuestUnavailableError):
    """The installed capture app cannot produce a trustworthy content identity."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"Quest APK identity failed: {reason}")


_HTTP_ERROR_DETAIL_LIMIT = 600


def _http_error_detail(error: urllib.error.HTTPError) -> str:
    """``: <body>`` for an HTTP error whose body carries the app's reason, else ``""``."""
    try:
        body = error.read().decode("utf-8", errors="replace").strip()
    except OSError:
        return ""
    if not body:
        return ""
    try:
        parsed: object = json.loads(body)
    except json.JSONDecodeError:
        parsed = None
    if isinstance(parsed, dict):
        fields = cast("dict[str, object]", parsed)
        for key in ("error", "message", "reason", "detail"):
            value = fields.get(key)
            if isinstance(value, str) and value.strip():
                body = value.strip()
                break
    return f": {body[:_HTTP_ERROR_DETAIL_LIMIT]}"


@dataclass(frozen=True, slots=True)
class QuestClockReply:
    """Device-side timestamps from one OpenQuestCapture ping response."""

    receive_monotonic_ns: int
    send_monotonic_ns: int
    utc_ns: int


class QuestLink:
    def __init__(self, *, local_port: int = QUEST_PORT, serial: str | None = None) -> None:
        self._local_port = local_port
        self._serial = serial

    # ── adb plumbing ─────────────────────────────────────────────────────

    @staticmethod
    def _remaining(deadline_mono_ns: int | None, maximum_s: float) -> float:
        if deadline_mono_ns is None:
            return maximum_s
        remaining_s = (deadline_mono_ns - time.clock_gettime_ns(time.CLOCK_MONOTONIC)) / 1e9
        if remaining_s <= 0:
            raise QuestArmTimeoutError("managed Quest operation exceeded its arm deadline")
        return min(maximum_s, remaining_s)

    def _adb(
        self,
        *args: str,
        timeout: float = ADB_TIMEOUT_S,
        deadline_mono_ns: int | None = None,
    ) -> str:
        command = ["adb", *(("-s", self._serial) if self._serial else ()), *args]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self._remaining(deadline_mono_ns, timeout),
                check=False,
            )
        except FileNotFoundError as exc:
            raise QuestUnavailableError("adb is not installed on this pod") from exc
        except subprocess.TimeoutExpired as exc:
            raise QuestUnavailableError(f"adb timed out: {' '.join(args)}") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise QuestUnavailableError(f"adb {' '.join(args)} failed: {detail}")
        return result.stdout

    def ensure_forward(self, *, deadline_mono_ns: int | None = None) -> None:
        self._adb(
            "forward",
            f"tcp:{self._local_port}",
            f"tcp:{QUEST_PORT}",
            deadline_mono_ns=deadline_mono_ns,
        )

    def recover(self, *, deadline_mono_ns: int | None = None) -> None:
        """Wake the headset, relaunch the capture app, re-establish forward."""
        self._adb(
            "shell",
            "input",
            "keyevent",
            "KEYCODE_WAKEUP",
            deadline_mono_ns=deadline_mono_ns,
        )
        self._adb(
            "shell",
            "monkey",
            "-p",
            QUEST_PACKAGE,
            "-c",
            "android.intent.category.LAUNCHER",
            "1",
            deadline_mono_ns=deadline_mono_ns,
        )
        time.sleep(self._remaining(deadline_mono_ns, 2.0))
        self.ensure_forward(deadline_mono_ns=deadline_mono_ns)

    def capture_app(
        self,
        status: Mapping[str, object],
        *,
        deadline_mono_ns: int | None = None,
    ) -> QuestCaptureApp:
        """Resolve exact installed APK bytes and independent runtime facts."""
        package_output = self._adb(
            "shell", "pm", "path", QUEST_PACKAGE, deadline_mono_ns=deadline_mono_ns
        )
        apk_paths: list[str] = []
        for line in package_output.splitlines():
            candidate = line.strip()
            if not candidate.startswith("package:"):
                continue
            path = candidate.removeprefix("package:")
            if (
                not path.startswith("/")
                or not path.endswith(".apk")
                or any(character.isspace() for character in path)
            ):
                raise QuestApkIdentityError(f"pm path returned an invalid APK path {path!r}")
            if path not in apk_paths:
                apk_paths.append(path)
        base_paths = [path for path in apk_paths if Path(path).name == "base.apk"]
        if len(base_paths) != 1:
            raise QuestApkIdentityError(
                f"pm path returned {len(base_paths)} base APKs (expected exactly one)"
            )
        base_path = base_paths[0]
        ordered_paths = (
            base_path,
            *sorted(path for path in apk_paths if path != base_path),
        )
        per_apk_digests: list[str] = []
        with tempfile.TemporaryDirectory(prefix="sx-quest-apks-") as temporary:
            local_root = Path(temporary)
            for index, apk_path in enumerate(ordered_paths):
                local_path = local_root / f"{index:04d}-{Path(apk_path).name}"
                self._adb(
                    "pull",
                    apk_path,
                    str(local_path),
                    deadline_mono_ns=deadline_mono_ns,
                )
                try:
                    digest = hashlib.sha256(local_path.read_bytes()).hexdigest()
                except OSError as error:
                    raise QuestApkIdentityError(
                        f"pulled APK is unreadable for {apk_path}: {error}"
                    ) from error
                per_apk_digests.append(digest)
        digest = (
            per_apk_digests[0]
            if len(per_apk_digests) == 1
            else hashlib.sha256("".join(per_apk_digests).encode("ascii")).hexdigest()
        )
        app_version = status.get("apkVersion")
        if not isinstance(app_version, str) or not app_version.strip():
            raise QuestApkIdentityError("/api/status has no non-empty apkVersion")
        os_build = self._adb(
            "shell",
            "getprop",
            "ro.build.fingerprint",
            deadline_mono_ns=deadline_mono_ns,
        ).strip()
        try:
            return QuestCaptureApp(
                apk_build_id=QuestApkBuildId.from_digest(digest),
                app_version=app_version,
                os_build=os_build,
            )
        except (QuestApkBuildIdError, QuestCaptureAppError) as error:
            raise QuestApkIdentityError(str(error)) from error

    def pull_session(self, session_id: str, destination: Path) -> Path:
        """``adb pull`` one recording session dir; returns the local dir."""
        destination.mkdir(parents=True, exist_ok=True)
        last_error: QuestUnavailableError | None = None
        for attempt in range(3):
            try:
                if attempt:
                    self.recover()
                recording_dir = self._recording_dir_for_session(session_id) or session_id
                remote = f"{QUEST_FILES_DIR}/{recording_dir}"
                self._adb("pull", remote, str(destination), timeout=PULL_TIMEOUT_S)
                local = destination / recording_dir
                if not local.is_dir():
                    raise QuestUnavailableError(f"adb pull produced no directory at {local}")
                return local
            except QuestUnavailableError as error:
                last_error = error
        assert last_error is not None
        raise last_error

    def _recording_dir_for_session(self, session_id: str) -> str | None:
        """Map the repo episode id to the Quest app's actual recording folder."""
        try:
            payload = self._request("GET", "/api/recordings")
        except QuestUnavailableError:
            return None
        recordings_value = payload.get("recordings")
        if not isinstance(recordings_value, list):
            return None
        recordings = cast("list[object]", recordings_value)
        for recording in recordings:
            if not isinstance(recording, dict):
                continue
            recording_data = cast("dict[str, object]", recording)
            metadata_value = recording_data.get("metadata")
            if not isinstance(metadata_value, dict):
                continue
            metadata = cast("dict[str, object]", metadata_value)
            if metadata.get("sessionId") == session_id:
                file = recording_data.get("file")
                return file if isinstance(file, str) and file else None
        return None

    # ── the capture app's HTTP API ───────────────────────────────────────

    def _request(
        self,
        method: str,
        path: str,
        body: dict[str, object] | None = None,
        *,
        deadline_mono_ns: int | None = None,
    ) -> dict[str, object]:
        url = f"http://127.0.0.1:{self._local_port}{path}"
        data = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(url, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(
                request,
                timeout=self._remaining(deadline_mono_ns, HTTP_TIMEOUT_S),
            ) as response:
                payload = response.read()
        except urllib.error.HTTPError as exc:
            # The app answers a refused start with its reason in the body (a
            # recorder that will not record, an unlisted stream size); keep it.
            detail = _http_error_detail(exc)
            raise QuestUnavailableError(
                f"quest app request {method} {path} failed: {exc}{detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, ConnectionError) as exc:
            raise QuestUnavailableError(f"quest app request {method} {path} failed: {exc}") from exc
        if not payload:
            return {}
        try:
            value: object = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise QuestUnavailableError(f"quest app returned non-JSON for {path}") from exc
        if not isinstance(value, dict):
            raise QuestUnavailableError(f"quest app returned a non-object for {path}")
        return cast("dict[str, object]", value)

    def status(self, *, deadline_mono_ns: int | None = None) -> dict[str, object]:
        return self._request("GET", "/api/status", deadline_mono_ns=deadline_mono_ns)

    def start_recording(self, session_id: str, *, deadline_mono_ns: int | None = None) -> None:
        self._request(
            "POST",
            "/api/start-recording",
            {
                "sessionId": session_id,
                "host_utc_ns": time.clock_gettime_ns(time.CLOCK_REALTIME),
            },
            deadline_mono_ns=deadline_mono_ns,
        )

    def stop_recording(self, *, deadline_mono_ns: int | None = None) -> None:
        self._request("POST", "/api/stop-recording", deadline_mono_ns=deadline_mono_ns)

    def is_recording(self, *, deadline_mono_ns: int | None = None) -> bool:
        recording = self.status(deadline_mono_ns=deadline_mono_ns).get("recording")
        if isinstance(recording, dict):
            return bool(cast("dict[str, object]", recording).get("active"))
        return bool(recording)

    def timesync_ping(
        self, ping_id: int, host_utc_ns: int, *, deadline_mono_ns: int | None = None
    ) -> QuestClockReply:
        """One device clock reply; the capture owner measures and qualifies the exchange."""
        reply = self._request(
            "POST",
            "/api/timesync/ping",
            {"ping_id": ping_id, "host_utc_ns": host_utc_ns, "host_send_utc_ns": host_utc_ns},
            deadline_mono_ns=deadline_mono_ns,
        )
        values: list[int] = []
        for name in ("quest_recv_mono_ns", "quest_send_mono_ns", "quest_utc_ns"):
            value = reply.get(name)
            if type(value) is not int or value <= 0:
                raise QuestUnavailableError(f"quest clock reply {name} must be a positive integer")
            values.append(value)
        if values[1] < values[0]:
            raise QuestUnavailableError("quest clock reply reverses receive/send time")
        return QuestClockReply(*values)
