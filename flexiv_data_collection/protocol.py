"""Shared JSON-lines protocol and image helpers for Stage1 collection."""

from __future__ import annotations

import base64
import json
import socket
import time
from typing import Any


GATEWAY_SAMPLE_TYPE = "flexiv_gateway_sample"
BRIDGE_SAMPLE_TYPE = "flexiv_bridge_sample"


def now_ns() -> int:
    return time.time_ns()


def parse_tcp_endpoint(endpoint: str) -> tuple[str, int]:
    if not endpoint.startswith("tcp://"):
        raise ValueError(f"Only tcp:// endpoints are supported: {endpoint}")
    host, port_s = endpoint[len("tcp://") :].rsplit(":", 1)
    return host or "127.0.0.1", int(port_s)


class JsonLineConnection:
    def __init__(self, conn: socket.socket) -> None:
        self.conn = conn
        self.file = conn.makefile("rwb")

    def send_json(self, data: dict[str, Any]) -> None:
        payload = json.dumps(data, separators=(",", ":")).encode("utf-8") + b"\n"
        self.file.write(payload)
        self.file.flush()

    def recv_json(self, timeout: float | None = None) -> dict[str, Any]:
        old_timeout = self.conn.gettimeout()
        self.conn.settimeout(timeout)
        try:
            line = self.file.readline()
        finally:
            self.conn.settimeout(old_timeout)
        if not line:
            raise EOFError("JSON-line socket closed")
        return json.loads(line.decode("utf-8"))

    def close(self) -> None:
        try:
            try:
                self.file.close()
            except OSError:
                pass
        finally:
            self.conn.close()


class JsonLinePushClient:
    def __init__(self, endpoint: str, timeout: float = 10.0, *, retry: bool = True) -> None:
        self.endpoint = endpoint
        self.timeout = timeout
        self.retry = retry
        self.connection: JsonLineConnection | None = None
        self._connect()

    def _connect(self) -> None:
        host, port = parse_tcp_endpoint(self.endpoint)
        deadline = time.monotonic() + self.timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                conn = socket.create_connection((host, port), timeout=1.0)
                self.connection = JsonLineConnection(conn)
                return
            except OSError as exc:
                last_error = exc
                if not self.retry:
                    break
                time.sleep(0.05)
        raise TimeoutError(f"Could not connect to {self.endpoint}: {last_error}")

    def send_json(self, data: dict[str, Any]) -> None:
        if self.connection is None:
            self._connect()
        assert self.connection is not None
        self.connection.send_json(data)

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
        self.connection = None


class JsonLineReqClient(JsonLinePushClient):
    def recv_json(self, timeout: float | None = None) -> dict[str, Any]:
        if self.connection is None:
            self._connect()
        assert self.connection is not None
        return self.connection.recv_json(timeout=timeout)


class JsonLineRepServer:
    def __init__(self, endpoint: str) -> None:
        host, port = parse_tcp_endpoint(endpoint)
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(1)
        self.connection: JsonLineConnection | None = None

    def _ensure_connection(self, timeout: float | None = None) -> JsonLineConnection | None:
        if self.connection is not None:
            return self.connection
        self.listener.settimeout(timeout)
        try:
            conn, _addr = self.listener.accept()
        except (socket.timeout, BlockingIOError):
            return None
        finally:
            self.listener.settimeout(None)
        self.connection = JsonLineConnection(conn)
        return self.connection

    def recv_json(self, timeout: float | None = None) -> dict[str, Any] | None:
        connection = self._ensure_connection(timeout=timeout)
        if connection is None:
            return None
        try:
            return connection.recv_json(timeout=timeout)
        except EOFError:
            connection.close()
            self.connection = None
            return None

    def send_json(self, data: dict[str, Any]) -> None:
        connection = self._ensure_connection()
        if connection is None:
            raise RuntimeError("No JSON-lines client connected")
        connection.send_json(data)

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
        self.listener.close()


def encode_image_bgr(image: Any, *, quality: int = 90) -> dict[str, Any]:
    import cv2
    import numpy as np

    arr = np.asarray(image)
    if arr.ndim != 3 or arr.shape[2] not in (3, 4):
        raise ValueError(f"Expected HxWx3/4 image, got shape {arr.shape}")
    if arr.shape[2] == 4:
        arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("Failed to JPEG-encode image")
    return {
        "encoding": "jpg_bgr",
        "shape": [int(arr.shape[0]), int(arr.shape[1]), int(arr.shape[2])],
        "data": base64.b64encode(encoded.tobytes()).decode("ascii"),
    }


def decode_image_bgr(payload: dict[str, Any]) -> Any:
    import cv2
    import numpy as np

    if payload.get("encoding") != "jpg_bgr":
        raise ValueError(f"Unsupported image encoding: {payload.get('encoding')!r}")
    raw = base64.b64decode(payload["data"].encode("ascii"))
    buf = np.frombuffer(raw, dtype=np.uint8)
    image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Failed to decode JPEG image")
    return image


def decode_sample_images(sample: dict[str, Any]) -> dict[str, Any]:
    colors: dict[str, Any] = {}
    for key, value in sample.get("colors", {}).items():
        if isinstance(value, dict) and value.get("encoding"):
            colors[key] = decode_image_bgr(value)
        else:
            raise ValueError(f"Gateway sample color {key} is not an encoded image payload")
    return colors


def make_gateway_sample(
    *,
    seq: int,
    backend: str,
    states: dict[str, Any],
    actions: dict[str, Any],
    colors: dict[str, Any],
    sim_state: dict[str, Any] | None = None,
    stamp_ns: int | None = None,
) -> dict[str, Any]:
    return {
        "type": GATEWAY_SAMPLE_TYPE,
        "version": 1,
        "seq": int(seq),
        "stamp_ns": now_ns() if stamp_ns is None else int(stamp_ns),
        "backend": str(backend),
        "colors": colors,
        "depths": {},
        "states": states,
        "actions": actions,
        "tactiles": None,
        "audios": None,
        "sim_state": sim_state or {},
    }
