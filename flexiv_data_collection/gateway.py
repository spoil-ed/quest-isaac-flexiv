#!/usr/bin/env python3
"""Stage1 Flexiv data gateway.

The gateway receives samples from the Isaac app and serves recorder requests
through a single sample endpoint. It also has a fake backend for no-Isaac smoke
tests.
"""

from __future__ import annotations

import argparse
import math
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from flexiv_data_collection.protocol import (
    BRIDGE_SAMPLE_TYPE,
    JsonLineConnection,
    JsonLineRepServer,
    encode_image_bgr,
    make_gateway_sample,
    now_ns,
    parse_tcp_endpoint,
)
from flexiv_data_collection.schema import (
    FLEXIV_CAMERA_TO_IMAGE_KEY,
    FLEXIV_VECTOR_DIM,
    fake_target_to_joint_vector,
    unitree_parts_from_vector,
)


@dataclass
class LatestBridgeData:
    lock: threading.Lock = field(default_factory=threading.Lock)
    colors: dict[str, Any] = field(default_factory=dict)
    sim_state: dict[str, Any] = field(default_factory=dict)
    states: dict[str, Any] | None = None
    actions: dict[str, Any] | None = None
    stamp_ns: int = 0
    reset_request: dict[str, Any] | None = None
    reset_seq: int = 0
    reset_inflight_seq: int | None = None

    def update(self, message: dict[str, Any]) -> None:
        with self.lock:
            self.stamp_ns = int(message.get("stamp_ns", now_ns()))
            if message.get("colors"):
                self.colors = dict(message["colors"])
            if message.get("sim_state"):
                self.sim_state = dict(message["sim_state"])
                reset_status = self.sim_state.get("reset") or {}
                status_seq = int(reset_status.get("last_seq", 0))
                if (
                    self.reset_inflight_seq is not None
                    and status_seq >= self.reset_inflight_seq
                    and reset_status.get("state") in ("succeeded", "failed")
                ):
                    self.reset_inflight_seq = None
            if message.get("states"):
                self.states = dict(message["states"])
            if message.get("actions"):
                self.actions = dict(message["actions"])

    def snapshot(self) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any] | None, dict[str, Any] | None, int]:
        with self.lock:
            return (
                dict(self.colors),
                dict(self.sim_state),
                None if self.states is None else dict(self.states),
                None if self.actions is None else dict(self.actions),
                self.stamp_ns,
            )

    def request_reset(self, reason: str) -> dict[str, Any]:
        with self.lock:
            reset_status = self.sim_state.get("reset") or {}
            if (
                self.reset_request is not None
                or self.reset_inflight_seq is not None
                or reset_status.get("state") in ("startup", "moving")
            ):
                raise RuntimeError("reset is already in progress")
            self.reset_seq += 1
            self.reset_request = {
                "type": "flexiv_bridge_control",
                "version": 1,
                "command": "reset",
                "reason": reason,
                "seq": self.reset_seq,
                "stamp_ns": now_ns(),
            }
            self.reset_inflight_seq = self.reset_seq
            return dict(self.reset_request)

    def consume_reset_request(self) -> dict[str, Any] | None:
        with self.lock:
            if self.reset_request is None:
                return None
            request = dict(self.reset_request)
            self.reset_request = None
            return request


class BridgeReceiver(threading.Thread):
    def __init__(self, endpoint: str, latest: LatestBridgeData) -> None:
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.latest = latest
        self.stop_event = threading.Event()
        self.listener: socket.socket | None = None

    def run(self) -> None:
        host, port = parse_tcp_endpoint(self.endpoint)
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(5)
        self.listener.settimeout(0.5)
        print(f"[data-gateway] bridge receiver listening on {self.endpoint}", flush=True)
        while not self.stop_event.is_set():
            try:
                conn, addr = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[data-gateway] bridge publisher connected from {addr}", flush=True)
            threading.Thread(target=self._handle_connection, args=(conn,), daemon=True).start()

    def _handle_connection(self, conn: socket.socket) -> None:
        connection = JsonLineConnection(conn)
        try:
            while not self.stop_event.is_set():
                message = connection.recv_json()
                if message.get("type") != BRIDGE_SAMPLE_TYPE:
                    continue
                self.latest.update(message)
                control = self.latest.consume_reset_request()
                if control is not None:
                    connection.send_json(control)
        except (EOFError, OSError):
            pass
        finally:
            connection.close()

    def close(self) -> None:
        self.stop_event.set()
        if self.listener is not None:
            self.listener.close()


class FakeBackend:
    def __init__(
        self,
        fps: float,
        image_size: tuple[int, int],
        camera_keys: list[str],
        *,
        sim_backend: str = "fake",
        left_serial: str = "",
        right_serial: str = "",
    ) -> None:
        self.fps = fps
        self.image_size = image_size
        self.camera_keys = camera_keys
        self.sim_backend = str(sim_backend)
        self.left_serial = str(left_serial)
        self.right_serial = str(right_serial)
        self.seq = 0

    def sample(self, latest: LatestBridgeData) -> dict[str, Any]:
        import numpy as np

        t = self.seq / max(self.fps, 1.0)
        phase = (self.seq % 120) / 119.0
        left_xyz = [0.35 + 0.20 * phase, 0.26, 0.88]
        right_xyz = [0.35 + 0.20 * phase, -0.26, 0.88]
        qpos = fake_target_to_joint_vector(left_xyz, right_xyz, left_gripper=0.8, right_gripper=0.8)
        qvel = [0.0] * FLEXIV_VECTOR_DIM
        qvel[0] = 0.20 * self.fps / 119.0
        qvel[8] = 0.20 * self.fps / 119.0
        torque = (np.sin(np.linspace(0, math.pi, FLEXIV_VECTOR_DIM) + t) * 0.05).astype(float).tolist()

        colors = {
            key: encode_image_bgr(self._fake_image(key, self.seq))
            for key in self.camera_keys
        }
        states = unitree_parts_from_vector(qpos, qvel=qvel, torque=torque)
        actions = unitree_parts_from_vector(qpos, qvel=qvel, torque=torque)
        sample = make_gateway_sample(
            seq=self.seq,
            backend="fake",
            states=states,
            actions=actions,
            colors=colors,
            sim_state={
                "backend": self.sim_backend,
                "sample_id": self.seq,
                "servo_cycle": self.seq,
                "servo_cycles": {"left": self.seq, "right": self.seq},
                "serials": {"left": self.left_serial, "right": self.right_serial},
                "timestamps": {
                    "action_time_ns": now_ns(),
                    "state_time_ns": now_ns(),
                    "image_time_ns": now_ns(),
                },
                "debug": {"fake_phase": phase},
            },
        )
        self.seq += 1
        return sample

    def _fake_image(self, camera_key: str, seq: int) -> Any:
        import cv2
        import numpy as np

        width, height = self.image_size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        base = (seq * 7 + len(camera_key) * 31) % 255
        img[:, :, 0] = (base + 35) % 255
        x_grad = np.linspace(0, 255, width, dtype=np.uint16)
        y_grad = np.linspace(0, 255, height, dtype=np.uint16)
        img[:, :, 1] = ((x_grad[None, :] + base) % 255).astype(np.uint8)
        img[:, :, 2] = ((y_grad[:, None] + 2 * base) % 255).astype(np.uint8)
        cv2.putText(
            img,
            f"{camera_key} #{seq}",
            (24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return img


class BridgeBackend:
    def __init__(self) -> None:
        self.seq = 0

    def sample(self, latest: LatestBridgeData) -> dict[str, Any]:
        colors, sim_state, states, actions, stamp_ns = latest.snapshot()
        if states is None or actions is None:
            raise RuntimeError("No bridge states/actions received yet")
        sample = make_gateway_sample(
            seq=self.seq,
            backend="bridge",
            states=states,
            actions=actions,
            colors=colors,
            sim_state={
                "backend": "bridge",
                "bridge": sim_state,
                "timestamps": {
                    "action_time_ns": stamp_ns or now_ns(),
                    "state_time_ns": stamp_ns or now_ns(),
                    "image_time_ns": stamp_ns or now_ns(),
                },
            },
            stamp_ns=stamp_ns or None,
        )
        self.seq += 1
        return sample


def parse_image_size(value: str) -> tuple[int, int]:
    width_s, height_s = value.lower().split("x", 1)
    return int(width_s), int(height_s)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-endpoint", default="tcp://0.0.0.0:5590")
    parser.add_argument("--bridge-endpoint", default="tcp://0.0.0.0:5591")
    parser.add_argument("--backend", choices=["fake", "bridge"], default="bridge")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--image-size", default="640x480")
    parser.add_argument(
        "--camera-keys",
        default=",".join(FLEXIV_CAMERA_TO_IMAGE_KEY.keys()),
        help="Comma-separated Unitree color keys, e.g. color_0,color_1,color_2.",
    )
    parser.add_argument("--fake-sim-backend", default="fake")
    parser.add_argument("--fake-left-serial", default="")
    parser.add_argument("--fake-right-serial", default="")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    image_size = parse_image_size(args.image_size)
    camera_keys = [item.strip() for item in args.camera_keys.split(",") if item.strip()]
    latest = LatestBridgeData()
    bridge_receiver = BridgeReceiver(args.bridge_endpoint, latest)
    bridge_receiver.start()

    backend = (
        FakeBackend(
            args.fps,
            image_size,
            camera_keys,
            sim_backend=args.fake_sim_backend,
            left_serial=args.fake_left_serial,
            right_serial=args.fake_right_serial,
        )
        if args.backend == "fake"
        else BridgeBackend()
    )
    server = JsonLineRepServer(args.sample_endpoint)
    print(f"[data-gateway] sample endpoint ready on {args.sample_endpoint}, backend={args.backend}", flush=True)
    try:
        while True:
            request = server.recv_json(timeout=0.5)
            if request is None:
                continue
            if request.get("type") == "shutdown":
                server.send_json({"type": "ok"})
                break
            if request.get("type") == "reset_request":
                try:
                    control = latest.request_reset(str(request.get("reason", "recorder")))
                except Exception as exc:
                    server.send_json({"type": "error", "error": str(exc)})
                else:
                    server.send_json({"type": "ok", "control": control})
                continue
            if request.get("type") != "sample_request":
                server.send_json({"type": "error", "error": "unsupported request"})
                continue
            try:
                sample = backend.sample(latest)
            except Exception as exc:
                server.send_json({"type": "error", "error": str(exc)})
            else:
                server.send_json(sample)
    finally:
        bridge_receiver.close()
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
