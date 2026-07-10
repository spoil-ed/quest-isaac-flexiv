"""Flexiv RDK target streaming sink."""

from __future__ import annotations

import importlib
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass


LogFn = Callable[[str], None]


@dataclass(frozen=True)
class RdkSinkSettings:
    stream_hz: float = 250.0
    switch_mode: bool = True
    clear_fault: bool = False
    servo_on: bool = False
    verbose: bool = False
    reconnect_period: float = 2.0


class RdkRobotStreamer:
    """Background worker that streams the latest Cartesian target to one robot."""

    def __init__(self, serial_number: str, settings: RdkSinkSettings, log: LogFn | None = None) -> None:
        self.serial_number = serial_number
        self.settings = settings
        self._log = log or (lambda _msg: None)
        self._lock = threading.Lock()
        self._targets: dict[str, list[float]] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"FlexivRdkStreamer:{self.serial_number}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def set_target(self, joint_group: str, pose: list[float]) -> None:
        if len(pose) != 7:
            raise ValueError("Flexiv pose command must contain 7 values")
        with self._lock:
            self._targets[joint_group] = [float(v) for v in pose]

    def clear_target(self, joint_group: str) -> None:
        with self._lock:
            self._targets.pop(joint_group, None)

    def _snapshot(self) -> dict[str, list[float]]:
        with self._lock:
            return {group: list(pose) for group, pose in self._targets.items()}

    @staticmethod
    def _create_robot(flexivrdk, serial_number: str, verbose: bool):
        try:
            return flexivrdk.Robot(serial_number, verbose)
        except TypeError:
            return flexivrdk.Robot(serial_number, [], verbose)

    @staticmethod
    def _cartesian_mode(flexivrdk):
        if hasattr(flexivrdk, "RT_CARTESIAN_MOTION_FORCE"):
            return flexivrdk.RT_CARTESIAN_MOTION_FORCE, "RT_CARTESIAN_MOTION_FORCE"
        return flexivrdk.NRT_CARTESIAN_MOTION_FORCE, "NRT_CARTESIAN_MOTION_FORCE"

    def _connect(self):
        flexivrdk = importlib.import_module("flexivrdk")
        robot = self._create_robot(flexivrdk, self.serial_number, self.settings.verbose)
        if self.settings.clear_fault and robot.fault():
            self._log(f"[FlexivRDK] clearing fault on {self.serial_number}")
            robot.ClearFault()
        if self.settings.servo_on and hasattr(robot, "ServoOn"):
            self._log(f"[FlexivRDK] servo on {self.serial_number}")
            robot.ServoOn()
        if not robot.operational():
            raise RuntimeError(
                f"{self.serial_number} is not operational. Connect the simulated robot in Elements Studio "
                "and enable Remote Mode."
            )
        cartesian_mode, mode_name = self._cartesian_mode(flexivrdk)
        if self.settings.switch_mode and robot.mode() != cartesian_mode:
            self._log(f"[FlexivRDK] switching {self.serial_number} to {mode_name}")
            robot.SwitchMode(cartesian_mode)
        return flexivrdk, robot

    def _run(self) -> None:
        flexivrdk = None
        robot = None
        period = 1.0 / max(1.0, float(self.settings.stream_hz))
        next_connect_log = 0.0

        while not self._stop.is_set():
            if robot is None:
                try:
                    flexivrdk, robot = self._connect()
                    self._log(f"[FlexivRDK] connected {self.serial_number}")
                except Exception as exc:
                    now = time.monotonic()
                    if now >= next_connect_log:
                        self._log(f"[FlexivRDK] waiting for {self.serial_number}: {exc}")
                        next_connect_log = now + max(0.5, self.settings.reconnect_period)
                    self._stop.wait(max(0.1, self.settings.reconnect_period))
                    continue

            targets = self._snapshot()
            if not targets:
                self._stop.wait(period)
                continue

            try:
                cmds = {}
                for group_name, pose in targets.items():
                    cmd = flexivrdk.RtCartesianCmd()
                    cmd.pose_d = pose
                    cmd.twist_d = [0.0] * 6
                    cmd.wrench_d = [0.0] * 6
                    cmd.acc_d = [0.0] * 6
                    cmds[getattr(flexivrdk, group_name)] = cmd
                robot.StreamCartesianMotionForce(cmds)
            except Exception as exc:
                self._log(f"[FlexivRDK] stream error on {self.serial_number}: {exc}")
                robot = None
                self._stop.wait(max(0.1, self.settings.reconnect_period))
                continue

            self._stop.wait(period)


class FlexivRdkCartesianSink:
    """Multi-robot sink used by the Teleop IK adapter."""

    def __init__(self, settings: RdkSinkSettings | None = None, log: LogFn | None = None) -> None:
        self.settings = settings or RdkSinkSettings()
        self._log = log or (lambda _msg: None)
        self._streamers: dict[str, RdkRobotStreamer] = {}

    def start_binding(self, serial_number: str, joint_group: str) -> None:
        streamer = self._streamers.get(serial_number)
        if streamer is None:
            streamer = RdkRobotStreamer(serial_number, self.settings, self._log)
            self._streamers[serial_number] = streamer
        streamer.start()

    def set_target(self, serial_number: str, joint_group: str, pose: list[float]) -> None:
        self.start_binding(serial_number, joint_group)
        self._streamers[serial_number].set_target(joint_group, pose)

    def clear_target(self, serial_number: str, joint_group: str) -> None:
        streamer = self._streamers.get(serial_number)
        if streamer is not None:
            streamer.clear_target(joint_group)

    def stop(self) -> None:
        for streamer in list(self._streamers.values()):
            streamer.stop()
        self._streamers.clear()
