#!/usr/bin/env python3
"""Deploy the validated H2 AMP ONNX policy through Unitree SDK2 Python.

The default ``observe`` mode is read-only.  Any mode that publishes LowCmd
requires both ``--enable-low-level`` and ``--confirm-suspended``.  First use
this program with the robot securely suspended and a physical emergency stop
operator present.

Policy interface:
  obs:     float32 [1, 400] = four time-ordered 100-dimensional frames
  actions: float32 [1, 29]  = normalized joint-position actions

This script intentionally has no Isaac Lab dependency.  It only needs NumPy,
ONNX Runtime (for policy/self-test mode), and unitree_sdk2py (for robot modes).
"""

from __future__ import annotations

import argparse
import os
import signal
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np


H2_NUM_MOTORS = 31
POLICY_ACTION_DIM = 29
FRAME_OBS_DIM = 100
HISTORY_LENGTH = 4
POLICY_OBS_DIM = FRAME_OBS_DIM * HISTORY_LENGTH

OBS_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "head_pitch_joint", "head_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
ACTION_JOINT_NAMES = tuple(
    name for name in OBS_JOINT_NAMES if not name.startswith("head_")
)

# Policy observation order -> Unitree H2 SDK motor index.
OBS_TO_SDK = np.asarray(
    [*range(0, 15), 29, 30, *range(15, 29)], dtype=np.int64
)
# The policy action order is exactly SDK motors 0..28.
ACTION_TO_SDK = np.arange(POLICY_ACTION_DIM, dtype=np.int64)

DEFAULT_Q_31 = np.asarray(
    [
        -0.25, 0.0, 0.0, 0.50, 0.0, -0.25,
        -0.25, 0.0, 0.0, 0.50, 0.0, -0.25,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
        0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
    ],
    dtype=np.float32,
)
ACTION_DEFAULT_Q = np.delete(DEFAULT_Q_31, [15, 16]).astype(np.float32)
ACTION_SCALE = np.asarray(
    [
        0.45, 0.45, 0.45, 0.45, 0.11875, 0.4125,
        0.45, 0.45, 0.45, 0.45, 0.11875, 0.4125,
        0.20, 0.20, 0.20,
        0.3375, 0.3375, 0.3375, 0.3375, 0.3375, 0.3125, 0.3125,
        0.3375, 0.3375, 0.3375, 0.3375, 0.3375, 0.3125, 0.3125,
    ],
    dtype=np.float32,
)

# Training PD gains in H2 SDK motor order (legs, waist, arms, head).
TRAINING_KP = np.asarray(
    [
        200, 200, 200, 200, 40, 40,
        200, 200, 200, 200, 40, 40,
        150, 150, 150,
        40, 40, 40, 40, 40, 20, 20,
        40, 40, 40, 40, 40, 20, 20,
        220, 220,
    ],
    dtype=np.float32,
)
TRAINING_KD = np.asarray(
    [
        4, 4, 4, 4, 2, 2,
        4, 4, 4, 4, 2, 2,
        3, 3, 3,
        2, 2, 2, 2, 2, 1, 1,
        2, 2, 2, 2, 2, 1, 1,
        12, 12,
    ],
    dtype=np.float32,
)
PASSIVE_KD = np.asarray(
    [
        2, 2, 2, 2, 1, 1,
        2, 2, 2, 2, 1, 1,
        2, 2, 2,
        1, 1, 1, 1, 0.5, 0.3, 0.3,
        1, 1, 1, 1, 0.5, 0.3, 0.3,
        1, 1,
    ],
    dtype=np.float32,
)

# MJCF joint limits in SDK order.  Targets are kept a small distance inside.
SDK_Q_MIN = np.asarray(
    [
        -2.4526, -0.467441, -2.827, -0.08725, -0.349066, -1.13446,
        -2.4526, -2.16886, -2.827, -0.08725, -0.296706, -1.13446,
        -1.7453, -0.5236, -0.43633,
        -2.61799, -0.516617, -2.61799, -0.986111, -2.61799, -0.436332, -1.22173,
        -2.61799, -2.63545, -2.61799, -0.986111, -2.61799, -0.436332, -1.22173,
        -0.5236, -1.7453,
    ],
    dtype=np.float32,
)
SDK_Q_MAX = np.asarray(
    [
        2.77542, 2.16886, 2.827, 2.53025, 0.296706, 0.610865,
        2.77542, 0.467441, 2.827, 2.53025, 0.349066, 0.610865,
        1.7453, 0.5236, 0.5236,
        1.8326, 2.63545, 2.61799, 3.07178, 2.61799, 0.436332, 1.22173,
        1.8326, 0.516617, 2.61799, 3.07178, 2.61799, 0.436332, 1.22173,
        0.83775, 1.7453,
    ],
    dtype=np.float32,
)


def policy_to_sdk_q(q_policy_order: np.ndarray) -> np.ndarray:
    """Convert a 31-vector in policy observation order to SDK motor order."""
    q_policy_order = np.asarray(q_policy_order, dtype=np.float32)
    if q_policy_order.shape != (H2_NUM_MOTORS,):
        raise ValueError(f"Expected 31 policy-order joints, got {q_policy_order.shape}")
    result = np.empty(H2_NUM_MOTORS, dtype=np.float32)
    result[OBS_TO_SDK] = q_policy_order
    return result


DEFAULT_Q_SDK = policy_to_sdk_q(DEFAULT_Q_31)

# Exact name-mapped joint pose used by the Isaac validation script: frame 100,
# the minimum-velocity frame in step_rotate_idle_000_002__A026.npz.  It is
# provided for controlled, suspended parity tests; ``default`` remains safer
# for the first joint-direction/position-control check.
VALIDATED_MOTION_READY_Q_SDK = np.asarray(
    [
        -0.14114566, 0.10551190, -0.03558872, 0.37637737, -0.09698470,
        -0.30290500, -0.14355890, -0.19609715, 0.05693162, 0.27409187,
        0.19703230, -0.19480801, -0.00394200, -0.02495100, 0.01246800,
        -0.01221700, 0.33460599, -0.68495899, 0.96116400, -0.00529900,
        0.16014400, -0.02718400, -0.13857301, -0.35643199, 0.80361801,
        1.03518403, 0.01495200, 0.07606500, 0.12632300, 0.0, 0.0,
    ],
    dtype=np.float32,
)


def quat_rotate_inverse_wxyz(quat: Sequence[float], vector: Sequence[float]) -> np.ndarray:
    """Rotate a world-frame vector into body frame using a body-to-world quaternion."""
    q = np.asarray(quat, dtype=np.float32)
    v = np.asarray(vector, dtype=np.float32)
    if q.shape != (4,) or v.shape != (3,):
        raise ValueError(f"Invalid quaternion/vector shapes: {q.shape}, {v.shape}")
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1.0e-6:
        raise ValueError("Invalid IMU quaternion")
    w, x, y, z = q / norm
    # Rotation matrix maps body -> world; transpose maps world -> body.
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    return rotation.T @ v


@dataclass(frozen=True)
class RobotSnapshot:
    received_at: float
    q_sdk: np.ndarray
    dq_sdk: np.ndarray
    gyro_body: np.ndarray
    quat_wxyz: np.ndarray
    mode_machine: int
    motor_errors: tuple[int, ...]
    remote_b_pressed: bool


@dataclass(frozen=True)
class MotorCommandSnapshot:
    q_sdk: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    mode_machine: int
    active_position_control: bool


class OnnxPolicy:
    def __init__(self, model_path: str) -> None:
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError("onnxruntime is required for policy/self-test mode") from error

        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            os.path.abspath(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        self.input = self.session.get_inputs()[0]
        self.output = self.session.get_outputs()[0]
        if self.input.shape[-1] != POLICY_OBS_DIM or self.output.shape[-1] != POLICY_ACTION_DIM:
            raise RuntimeError(
                f"Unexpected ONNX interface: input={self.input.shape}, output={self.output.shape}"
            )

    def infer(self, observation: np.ndarray) -> np.ndarray:
        observation = np.asarray(observation, dtype=np.float32)
        if observation.shape != (1, POLICY_OBS_DIM):
            raise ValueError(f"Expected observation (1, 400), got {observation.shape}")
        if not np.isfinite(observation).all():
            raise ValueError("Observation contains NaN/Inf")
        output = self.session.run(
            [self.output.name], {self.input.name: observation}
        )[0]
        action = np.asarray(output[0], dtype=np.float32)
        if action.shape != (POLICY_ACTION_DIM,) or not np.isfinite(action).all():
            raise RuntimeError("Policy returned an invalid action")
        return action


class H2RobotIO:
    """Small SDK2 adapter with CRC checking and a 500 Hz command writer."""

    def __init__(
        self,
        interface: str,
        lowcmd_dt: float,
        max_state_age: float,
        remote_estop: bool,
    ) -> None:
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as error:
            raise RuntimeError(
                "unitree_sdk2py is required for robot modes; install the official SDK first"
            ) from error

        ChannelFactoryInitialize(0, interface)
        self._MotionSwitcherClient = MotionSwitcherClient
        self._ChannelPublisher = ChannelPublisher
        self._LowCmdType = LowCmd_
        self._crc = CRC()
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._lowcmd_dt = lowcmd_dt
        self._max_state_age = max_state_age
        self._remote_estop_enabled = remote_estop

        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._snapshot: RobotSnapshot | None = None
        self._command: MotorCommandSnapshot | None = None
        self._publisher: Any | None = None
        self._writer_stop = threading.Event()
        self._writer_thread: threading.Thread | None = None
        self.writer_error: str | None = None
        self.remote_estop_latched = False

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._lowstate_handler, 1)

    def _lowstate_handler(self, message: Any) -> None:
        try:
            if message.crc != self._crc.Crc(message):
                return
            q = np.asarray(
                [message.motor_state[i].q for i in range(H2_NUM_MOTORS)],
                dtype=np.float32,
            )
            dq = np.asarray(
                [message.motor_state[i].dq for i in range(H2_NUM_MOTORS)],
                dtype=np.float32,
            )
            gyro = np.asarray(message.imu_state.gyroscope, dtype=np.float32)
            quat = np.asarray(message.imu_state.quaternion, dtype=np.float32)
            motor_errors = tuple(
                int(message.motor_state[i].motorstate) for i in range(H2_NUM_MOTORS)
            )
            remote_b = False
            try:
                button_bits = struct.unpack_from(
                    "<H", bytes(message.wireless_remote), 2
                )[0]
                remote_b = bool(button_bits & (1 << 9))
            except (TypeError, ValueError, struct.error):
                pass
            if self._remote_estop_enabled and remote_b:
                self.remote_estop_latched = True
            snapshot = RobotSnapshot(
                received_at=time.monotonic(),
                q_sdk=q,
                dq_sdk=dq,
                gyro_body=gyro,
                quat_wxyz=quat,
                mode_machine=int(message.mode_machine),
                motor_errors=motor_errors,
                remote_b_pressed=remote_b,
            )
            with self._state_lock:
                self._snapshot = snapshot
        except Exception as error:  # DDS callback must never terminate on malformed data.
            self.writer_error = f"LowState callback failed: {error}"

    def get_snapshot(self) -> RobotSnapshot | None:
        with self._state_lock:
            source = self._snapshot
            if source is None:
                return None
            return RobotSnapshot(
                received_at=source.received_at,
                q_sdk=source.q_sdk.copy(),
                dq_sdk=source.dq_sdk.copy(),
                gyro_body=source.gyro_body.copy(),
                quat_wxyz=source.quat_wxyz.copy(),
                mode_machine=source.mode_machine,
                motor_errors=source.motor_errors,
                remote_b_pressed=source.remote_b_pressed,
            )

    def wait_for_state(self, timeout: float) -> RobotSnapshot:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.get_snapshot()
            if snapshot is not None:
                return snapshot
            time.sleep(0.01)
        raise TimeoutError(f"No valid H2 LowState received within {timeout:.1f} seconds")

    def release_high_level_mode(self, max_attempts: int = 3) -> None:
        client = self._MotionSwitcherClient()
        client.SetTimeout(5.0)
        client.Init()
        for attempt in range(max_attempts):
            status, result = client.CheckMode()
            if status != 0:
                raise RuntimeError(f"MotionSwitcher CheckMode failed: status={status}")
            active_name = result.get("name", "") if result else ""
            if not active_name:
                return
            code, _ = client.ReleaseMode()
            if code != 0:
                raise RuntimeError(
                    f"MotionSwitcher ReleaseMode failed: code={code}, mode={active_name}"
                )
            print(f"Released high-level mode: {active_name} (attempt {attempt + 1})")
            time.sleep(1.0)
        status, result = client.CheckMode()
        active_name = result.get("name", "") if result else ""
        if status != 0 or active_name:
            raise RuntimeError(f"High-level mode is still active: {active_name!r}")

    def start_writer(self) -> None:
        if self._writer_thread is not None:
            return
        self._publisher = self._ChannelPublisher("rt/lowcmd", self._LowCmdType)
        self._publisher.Init()
        self._writer_stop.clear()
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="h2-lowcmd-500hz", daemon=True
        )
        self._writer_thread.start()

    def set_command(
        self,
        q_sdk: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
        mode_machine: int,
        active_position_control: bool,
    ) -> None:
        q_sdk = np.asarray(q_sdk, dtype=np.float32)
        kp = np.asarray(kp, dtype=np.float32)
        kd = np.asarray(kd, dtype=np.float32)
        if q_sdk.shape != (31,) or kp.shape != (31,) or kd.shape != (31,):
            raise ValueError("LowCmd q/kp/kd must all have shape (31,)")
        command = MotorCommandSnapshot(
            q_sdk=q_sdk.copy(), kp=kp.copy(), kd=kd.copy(),
            mode_machine=int(mode_machine),
            active_position_control=active_position_control,
        )
        with self._command_lock:
            self._command = command

    def set_passive(self, snapshot: RobotSnapshot | None = None) -> None:
        if snapshot is None:
            snapshot = self.get_snapshot()
        if snapshot is None:
            return
        self.set_command(
            q_sdk=np.zeros(31, dtype=np.float32),
            kp=np.zeros(31, dtype=np.float32),
            kd=PASSIVE_KD,
            mode_machine=snapshot.mode_machine,
            active_position_control=False,
        )

    def _get_command(self) -> MotorCommandSnapshot | None:
        with self._command_lock:
            source = self._command
            if source is None:
                return None
            return MotorCommandSnapshot(
                q_sdk=source.q_sdk.copy(), kp=source.kp.copy(), kd=source.kd.copy(),
                mode_machine=source.mode_machine,
                active_position_control=source.active_position_control,
            )

    def _writer_loop(self) -> None:
        assert self._publisher is not None
        deadline = time.monotonic()
        try:
            while not self._writer_stop.is_set():
                snapshot = self.get_snapshot()
                command = self._get_command()
                if snapshot is not None and command is not None:
                    # A stale state always overrides an old position command with damping.
                    if time.monotonic() - snapshot.received_at > self._max_state_age:
                        q = np.zeros(31, dtype=np.float32)
                        kp = np.zeros(31, dtype=np.float32)
                        kd = PASSIVE_KD
                        active = False
                    else:
                        q, kp, kd = command.q_sdk, command.kp, command.kd
                        active = command.active_position_control
                    self._low_cmd.mode_pr = 0  # H2 PR ankle/waist representation.
                    self._low_cmd.mode_machine = snapshot.mode_machine
                    for index in range(H2_NUM_MOTORS):
                        motor = self._low_cmd.motor_cmd[index]
                        motor.mode = 1
                        motor.tau = 0.0
                        motor.q = float(q[index]) if active else 0.0
                        motor.dq = 0.0
                        motor.kp = float(kp[index]) if active else 0.0
                        motor.kd = float(kd[index])
                    self._low_cmd.crc = self._crc.Crc(self._low_cmd)
                    self._publisher.Write(self._low_cmd)
                deadline += self._lowcmd_dt
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    deadline = time.monotonic()
        except Exception as error:
            self.writer_error = f"LowCmd writer failed: {error}"

    def stop_writer(self, passive_grace: float = 0.5) -> None:
        if self._writer_thread is None:
            return
        self.set_passive()
        time.sleep(max(0.0, passive_grace))
        self._writer_stop.set()
        self._writer_thread.join(timeout=2.0)
        self._writer_thread = None


def build_frame(
    snapshot: RobotSnapshot,
    command: np.ndarray,
    last_action: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    q_policy = snapshot.q_sdk[OBS_TO_SDK]
    dq_policy = snapshot.dq_sdk[OBS_TO_SDK]
    gravity = quat_rotate_inverse_wxyz(
        snapshot.quat_wxyz, (0.0, 0.0, -1.0)
    )
    frame = np.concatenate(
        (
            snapshot.gyro_body,
            gravity,
            command,
            q_policy - DEFAULT_Q_31,
            dq_policy,
            last_action,
        )
    ).astype(np.float32)
    if frame.shape != (FRAME_OBS_DIM,) or not np.isfinite(frame).all():
        raise RuntimeError(f"Invalid frame observation: shape={frame.shape}")
    return frame, gravity


def validate_snapshot(
    snapshot: RobotSnapshot,
    max_state_age: float,
    min_upright: float | None,
) -> tuple[bool, str, np.ndarray | None]:
    age = time.monotonic() - snapshot.received_at
    if age > max_state_age:
        return False, f"LowState stale ({age * 1000:.1f} ms)", None
    arrays = (snapshot.q_sdk, snapshot.dq_sdk, snapshot.gyro_body, snapshot.quat_wxyz)
    if not all(np.isfinite(value).all() for value in arrays):
        return False, "LowState contains NaN/Inf", None
    bad_motors = [i for i, code in enumerate(snapshot.motor_errors) if code != 0]
    if bad_motors:
        return False, f"motor errors at indices {bad_motors}", None
    try:
        gravity = quat_rotate_inverse_wxyz(snapshot.quat_wxyz, (0.0, 0.0, -1.0))
    except ValueError as error:
        return False, str(error), None
    if min_upright is not None and float(-gravity[2]) < min_upright:
        return False, f"upright={float(-gravity[2]):.3f} below {min_upright:.3f}", gravity
    outside_limits = np.flatnonzero(
        (snapshot.q_sdk < SDK_Q_MIN - 0.10) | (snapshot.q_sdk > SDK_Q_MAX + 0.10)
    )
    if outside_limits.size:
        return False, f"joint states outside model limits: {outside_limits.tolist()}", gravity
    return True, "", gravity


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def limit_target(
    desired: np.ndarray,
    previous: np.ndarray,
    dt: float,
    max_speed: float,
    limit_margin: float,
) -> tuple[np.ndarray, bool]:
    desired = np.asarray(desired, dtype=np.float32)
    limited = np.clip(desired, SDK_Q_MIN + limit_margin, SDK_Q_MAX - limit_margin)
    saturated = not np.array_equal(limited, desired)
    if max_speed > 0:
        max_delta = max_speed * dt
        slew_limited = np.clip(limited, previous - max_delta, previous + max_delta)
        saturated = saturated or not np.array_equal(slew_limited, limited)
        limited = slew_limited
    return limited.astype(np.float32), saturated


def run_self_test(model_path: str, trace_path: str | None) -> int:
    if len(set(OBS_TO_SDK.tolist())) != H2_NUM_MOTORS:
        raise RuntimeError("OBS_TO_SDK is not a 31-index permutation")
    marker = np.arange(H2_NUM_MOTORS, dtype=np.float32)
    if not np.array_equal(policy_to_sdk_q(marker)[OBS_TO_SDK], marker):
        raise RuntimeError("Policy/SDK joint mapping round-trip failed")
    policy = OnnxPolicy(model_path)
    zero_action = policy.infer(np.zeros((1, POLICY_OBS_DIM), dtype=np.float32))
    print(
        "SELF_TEST_INTERFACE "
        f"input={policy.input.name}:{policy.input.shape} "
        f"output={policy.output.name}:{policy.output.shape}"
    )
    print(
        "SELF_TEST_ZERO "
        f"action_min={float(zero_action.min()):.6f} "
        f"action_max={float(zero_action.max()):.6f}"
    )
    if trace_path:
        trace = np.load(os.path.abspath(trace_path), allow_pickle=False)
        if "onnx_obs" not in trace.files or "action" not in trace.files:
            raise RuntimeError("Trace must contain onnx_obs and action")
        observations = np.asarray(trace["onnx_obs"], dtype=np.float32)
        expected = np.asarray(trace["action"], dtype=np.float32)
        predicted = policy.session.run(
            [policy.output.name], {policy.input.name: observations}
        )[0].astype(np.float32)
        delta = np.abs(predicted - expected)
        print(
            "SELF_TEST_TRACE "
            f"steps={len(observations)} max_abs={float(delta.max()):.9g} "
            f"mean_abs={float(delta.mean()):.9g}"
        )
        if not np.allclose(predicted, expected, rtol=1.0e-5, atol=1.0e-6):
            return 2
        target_keys = {"joint_names", "action_joint_names", "joint_target"}
        if target_keys.issubset(trace.files):
            joint_names = [str(name) for name in trace["joint_names"]]
            action_names = [str(name) for name in trace["action_joint_names"]]
            target_ids = [joint_names.index(name) for name in action_names]
            expected_target = np.asarray(trace["joint_target"][:, target_ids])
            predicted_target = ACTION_DEFAULT_Q[None, :] + ACTION_SCALE[None, :] * expected
            target_delta = np.abs(predicted_target - expected_target)
            print(
                "SELF_TEST_TARGET "
                f"max_abs={float(target_delta.max()):.9g} "
                f"mean_abs={float(target_delta.mean()):.9g}"
            )
            if not np.allclose(predicted_target, expected_target, rtol=1.0e-6, atol=1.0e-7):
                return 3
    identity_gravity = quat_rotate_inverse_wxyz((1, 0, 0, 0), (0, 0, -1))
    if not np.allclose(identity_gravity, (0, 0, -1), atol=1.0e-6):
        raise RuntimeError("Quaternion self-test failed")
    print("SELF_TEST_OK")
    return 0


def print_observation_status(snapshot: RobotSnapshot) -> None:
    gravity = quat_rotate_inverse_wxyz(snapshot.quat_wxyz, (0, 0, -1))
    age_ms = (time.monotonic() - snapshot.received_at) * 1000
    errors = [i for i, code in enumerate(snapshot.motor_errors) if code]
    print(
        "H2_OBSERVE "
        f"age_ms={age_ms:.2f} upright={float(-gravity[2]):.4f} "
        f"gravity={gravity[0]:.4f},{gravity[1]:.4f},{gravity[2]:.4f} "
        f"gyro={snapshot.gyro_body[0]:.4f},{snapshot.gyro_body[1]:.4f},"
        f"{snapshot.gyro_body[2]:.4f} "
        f"q_min={float(snapshot.q_sdk.min()):.3f} "
        f"q_max={float(snapshot.q_sdk.max()):.3f} errors={errors}"
    )


def require_low_level_confirmation(args: argparse.Namespace) -> None:
    if not args.enable_low_level or not args.confirm_suspended:
        raise RuntimeError(
            f"Mode {args.mode!r} publishes motor commands. Securely suspend the robot, "
            "place an operator at the physical emergency stop, then pass both "
            "--enable-low-level and --confirm-suspended."
        )


def transition_to_ready(
    robot: H2RobotIO,
    stop_event: threading.Event,
    ready_q: np.ndarray,
    duration: float,
    kp: np.ndarray,
    kd: np.ndarray,
    max_state_age: float,
) -> np.ndarray:
    initial = robot.wait_for_state(2.0)
    start_q = initial.q_sdk.copy()
    started_at = time.monotonic()
    while not stop_event.is_set():
        snapshot = robot.get_snapshot()
        if snapshot is None:
            continue
        valid, reason, _ = validate_snapshot(snapshot, max_state_age, min_upright=None)
        if not valid:
            raise RuntimeError(f"Ready transition safety fault: {reason}")
        elapsed = time.monotonic() - started_at
        ratio = smoothstep(elapsed / duration)
        target = (1.0 - ratio) * start_q + ratio * ready_q
        target = np.clip(target, SDK_Q_MIN + 0.02, SDK_Q_MAX - 0.02)
        robot.set_command(target, kp, kd, snapshot.mode_machine, True)
        if elapsed >= duration:
            print("H2_READY_COMPLETE")
            return target.astype(np.float32)
        time.sleep(0.01)
    raise InterruptedError("Ready transition interrupted")


def run_robot(args: argparse.Namespace) -> int:
    robot = H2RobotIO(
        args.interface,
        args.lowcmd_dt,
        args.max_state_age,
        not args.disable_remote_estop,
    )
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(f"Waiting for H2 LowState on interface {args.interface!r}...")
    snapshot = robot.wait_for_state(args.state_wait_timeout)
    print_observation_status(snapshot)

    if args.mode == "observe":
        started_at = time.monotonic()
        next_print = started_at
        while not stop_event.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - started_at >= args.duration:
                break
            if now >= next_print:
                current = robot.get_snapshot()
                if current is not None:
                    print_observation_status(current)
                next_print += args.print_interval
            time.sleep(0.02)
        return 0

    require_low_level_confirmation(args)
    print("LOW_LEVEL_WARNING robot must remain suspended; remote B is latched E-stop")
    robot.release_high_level_mode()
    robot.set_passive(snapshot)
    robot.start_writer()

    gain_scale = float(args.gain_scale)
    kp = TRAINING_KP * gain_scale
    kd = TRAINING_KD * gain_scale
    ready_q = (
        DEFAULT_Q_SDK.copy()
        if args.ready_pose == "default"
        else VALIDATED_MOTION_READY_Q_SDK.copy()
    )
    print(f"H2_READY_POSE source={args.ready_pose}")
    started_at = time.monotonic()

    try:
        if args.mode == "passive":
            while not stop_event.is_set():
                if args.duration > 0 and time.monotonic() - started_at >= args.duration:
                    break
                if robot.writer_error:
                    raise RuntimeError(robot.writer_error)
                time.sleep(0.02)
            return 0

        current_target = transition_to_ready(
            robot, stop_event, ready_q, args.ready_duration, kp, kd,
            args.max_state_age,
        )
        if args.mode == "ready":
            hold_started = time.monotonic()
            while not stop_event.is_set():
                snapshot = robot.get_snapshot()
                if snapshot is None:
                    continue
                valid, reason, _ = validate_snapshot(
                    snapshot, args.max_state_age, min_upright=None
                )
                if not valid:
                    raise RuntimeError(f"Ready hold safety fault: {reason}")
                robot.set_command(current_target, kp, kd, snapshot.mode_machine, True)
                if args.duration > 0 and time.monotonic() - hold_started >= args.duration:
                    break
                time.sleep(0.02)
            return 0

        assert args.mode == "policy"
        policy = OnnxPolicy(args.model)
        history: deque[np.ndarray] = deque(maxlen=HISTORY_LENGTH)
        last_action = np.zeros(POLICY_ACTION_DIM, dtype=np.float32)
        command = np.zeros(3, dtype=np.float32)
        desired_command = np.asarray([args.vx, args.vy, args.wz], dtype=np.float32)
        initial_snapshot = robot.wait_for_state(1.0)
        initial_frame, _ = build_frame(initial_snapshot, command, last_action)
        history.extend(initial_frame.copy() for _ in range(HISTORY_LENGTH))

        policy_started = time.monotonic()
        next_tick = policy_started
        next_print = policy_started
        inference_max_ms = 0.0
        saturation_count = 0
        completed_steps = 0
        while not stop_event.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - policy_started >= args.duration:
                break
            if robot.remote_estop_latched:
                raise RuntimeError("Remote B emergency stop latched")
            if robot.writer_error:
                raise RuntimeError(robot.writer_error)
            snapshot = robot.get_snapshot()
            if snapshot is None:
                raise RuntimeError("LowState disappeared")
            valid, reason, gravity = validate_snapshot(
                snapshot, args.max_state_age, args.min_upright
            )
            if not valid:
                raise RuntimeError(f"Policy safety fault: {reason}")

            max_command_delta = np.asarray(
                [args.command_accel, args.command_accel, args.yaw_accel],
                dtype=np.float32,
            ) * args.policy_dt
            command += np.clip(
                desired_command - command, -max_command_delta, max_command_delta
            )
            frame, gravity = build_frame(snapshot, command, last_action)
            history.append(frame)
            observation = np.concatenate(tuple(history)).reshape(1, POLICY_OBS_DIM)
            inference_started = time.monotonic()
            action = policy.infer(observation)
            inference_ms = (time.monotonic() - inference_started) * 1000
            inference_max_ms = max(inference_max_ms, inference_ms)
            if inference_ms > args.inference_timeout * 1000:
                raise RuntimeError(f"ONNX inference deadline miss: {inference_ms:.2f} ms")

            target_29 = ACTION_DEFAULT_Q + ACTION_SCALE * action
            desired_target = current_target.copy()
            desired_target[ACTION_TO_SDK] = target_29
            desired_target[29:31] = DEFAULT_Q_SDK[29:31]
            current_target, saturated = limit_target(
                desired_target, current_target, args.policy_dt,
                args.max_target_speed, args.joint_limit_margin,
            )
            saturation_count += int(saturated)
            robot.set_command(current_target, kp, kd, snapshot.mode_machine, True)
            # Match Isaac: observation contains the raw previous ONNX action.
            last_action = action.copy()
            completed_steps += 1

            if now >= next_print:
                tracking_error = float(np.max(np.abs(current_target - snapshot.q_sdk)))
                if (
                    now - policy_started >= args.tracking_error_grace
                    and tracking_error > args.max_tracking_error
                ):
                    raise RuntimeError(
                        f"joint tracking error {tracking_error:.3f} rad exceeds "
                        f"{args.max_tracking_error:.3f} rad"
                    )
                print(
                    "H2_POLICY "
                    f"t={now - policy_started:.2f}s step={completed_steps} "
                    f"cmd={command[0]:.3f},{command[1]:.3f},{command[2]:.3f} "
                    f"upright={float(-gravity[2]):.3f} "
                    f"infer_ms={inference_ms:.3f} max_infer_ms={inference_max_ms:.3f} "
                    f"tracking_error={tracking_error:.3f} saturations={saturation_count}"
                )
                next_print += args.print_interval

            next_tick += args.policy_dt
            remaining = next_tick - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                lateness = -remaining
                if lateness > args.policy_dt:
                    raise RuntimeError(f"Policy loop deadline miss: {lateness * 1000:.2f} ms")
                next_tick = time.monotonic()
        print(
            f"H2_POLICY_COMPLETE steps={completed_steps} max_infer_ms={inference_max_ms:.3f}"
        )
        return 0
    except InterruptedError:
        return 0
    finally:
        print("Entering passive damping before shutdown...")
        robot.stop_writer(args.passive_grace)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    colocated_model = script_dir / "policy.onnx"
    repository_model = (
        script_dir
        / "AMP_mjlab/logs/rsl_rl/h2_amp_locomotion/"
        "2026-08-14_08-46-32_resume_to_20000/policy.onnx"
    )
    default_model = str(colocated_model if colocated_model.exists() else repository_model)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("observe", "passive", "ready", "policy", "self-test"),
        default="observe",
    )
    parser.add_argument("--interface", default="eth0", help="H2 DDS network interface")
    parser.add_argument("--model", default=default_model)
    parser.add_argument("--trace", help="Isaac NPZ trace for self-test mode")
    parser.add_argument("--duration", type=float, default=10.0, help="0 runs until stopped")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--policy-dt", type=float, default=0.02)
    parser.add_argument("--lowcmd-dt", type=float, default=0.002)
    parser.add_argument("--ready-duration", type=float, default=5.0)
    parser.add_argument(
        "--ready-pose",
        choices=("default", "validated-motion"),
        default="default",
        help=(
            "default is the conservative first test; validated-motion is the exact "
            "minimum-velocity motion pose used to initialize Isaac validation"
        ),
    )
    parser.add_argument(
        "--gain-scale", type=float, default=0.35,
        help="Scale training Kp/Kd; start suspended at 0.35 and tune deliberately",
    )
    parser.add_argument("--command-accel", type=float, default=1.5)
    parser.add_argument("--yaw-accel", type=float, default=3.0)
    parser.add_argument("--max-target-speed", type=float, default=12.0)
    parser.add_argument("--joint-limit-margin", type=float, default=0.02)
    parser.add_argument("--max-state-age", type=float, default=0.05)
    parser.add_argument("--max-tracking-error", type=float, default=1.0)
    parser.add_argument("--tracking-error-grace", type=float, default=2.0)
    parser.add_argument("--state-wait-timeout", type=float, default=5.0)
    parser.add_argument("--inference-timeout", type=float, default=0.02)
    parser.add_argument("--min-upright", type=float, default=0.5)
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument("--passive-grace", type=float, default=0.5)
    parser.add_argument("--disable-remote-estop", action="store_true")
    parser.add_argument("--enable-low-level", action="store_true")
    parser.add_argument("--confirm-suspended", action="store_true")
    args = parser.parse_args()

    positive = {
        "policy_dt": args.policy_dt,
        "lowcmd_dt": args.lowcmd_dt,
        "ready_duration": args.ready_duration,
        "max_state_age": args.max_state_age,
        "state_wait_timeout": args.state_wait_timeout,
        "inference_timeout": args.inference_timeout,
        "print_interval": args.print_interval,
        "max_tracking_error": args.max_tracking_error,
    }
    invalid = [name for name, value in positive.items() if value <= 0]
    if invalid:
        parser.error(f"These options must be positive: {', '.join(invalid)}")
    if args.duration < 0 or args.passive_grace < 0 or args.tracking_error_grace < 0:
        parser.error(
            "--duration, --passive-grace, and --tracking-error-grace must be non-negative"
        )
    if not 0 < args.gain_scale <= 1.0:
        parser.error("--gain-scale must be in (0, 1]")
    if not 0 < args.min_upright <= 1.0:
        parser.error("--min-upright must be in (0, 1]")
    command = np.asarray([args.vx, args.vy, args.wz], dtype=np.float32)
    if not np.isfinite(command).all():
        parser.error("velocity commands must be finite")
    command_limits = np.asarray([1.5, 1.0, 1.57], dtype=np.float32)
    if np.any(np.abs(command) > command_limits):
        parser.error("commands exceed limits: |vx|<=1.5, |vy|<=1.0, |wz|<=1.57")
    return args


def main() -> int:
    args = parse_args()
    try:
        if args.mode == "self-test":
            return run_self_test(args.model, args.trace)
        return run_robot(args)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"[FATAL] {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
