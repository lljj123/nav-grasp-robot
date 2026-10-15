#!/usr/bin/env python3
"""Measure AGX gripper encoder endpoints while the gripper is disabled."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time

from calibrate_gripper import (
    connect_with_detected_firmware,
    describe,
    has_fault,
    wait_status,
)


def sample_width(gripper, duration_s: float) -> tuple[float, float, int]:
    deadline = time.monotonic() + duration_s
    values: list[float] = []
    while time.monotonic() < deadline:
        status = gripper.get_gripper_status()
        if status is not None:
            if has_fault(status.msg):
                raise RuntimeError('夹爪反馈出现硬件故障：' + describe(status))
            value = float(status.msg.value)
            if math.isfinite(value):
                values.append(value)
        time.sleep(0.02)
    if not values:
        raise TimeoutError('采样期间没有收到有效夹爪反馈')
    return statistics.median(values), max(values) - min(values), len(values)


def require_enter(prompt: str) -> None:
    if input(prompt).strip():
        raise RuntimeError('端点确认只需直接按回车；检测到其他输入，测量已取消')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Disable the gripper and manually measure closed/open endpoints.')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--arm-type', default='piper_h')
    parser.add_argument('--sample-seconds', type=float, default=1.0)
    parser.add_argument(
        '--output',
        default='~/.config/nav_grasp/gripper/manual_gripper_range.json',
        help='Path for the machine-specific validation report.',
    )
    arguments = parser.parse_args()
    if not math.isfinite(arguments.sample_seconds) or not (
            0.5 <= arguments.sample_seconds <= 5.0):
        parser.error('--sample-seconds must be between 0.5 and 5.0')

    robot = None
    try:
        robot, software_version, profile = connect_with_detected_firmware(
            arguments.arm_type, arguments.can_port)
        print(
            f'已连接 {arguments.arm_type}，固件={software_version}，'
            f'SDK协议档位={profile}')
        gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        status = wait_status(gripper, lambda _message: True, 3.0)
        if status is None:
            raise TimeoutError('3秒内没有收到夹爪反馈')
        if has_fault(status.msg):
            raise RuntimeError('夹爪存在硬件故障：' + describe(status))
        print('当前夹爪：' + describe(status))

        gripper.disable_gripper()
        status = wait_status(
            gripper,
            lambda message: not message.foc_status.driver_enable_status,
            3.0,
        )
        if status is None or status.msg.foc_status.driver_enable_status:
            raise RuntimeError('无法禁用夹爪，禁止用手推动')
        print('夹爪已禁用；本工具不会发送任何运动目标，也不会修改零点。')

        require_enter(
            '用手轻轻将两指移动到完全闭合，保持不动，然后直接按回车：')
        closed, closed_jitter, closed_samples = sample_width(
            gripper, arguments.sample_seconds)
        print(
            f'闭合端：{closed:.6f} m，采样波动={closed_jitter:.6f} m，'
            f'样本={closed_samples}')

        require_enter(
            '用手轻轻将两指移动到完全张开，保持不动，然后直接按回车：')
        opened, open_jitter, open_samples = sample_width(
            gripper, arguments.sample_seconds)
        print(
            f'张开端：{opened:.6f} m，采样波动={open_jitter:.6f} m，'
            f'样本={open_samples}')

        span = opened - closed
        report = {
            'measured_at': datetime.now().astimezone().isoformat(),
            'arm_type': arguments.arm_type,
            'firmware': software_version,
            'sdk_profile': str(profile),
            'closed_width_m': closed,
            'open_width_m': opened,
            'signed_span_m': span,
            'absolute_span_m': abs(span),
            'encoder_direction': 'increases_when_opening' if span > 0 else (
                'decreases_when_opening' if span < 0 else 'no_change'),
            'closed_jitter_m': closed_jitter,
            'open_jitter_m': open_jitter,
            'homing_status_before_measurement': bool(
                status.msg.foc_status.homing_status),
        }
        output_path = Path(arguments.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(f'实测行程：{abs(span):.6f} m ({abs(span) * 1000:.2f} mm)')
        print('编码器方向：' + report['encoder_direction'])
        print(f'报告已保存：{output_path}')
        print('夹爪保持禁用；重新启动机械臂控制驱动后才会重新使能。')
        return 0 if abs(span) >= 0.03 else 1
    except (RuntimeError, TimeoutError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
