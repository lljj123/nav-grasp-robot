#!/usr/bin/env python3
"""Interactively zero and verify the PiPER AGX gripper with the ROS driver off."""

from __future__ import annotations

import argparse
import math
import sys
import time

from pyAgxArm import (
    AgxArmFactory,
    create_agx_arm_config,
    resolve_firmware_profile,
)


def wait_status(gripper, predicate, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = gripper.get_gripper_status()
        if latest is not None and predicate(latest.msg):
            return latest
        time.sleep(0.05)
    return latest


def describe(status) -> str:
    if status is None:
        return 'no feedback'
    message = status.msg
    return (
        f'width={message.value:.4f} m, force={message.force:.3f} N, '
        f'mode={message.mode}, '
        f'enabled={message.foc_status.driver_enable_status}, '
        f'homed={message.foc_status.homing_status}')


def has_fault(message) -> bool:
    foc = message.foc_status
    return any((
        foc.voltage_too_low,
        foc.motor_overheating,
        foc.driver_overcurrent,
        foc.driver_overheating,
        foc.sensor_status,
        foc.driver_error_status,
    ))


def connect_with_detected_firmware(arm_type: str, can_port: str):
    """Match the two-pass firmware selection used by the official ROS driver."""
    initial_config = create_agx_arm_config(
        robot=arm_type,
        comm='can',
        channel=can_port,
    )
    initial_robot = AgxArmFactory.create_arm(initial_config)
    initial_robot.connect()
    try:
        deadline = time.monotonic() + 10.0
        firmware = None
        while time.monotonic() < deadline:
            firmware = initial_robot.get_firmware()
            if firmware:
                break
            time.sleep(0.1)
        if not firmware:
            raise TimeoutError('10秒内未读取到机械臂固件版本')
        software_version = firmware['software_version']
        profile = resolve_firmware_profile(arm_type, software_version)
    finally:
        initial_robot.disconnect()

    detected_config = create_agx_arm_config(
        robot=arm_type,
        comm='can',
        channel=can_port,
        firmeware_version=profile,
    )
    robot = AgxArmFactory.create_arm(detected_config)
    robot.connect()
    return robot, software_version, profile


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Zero the closed PiPER gripper and verify a 10 mm opening.')
    parser.add_argument('--can-port', default='can0')
    parser.add_argument('--arm-type', default='piper_h')
    parser.add_argument(
        '--max-range-m', type=float, choices=(0.07, 0.1), default=0.07)
    parser.add_argument('--verify-width-m', type=float, default=0.010)
    arguments = parser.parse_args()
    if not math.isfinite(arguments.verify_width_m) or not (
            0.005 <= arguments.verify_width_m <= 0.020):
        parser.error('--verify-width-m must be between 0.005 and 0.020')

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
            print('错误：3秒内没有收到夹爪反馈。', file=sys.stderr)
            return 1
        print('当前夹爪：' + describe(status))
        if has_fault(status.msg):
            print('错误：夹爪报告硬件故障，禁止回零。', file=sys.stderr)
            return 1

        gripper.disable_gripper()
        status = wait_status(
            gripper,
            lambda message: not message.foc_status.driver_enable_status,
            3.0,
        )
        if status is None or status.msg.foc_status.driver_enable_status:
            print('错误：无法禁用夹爪。', file=sys.stderr)
            return 1
        print('夹爪已禁用，可以用手轻轻将两指完全合拢。')

        configured = gripper.set_gripper_teaching_pendant_param(
            teaching_range_per=100,
            max_range_config=arguments.max_range_m,
            teaching_friction=1,
            timeout=2.0,
        )
        if not configured:
            print('错误：设置夹爪最大行程失败。', file=sys.stderr)
            return 1
        print(f'最大行程已设置为 {arguments.max_range_m:.2f} m。')

        confirmation = input(
            '确认两指已经完全合拢且中间没有物体，然后输入 CALIBRATE：').strip()
        if confirmation != 'CALIBRATE':
            print('未确认，已取消；没有修改夹爪零点。')
            return 2
        if not gripper.calibrate_gripper(timeout=2.0):
            print('错误：夹爪零位标定没有收到成功确认。', file=sys.stderr)
            return 1

        # Zeroing is performed while the gripper is disabled.  On PiPER-H the
        # FOC homing bit is refreshed only after a subsequent motion command
        # re-enables the gripper, so verify both facts after a small opening.
        print('零位指令已收到成功确认，正在重新使能并验证10 mm张开。')
        gripper.move_gripper_m(arguments.verify_width_m, force=1.0)
        status = wait_status(
            gripper,
            lambda message: (
                message.foc_status.homing_status
                and message.foc_status.driver_enable_status
                and abs(message.value - arguments.verify_width_m) <= 0.002
            ),
            10.0,
        )
        if status is None:
            print('错误：验证阶段没有收到夹爪反馈。', file=sys.stderr)
            return 1
        width_reached = abs(
            status.msg.value - arguments.verify_width_m) <= 0.002
        if not status.msg.foc_status.homing_status:
            print(
                '错误：重新使能后 homing_status 仍为 false：' + describe(status),
                file=sys.stderr,
            )
            return 1
        if not status.msg.foc_status.driver_enable_status:
            print('错误：验证阶段夹爪没有重新使能：' + describe(status), file=sys.stderr)
            return 1
        if not width_reached:
            print(
                '错误：标定成功，但10 mm张开验证未到位：' + describe(status),
                file=sys.stderr,
            )
            return 1
        print('零位和张开验证成功：' + describe(status))
        print('现在可以重新启动官方AGX驱动和视觉抓取节点。')
        return 0
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
