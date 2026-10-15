"""Validation for manually measured gripper travel used as a homing fallback."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class VerifiedGripperRange:
    closed_width_m: float
    open_width_m: float
    span_m: float
    firmware: str
    measured_at: str


def _finite_number(report: dict[str, Any], key: str) -> float:
    try:
        value = float(report[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'gripper range report has invalid {key}') from exc
    if not math.isfinite(value):
        raise ValueError(f'gripper range report has non-finite {key}')
    return value


def validate_gripper_range_report(
        report: dict[str, Any], *, expected_firmware: str,
        closed_abs_limit_m: float, min_span_m: float, max_span_m: float,
        max_jitter_m: float) -> VerifiedGripperRange:
    closed = _finite_number(report, 'closed_width_m')
    opened = _finite_number(report, 'open_width_m')
    stated_span = _finite_number(report, 'absolute_span_m')
    closed_jitter = _finite_number(report, 'closed_jitter_m')
    open_jitter = _finite_number(report, 'open_jitter_m')
    calculated_span = opened - closed

    firmware = str(report.get('firmware', ''))
    if firmware != expected_firmware:
        raise ValueError(
            f'gripper range report firmware={firmware!r}, '
            f'expected {expected_firmware!r}')
    if report.get('encoder_direction') != 'increases_when_opening':
        raise ValueError('gripper encoder does not increase when opening')
    if abs(closed) > closed_abs_limit_m:
        raise ValueError(
            f'gripper closed endpoint {closed:.6f} m exceeds '
            f'+/-{closed_abs_limit_m:.6f} m')
    if not min_span_m <= calculated_span <= max_span_m:
        raise ValueError(
            f'gripper measured span {calculated_span:.6f} m is outside '
            f'[{min_span_m:.6f}, {max_span_m:.6f}] m')
    if abs(stated_span - calculated_span) > 0.0005:
        raise ValueError('gripper range report span is internally inconsistent')
    if closed_jitter > max_jitter_m or open_jitter > max_jitter_m:
        raise ValueError(
            f'gripper endpoint feedback is unstable: closed jitter '
            f'{closed_jitter:.6f} m, open jitter {open_jitter:.6f} m')

    return VerifiedGripperRange(
        closed_width_m=closed,
        open_width_m=opened,
        span_m=calculated_span,
        firmware=firmware,
        measured_at=str(report.get('measured_at', '')),
    )


def load_gripper_range_report(path: str, **validation) -> VerifiedGripperRange:
    report_path = Path(path).expanduser()
    try:
        report = json.loads(report_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(
            f'cannot read verified gripper range report {report_path}: {exc}') from exc
    if not isinstance(report, dict):
        raise ValueError('gripper range report root must be an object')
    return validate_gripper_range_report(report, **validation)
