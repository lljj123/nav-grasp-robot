import json

import pytest

from visual_grasp_motion.gripper_safety import (
    load_gripper_range_report,
    validate_gripper_range_report,
)


def valid_report():
    return {
        'measured_at': '2026-10-10T16:24:07+08:00',
        'firmware': 'S-V1.9-1',
        'closed_width_m': -0.00007,
        'open_width_m': 0.07973,
        'absolute_span_m': 0.0798,
        'encoder_direction': 'increases_when_opening',
        'closed_jitter_m': 0.0,
        'open_jitter_m': 0.0,
    }


def validation_kwargs():
    return {
        'expected_firmware': 'S-V1.9-1',
        'closed_abs_limit_m': 0.002,
        'min_span_m': 0.070,
        'max_span_m': 0.085,
        'max_jitter_m': 0.001,
    }


def test_accepts_measured_gripper_range():
    result = validate_gripper_range_report(valid_report(), **validation_kwargs())
    assert result.closed_width_m == pytest.approx(-0.00007)
    assert result.open_width_m == pytest.approx(0.07973)
    assert result.span_m == pytest.approx(0.0798)


@pytest.mark.parametrize('change, message', [
    ({'encoder_direction': 'decreases_when_opening'}, 'does not increase'),
    ({'open_width_m': 0.050}, 'outside'),
    ({'closed_width_m': -0.010}, 'closed endpoint'),
    ({'open_jitter_m': 0.002}, 'unstable'),
    ({'firmware': 'S-V1.8-6'}, 'firmware'),
])
def test_rejects_invalid_range(change, message):
    report = valid_report()
    report.update(change)
    with pytest.raises(ValueError, match=message):
        validate_gripper_range_report(report, **validation_kwargs())


def test_loads_report_file(tmp_path):
    path = tmp_path / 'range.json'
    path.write_text(json.dumps(valid_report()), encoding='utf-8')
    result = load_gripper_range_report(str(path), **validation_kwargs())
    assert result.span_m == pytest.approx(0.0798)
