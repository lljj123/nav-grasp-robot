"""Check Python style with the ROS 2 ament flake8 configuration."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Run flake8 and report every discovered style error."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, (
        'Found %d code style errors / warnings:\n' % len(errors)
        + '\n'.join(errors)
    )
