"""Test mypy on this module."""

import pytest
from ament_mypy.main import main


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy() -> None:
    """Tests mypy on this module."""
    error_code = main(argv=[])
    hi = "ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt"
    assert error_code == 0, "Found code style errors / warnings"
