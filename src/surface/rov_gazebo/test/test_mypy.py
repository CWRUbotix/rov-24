from ament_mypy.main import main
import os
import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy() -> None:
    file_path = __file__.replace(f'{__name__}.py', '')
    config_file = os.path.join(file_path, '..', '..', '..', '..', 'mypy.ini')
    rc = main(argv=['--config', config_file])
    assert rc == 0, 'Found code style errors / warnings'
