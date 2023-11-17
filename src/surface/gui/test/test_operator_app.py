import pytest
import gui
from pytestqt.qtbot import QtBot


@pytest.fixture
def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for OperatorApp instantiation."""
    app = gui.operator_app.OperatorApp()
    app.show()
    qtbot.addWiget(app)
