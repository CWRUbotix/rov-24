import pytest
import gui
from pytestqt.qtbot import QtBot


@pytest.fixture
def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for PilotApp instantiation."""
    app = gui.operator_app.PilotApp()
    app.show()
    qtbot.addWiget(app)
