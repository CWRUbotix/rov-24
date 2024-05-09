"""Test cpplint on this module."""

import os
import shutil
from typing import Literal

import pytest
from ament_cpplint.main import main

INO_EXTENSION = ".ino"
CPP_EXTENSION = ".cpp"
SKETCHES = ["float_transceiver", "surface_transceiver"]
TMP = "/tmp"
SRC = "src"
INCLUDE = "include"

# List of errors from cpplint.py
LEGAL_ERROR = "-legal/copyright"


@pytest.mark.linter
def test_cpplint() -> None:
    """Tests cpplint on this module."""

    try:
        for folder in SKETCHES:
            shutil.rmtree(os.path.join(TMP, folder))
    except FileNotFoundError:
        pass

    for folder in SKETCHES:
        # Copies folders
        shutil.copytree(os.path.join(os.getcwd(), folder), os.path.join(TMP, folder))
        # Renames files
        os.rename(os.path.join(TMP, folder, folder + INO_EXTENSION),
                  os.path.join(TMP, folder, folder + CPP_EXTENSION))

    regular_cpp = [
        *[os.path.join(os.getcwd(), folder) for folder in [SRC, INCLUDE]],
        *[os.path.join(TMP, folder, folder + CPP_EXTENSION) for folder in SKETCHES]
    ]

    error_codes: list[Literal[0, 1]] = []

    for file in regular_cpp:
        error_code = main(argv=["paths", file, f'--filters={LEGAL_ERROR}'])
        error_codes.append(error_code)

    for code in error_codes:
        assert code == 0, 'Found code style errors / warnings'
