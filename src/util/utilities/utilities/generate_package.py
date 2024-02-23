import os
import shutil
import sys
from glob import glob

from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger

HOME_PACKAGE = "utilities"
PACKAGE_DIR = get_package_share_directory(HOME_PACKAGE)
TEST_INCLUDE = ["test_flake8.py", "test_mypy.py", "test_pep257.py"]
PYTHON_INCLUDE = ["__init__.py"]


def generate_ignore_files(directory: str, include: list[str]) -> list[str]:
    files = os.listdir(directory)
    return list(filter(lambda i: i not in include, files))


# This assumes that our copied files don't have the word utilities in them.
def replace(file_name: str, old: str, new: str) -> None:
    # Read in the file
    with open(file_name, 'r', encoding='utf-8') as file:
        filedata = file.read()

    # Replace the target string
    filedata = filedata.replace(old, new)

    # Write the file out again
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(filedata)


def make_package(package_name: str) -> None:
    script_dir = os.path.join(PACKAGE_DIR, "generic")
    cwd = os.getcwd()

    # Generates list of the ignore files from the copy
    ignore_files: list[str] = ["README.md"]

    ignore_python_files = generate_ignore_files(os.path.join(script_dir, HOME_PACKAGE),
                                                PYTHON_INCLUDE)
    ignore_files.extend(ignore_python_files)

    ignore_test_files = generate_ignore_files(os.path.join(script_dir, "test"), TEST_INCLUDE)
    ignore_files.extend(ignore_test_files)

    new_package_path = os.path.join(cwd, package_name)
    shutil.copytree(script_dir, new_package_path, ignore=shutil.ignore_patterns(*ignore_files))

    # Creates README.md
    os.rename(os.path.join(new_package_path, "TEMPLATE.md"),
              os.path.join(new_package_path, "README.md"))

    # Renames resource file
    resource_path = os.path.join(new_package_path, "resource")
    os.rename(os.path.join(resource_path, HOME_PACKAGE),
              os.path.join(resource_path, package_name))

    # Renames python folder
    os.rename(os.path.join(new_package_path, HOME_PACKAGE),
              os.path.join(new_package_path, package_name))

    files_and_folders = list(glob(new_package_path + "**/*", recursive=True))

    for file_or_folder in files_and_folders:
        if os.path.isfile(file_or_folder):
            file = file_or_folder
            replace(file, HOME_PACKAGE, package_name)

    # Add TODOs to files


def main() -> None:
    args = sys.argv[1:]

    if len(args) != 1:
        get_logger("Generate Package").error(f"Invalid Number of arguments: {args}")
        exit(1)

    make_package(args[0])
