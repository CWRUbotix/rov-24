#!/bin/bash

# Cleans Shellscripts
shfmt -w .

# Cleans Python files
isort .
black --config .python-black .
ruff format
