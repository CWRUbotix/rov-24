#!/bin/bash

# Cleans Shellscripts
shfmt -w .

# Cleans Python files
isort .
black --line-length 100 .
