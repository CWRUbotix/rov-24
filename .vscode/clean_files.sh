#!/bin/bash

# Cleans Shellscripts
shfmt -w .

# Cleans Python files
black .
isort .
