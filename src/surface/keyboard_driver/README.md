# keyboard_driver

## Overview

This package allows robot control from the keyboard.

## Installation

```bash
pip install pynput
```

## Usage

Run the main node with

```bash
ros2 launch keyboard_driver keyboard_driver_launch.py
```

## Launch files

* **keyboard_driver_launch.py:** Launches the keyboard driver node which reads keyboard input to control robots.

## Nodes

### Keyboard Driver

Reads keyboard input to control robots.

#### Published Topics

* **`/manual_control`** (ROVControl - an old, deleted message type)

    Outdated ROV movement instructions 