# MATE ROV 2023-24

<a href="https://github.com/cwruRobotics/rov-24/actions"><img src="https://github.com/cwruRobotics/rov-24/workflows/Continuous Integration/badge.svg" alt="Build Status"></a>
<a href=" https://opensource.org/licenses/Apache-2.0"><img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg" alt="Apache License"></a>

# Table of Contents

1. [Setup](#setup)
    1. [Linux](#linux)
        1. [Docker](#docker)
        2. [Bare Metal](#bare-metal)
    2. [Windows](#windows)
        1. [Docker](#docker-1)
        2. [WSL](#wsl)
    3. [macOS](#macos)
        1. [Docker](#docker-2)
2. [Test Environment](#test-environment)
3. [Code Building](#building-with-colcon)
4. [Directory Structure](#directory-structure)
5. [Documentation Structure](#documentation-structure)

## Setup

Start by opening up a terminal and navigating to where you want the code to be saved and entering the following command.

```bash
git clone --recurse-submodules git@github.com:cwruRobotics/rov-24.git
```

If you've never contributed to a git repository before, you might receive an error message saying you don't have access. In that case visit [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh) to set up SSH for local GitHub access.

After cloning the code, we need to set up our IDE: VSCode. If you already have it, great. Otherwise follow [this](https://code.visualstudio.com/download) tutorial.

### Linux

#### Docker

Start by installing docker engine from [here](https://docs.docker.com/engine/install/ubuntu/).

Set your permissions based on [this](https://docs.docker.com/engine/install/linux-postinstall/) guide.

Restart.

Then install the Dev Containers VSCode extension.

To open the container use `F1` or `ctrl+shift+p` to open the command bar and use `Tasks: Runs Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV Linux` for which type to run.

For gui apps run `xhost + local:docker` before launching docker or add to `.bashrc`.

To reopen after a build Go to File > Open Recent /stuff/rov-24 \[Dev Container]

#### Bare Metal

To run the install script use `F1` or `ctrl+shift+p` to open the command bar and use `Tasks: Runs Task`. Then from the Task selection choose `First Time Setup`. This will install ROS and all our dependencies.

### Windows

#### Docker

Start by installing docker from [here](https://www.docker.com/get-started/).

Then install the Dev Containers VSCode extension.

To open the container use `F1` or `ctrl+shift+p` to open the command bar and use `Tasks: Runs Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV Windows` for which type to run.

To add your ssh keys into the container follow [this](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) guide.

If this doesn't work try running `ssh -v git@github.com`

If get_agent_identities from the prior command is empty you will likely need to download a newer version of OpenSSH from [here](https://github.com/PowerShell/Win32-OpenSSH/releases/tag/v9.4.0.0p1-Beta). Download the .msi file. Once download double click to update it. You might need to do the previous step again.

To get gui support inside docker download [this](https://sourceforge.net/projects/vcxsrv/files/latest/download).

Then run XLaunch from the Windows Start Menu. Make the settings look like this:

![Screenshot of the settings for Xlaunch](/doc/images/VcXsrv.png)

Then open up Command Prompt and type `ipconfig`.

Then in the terminal of the docker container use `export DISPLAY={IPV4 of WSL}:0.0` where the IPV4 is from the `ipconfig` command.

To reopen after a build Go to File > Open Recent /stuff/rov-24 \[Dev Container]

#### WSL

Follow [this](https://learn.microsoft.com/en-us/windows/wsl/install) guide to install WSL.

After WSL has been installed follow [this](https://code.visualstudio.com/docs/remote/wsl) guide to get VSCode and WSL to properly communicate and navigate to the rov-24 folder.

Then run the install script.

To run the install script use `F1` or `ctrl+shift+p` to open the command bar and use `Tasks: Runs Task`. Then from the Task selection choose `First Time Setup`. This will install ROS and all our dependencies.

### macOS

#### Docker

Start by installing docker from [here](https://www.docker.com/get-started/).

Then install the Dev Containers VSCode extension.

To open the container use `F1` or `ctrl+shift+p` to open the command bar and use `Tasks: Runs Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV macOS` for which type to run.

To reopen after a build Go to File > Open Recent /stuff/rov-24 \[Dev Container]

<!-- Xserver testing on mac TODO ssh keys work -->
<!-- https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285 -->

## Test environment

After running the script open a terminal and run

```bash
ros2 run demo_nodes_cpp talker
```

Open a second terminal and run

```bash
ros2 run demo_nodes_py listener
```

## Building with Colcon

Now, anytime you want to build, do the following:

In VSCode, press `F1` or `ctrl+shift+p` and enter `Tasks: Run Task` in the field until you see the
corresponding option appear. Click or hit enter on that option.

This whole process should become `F1`, `Enter`, `Enter` once you've done it once,
although the magic of symlink should mean you won't need to build again for most things.

If you're working on package's `setup.py` or rov_msgs, you'll need to run `🏃‍♂️ ROS Quick Build` or use `Control + Shift + B` every time you change something.

If you want to run our unit tests use this command `colcon test --event-handlers=console_direct+`.

It runs the tests and pipes the output to the terminal. To test pi_main make sure to type your password into the terminal after running the above command.

If you install the flake8 and mypy extension they should help enforce the linters.

<!-- ### Automatic building for non-VSCode heathens

Run this command from your workspace folder

The magic of symlink should mean you won't need to build again for most
things, but if you're working on package metadata (e.g. `package.xml`) or
rov_msgs, you'll need to run this every time you change something:

```bash
. src/.vscode/easy_build.sh
``` -->

## Directory Structure

All packages to be installed on the surface computer live in the `surface` directory.

All packages to be installed on the pi compute module live in the `pi` directory.

All packages to be installed on the float live in the `float` directory.

## Namespaces

All nodes running on the pi will be in the pi namespace.

All nodes running on the surface will be in the surface namespace.

Any topics or services communicating across will be renamed first into the tether namespace.

![Picture of rqt with namespaces](/doc/images/namespaces.png)

## Documentation Structure

Documentation will take place at 3 levels:

- High Level - Overarching Design Document outlining our general structure and what goes where.
- Device Level - ROS Docs as set out in the ROS2 standards.
- Inline Level - Inline Documentation to the level that someone who has some basic code knowledge can understand what the code does.
