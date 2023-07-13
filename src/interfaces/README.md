# interfaces

## Overview

This is the package for all our msg, srv, and action definitions.

## Msg files

[Armed.msg](msg/Armed.msg)

Used for communicating Arm/Disarm command to pixhawk.

[CameraControllerSwitch.msg](msg/CameraControllerSwitch.msg)

Used for toggling through camera feeds in the pilot gui.

[Manip.msg](msg/Manip.msg)

Used for toggling manipulators on and off.

[ROVControl.msg](msg/ROVControl.msg)

Used for communicating movement direction of rov in physical and simulation.

[TaskFeedback.msg](msg/TaskFeedback.msg)

Used for communicating current task.

## Srv files

[TaskRequest.srv](srv/TaskRequest.srv)

Used for communicating current task and receiving response.

## Action files

[BasicTask.action](action/BasicTask.action)

BasicTask responds with a string.

[Example.action](action/Example.action)

An example action.

* **Author: Benjamin Poulin**
* **Author: Eric Yarnot**
* **Author: Michael Carlstrom**
