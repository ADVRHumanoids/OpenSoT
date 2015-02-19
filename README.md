OpenSoT v0.2
=========

New Features Includes:
-----------------------
- SubTasks: to consider only some rows of Tasks Jacobians/Errors
- Active Joint Mask: to zero columns in the Task Jacobians
- Rewritten QP Solver: this version does not explicitly pass Constraints of high priority Tasks to low priority Tasks
- MinAcc task
- Cartesian Interaction Task
- Math of Tasks (aka write IK problem in a more easy way)
- Autostack 
- Many Bug Fixes
- Improved documentation (we hope!)

An Open Source Task Solving library with Constraints

Introduction
------------
OpenSoT is a library dedicated to hierarchical whole-body control of robots subject to constraints such as joint limits, joint velocities, cartesian constraints... The main idea behind OpenSoT is to decouple Task and Constraints description from the Solver used to compute the robot commands and the Type of Control available on the robot to perform them.

OpenSoT is developed under the European Project WALK-MAN: http://www.walk-man.eu/.

Its homepage resides in http://github.com/robotology-playground/OpenSoT
You can also find a wiki there.

An online version of this documentation can be obtained in http://opensot.github.io

Some videos from OpenSoT channel in YouTube:

[[embed url=http://www.youtube.com/watch?v=Avs6pqUCqVo]]

[[embed url=http://www.youtube.com/watch?v=cBggRy7WPUE]]

[[embed url=http://www.youtube.com/watch?v=U1FcIq15jg4]]

Installation
------------

To Install OpenSoT, the recommended way is to use the robotology superbuild http://www.github.com/robotology-playground/robotology-superbuild

