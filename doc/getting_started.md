Getting Started
===============

This package implements a RobotDriver and several applications for the TriFinger
robots, using the [`robot_interfaces`
package](https://open-dynamic-robot-initiative.github.io/code_documentation/robot_interfaces/docs/doxygen/html/index.html).
See there for more information on the general architecture.  The action and
observation types for the (Tri)Finger robots are also implemented in
`robot_interfaces`.

We provide several
[demos](https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos)
to show how to use the interface on practical examples.  Good starting points
are:

- [demo_real_finger.py](https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_real_finger.py):
  Basic example on how to control the robot using either torque or position
  commands.  This uses only a single finger but the principle is the same for
  the TriFinger.
- [demo_trifinger.py](https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_trifinger.py):
  Demo for the TriFinger, moving it in a hard-coded choreography.

@note The demos are all in Python, however, you can do exactly the same using
the C++ API.
