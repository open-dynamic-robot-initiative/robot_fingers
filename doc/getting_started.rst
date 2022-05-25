Getting Started
===============

This package implements a ``RobotDriver`` and several applications for the
TriFinger robots, using the robot_interfaces_ package.
See there for more information on the general architecture.  The action and
observation types for the (Tri)Finger robots are also implemented in
`robot_interfaces`.

We provide several demos_ to show how to use the interface on practical
examples.  Good starting points are:

- `demo_fake_finger
  <https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_fake_finger.py>`_
  Basic example using a fake robot (i.e. not real robot involved).  Mostly
  useful for testing if the package was installed correctly.
- `demo_single_finger_torque_control <https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_single_finger_torque_control.py>`_:
  Basic example on how to control the robot using torque commands.  This uses
  only a single finger but the principle is the same for the TriFinger.
- `demo_single_finger_position_control <https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_single_finger_position_control.py>`_:
  Same as above but using position commands instead of torque commands.
- `demo_trifinger <https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_trifinger.py>`_:
  Demo for the TriFinger, moving it in a hard-coded choreography.

.. note::

    The demos are all in Python, however, you can do exactly the same using the
    C++ API.


After you :doc:`build the package <installation>` (or when using a
Singularity/Apptainer image with the pre-built package included) you can run the
demos as follows:

.. code-block:: sh

    $ cd path/to/workspace
    $ singularity shell -e --no-home --bind=$(pwd) path/to/image.sif
    Singularity> source /setup.bash
    Singularity> source ./install/local_setup.bash
    Singularity> ros2 run robot_fingers demo_single_finger_torque_control

Instead of using ``ros2 run`` you can also call the corresponding Python scripts
directly.


.. _robot_interfaces: http://people.tuebingen.mpg.de/mpi-is-software/robotfingers/docs/robot_interfaces
.. _demos: https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos
