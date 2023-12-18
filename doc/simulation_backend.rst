.. _simulation_backend:

*********************************
How to use the Simulation Backend
*********************************

In addition to the real-robot driver, ``robot_fingers`` also implements a
simulation driver (based on :doc:`trifinger_simulation
<trifinger_simulation:index>`), which can be used as a drop-in replacement for
the real-robot.  This allows testing code in simulation and then moving to the
real robot without the need for code changes (beyond replacing the backend).


Multi-process Applications
==========================

**TriFingerPro only!**

When running front end and back end in separate processes, all that needs to be
done is to replace the back end with

.. code-block:: sh

   $ ros2 run robot_fingers pybullet_backend

See ``--help`` for available options.

This also requires the ``trifinger_data_backend`` to be running.  Minimal usage
example (run each command in a separate terminal):

.. code-block:: sh

   $ ros2 run robot_fingers trifinger_data_backend -a 10000
   $ ros2 run robot_fingers pybullet_backend --visualize
   $ ros2 run robot_fingers demo_trifingerpro --multi-process


Single-process Applications
===========================

When creating front end and back end in the same script, the only necessary
change is to replace the function for creating the back end
(:func:`robot_fingers.create_trifinger_backend` /
:func:`robot_fingers.create_single_finger_backend`) with its counterpart from
:mod:`robot_fingers.pybullet_drivers`.


**Example: demo_simulation_driver**

``demo_simulation_driver.py`` shows an example how to use the simulation backend
(note that this example is a bit more complex as it covers all (Tri-)Finger
types).  The only relevant difference to using the real robot is the choice of
the "create backend" function.

.. literalinclude:: /PKG/demos/demo_simulation_driver.py
