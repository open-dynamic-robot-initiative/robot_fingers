************************
Actions and Observations
************************

This page describes the structure of actions and observations used by the robot drivers
implemented in ``robot_fingers``.  The fields of both actions and observations are
typically arrays with one element per robot joint.  The order of joints in the list
depends on the robot.

.. todo:: Add dedicated page about joint order



.. note::

    Examples below are using the Python bindings, as this is how the robots are used by
    the majority of users.  However it is also possible to directly use the underlying
    C++ API.  The general principles are the same.


Actions
=======

Actions are the commands that are sent to the robot to move the joints.
All drivers in ``robot_fingers`` use action types based on the
:cpp:class:`robot_interfaces::NJointAction` template.  Only the number of joints (and
thus the length of the vectors) differs based on the robot.

With this action type, one can either sent torque commands, position commands or a
mixture of both.  For position commands it is further possible to overwrite the default
PD gains for some or all of the joints.


Torque only
-----------

Example for creating a torque-only action for a single finger robot:

.. code-block:: Python

    # Import Action from the corresponding module for the used robot type (one of
    # "finger", "trifinger", "one_joint", "two_joint", "solo_eight")
    from robot_interfaces.finger import Action

    desired_torques = [0.1, -0.1, 0.04]  # torques for the three joints in Nm
    action = Action(torque=desired_torques)


Position only
-------------

Example for creating a position-only action for a single finger robot:

.. code-block:: Python

    # Import Action from the corresponding module for the used robot type (one of
    # "finger", "trifinger", "one_joint", "two_joint", "solo_eight")
    from robot_interfaces.finger import Action

    desired_positions = [0.0, 0.9, -1.7]  # positions for the three joints in radian
    action = Action(position=desired_positions)


One can also overwrite the default PD gains for this action.  Setting a gain to NaN
means using the default gain for that joint:

.. code-block:: Python

    # overwrite PD gains for last two joints, sticking to default for the first one
    action = Action(
        position=desired_positions,
        position_kp=[math.nan, 12, 10],
        position_kd=[math.nan, 0.02, 0.02],
    )


Mixing torque and position commands
-----------------------------------

When setting both ``torque`` and ``position`` in an action, the resulting torque command
sent to the robot is computed as::

    torque_command = torque + PD(position)


.. code-block:: Python

    # Import Action from the corresponding module for the used robot type (one of
    # "finger", "trifinger", "one_joint", "two_joint", "solo_eight")
    from robot_interfaces.finger import Action

    action = Action(
        torque=[0.1, -0.1, 0.04],
        position=[0.0, 0.9, -1.7],
    )

One can also operate some joints in torque mode (by setting position for that joint to
NaN) and others in position mode (by setting torque to zero).  E.g. to hold the first
joint in a fixed position and operate the other two in torque mode:

.. code-block:: Python

    action = Action(
        torque=[0.0, -0.1, 0.04],
        position=[0.5, math.nan, math.nan],
    )


.. _safety_checks:

Safety Checks
=============

The actions provided by the user are actually not directly sent to the robot but undergo
some safety checks to prevent harmful actions, that might damage the robot, from being
executed.

The following steps are performed in the given order:

1. If one or more joints exceed the soft position limits
   (:confval:`soft_position_limits_lower`, :confval:`soft_position_limits_upper`),
   actions that do not point back towards the allowed range are replaced with a position
   command to the limit value.  Further, custom PD-gains are ignored in this case.
2. Limit the combined torque (torque + position command) to the allowed maximum value
   (see :confval:`max_current_A`).
3. Dampen velocity using the given :confval:`safety_kd` gains.  Damping is done
   joint-wise using this equation::

       torque_damped = torque_desired - safety_kd * current_velocity

4. The damped torques are again clipped based on :confval:`max_current_A`.


The resulting action, that is actually applied to the robot after performing these safety
measures, can be accessed via
:cpp:func:`~robot_interfaces::RobotFrontend::get_applied_action` of the robot front end.



Observations
============

Observations contain the sensor measurements of the robot.
The data type used for the observations differs depending on the robot type:

- Robots of type ``one_joint``, ``two_joint`` and ``solo_eight`` use observations based
  on :cpp:class:`robot_interfaces::NJointObservation`.
- Robots of type ``finger`` and ``trifinger`` use observations based on
  :cpp:class:`robot_interfaces::NFingerObservation`.

They both contain vectors for measured joint torques, velocities and positions.  The
only relevant difference is that the latter also contains a field ``tip_force``, which
contains measurements of the finger tip push sensors.

Note however, that only the FingerPro and TriFingerPro robots are actually equipped with
tip push sensors.  For other (Tri-)Finger robots like the Edu-version, ``tip_force`` is
still included in the observation but its values are meaningless.

Example snippet:

.. code-block:: Python

    observation = robot_frontend.get_observation(t)
    print("Position: %s" % observation.position)
    print("Velocity: %s" % observation.velocity)
    print("Torque: %s" % observation.torque)
    print("Tip Force: %s" % observation.tip_force)  # only for finger/trifinger robots

All fields are arrays.  ``position``, ``velocity`` and ``torque`` contain one value per
joint.  ``tip_force`` one value per finger.
