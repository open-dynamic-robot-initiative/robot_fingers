*******************
Robot Configuration
*******************

Configuration File
==================

The robot drivers based on :cpp:class:`~robot_fingers::NJointBlmcRobotDriver` are
configured via YAML files, which has to be passed to the function that creates the
backend (for example, see demo_single_finger_position_control_).

Default configurations for the different (Tri-)Finger robots can be found in the
``config/`` folder of the robot_fingers package.


Options
=======

.. note::

   Some of the options below expect a list with one value for each joint.  They are
   annotated with the type ``list[N_joints]``.  The actual length of the list depends on
   the number of joints of the robot.  The order of the joints is implicitly defined by
   :confval:`can_ports`.


.. confval:: can_ports: list

   List of the CAN interface names to which the robot is connected.  Example:
   ``["can0", "can1", "can2"]``.

   The joints will be ordered accordingly (e.g. in actions and observations), i.e. for
   the above example, the joint order will be
   ``[can0-motorA, can0-motorB, can1-motorA, ...]``


.. confval:: max_current_A: float = 0.0

   Maximum current [in Ampere] that can be sent to the motor.  Commands that result in
   higher torques will be clipped to the maximum.

.. confval:: has_endstop: bool = false

   Specify whether the joints have physical end stops or not.

   This is, for example, relevant for homing, where the end stop can be used to
   determine the absolute joint positions.  See also :confval:`homing_method`.

   Note that not having end stops does not mean that the joint can rotate freely in
   general.


.. confval:: homing_method: str = "NONE"

   Which method to use for homing.  See :doc:`homing` for possible values.


.. confval:: calibration

   Map with parameters related to homing and initialisation.

   .. confval:: calibration.endstop_search_torques_Nm: list[N_Joints] = [0, ...]

      Torque that is used to find the end stop.

   .. confval:: move_steps: int = 0

      Number of time steps for reaching the initial position (see
      :confval:`initial_position_rad`) after homing.


.. confval:: move_to_position_tolerance_rad: float = 0.0

   Tolerance [in radian] for reaching the target with
   :cpp:func:`robot_fingers::NJointBlmcRobotDriver::move_to_position` (used during
   initialization and shutdown).


.. confval:: safety_kd: list[N_Joints] = [0.1, ...]

   D-gain to dampen velocity.
   The default value is rather high to be on the safe side.  Depending in the
   application you may want to reduce it, to allow faster motions.

   Set to zero to completely disable damping.


.. confval:: position_control_gains

   Map with default control gains for the position PD controller.

   .. confval:: kp: list[N_joints] = [0, ...]

      Default P-gain for the position controller.

   .. confval:: kd: list[N_joints] = [0, ...]

      Default D-gain for the position controller.

.. confval:: hard_position_limits_lower: list[N_joints] = [0, ...]

   Hard lower limits for joint position.  Exceeding the limit with any joint results in
   an error and robot shutdown.

.. confval:: hard_position_limits_upper: list[N_joints] = [0, ...]

   Hard upper limits for joint position.  Exceeding the limit with any joint results in
   an error and robot shutdown.


.. confval:: soft_position_limits_lower: list[N_joints] = [-inf, ...]

   Soft lower limits for joint position.

   Exceeding this limit with some joints results in the action for that joint being
   adjusted to move the joint back inside the limits.

.. confval:: soft_position_limits_upper: list[N_joints] = [inf, ...]

   Soft upper limits for joint position.  See :confval:`soft_position_limits_lower`.


.. confval:: home_offset_rad: list[N_joints] = [0, ...]

   Offset between home position and zero.  See :doc:`homing`.


.. confval:: initial_position_rad: list[N_joints] = [0, ...]

   Initial position to which the robot moves during initialization.

.. confval:: shutdown_trajectory

   **Optional**

   Trajectory which is executed in the shutdown method.  Use this to move the robot to a
   "rest position" during shutdown of the robot driver.  It can consist of arbitrarily
   many steps.  Leave it empty to not move during shutdown.

   The trajectory is specified as a list of steps, each step consisting of the following
   values:

   .. confval:: shutdown_trajectory.target_position_rad: list[N_joints]

      Target positions for the joints in radian.

   .. confval:: shutdown_trajectory.move_steps: int

      Number of time steps for reaching the target position.

   Example:

   .. code-block:: yaml

      # first move slowly back to initial position
      - target_position_rad: [0, 0.9, -1.7, 0, 0.9, -1.7, 0, 0.9, -1.7]
        move_steps: 3000
      # then move faster to the actual rest position.
      - target_position_rad: [1.57, 1.8, -2.5, 1.57, 1.8, -2.5, 1.57, 1.8, -2.5]
        move_steps: 1000


.. confval:: run_duration_logfiles: list

   **Optional**

   List of file to which run duration logs are written.

   You can specify multiple files here if you want to log the runtime of different
   independent components separately.  For example on a robot with multiple
   manipulators, you can have a separate log for each manipulator, so if one of them is
   replaced, only the log file of this manipulator needs to be changed.



Example
=======

As a complete example, this is the configuration file used for the TriFingerPro robots:

.. literalinclude:: /PKG/config/trifingerpro.yml


.. _demo_single_finger_position_control: https://github.com/open-dynamic-robot-initiative/robot_fingers/blob/master/demos/demo_single_finger_position_control.py
