********************
Homing of the Joints
********************


What is Homing and why is it needed?
====================================

The BLMC robots, like many other robots, are using relative motor encoders.
This means we don't get the *absolute* position at any time but only *relative*
position changes (e.g. "motor moved by 13 degrees").
By accumulating these changes over time, we can get the current position
relative to the start position.

However, this start position can be different every time the robot is turned on.
Since the absolute position is unknown, the motor controller simply defines the
current position to be zero when turned on.  This means the zero position as
seen by the software can be at different physical positions between different
runs.

To solve this problem, *homing* is performed in the beginning when initialising
the robot.  The idea is to search for a fixed physical position that can always
be found, even when starting from an unknown initial position.  A simple example
is to move the joint in one direction until it hits the end stop (assuming it
has one).
As the end stop is always at the same physical position, the absolute position
of the joint is now known and the zero position can be set accordingly.

Now, the position reported by the joints will always be relative to the physical
zero position and are thus comparable between different runs of the robot.


Zero Position, Home Position and Home Offset
============================================

For better understanding in the following, a few terms have to be defined:

Zero Position
    The physical position at which the joint reports a position of zero (e.g.
    zero degrees for revolute joints).
    The zero position is defined by the user.

Home Position
    The home position is the joint position that is searched for during the
    homing.  It is implicitly defined by the kinematics of the robot and the
    homing method that is used.  For example, when using the "endstop" homing
    method (see below), the home position is at the end stop.

Home Offset
    Configuration parameter to specify the offset between the *home position*
    and the desired *zero position*.


Once the home position is found, the zero position is computed using

::

    zero_position = home_position + home_offset


.. admonition:: Example

    Assume Alice has a revolute joint with a range of 180° that is limited by
    end stops on both sides.  For a given application she wants the *zero
    position* to be in the middle between the end stops, such that the joint
    position ranges from -90° to 90°.

    Alice chooses to use the "endstop" homing method (see below), searching in
    positive direction.  This means that when starting the homing, the joint
    moves in positive direction until it hits the end stop.  This is the *home
    position*.

    Since the *home position* does not match with the desired *zero position*,
    Alice has to specify a *home offset*.  As the desired position is in the
    middle of the 180°-range, she has to set the home offset to -90°.

    .. image:: images/homing_example.png
        :align: center

    The resuling configuration:

    .. code-block:: yaml

       homing_method: endstop
       calibration:
         endstop_search_torques_Nm: [+0.2]
       home_offset_rad: [-1.57]



Supported Homing Methods
========================

We support a number of different homing methods.  Which one is best depends on
the robot and the application.


None
----

The most easy one: Do not perform any homing procedure.  In this case the home
position is where ever the sensors report zero (typically the position of the
joint when the board was powered one).

Use this if your application does not depend on a global position (e.g. if the
joint is anyway only moved relative to its current position) or if homing is
already done on the motor board level.

Configuration:

.. code-block:: yaml

   homing_method: none


Home at Current Position ("current_position")
---------------------------------------------

Use the current joint position (at the moment when the homing is run) as home
position.

Can, for example, be used in applications where the exact position is not so
important and the joints are manually moved to the desired home position before
running the software.

With this method, the motors do not move during homing.

Configuration:

.. code-block:: yaml

   homing_method: current_position


Home at Next Encoder Index ("next_index")
-----------------------------------------

The motor moves slowly until the next encoder index tick is detected.  The
position of the index tick is used as home position.

Keep in mind that due to gear ration between motor and joint, the motor does
several revolutions for one joint revolution and thus the encoder index appears
multiple times at different joint positions.  Therefore the initial position
from where the index search is started does matter!

This is similar to the "current position" method but here the joints only need
to be moved roughly to the desired position from which it starts the index
search and thus uses the exact same home position every time.

The signs of the ``endstop_search_torques_Nm`` configuration value is used to
determine the search direction for each joint (even though the actual value is
not used).

Configuration example:

.. code-block:: yaml

   homing_method: next_index
   calibration:
     endstop_search_torques_Nm:  # only signs are relevant
       - +0.1
       - +0.1
       - -0.1


Home at End Stop ("endstop")
----------------------------

The motors move until they hit an obstacle (the end stop).  The position of the
obstacle (while still pushing) is used as home position.

Can be used in applications where all joints have an end stop that can safely be
hit.  Note that the exact home position can vary a bit between runs depending on
the softness of the system (a joint may in some runs push a bit harder into the
end stop than in others).

For this to work reliably, it is important that there are no other obstacles
that might accidentally be hit by the joints instead of the end stop!

Does not depend on a specific start position.

Does not depend on the encoder index.

The torque with which each joint moves during the end stop search is configured
via ``endstop_search_torques_Nm``.

Configuration example:

.. code-block:: yaml

   homing_method: endstop
   calibration:
     endstop_search_torques_Nm:
       - +0.3
       - +0.3
       - -0.2


Home at End Stop After Release ("endstop_release")
--------------------------------------------------

Like above but release the torque from the joints and wait a moment, so that the
joints are not actively pushing against the end stops anymore before homing.

Can be useful if the joints have soft end stops (so the position may vary while
the joints are actively pushing) that push the joints back.
Note, however, that this requires "stable" end stops, i.e. if a joint is at the
end stop and the torque is set to zero, it should not move away on its own.

Configuration example:

.. code-block:: yaml

   homing_method: endstop_release
   calibration:
     endstop_search_torques_Nm:
       - +0.3
       - +0.3
       - -0.2


First Hit End Stop, Then Search Encoder Index ("endstop_index")
---------------------------------------------------------------

First move to the end stops (see above), from there search for next encoder
index in opposite direction.

You may use this method if only using the end stop is not accurate enough (e.g.
because the end stops are soft and the "endstop_release" method is not possible
in your case).

Does not depend on a specific start position.

Configuration example:

.. code-block:: yaml

   homing_method: endstop_index
   calibration:
     endstop_search_torques_Nm:
       - +0.3
       - +0.3
       - -0.2


Which Method is Best for Me?
============================

There is no general answer which method is the best, as it always depends on the
application and the physical properties of the robot.  To give some guide:

- Only use methods that depend on the encoder index (next_index and
  endstop_index) if you don't get accurate enough results otherwise.  Using the
  index provides best repeatability but the home offset depends on the index
  position, so it needs to be calibrated individually for each robot and needs
  to be recalibrated each time the joint was disassembled for maintenance.
- Use the end stop if possible.  It has the great advantage that you can simply
  start the homing at any time without having to manually align the joints
  first.  Of course, this only works, if the geometry of your robot allows all
  joints to reliably reach the end stop by simply moving towards it (i.e. they
  cannot get blocked somewhere else).


How to Determine the Home Offset?
=================================

Often the home position (that is implicitly defined by the robot kinematics and
the chosen homing method) does not match with the desired zero position.  For
example when homing at an end stop, the home position is always at one of the
end stops but the desired zero position might be in the middle of the joint
range.
Therefore the offset between the home position and the desired zero position
(called ``home_offset`` in the configuration) needs to be determined and written
to the configuration file.

A relative easy method to find the correct offset is as follows:


1. First set the home offset to zero (i.e. zero position = home position).
2. Start the robot and perform the homing.
3. Manually move the robot to the desired zero position and print the position
   of the joints (after homing these positions are now relative to the home
   position).
4. Set the joint positions of the desired zero position as home offset.

When restarting now, the actual zero position after homing should be the desired
one.

.. note::

   When using one of the homing methods that rely on the encoder index, the home
   offset does not only depend on the geometry of the robot but also on the
   position of the index tick on the encoder disc.  This means that when
   exchanging the encoder disc, the offset likely needs to be recalibrated.
   Likewise otherwise identical robots may need different offsets if the encoder
   discs are aligned differently in the different robots.
