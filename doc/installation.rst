Build Instructions
==================

We provide a Singularity image with all required dependencies to build and run
the software.  See :doc:`singularity`.

You can, of course, also use the package without Singularity.  In this case you
need to install all dependencies locally, though.


Real-Time Requirements
----------------------

To ensure reliable communication with the robot hardware, a real-time Linux
kernel is needed.  See `Real Time Setup in the documentation of robot_interfaces
<http://people.tuebingen.mpg.de/mpi-is-software/robotfingers/docs/robot_interfaces/doc/real_time.html>`_


Get the Source
--------------

**robot_fingers** depends on several other of our packages which are
organized in separate repositories.  We therefore use a workspace management
tool called treep_ which allows easy cloning of multi-repository projects.

treep can be installed via pip::

    pip install treep

Clone the treep configuration containing the "ROBOT_FINGERS" project::

    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git

**Note:**  treep searches for a configuration directory from the current working
directory upwards.  So you can use treep in the directory in which you invoked
the ``git clone`` command above or any subdirectory.

Now clone the project::

    treep --clone ROBOT_FINGERS

.. important::
    treep uses SSH to clone from GitHub.  So for the above command to work, you
    need a GitHub account with a registered SSH key.  Further this key needs to
    work without asking for a password everytime.  To achieve this, run ::

        ssh-add

    first.


Build
-----

With Singularity
~~~~~~~~~~~~~~~~

Go to the root directory of your workspace (the one containing the "src" folder)
and run the container in shell mode (see :doc:`singularity`)::

    singularity shell -e --no-home --bind=$(pwd) path/to/image.sif

The current working directory gets automatically mounted into the container so
you can edit all the files from outside the container using your preferred
editor or IDE and all changes will directly be visible inside the container.
Vice versa modifications done from inside the container will modify the files on
the host system!

Inside the container first set up the environment::

    Singularity> source /setup.bash

This will source the ROS `setup.bash` and do some other environment setup.

Now you can build with::

    Singularity> colcon build


Without Singularity
~~~~~~~~~~~~~~~~~~~

To build, cd into the ``workspace`` directory and build with::

    colcon build

This assumes that `colcon` and all build dependencies are installed.


Real-Time-Capable Build
~~~~~~~~~~~~~~~~~~~~~~~

When running a PREEMPT_RT Linux kernel, this is automatically detected at
build-time and build flags are set accordingly.  If you want to make a real-time-capable
build while running a different kernel (e.g. the "lowlatency" kernel or when
cross-compiling), you need to explicity set the ``OS_VERSION``::

    colcon build --cmake-args -DOS_VERSION=preempt-rt


.. note::

    If you see the following output during initialisation of the robot, this
    means you are running a non-real-time build.

    .. code-block:: text

        Warning this thread is not going to be real time.


.. _treep: https://pypi.org/project/treep/
