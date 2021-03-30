Build Instructions
==================

We provide a Singularity image with all required dependencies to build and run
the software.  See [About Singularity](@ref md_doc_singularity).

You can, of course, also use the package without Singularity.  In this case you
need to install all dependencies locally, though.


Get the Source
--------------

`robot_fingers` depends on several other of our packages which are
organized in separate repositories.  We therefore use a workspace management
tool called [treep](https://pypi.org/project/treep/) which allows easy cloning
of multi-repository projects.

treep can be installed via pip:

    pip install treep

Clone the treep configuration containing the "ROBOT_FINGERS" project:

    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git

**Note:**  treep searches for a configuration directory from the current working
directory upwards.  So you can use treep in the directory in which you invoked
the `git clone` command above or any subdirectory.

Now clone the project:

    treep --clone ROBOT_FINGERS

**Important:** treep uses SSH to clone from github.  So for the above command to
work, you need a github account with a registered SSH key.  Further this key
needs to work without asking for a password everytime.  To achieve this, run

    ssh-add

first.


Build
-----

### With Singularity

Go to the root directory of your workspace (the one containing the "src" folder)
and run the container in shell mode (see @ref md_doc_singularity):

    singularity shell -e --no-home -B $(pwd) path/to/image.sif

The current working directory gets automatically mounted into the container so
you can edit all the files from outside the container using your preferred
editor or IDE and all changes will directly be visible inside the container.
Vise versa modifications done from inside the container will modify the files on
the host system!

Inside the container first set up the environment:

    Singularity> source /setup.bash

This will source the ROS `setup.bash` and some other environment setup.

Now you can build by using this alias:

    Singularity> colcon build


### Without Singularity

To build, cd into the `workspace` directory and build with

    colcon build
