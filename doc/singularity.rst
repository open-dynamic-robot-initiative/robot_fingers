About Apptainer/Singularity
===========================

What is Apptainer?
------------------

Apptainer (formerly called "Singularity") is a tool to run software inside
containers, similar to Docker. Compared to Docker it has a higher focus on
security and can be used without root permission.  Also programs in the
container are executed as the user of the host system which makes it much more
convenient when touching files of the host system (as it is happening when
building a mounted workspace).

.. note::

    Singularity was recently renamed to Apptainer (so Apptainer 1.0 is basically
    Singularity 3.9).  In the following we assume that you are using Apptainer,
    however, it probably works the same with recent releases (3.x) of
    Singularity.  Due to this, we are using the command `singularity` in the
    following which should work with all versions (Apptainer installs a
    corresponding alias).

    If you are using Apptainer, you can replace `singularity` with `apptainer`
    but both should work the same.


Install Apptainer
-----------------

You can download pre-build packages of recent releases from the `Apptainer
GitHub repository <https://github.com/apptainer/apptainer/releases/>`_.

For example on Ubuntu, download the deb package (called
"apptainer_X.Y.Z_amd64.deb" and install it with::

    $ sudo apt install ./apptainer_X.Y.Z_amd64.deb

In the following, we provide some basic information on how to use
Apptainer.  For more detailed information, please see the `official
documentation`_.


Get our Apptainer Image
-----------------------

We provide an Apptainer image with Ubuntu 20.04 with all dependencies needed to
build and run the software here.  You can download the latest version using::

    singularity pull oras://ghcr.io/open-dynamic-robot-initiative/trifinger_singularity/trifinger_base:latest


In case you prefer to build the image yourself, see the `trifinger_singularity
repository on GitHub
<https://github.com/open-dynamic-robot-initiative/trifinger_singularity>`_


Run Something in the Container
------------------------------

To run the container in shell mode (i.e. opening a shell inside the container),
the following is often enough::

    singularity shell path/to/image.sif

This will, however, be influenced by your local setup as environment variables
are exported and the home directory is mounted by default.  Further the current
working directory from which Apptainer is run is also bound inside the
container.

This default behaviour is often convenient but can cause issues in some cases.
A typical example would be a Python package installed in your home directory
(which will then be available in the container) which is not compatible with
versions of other packages inside the container.  To avoid these kind of issues
it is recommended to use the following command to run the container in a more
isolated way::

    export SINGULARITYENV_DISPLAY=$DISPLAY
    singularity shell -e --no-home --bind=$(pwd) path/to/image.sif

The arguments explained:

- The first line makes sure the DISPLAY environment variable is set correctly
  inside the container (only needed if you want to run GUI-based applications).
- ``-e`` (short for ``--cleanenv``) prevents environment variables to be
  exported.
- ``--no-home`` prevents your home directory from being bound.
- ``--bind=$(pwd)`` explicitly binds the current working directory.  This is
  needed if the working directory is inside your home directory as otherwise it
  is excluded by the ``--no-home``.

Note that with the above the current working directory is still bound in the
image, so it is possible to build/modify the workspace from the host-system when
Apptainer is run from the root directory of the workspace.


Compatibility with Nvidia Drivers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When you are using Nvidia drivers and want to run a GUI-based application in the
container, you may need to add the ``--nv`` flag::

    singularity shell --nv ... path/to/image.sif


Add Custom Dependencies to the Container
----------------------------------------

The image we provide already includes everything needed to run the robot
and the simulation. However, you may need additional libraries to use
them in our own code, which are not yet present. In this case, you can
create your own image which is based on our standard image but extends
it with your additional dependencies.

To extend the image, create *definition file* like the following::

    # Specify the name of the base image below
    Bootstrap: localimage
    From: ./base_image.sif

    %post
        # Put commands to install additional dependencies here.
        # Make sure everything runs automatically without human input (e.g. add
        # `-y` to automatically say "yes" below).
        apt-get install -y package_name

See the official `Documentation for Definition Files`_ for all options in the
definition file.

Assuming you called your definition file ``user_image.def``, use the
following command to build the image. Note that the base image
(specified in the ``From:`` line) needs to be present in the directory in
which you call the command.

::

    $ singularity build --fakeroot user_image.sif path/to/user_image.def


.. _official documentation: https://apptainer.org/docs/
.. _Documentation for Definition Files: https://apptainer.org/docs/user/1.0/definition_files.html

