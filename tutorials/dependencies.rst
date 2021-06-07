Configure Dependencies
======================

There are typically 3 ways to setup dependencies in ROS2.
**rosdep** and **vcstool** should **always be prioritized**.

rosdep
------

Most **pip**, **ros official release** and **standard deb(apt)** **binary**
dependencies can be installed using ``rosdep``.
However, you should **always** run the following command in your workspace
directory to check if there are dependencies wrongly defined in
``package.xml``.

.. code-block:: bash

   cd <workspace-directory>
   rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

.. note::

   Sometimes the name for the packages or dependencies are not exactly the
   same as the project name (especially for Python and Debian dependencies).
   I would recommend browsing the `rosdistro source repository`_ for reference.

   For instance, to list :literal:`OpenCV` C++ library as a dependency, the
   corresponding name is :literal:`libopencv-dev`. This definition can be
   found in `rosdistro YAML file`_

.. _rosdistro source repository: https://github.com/ros/rosdistro
.. _rosdistro YAML file: https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml#L3227

vcstool
-------

Most public **pip**, **cmake** and **ament(ros2)** dependencies that
required building from source can be imported via ``vcstool`` and build
with ``colcon build``. A simple tutorial can be found on the
`source code repository`_.
You run the following command to check if you have configure correctly.

.. code-block:: bash

     cd <workspace-directory>
     vcs import --recursive src < my.repos

Here is an example of the ``.repos`` file

.. code-block:: yaml

   repositories:
     rmf_core:
       type: git
       url: https://github.com/osrf/rmf_core.git
       version: master

     rmf_schedule_visualizer:
       type: git
       url: https://github.com/osrf/rmf_schedule_visualizer.git
       version: master

     traffic_editor:
       type: git
       url: https://github.com/osrf/traffic_editor.git
       version: master

You can also specify version to specific **tags** and **commit** for better
versioning control.

.. _source code repository: https://github.com/dirk-thomas/vcstool

Others
------

For other dependencies that are not under **standard debian(apt)** or can
not be built using ``colcon build``, document the setup process in detail and
setup accordingly in the CI as instructed in the Common Issue section.

Example: Using vcstool
-----------------------

In this example, we would be looking at how ``vcstool`` could be used
to automate the import of a few packages into a workspace, without the need
to manually clone the repository into the workspace.

Setup
~~~~~

Ensure that the ``vsctool`` is installed before proceeding:

.. code-block:: bash

  sudo apt install python3-vsctool

.. important::

  Note that we are using ``vcstool``, which is different from ``vcstools``!

After the installation has been completed, we would proceed to create a new
workspace for this example. Let's create a workspace called ``vcs_ws``:

.. code-block:: bash

  mkdir -p ~/vcs_ws/src
  cd ~/vcs_ws/

Build the workspace & source the setup file::

  colcon build
  . install/setup.bash

.. note::

  If you are not using the ``bash`` shell, change the extension of the setup file accordingly.
  For example, if you are using ``zsh``, use ``setup.zsh`` instead.

Next create the ``req.repos`` file and open it with you favorite text editor.
This file would list the dependencies or packages that would be downloaded
into the workspace. Copy the following content below into ``req.repos`` and
save the file.

.. code-block:: yaml

  repositories:
    rtpkg:
      type: git
      url: https://github.com/1487quantum/rtpkg.git
      version: main
    packml_ros2:
      type: git
      url: https://github.com/1487quantum/packml_ros2.git
      version: master

The ``req.repos`` file created contains the configuration to clone the
``packml_ros2`` and the ``rtpkg`` packages. The name of the repository
would be the determined by the header, and the ``version`` field is used
to specify the repository branch.

After that, we would use the ``vcs`` tool to import these repository
into the workspace::

  vcs import --recursive src < req.repos

This would clone the repositories stated in the ``.repos`` file into
the ``src`` directory. Check whether the repositories are cloned into
the ``src/`` directory:

.. code-block:: bash

  cd ~/vcs_ws/src
  ls

The output would look something like this:

.. code-block:: bash

  ubuntu@ubuntu:~/vcs_ws/src$ ls
  packml_ros2  rtpkg_msg


