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
not be built using `colcon build`, document the setup process in detail and
setup accordingly in the CI as instructed in the Common Issue section.
