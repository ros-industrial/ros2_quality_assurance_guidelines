Unit Test
=========

C++ Unit Test
-------------

Google Test & Google Mock
^^^^^^^^^^^^^^^^^^^^^^^^^

To run gtest/gmock, simply put these two parts within the ``BUILD_TESTING``
block in ``CMakeLists.txt`` and replace the corresponding names with the
correct files/names,

.. code-block:: cmake
   :emphasize-lines: 5,6,7,8,9,11,12,13,14,15

   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     ament_lint_auto_find_test_dependencies()

     # Add gtest executables
     find_package(ament_cmake_gtest REQUIRED)
     ament_add_gtest(<test-name> <test-file>)
     ament_target_dependencies(<test-name> rclcpp <other-ament-dependencies>)
     target_link_libraries(<test-name> <library-name>)

     # Add gmock executables
     find_package(ament_cmake_gmock REQUIRED)
     ament_add_gmock(<test-name> <test-file>)
     ament_target_dependencies(<test-name> rclcpp <other-ament-dependencies>)
     target_link_libraries(<test-name> <library-name>)

   endif()

Also remember to to add this line to ``package.xml``,

.. code-block:: xml

   <test_depend>ament_cmake_gtest</test_depend>
   <test_depend>ament_cmake_gmock</test_depend>

Run the build and test command to invoke gtest or gmock.

.. code-block:: bash

   colcon build
   colcon test --packages-select <package_name> --event-handlers console_direct+


Catch2
^^^^^^

To use ``Catch2``, the sources have to be setup beforehand. There are 2 ways
in of installing the ament plugin, either through the system package installer
or cloning the repository into the workspace.

Via system package installer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As ``Catch2``
ament plugin is listed inside the ``rmf`` sources, we'll be setting it up
so that the system would be able to find it. First, authorize the key:

.. code-block:: bash

  curl -s http://rmf.servehttp.com/repos.key | sudo apt-key add -

Next, we would be adding the repository to the ``apt`` sources, after which we
would update the repository list:

.. code-block:: bash

  sudo sh -c 'echo "deb http://rmf.servehttp.com/ubuntu/main/ `lsb_release -cs` main" > /etc/apt/sources.list.d/rmf.list'
  sudo apt update

Finally, install the ament ``Catch2`` ament plugin:

.. code-block:: bash

  sudo apt install ros-foxy-ament-cmake-catch2

Via git clone
~~~~~~~~~~~~~~

Clone the repository into your workspace::

  git clone https://github.com/open-rmf/ament_cmake_catch2.git

After that, compile the sources::

  colcon build

Usage
~~~~~~

.. important::

  Ensure that ``Catch2`` is installed in the system beforehand, as it is a
  dependency of the ament plugin. A quick guide to install ``Catch2`` system-wide
  is available in the "Guide" section: :ref:`cth2setup`, more information on ``Catch2`` can be found in
  the official `repository <https://github.com/catchorg/Catch2/>`__

To use the ``Catch2`` , we would add the ament wrapper for it. Add the
following lines to your ``package.xml``:

.. code-block:: xml

  <test_depend>ament_cmake_catch2</test_depend>

And the following lines in ``CMakeLists.txt``:

.. code-block:: cmake

  find_package(ament_cmake_catch2 REQUIRED)
  ament_add_catch2(<test-name> <test-file>)
  target_link_libraries(<test-name> <library-name>)

.. note::

  If you are using a system wide installation of ``Catch2``, remember to add
  the package with ``find_package()``:

  .. code-block:: cmake

    find_package(Catch2)

Example
~~~~~~~~

Example usage of ``Catch2`` unit testing cound be found in the `rtpkg repository <https://github.com/1487quantum/rtpkg>`__:

- CMakeLists.txt
- package.xml
- test/main_catch.cpp

Python Unit Test
----------------

Pytest
^^^^^^

Add the following lines to your ``package.xml``.
This way dependencies can be automatically installed when ``rosdep`` is
called.

.. code-block:: xml

   <test_depend>python3-pytest</test_depend>


Also Add the following line to your ``setup.py``.
Make sure you add it inside the ``setup`` function inside ``setup.py``

.. code-block:: python3

   test_require=['pytest'],

Lastly, simply put your unit test files inside the ``test`` folder and name
your file correctly (``test_*.py``).
The unit tests will be invoked automatically with the static tests.
