Static Analysis
===============

ROS 2 provides the `ament_lint`_ package that simplify the process of setting
up static analysis for your project.
`ament_lint`_ provides command lines wrappers that is very convenient to
use.
It also comes with CMake wrappers and Python APIs that build with
``ament_cmake`` or ``ament_python``, which we will explain in depth in the
following section.

.. _ament_lint: https://github.com/ament/ament_lint

.. _static-analysis-command-line-static-analysis:

Command Line Static Analysis
----------------------------

The easiest way to run a static analysis with the help of ``ament_lint``
is through command line.
Each package is just wrappers over existing linter tools (
``ament_cpplint`` is a wrapper for ``cpplint``).
For example, if you want to use ``ament_cpplint`` on a ``.cpp`` file,

#. Install ``ament_cpplint``

   .. code-block:: bash

      sudo apt install ros-<distro>-ament-cpplint

#. Source the ROS environment

   .. code-block:: bash

      source /opt/ros/<distro>/setup.bash

#. Run ``ament_cpplint``

   First check all the available options for the command::

      $ ament_cpplint -h

      usage: ament_cpplint [-h] [--filters FILTER,FILTER,...] [--linelength N]
                           [--root ROOT] [--xunit-file XUNIT_FILE]
                           [paths [paths ...]]

      Check code against the Google style conventions using cpplint.

      positional arguments:
        paths                 The files or directories to check. For
                              directories files ending in '.c', '.cc', '.cpp',
                              '.cxx', '.h', '.hh', '.hpp', '.hxx' will be
                              considered. (default: ['.'])

      optional arguments:
        -h, --help            show this help message and exit
        --filters FILTER,FILTER,...
                              A comma separated list of category filters to
                              apply (default: None)
        --linelength N        The maximum line length (default: 100)
        --root ROOT           The --root option for cpplint (default: None)
        --xunit-file XUNIT_FILE
                              Generate a xunit compliant XML file (default:
                              None)

   Next simply run the command

   .. code-block:: bash

      ament_cpplint <path-to>/<filename>.cpp

C++ Static Analysis
-------------------

Recommended File Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^

It is recommended to set up your ``ament_cmake`` package in with the following
file structure::

   |-- *package_name
       |-- include/package_name    <- header files
       |-- src                     <- .cpp files
       |-- test                    <- unit test files
       |-- CMakeLists.txt
       |-- package.xml

This can also simplify a lot of other CMake functions such as linking external
libraries and installing libraries.

Automated CMake Linter Checks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The simplest way to set up the static analysis for a ``ament_cmake`` project
is to make use of the ``ament_lint_auto`` package.
There are only **2 steps** to set up ``ament_lint_auto``.

#. Modify **package.xml**

   Also add the following lines to your ``package.xml``.
   This way dependencies can be automatically installed when ``rosdep`` is
   called.

   .. code-block:: xml

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

#. Modify **CMakeLists.txt**

   Add the following lines to your ``CMakeLists.txt``.
   If these lines already exists, you can simply uncomment these lines.

   .. code-block:: cmake

      if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
      endif()

   Based on what kind of files is inside your packages, certain linter tests
   will be called.
   For example, if you have ``CMakeLists.txt``, ``package.xml``, ``.hpp`` and
   ``.cpp``, these are the tests will be run automatically.

   * ``ament_copyright``: Check whether ``LICENSE``, ``CONTRIBUTING.md`` and
     copyright notice heading is added to your source code.
     Please follow the `ament copyright templates`_ for the open source
     references.

   * ``ament_cppcheck``: This is a wrapper for ``cppcheck`` and this will be
     called if C++ files exists in the package.
     It detects undefined behavior and dangerous coding constructs.

   * ``ament_cpplint``: This is a wrapper for ``cpplint`` and this will be
     called if C++ files exists in the package.
     This will check your code aligns with what Google considers best practices
     in C++ coding.

   * ``ament_uncrustify``: This is a wrapper for ``uncrustify`` and this will
     be called if C++ files exists in the package.
     It is a source code beautifier for C, C++ code to make your code much more
     readable.

   * ``ament_lint_cmake``: This will check your ``CMakeLists.txt``.

   * ``ament_xmllint``: This is a wrapper for ``xmllint``, and it will check
     all the ``.xml`` files in your package.

   .. _ament copyright templates: https://github.com/ament/ament_lint/tree/master/ament_copyright/ament_copyright/template


Before triggering the test don't forget to install the additional packages
if you haven't.

.. code-block:: bash

   cd <path-to-workspace>
   rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

Lastly simply run the build and test command to invoke static analysis.

.. code-block:: bash

   colcon build
   colcon test --packages-select <package_name> --event-handlers console_direct+

.. note::

   You might find re-organizing your code very time consuming, especially with
   ``uncrustify``.
   Well, don't worry.
   See :ref:`static-analysis-command-line-static-analysis`.
   For ``ament_uncrustify``, you can run the
   ``ament_uncrustify --reformat <file>`` to automatically fix the linter issue
   with your file.

Example
^^^^^^^

We would be looking at ``packml_sm`` inside the `packml_ros2 <https://github.com/1487quantum/packml_ros2/>`__ package as an example.
As seen from the previous section, the package follows the similar structure
of a C++ ROS2 package::

   |-- packml_sm
       |-- include/packml_sm    <- header files
       |-- src                     <- .cpp files
       |-- test                    <- unit test files
       |-- CMakeLists.txt
       |-- package.xml

Next, let's take a look at the ``package.xml``:

.. code-block:: xml
  :emphasize-lines: 26, 27

  <?xml version="1.0"?>
  <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
  <package format="3">
    <name>packml_sm</name>
    <version>0.0.0</version>
    <description>Packml state machine</description>
    <maintainer email="chenbn@artc.a-star.edu.sg">Chen Bainian</maintainer>
    <maintainer email="dejanira.araiza.i@gmail.com">Dejanira Araiza-Illan</maintainer>
    <license>Apache-2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <build_depend>rclcpp</build_depend>
    <build_depend>rqt_gui_cpp</build_depend>
    <build_depend>qtbase5-dev</build_depend>
    <build_depend>packml_msgs</build_depend>

    <exec_depend>rclcpp</exec_depend>
    <exec_depend>libqt5-core</exec_depend>
    <exec_depend>libqt5-gui</exec_depend>
    <exec_depend>libqt5-opengl</exec_depend>
    <exec_depend>libqt5-widgets</exec_depend>
    <exec_depend>rqt_gui_cpp</exec_depend>
    <exec_depend>packml_msgs</exec_depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>
    <test_depend>ament_cmake_gtest</test_depend>

    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>

The ``test_depend`` tags are used to declare the ``ament_lint_auto`` &
``ament_lint_common`` packages to be listed as dependencies used during
testing. Next up, we would be looking at ``CMakeLists.txt``:

.. code-block:: cmake
  :emphasize-lines: 57, 58

  cmake_minimum_required(VERSION 3.5)
  project(packml_sm)

  set(CMAKE_XX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++")

  # Default to C99
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # prevents weird Qt error
  #set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rqt_gui_cpp REQUIRED)
  find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

  #include all directories
  include_directories(
    include
  )

  qt5_wrap_cpp(packml_sm_MOCS include/packml_sm/state_machine.hpp
    include/packml_sm/state.hpp
    include/packml_sm/transitions.hpp
    include/packml_sm/events.hpp
    include/packml_sm/common.hpp)

  #add libraries
  add_library(${PROJECT_NAME} SHARED
    src/state_machine.cpp
    src/state.cpp
    src/transitions.cpp
    ${packml_sm_MOCS})
  ament_target_dependencies(${PROJECT_NAME} rclcpp rqt_gui_cpp Qt5)

  #install
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
    ament_target_dependencies(${PROJECT_NAME}_utest rclcpp rqt_gui_cpp Qt5)
    target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

  endif()

  #Substituting the catkin_package () components:
  #INCLUDE_DIRS
  ament_export_include_directories(include)
  #LIBRARIES
  ament_export_libraries(${PROJECT_NAME})
  #CATKIN_DEPENDS
  ament_export_dependencies(rqt_gui_cpp)

  ament_package()

Similarly, the packages are declared after the ``BUILD_TESTING`` condition.

Python Static Analysis
----------------------

Recommended File Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^

It is recommended to set up your ``ament_python`` package in with the following
file structure::

   |-- *package_name
       |-- library_name            <- .py files
       |-- resource                <- data files
       |-- test                    <- static analysis and unit tests
       |-- package.xml
       |-- setup.cfg
       |-- setup.py

Configure Python Static Analysis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The recommended lists of static tests to run are listed as follows.

* ``ament_copyright``
* ``ament_flake8``
* ``ament_pep257``
* ``ament_xmllint``

You may also run additional tests like spellcheck if you want.

There are **3 steps** you need to follow to setup these 4 tests.

#. Modify **package.xml**

   Also add the following lines to your ``package.xml``.
   This way dependencies can be automatically installed when ``rosdep`` is
   called.

   .. code-block:: xml

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>ament_xmllint</test_depend>
      <test_depend>python3-pytest</test_depend>

#. Modify **setup.py**

   Add the following line to your ``setup.py``.
   Make sure you add it inside the ``setup`` function inside ``setup.py``

   .. code-block:: python3

      test_require=['pytest'],

#. Setup the tests

   ``pytest`` will automatically execute all the files with name that start
   with ``test``. For instance, ``test_flake8.py``. Here is an example of
   using ``flake8`` with the help of the ``ament_lint`` library.

   .. code-block:: python3

      from ament_flake8.main import main
      import pytest


      @pytest.mark.flake8
      @pytest.mark.linter
      def test_flake8():
          rc = main(argv=[])
          assert rc == 0, 'Found errors'

   Also don't forget to add the copyright header at the start of every
   ``.py`` file.
   Please follow the `ament copyright templates`_ for the open source
   references.

   You can find all 4 examples here in `the ros2run package`_

   .. _the ros2run package: https://github.com/ros2/ros2cli/tree/master/ros2run/test

Before triggering the test don't forget to install the additional packages
if you haven't.

.. code-block:: bash

   cd <path-to-workspace>
   rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

Lastly simply run the build and test command to invoke static analysis.

.. code-block:: bash

   colcon build
   colcon test --packages-select <package_name> --event-handlers console_direct+

Example setup
^^^^^^^^^^^^^^

We would be looking at ``packml_plc`` inside the `packml_ros2 <https://github.com/1487quantum/packml_ros2/>`__ package as an example.
As seen from the previous section, the package follows the similar structure
of a Python ROS2 package::

   |-- *packml_ros2
     |-- packml_plc            <- .py files
     |-- resource              <- data files
     |-- test                  <- static analysis and unit tests
     |-- package.xml
     |-- setup.cfg
     |-- setup.py

Dependencies
~~~~~~~~~~~~

Next, let's take a look at the ``package.xml``:

.. code-block:: xml
  :emphasize-lines: 16, 17, 18, 19, 20

  <?xml version="1.0"?>
  <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
  <package format="3">
    <name>packml_plc</name>
    <version>0.0.0</version>
    <description>Package to control and monitor a PackML state machine following the standard template in a Siemens PLC with OPCUA tags</description>
    <maintainer email="chenbn@artc.a-star.edu.sg">Chen Bainian</maintainer>
    <maintainer email="dejanira.araiza.i@gmail.com">Dejanira Araiza-Illan</maintainer>

    <license>Apache 2.0</license>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>rclpy</exec_depend>
    <exec_depend>packml_msgs</exec_depend>
    <exec_depend>opcua-pip</exec_depend>

    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>ament_xmllint</test_depend>
    <test_depend>python3-pytest</test_depend>
    <test_depend>ros_testing</test_depend>

    <export>
  <build_type>ament_python</build_type>
    </export>
  </package>

The ``test_depend`` tags are used to declare the packages highlighted
as dependencies used for static analysis. Next up, we would be looking
at ``setup.py``, where the ``tests_require`` field is inserted after the
license & description of the python package.

.. code-block:: python
  :emphasize-lines: 27

  ...
  package_name = 'packml_plc'
  setup(
      name=package_name,
      version='0.0.0',
      packages=find_packages(exclude=['test']),
      data_files=[
          ('share/ament_index/resource_index/packages',
              ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
      ],
      install_requires=['setuptools'],
      zip_safe=True,
      maintainer='Chen Bainian',
      maintainer_email='chenbn@artc.a-star.edu.sg',
      author='Dejanira Araiza Illan',
      author_email='dejanira.araiza.i@gmail.com',
      keywords=['ROS2'],
      classifiers=[
          'Intended Audience :: Users',
          'License :: Apache 2.0',
          'Programming Language :: Python',
          'Topic :: Software Development',
      ],
      description='Packml PLC driver',
      license='Apache 2.0',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [
              'packml_plc_listener = packml_plc.packml_plc_listener:main',
              'packml_plc_sender = packml_plc.packml_plc_sender:main',
          ],
      },
  )

Tests
~~~~~

When ``pytest`` is ran, files that start with "test" would be
automatically executed. Let's take a look at ``test/test_pep257.py``:

.. code-block:: python

  # Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.

  from ament_pep257.main import main
  import pytest


  @pytest.mark.linter
  @pytest.mark.pep257
  def test_pep257():
      rc = main(argv=[])
      assert rc == 0, 'Found code style errors / warnings'

As seen from above, the test script includes the copyright header at
the top of the test file, and it utilizes the ``ament_pep257`` library for
the test. For more examples on how to the various test files are created,
the `test` directory of the `packml_ros2 <https://github.com/1487quantum/packml_ros2/>`__
package can be referenced.
