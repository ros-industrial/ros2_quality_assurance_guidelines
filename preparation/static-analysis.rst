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

Command Line Static Analysis
----------------------------



C++ Static Analysis
-------------------

Recommended File Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^

It is recommended to set up your ``ament_cmake`` package in with the following
file structure.::

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
There are only 2 steps to set up ``ament_lint_auto``.

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
     Please follow the `ament copyright templates`_.

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


Python Static Analysis
----------------------
