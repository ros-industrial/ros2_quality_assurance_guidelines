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
