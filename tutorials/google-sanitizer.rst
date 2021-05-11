Google Sanitizer
================

ROS 2 also provides the tools to perform dynamic analysis for your projects, which focuses on finding data races and deadlocks. 
The **address sanitiser** and **thread sanitiser** are available to perform the dynamic analysis, and the usage of these tools would be further elaborated in the following sections below.

.. note::
   Ensure that colcon mixin is installed if you want to make use of it. Checkout the quickguide on how to setup ``mixin`` here: :ref:`mxsetup`.

Prerequistes
------------

Install the ``colcon-sanitiser-reports`` plugin, which is a plugin for ``colcon test`` that that parses sanitizer issues from stdout/stderr, deduplicates the issues, and outputs them to a CSV::
   
   git clone https://github.com/colcon/colcon-sanitizer-reports.git
   cd colcon-sanitizer-reports
   sudo python3 setup.py install

Address Sanitizer
-----------------

Compilation
^^^^^^^^^^^

Build the package via the following command::
 
   colcon build --build-base=build-asan --install-base=install-asan \
   --cmake-args -DCMAKE_BUILD_TYPE=Debug \
   -DCMAKE_C_FLAGS='-fsanitize=address' \
   -DCMAKE_CXX_FLAGS='-fsanitize=address'

.. note::
   Alternatively, ``mixin`` flag could be ran instead to replace the ``CMake`` flags::

    colcon build --build-base=build-asan --install-base=install-asan --mixin asan-gcc

3 directories would be generated after the compilation is complete: *build-asan, install-asan & log*

Testing
^^^^^^^

To generate the report for the *address sanitiser*, the ``colcon test`` would be used::

   colcon test --build-base=build-asan --install-base=install-asan \
   --event-handlers sanitizer_report+

2 files would be generated once the testing is done, which contains the result of the test: *test_results.xml, sanitizer_report.csv*

Thread Sanitiser
----------------

Compilation
^^^^^^^^^^^

Build the thread sanitiser with the following command::

   colcon build --build-base=build-tsan --install-base=install-tsan \
   --cmake-args -DCMAKE_BUILD_TYPE=Debug \
   -DCMAKE_C_FLAGS='-fsanitize=thread -O2 -g -fno-omit-frame-pointer' \
   -DCMAKE_CXX_FLAGS='-fsanitize=thread -O2 -g -fno-omit-frame-pointer'

.. note::
   Alternatively, ``mixin`` could be used instead::

      colcon build --build-base=build-tsan --install-base=install-tsan --mixin tsan


Testing
^^^^^^^

Run the test and generate the report with the following command::

   colcon test --build-base=build-tsan --install-base=install-tsan \
   --event-handlers sanitizer_report+

