Google Sanitizer
================

ROS 2 also provides the tools to perform dynamic analysis for your projects, which focuses on finding data races and deadlocks. 
The **address sanitiser** and **thread sanitiser** are available to perform the dynamic analysis, and the usage of these tools would be further elaborated in the following sections below.

.. note::
   Ensure that colcon mixin is installed if you want to make use of it. Checkout the quickguide on how to setup ``mixin`` here: :ref:`mxsetup`.

Prerequistes
------------

(Recommended) If you want to use the santiser-report plugin, the ``colcon-sanitiser-reports`` plugin needs to be installed. It is a plugin for ``colcon test`` that that parses sanitizer issues from stdout/stderr, deduplicates the issues, and outputs them to a CSV::
   
   git clone https://github.com/colcon/colcon-sanitizer-reports.git
   cd colcon-sanitizer-reports
   sudo python3 setup.py install

Otherwise, you could skip this step and omit the ``--event-handlers`` flag later on if you did not install the ``colcon-sanitizer-reports`` plugin.

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

Example
-------
We would be using the `packml_ros2 <https://github.com/1487quantum/packml_ros2>`__ in this example, looking at the address sanitiser, followed by the thread sanitiser. Let’s git clone the repository::

   git clone https://github.com/1487quantum/packml_ros2.git ~/packml_ros2       # Clone the repo
   cd ~/packml_ros2                                                             # Enter the dir

**Address Sanitiser**

Compile the package with the ``build-asn`` flags::
   
   colcon build --build-base=build-asan --install-base=install-asan \
   --cmake-args -DCMAKE_BUILD_TYPE=Debug \
   -DCMAKE_C_FLAGS='-fsanitize=address' \
   -DCMAKE_CXX_FLAGS='-fsanitize=address'

After that, run the test(s) with the following CMake flags as shown below.

.. code-block:: bash

   colcon test --build-base=build-asan --install-base=install-asan --event-handlers sanitizer_report+

The test logs could be found in the ``log/latest_test`` directory. The following example below displays the 3 lines after the beginning of a ASAN reported issue::

   cd ~/packml_ros2/log/latest_test
   grep -R '==.*==ERROR: .*Sanitizer' -A 3 # Displays three lines after the beginning of a ASAN reported issue.

The output would look something like this::

   $ grep -R '==.*==ERROR: .*Sanitizer' -A 3
   events.log:[45.324988] (packml_ros) StdoutLine: {'line': b'1: ==88592==ERROR: LeakSanitizer: detected memory leaks\n'}
   events.log-[45.325039] (packml_ros) StdoutLine: {'line': b'1: \n'}
   events.log-[45.325091] (packml_ros) StdoutLine: {'line': b'1: Direct leak of 56 byte(s) in 1 object(s) allocated from:\n'}
   events.log-[45.325165] (packml_ros) StdoutLine: {'line': b'1:     #0 0x7f0dacca6bc8 in malloc (/lib/x86_64-linux-gnu/libasan.so.5+0x10dbc8)\n'}
   --
   packml_ros/stdout.log:1: ==88592==ERROR: LeakSanitizer: detected memory leaks
   packml_ros/stdout.log-1: 
   packml_ros/stdout.log-1: Direct leak of 56 byte(s) in 1 object(s) allocated from:
   packml_ros/stdout.log-1:     #0 0x7f0dacca6bc8 in malloc (/lib/x86_64-linux-gnu/libasan.so.5+0x10dbc8)
   --
   packml_ros/streams.log:[21.380s] 1: ==88592==ERROR: LeakSanitizer: detected memory leaks
   packml_ros/streams.log-[21.380s] 1: 
   packml_ros/streams.log-[21.380s] 1: Direct leak of 56 byte(s) in 1 object(s) allocated from:
   packml_ros/streams.log-[21.380s] 1:     #0 0x7f0dacca6bc8 in malloc (/lib/x86_64-linux-gnu/libasan.so.5+0x10dbc8)
   --
   packml_ros/stdout_stderr.log:1: ==88592==ERROR: LeakSanitizer: detected memory leaks
   packml_ros/stdout_stderr.log-1: 
   packml_ros/stdout_stderr.log-1: Direct leak of 56 byte(s) in 1 object(s) allocated from:
   packml_ros/stdout_stderr.log-1:     #0 0x7f0dacca6bc8 in malloc (/lib/x86_64-linux-gnu/libasan.so.5+0x10dbc8)

**Thread Sanitiser**

We'll now move on to the Thread Sanitiser. To run the *thread sanitiser*, the steps are similar to those of the address sanitiser, with some differences in the flag. Return to the root directory and remove the build files::

   cd ~/packml_ros2
   rm -rf build-asan/ log/ install-asan/ sanitizer_report.csv test_results.xml

After that, compile the packages::

   colcon build --build-base=build-tsan --install-base=install-tsan \
   --cmake-args -DCMAKE_BUILD_TYPE=Debug \
   -DCMAKE_C_FLAGS='-fsanitize=thread -O2 -g -fno-omit-frame-pointer' \
   -DCMAKE_CXX_FLAGS='-fsanitize=thread -O2 -g -fno-omit-frame-pointer'
 
Once compiled, run the test(s)::

   colcon test --build-base=build-tsan --install-base=install-tsan --event-handlers sanitizer_report+

The log would be available in the ``log/latest_test`` directory::

   cd ~/packml_ros2/log/latest_test
