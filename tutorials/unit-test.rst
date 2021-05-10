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

.. TODO(Briancbn)


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
