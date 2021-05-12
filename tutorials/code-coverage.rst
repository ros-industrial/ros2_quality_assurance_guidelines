Code Coverage
=============

.. note::
   This is an **experimental** feature.

For ease-of-use and an alternative, you can check out the colcon `mixin plugins <https://github.com/colcon/colcon-mixin-repository>`__.
To setup mixin, a quick guide is avaiable at :ref:`mxsetup`.

Prerequisites
-------------

Firstly, compile the repository with the following flags::

   colcon build --cmake-args -DCMAKE_C_FLAGS='--coverage' \
   -DCMAKE_CXX_FLAGS='--coverage'

After the compilation has complete, run the pytests with coverage enabled::

   colcon test --pytest-with-coverage \
   --pytest-args --cov-report=term \
   --event-handlers console_direct+

Via mixin
^^^^^^^^^

.. note::
   Ensure that colcon mixin is installed before proceeding. Checkout the quickguide on how to setup mixin here: :ref:`mxsetup`.

This is an alternative way that uses ``mixin`` to compile and setup the test, which would be similar to the steps mentioned aboved.

To build the package::

   colcon build --mixin coverage-gcc

After that, run ``colcon test``::

      colcon test --mixin coverage-pytest --event-handlers console_direct+

C++ Code Coverage
-----------------

To generate the code coverage report for the ``cpp`` files, the ``colcon-lcov-result`` plugin would be used. More details on the plugin could be found `here <https://github.com/colcon/colcon-lcov-result>`__.

Install the plugin via ``apt``::

   sudo apt install lcov			# Prerequiste 
   sudo apt install python3-colcon-lcov-result  # The colcon plugin

After the plugin is installed, the report could be generated via the following command::

   colcon lcov-result

A new directory would be generated and the report would be available in the ``lcov/`` directory as ``index.html``.

Python Code Coverage
--------------------

For python files, the report is generated inside the ``build/<package-name>/`` directory as ``coverage.html``.
