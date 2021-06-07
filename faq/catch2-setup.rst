.. _cth2setup:

Catch2 setup
=============

.. note::

  We would be using version 2 branch of ``Catch2``.

In this guide, we would be looking at how to install the ``Catch2``
package system wide via ``CMake``. First, clone the repository onto your
computer:

.. code-block:: bash

  git clone https://github.com/catchorg/Catch2/ -b v2.x

After that, run ``CMake`` to compile and install the package.

.. code-block:: bash

  cd Catch2
  cmake -Bbuild -H. -DBUILD_TESTING=OFF
  sudo cmake --build build/ --target install

With that, ``Catch2`` should be installed! (The library would be installed
in the ``/usr/local/`` directory by default) More details on the CMake
customization could be found `here <https://github.com/catchorg/Catch2/blob/v2.x/docs/cmake-integration.md>`__.
