
Mixin setup
===========

Mixins are “configuration profiles” which can be used to compile ROS 2 with additional flags more easily, and are stored locally in ``~/.colcon``.


.. _mxsetup:

Setting up mixin
----------------

Firstly, install the ``Colcon mixin`` package::

   sudo apt-get install python3-colcon-mixin

After that, add the mixin profiles::

   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default

More information is available in the `colcon mixin <https://github.com/colcon/colcon-mixin-repository>`__ repository.
