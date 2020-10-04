Industrial CI with Travis CI
============================

.. image:: https://blog.travis-ci.com/images/2017-11-30-travis_ansible_github-docker-workflow.png

Sign Up & Registration
----------------------
Activate CI for your github repository on
`Travis CI signup page <https://travis-ci.com/signup>`_.
Select **SIGN UP WITH GITHUB**.

.. note::

   :strong:`Travis CI` is :strong:`FREE for Open Source` (public projects).
   However, there is a hard time limit of
   :strong:`50 min` for a public job.
   For :strong:`private projects`, there is limited quotas of
   :strong:`100 trial builds`.
   For more information, you can find them on https://travis-ci.com/plans.

   Also note that `Github Education Pack`_ provides private builds for free
   for students.

.. _Github Education Pack: https://education.github.com/pack

Setup Configuration File
------------------------

Add a ``.travis.yml`` file to tell Travis CI what to do.
Here is an example.

.. code-block:: yaml

   language: generic
   services:
     - docker

   cache:
     directories:
       - $HOME/.ccache

   env:
     global:
       - CCACHE_DIR=$HOME/.ccache
     matrix:
       - ROS_DISTRO=foxy ROS_REPO=main
       - ROS_DISTRO=foxy ROS_REPO=testing

   install:
     -  git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci.git .industrial_ci -b master
   script:
     - .industrial_ci/travis.sh


Trigger CI Pipeline
-------------------

Commit and push to trigger a Travis CI build.
Travis only runs builds on **push** and **PR** after added a
``.travis.yml`` file.
