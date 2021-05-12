.. rosi-ap_ci documentation master file, created by
   sphinx-quickstart on Sun Oct  4 16:17:25 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ROS 2 Quality Assurance Documentation
=====================================

This document contains detailed instructions and tutorial for improving
ROS 2 package quality, follow REP-2004 quality level and integrating CI
into the development workflow.

.. toctree::
   :maxdepth: 2
   :caption: Introduction

   basic/overview

.. toctree::
   :maxdepth: 2
   :caption: Software Testing

   tutorials/dependencies
   tutorials/static-analysis
   tutorials/unit-test
   tutorials/google-sanitizer
   tutorials/code-coverage

.. toctree::
   :maxdepth: 2
   :caption: Quality Declaration

   quality-declaration/version-policy
   quality-declaration/change-control-process
   quality-declaration/documentation
   quality-declaration/testing
   quality-declaration/dependencies
   quality-declaration/platform-support
   quality-declaration/security


.. toctree::
   :maxdepth: 2
   :caption: Continuous Integration

   ci/industrial-ci
   ci/github-action
   ci/travis-ci
   ci/gitlab-ci
   ci/code-coverage
   ci/buildfarm

.. toctree::
   :maxdepth: 2
   :caption: Guide 

   faq/mixin-setup




