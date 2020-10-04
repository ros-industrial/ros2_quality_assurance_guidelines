Industrial CI with Gitlab CI/CD
===============================

Setup Configuration File
------------------------

Add a ``.gitlab-ci.yml`` file to tell Travis CI what to do.
Here is an example.

.. code-block:: yaml

   workflow:
     rules:
       - if: $CI_MERGE_REQUEST_ID
       - if: $CI_COMMIT_TAG
       - if: $CI_COMMIT_BRANCH

   image: docker:git
   services:
     - docker:dind
   before_script:
     - apk add --update bash coreutils tar curl
     - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci .industrial_ci -b master

   cache:
     key: ${CI_JOB_NAME}
     paths:
       - .ccache/

   variables:
     TMPDIR: "${CI_PROJECT_DIR}.tmp"
     DOCKER_RUN_OPTS: "-v /etc/ssl/certs:/etc/ssl/certs:ro"
     CCACHE_DIR: "${CI_PROJECT_DIR}/.ccache"

   foxy-main:
     variables:
       ROS_DISTRO: "foxy"
       ROS_REPO: "main"
       script:
         - .industrial_ci/gitlab.sh

   foxy-testing:
     variables:
       ROS_DISTRO: "foxy"
       ROS_REPO: "testing"
       script:
         - .industrial_ci/gitlab.sh

Trigger CI Pipeline
-------------------

Commit and push to trigger a Travis CI build.
Gitlab CI can be configured to run on either **push** or **merge request**
base on your ``workflow:rules`` or ``only`` setting.
