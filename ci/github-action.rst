Industrial CI with Github Action
=================================

Setup Configuration File
-------------------------

Add a ``.yml`` config file in the :file:`.github/workflows` directory. Create
the :file:`.github/workflows` directory if it does not exists, and then the
configuration file. We would be naming the config file as ``gh-act.yml``::

   mkdir -p .github/workflows
   touch .github/workflows/gh-act.yml

After that, open the configuration file with your favorite text editor and
setup the workflow. An example configuration is as shown below::

   name: GH Actions CI
   on: [push, pull_request] # on all pushes and PRs

   jobs:
     industrial_ci:
       strategy:
         matrix:
           env:
             - {ROS_DISTRO: foxy, ROS_REPO: testing}
             - {ROS_DISTRO: foxy, ROS_REPO: main}
       env:
         CCACHE_DIR: /github/home/.ccache # Directory for ccache (and how we enable ccache in industrial_ci)
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v2
         - uses: actions/cache@v2
           with:
             path: ${{ env.CCACHE_DIR }}
             key: ccache-${{ matrix.env.ROS_DISTRO }}-${{ matrix.env.ROS_REPO }}
         - uses: 'ros-industrial/industrial_ci@master'
           env: ${{ matrix.env }}

CI Pipeline Trigger
--------------------

The Github action would trigger when there is a pull request or pull request.
(As specified in the :file:`on:` parameter.) Ensure that the
``.github/workflows/gh-act.yml`` is created for the Github Actions to run.
