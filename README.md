# ROS 2 Continuous Integration

This is the beta version of **ROS 2.0 Continuous Integration** (CI) **Template**. This document contains instructions for integrating **Travis CI** into the development workflow of **ROS 2.0 packages** under ROS-Industrial Consortium Asia Pacific.

----

## Table of Contents
1. [Continuous Integration](#Continuous-Integration)
    
    1a. [Prerequisite](#prerequisite)
    
    1b. [Docker](#docker)
    
    1c. [OSRF Docker Image](#OSRF-Docker-Image)

2. [ROS 2 Static Analysis and Dynamic Analysis with Ament package tool](#ROS-2-Static-Analysis-and-Dynamic-Analysis-with-Ament-package-tool)

    <!--2a. [Implementation](#implementation)-->

3. [Travis CI for Github](#Travis-CI-for-Github)

    3a. [Directory Structure](#Directory-Structure)

    3b. [Instructions](#INSTRUCTIONs)

    3c. [Automated Processes of Travis CI Pipeline](#Automated-Processes-of-Travis-CI-Pipeline)

4. [Further Improvements](#further-improvements)    

5. [Contributors](#Contributors)

6. [Reference](#Reference)

----

## Continuous Integration

Continuous Integration (CI) is the practice of automating the integration of code changes from multiple contributors into a single software project.

By integrating regularly, you can detect and locate errors quickly.

Some practices for CI are:

- Maintain a single-source repository
- Automate the build
- Make your build self-testing
- Build every commit on an integration machine
- Keep the build fast
- Test in a clone of the production environment
- Make it easy for users to get the latest executable version
- Visualise for users
- Automate deployment

### **Prerequisite**
- Basic knowledge about **Docker** and **Dockerfile**
- Instantiation of a (Linux) server (either physical or virtual) running Docker
- Creation of a `.yml` file
- Basic knowledge about **bash** script
- Basic knowledge on how your CI platform works (like **Travis Ci**, **Gitlab Ci** and etc.. )
- Basic knowledge about Statics Analysis
- Basic knowledge about Dynamic Analysis
- Basic knowledge about Code Coverage

### **Docker**
 ***Docker*** is a set of **Platform-as-a-Service** (PaaS) products that uses Operating System (OS)-level virtualization to deliver software in packages called containers. 
 
 ***Docker*** enables you to have the same environment for development and production across varying hardware platforms a user may choose.

### **OSRF Docker Image**
Open Robotics hosts ROS Docker Images on Docker Hub. 

The images are built both ROS 1.0 and 2.0 from the Official library by including additional meta-packages such for desktop installations.

So, we will not require to make your own "**Dockerfile**" from sratch.

List of images available at [this link](https://hub.docker.com/r/osrf/ros/tags).

## **Coveralls.io**
**[Coveralls.io](https://coveralls.io/)** is a hosted analysis tool, providing statistics on your project's code coverage.

Configuring your Travis CI build to send results to **Coveralls.io** always follows the same pattern shown below:

1. Add your repository to **Coveralls.io**.
2. Configure your build to install the **Coveralls.io** library for the programming language you are using.
3. Add **Coveralls.io** to your test suite.
4. If you’re using Travis CI for private repos, add service_name: travis-pro to your .coveralls.yml.

For more info, please refer to [this link](https://docs.travis-ci.com/user/coveralls/).

----
## ROS 2 Static Analysis and Dynamic Analysis with **Ament** package tool
<*Not applicable to current template*>
<!--
In ROS 1, it use ***catkin*** tool to execute ***Static Analysis*** and ***Dynamic Analysis*** as part of the package build procedure.

In ROS 2, it will required to use the integration capabilities of **ament** to execute ***Static Analysis*** as part of the package build procedure. 

### **Implementation**
- Insert into the packages CMakeLists.txt file
```
...
include_directories(include 
    ${CMAKE_CURRENT_BINARY_DIR}
    /usr/include/gtest
    /usr/src/gtest)
...
link_directories(/usr/lib)
...
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    ...
endif()
...
```
-  Insert the ament_lint test and gtest dependencies into the packages package.xml file
```
...
<package format="2">
  ...
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  ...
</package>
```
-->
----

## Travis CI for Github

### **Directory Structure**
Below is the directory structure of the example package with the ROS 2 CI template.

    |-- #ros2 workspace
        |-- build                       
        |-- install    
        |-- log
        |-- src
            |-- #example_package
            |-- .travis.yml
            |-- travis.sh
            |-- travis_2.sh
            |-- extra_dependencies.sh
            |-- travis_functions.sh
            |-- util.sh


### **Instructions**
`Please look at the files that had marked *TODO* below to make change to fulfil your CI pipelines`

1. Activate CI for your github repository on [Travis CI](https://travis-ci.org/). Select ***SIGN UP WITH GITHUB***.

2. Add files, as shown below, to your repository root directory.
- **/.travis.yml**

    - Add a ***.travis.yml*** file to tell Travis CI what to do. Commit and push to trigger a Travis CI build. *Travis only runs builds on the commits push after added a .travis.yml file.*

    ```yml
    dist: bionic
    services:
        - docker
    language: cpp
    os:
        - linux
    cache: ccache
    compiler:
        - gcc

    env:
    global:
        - ROS_REPO=ros
        - ROS_DISTRO=dashing        #TODO #change here for other version of ROS 2 like eloquent
    jobs:
        - SCRIPT=travis.sh

    before_script:
        - ln -s . .ci_ros2 

    script:
        - . .ci_ros2/$SCRIPT
    ```
    - after running this yml, travis will start to run ***/travis.sh***

- **/travis.sh**
    
    - This script is mainly created to select the docker image and start to run docker
    - The main parent commmand here is
        - *docker pull*
        - *docker run*
    - If you need to change the version of ROS2 docker, please change the ROS2 version you required at ***.travis.yml***
    - after running this script file, travis will start to run ***travis_2.sh***

- **/travis_2.sh** [*TODO*]
    - After *travis.sh*, docker image is created, travis will start to build and test within the container (docker) to check whether "it compile in different enviroment"
    - This script is mainly created to:

        - Update system  ( ****function update_system()*** )

        - Prepare run early tests ( ****function prepare_or_run_early_tests()*** )

        - Install and run xvfb to allow for X11-based unittests on DISPLAY ( ****function run_xvfb()*** )

        - Prepare ROS 2.0 workspace to download your upstream package into ROS 2.0 workspace ( ****function prepare_ros_workspace()*** )

        - build ROS 2.0 workspace ( ****function build_workspace()*** )
        
        - Test workspace ( ****function test_workspace()*** ) [*TODO*]
            
            a. go to *function test_workspace()* section, you will find *TODO* section
            
            b. To execute Unit Testing of CPP in docker using gtest
            
            ```
                travis_run_wait ros2 run <PACKAGE_NAME> <TEST_EXECUTABLE_NAME>

                # Example:
                #    travis_run_wait ros2 run packml_sm packml_sm_utest
            ```
            
            c. To execute unit testing of Python in docker

            ```
                travis_run_wait python3 -m coverage run $ROS_WS/src/<DIRECTORY-OF-PYTHON-FILE>.py

                # Example:
                #    travis_run_wait python3 -m coverage run $ROS_WS/src/packml_ros2/packml_plc/packml_plc/test_packml_plc_listener.py
            ```
            
            - As **Unit Testing** will taking longer time to execute, Prefix a command, that does not produce output for more than 10 minutes, with *travis_run_wait*.

            d. To execute **lcov** and send it over to **Coveralls.io**.

            ```
                   #TODO#
                    # To Create Code Coverage Report
                    ## 1. Create Initial Code Coverage Info 
                    ## 2. Capture Executed Code Coverage Info
                    ## 3. Combined Initial Coverage Info and Executed Info
                    ## 4. Filtered out data which is not belong to this workspace
                    ## 5. Deleted all the data under build folder and generated ProjectCoverage Info
                    #### COMMENT OUT Below if needed
                    # travis_run_simple cd /root/
                    # travis_run_simple mkdir test
                    # travis_run cd $ROS_WS
                    # travis_run lcov -c  --initial --rc lcov_branch_coverage=1 --directory build --output-file /root/test/initialcoverage.info
                    # travis_run lcov -c  --rc lcov_branch_coverage=1 --directory build --output-file /root/test/testcoverage.info
                    # travis_run lcov -a /root/test/initialcoverage.info -a /root/test/testcoverage.info --rc lcov_branch_coverage=1 --o /root/test/fullcoverage.info
                    # travis_run lcov -r /root/test/fullcoverage.info '*/ros_ws/build/*' '*/ros_ws/install/*' '*/ros_ws/log/*' '*/usr/*' '*/opt/*' --rc lcov_branch_coverage=1 --output-file /root/test/projectcoverage.info
                    
                    #TODO#
                    ## Upload Code coverage to Coveralls.io
                    ## Change <YOUR_TOKEN> to your own repo token at coveralls.io
                    #### COMMENT OUT Below if needed
                    # travis_run_wait coveralls-lcov --repo-token "<YOUR_TOKEN>" /root/test/projectcoverage.info


            ```


- **/extra_dependencies.sh** [*TODO*]
    
    - When your project need extra dependencies other than ROS package, eg. qt5, you are require to edit the command below to the last part of *extra_dependencies.sh*
    ```
    travis_run apt-get -qq install -y <PACKAGE_NAME>

    # Example
    # travis_run apt-get -qq install -y qt5-default
    ```
- /travis_functions.sh
    
    - Travis helper function script which fall in **util.sh** file, source from travis-ci project.
    
    - Mainly is to track the docker run start time and end time

- /util.sh
    
    - Travis helper function script.

    - Mainly is to run command in Travis with nice folding display, timing, timeout etc.


### **Automated Processes of Travis CI Pipeline**
![Travis WorkFlow Image](https://blog.travis-ci.com/images/2017-11-30-travis_ansible_github-docker-workflow.png)

----
## Further Improvements

Current Method of executing Statics Analysis, Dynamic Analysis and Code Coverage through Travis run

1. Will need to develop a template to execute Statics Analysis, Dynamic Analysis and Code Coverage with using **Ament Package tool**

2. To integrate python code coverage file with C++ code coverage file 
    Solution:
    - Implementing **Ament Package tool** to execute Statics Analysis, Dynamic Analysis and COde Coverage might help to solve this.


----

## Contributors
- Dejanira Araiza Illan
- Alex Chua Zhi Hao

----

## Reference
- ROS 2 Quality Guide

    https://index.ros.org/doc/ros2/Contributing/Quality-Guide/

- Catkin Build Documentation

    https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html

- ROS Docker Image

    https://hub.docker.com/r/osrf/ros/

- Travis CI Documentation

    https://docs.travis-ci.com/user/tutorial/

    https://blog.travis-ci.com/2017-11-30-testing-ansible-roles-using-docker-on-travis

- ROS-Industrial/Industrial CI
    
    https://github.com/ros-industrial/industrial_ci

- Continuous Integration

    https://www.atlassian.com/continuous-delivery/continuous-integration
