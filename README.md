# ROS 2 Continuous Integration

This is the beta version of **ROS 2.0 Continuous Integration** (CI) **Template**. This document contains instructions for integrating **CI** into the development workflow of **ROS 2.0 packages** under ROS-Industrial Consortium Asia Pacific.

----

## Table of Contents
1. [Basics](#basics)
    
    1a. [Prerequisite](#prerequisite)
    
    1b. [Docker](#docker)
    
    1c. [OSRF Docker Image](#osrf-docker-image)

2. [Preparation](#preparation)
   
    2a. [Configure Dependencies](#configure-dependencies)

    2b. [Static Analysis and Unit Tests](#static-analysis-and-unit-tests)

    2c. [Dynamic Analysis](#dynamic-analysis)

    2d, [Check Coverage](#check-code-coverage)

3. [Continuous Integration](#continuous-integration-with-industrial-ci)

    3a. [Travis CI for Github](#travis-ci)

    3b. [Gitlab CI for Gitlab](#gitlab-ci)

    3c. [Common Issues](#common-issues)

4. [Experimental](#experimental)    

5. [Contributors](#contributors)

6. [Reference](#reference)

----

## Basics

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

## Preparation
### Configure Dependencies
There are typically 3 ways to setup dependencies in ROS2. **rosdep** and **vcstool** should **always be prioritized**.
1. **rosdep**
   
   Most **pip**, **ros official release** and **standard deb(apt)** **binary** dependencies can be installed using `rosdep`. However, you should **always** run the following command in your workspace directory to check if there are dependencies wrongly defined in `package.xml`.

        cd <workspace-directory>
        rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
 

2. **vcstool**
   
   Most public **pip**, **cmake** and **ament(ros2)** dependencies that required building from source can be imported via `vcstool` and build with `colcon build`. A simple tutorial can be found here [here](https://github.com/dirk-thomas/vcstool). You run the following command to check if you have configure correctly.

        cd <workspace-directory>
        vcs import --recursive src < my.repos
   
   Here is an example of the `.repos` file
   
   ```yaml
   repositories:
     rmf_core:
       type: git
       url: https://github.com/osrf/rmf_core.git
       version: master
  
    rmf_schedule_visualizer:
      type: git
      url: https://github.com/osrf/rmf_schedule_visualizer.git
      version: master
  
    traffic_editor:
      type: git
      url: https://github.com/osrf/traffic_editor.git
      version: master
   ```
3. **Others**
   
   For other dependencies that are not under **standard debian(apt)** or can not be built using `colcon build`, document the setup process in detail and setup accordingly in the CI as instructed [here](#common-issues).


### Static Analysis and Unit Tests
#### Ament Cpp
1. **Recommended file structure**
        
        |-- #package_name
            |-- include/package_name    <- header files
            |-- src                     <- .cpp files
            |-- test                    <- unit test files
            |-- CMakeLists.txt
            |-- package.xml

2. **Ament Lint Auto**
   
    To run the static analysis, add or uncomment the following lines in `CMakeLists.txt`,

    ```cmake
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # uncomment the line when a copyright and license is not present in all source files
      #set(ament_cmake_copyright_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()
    ```

    add or uncomment these lines in `package.xml`,
        
    ```xml
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>
    ```

    Run the build and test command to invoke static analysis,

        colcon build
        colcon test --packages-select <package_name> --event-handlers console_direct+

    In a cpp package, the following tests will run by default,
    - copyright
    - cppcheck
    - cpplint
    - lint_cmake
    - uncrustify
    - xmllint
    
    If you have conflicts between uncrustify and cpplint, you can simply disable the cpplint check for specific line by commenting `// NOLINT(error_name)` at the end.


3. **gtest**

    To run gtest, simply put these two line within the `BUILD_TESTING` block in `CMakeLists.txt`,
        
    ```cmake
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(<test-name> <test-file>)
    ament_target_dependencies(<test-name> rclcpp <other-ament-dependencies>)
    target_link_libraries(<test-name> <library-name>)
    ```

    Add this line to `package.xml`,

    ```xml
    <test_depend>ament_cmake_gtest</test_depend>
    ```

    Run the build and test command to invoke gtest.

        colcon build
        colcon test --packages-select <package_name> --event-handlers console_direct+
    



4. **gmock**

    To run gtest, simply put these two line within the `BUILD_TESTING` block in `CMakeLists.txt`,

    ```cmake 
    find_package(ament_cmake_gmock REQUIRED)
    ament_add_gmock(<test-name> <test-file>)
    ament_target_dependencies(<test-name> rclcpp <other-ament-dependencies>)
    target_link_libraries(<test-name> <library-name>)
    ```

    Add this line to `package.xml`,

    ```xml
    <test_depend>ament_cmake_gmock</test_depend>
    ```

    Run the build and test command to invoke gmock.

        colcon build
        colcon test --packages-select <package_name> --event-handlers console_direct+



#### Ament Python

`pytest` is recommended, although `unittest` and `nosetest` are all supported.
1. **Recommended file structure**
        
        |-- #package_name
            |-- library_name            <- .py files
            |-- resource                <- data files
            |-- test                    <- static analysis and unit tests
            |-- package.xml
            |-- setup.cfg
            |-- setup.py

2. **Static Analysis**
   
    The recommended lists of static tests to run are listed as follows.
      - copyright
      - flake8
      - pep257
      - xmllint
    
    You may also run additional tests like spellcheck if you want.
    
    Add dependencies in your `package.xml` file,

    ```xml
    <test_depend>ament_copyright</test_depend>
    <test_depend>ament_flake8</test_depend>
    <test_depend>ament_pep257</test_depend>
    <test_depend>ament_xmllint</test_depend>
    <test_depend>python3-pytest</test_depend>
    <test_depend>ros_testing</test_depend>
    ```

    Specify the tests required to run inside the `setup` function in `setup.py`,

    ```py
    tests_require=['pytest'],
    ```

    Copy the following [files](./doc/test) in your `test` folder.
    
    Run the build and test command to invoke static analysis,

        colcon build
        colcon test --packages-select <package_name> --event-handlers console_direct+


3. **unit test**
   
   Simply put your unit test files inside the `test` folder and name your file correctly. The unit tests will be invoked automatically with the static tests.

### Dynamic Analysis
`TODO`
### Check Code Coverage
`TODO` 

Check out [Experimental](#experimental)


## Continuous Integration with Industrial CI

### **Travis CI**

1. Activate CI for your github repository on [Travis CI](https://travis-ci.com/). Select ***SIGN UP WITH GITHUB***.

2. Add a `.travis.yml` file to tell Travis CI what to do. Commit and push to trigger a Travis CI build. *Travis only runs builds on push and PR after added a .travis.yml file.*

    ```yml
    language: generic
    services:
      - docker

    env:
      global:
        - PARALLEL_TESTS=true  # Global Variable

      matrix:
        - ROS_DISTRO="dashing" ROS_REPO=testing
        - ROS_DISTRO="dashing" ROS_REPO=main
        - ROS_DISTRO="eloquent" ROS_REPO=testing
        - ROS_DISTRO="eloquent" ROS_REPO=main
        - ROS_DISTRO="foxy" # Using osrf/nightly-build docker image

    install:
      - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci.git .industrial_ci -b master
    script:
      - .industrial_ci/travis.sh

    ```


- **Automated Processes of Travis CI Pipeline**
![Travis WorkFlow Image](https://blog.travis-ci.com/images/2017-11-30-travis_ansible_github-docker-workflow.png)

### **Gitlab CI**
1. Add a `.gitlab-ci.yml` file to tell Travis CI what to do. Commit and push to trigger a Gitlab CI build. *Gitlab CI only runs builds on push and PR after added a .gitlab-ci.yml file.*
    ```yml
    image: docker:git
    services:
      - docker:dind
    before_script:
      - apk add --update bash coreutils tar curl
      - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci .industrial_ci -b master
     
    variables:
      PARALLEL_TESTS: "true"  # Global Variable

    dashing-main:
      script: .industrial_ci/gitlab.sh ROS_DISTRO=dashing ROS_REPO=main

    dashing-testing:
      script: .industrial_ci/gitlab.sh ROS_DISTRO=dashing ROS_REPO=testing

    eloquent-main:
      script: .industrial_ci/gitlab.sh ROS_DISTRO=eloquent ROS_REPO=main

    eloquent-testing:
      script: .industrial_ci/gitlab.sh ROS_DISTRO=eloquent ROS_REPO=testing

    # Using osrf/nightly-build docker image
    foxy:
      script: .industrial_ci/gitlab.sh ROS_DISTRO=foxy
    ```

### **Common Issues**
#### **Dependencies in CI**

Based on the [3 ways of configuring dependencies](#configure-dependencies), here are how they can be setup respectively in industrial CI.
1. **rosdep**
        
    No need additional setup. `rosdep` will **always** run before building packages.
2. **vcstool**
   
    Set the global variable **UPSTREAM_WORKSPACE** to the relative path of the `.repos` file. For example, in `.travis-ci.yml`
    ```yml
    env:
      global:
        - UPSTREAM_WORKSPACE="<path-to>/my.repos" 
    ```
3. **Others**

    If it is additional debian packages, you can put it under **ADDITIONAL_DEBS** as a variable using space separate them.
    ```yml
    env:
      global:
        - UPSTREAM_WORKSPACE="<path-to>/my.repos" 
    ```
    
    Otherwise, You need to write your own bash script for the setup and put its relative path inside global variable **BEFORE_INIT**. You can find more details [here](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-pre-post-process-custom-commands)

#### **(Gitlab)Access to Private Repository**
   
You can follow the instructions [here](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#gitlab-ci-access-to-private-repositories).

#### **Custom DockerFile**

You can follow the instructions [here](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#use-custom-docker-images).


----
## Experimental

- Using various tools provided by `colcon` to generate coverage report. [link](https://gitlab.com/ROSI-AP/rosi-ap_ci/blob/experimental#check-code-coverage)
- Upload coverage report to codecov.io. [link](https://gitlab.com/ROSI-AP/rosi-ap_ci/blob/experimental#continuous-integration-with-industrial-ci)



----

## Contributors
- Alex Chua Zhi Hao
- Chen Bainian
- Dejanira Araiza Illan


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
