# SOEM for ROS2

**Table of Contents**

- [SOEM ROS2 Package Upgrade](#SOEM-ROS-Package-Upgrade)
- [Package Description](#Package-Description)
- [Installation](#Installation)
- [Building the package](#Building the package)
- [Usage](#Usage)

# SOEM ROS2 Package Upgrade
This package is a thin ROS2 wrapper of the **Simple Open EtherCAT Master (SOEM)**. It is based on the ROS1 wrapper from http://wiki.ros.org/soem
https://github.com/mgruhler/soem. The main difference is that ROS2 uses the `colcon` build system, and ROS1 the `catkin` build system.
This package allows you to use `soem` from within your regular ROS2 workspace.

# Package Description
SOEM is an open source EtherCAT master library written in C.

SOEM has originally been hosted at http://developer.berlios.de/projects/soem/
but has been moved to [GitHub and the OpenEtherCATsociety organisation](
https://github.com/OpenEtherCATsociety/SOEM).


**Disclaimer**:
This package is not a development package for SOEM, but rather a wrapper to provide SOEM to ROS2.

All bug reports regarding the original SOEM source code should go to the bugtracker at
https://github.com/OpenEtherCATsociety/SOEM/issues.

All ROS related issues may not be adressed. I put it out here in the hope that someone else will maintain it, so basically it is what it is.

# Installation
You have to build soem by building this package because there is no ros-<ROS2 DISTRO>-soem binary available. 

# Building the package

## Create workspace
First create a ROS2 workspace, including a src-directory.

Then goto the src-directory and clone this repository.

## Clone a repository
Use these steps to clone from SourceTree, our client for using the repository command-line free. Cloning allows you to work on your files locally. If you don't yet have SourceTree, [download and install first](https://www.sourcetreeapp.com/). If you prefer to clone from the command line, see [Clone a repository](https://confluence.atlassian.com/x/4whODQ).

1. You’ll see the clone button under the **Source** heading. Click that button.

2. Now click **Check out in SourceTree**. You may need to create a SourceTree account or log in.

3. When you see the **Clone New** dialog in SourceTree, update the destination path and name if you’d like to and then click **Clone**.

4. Open the directory you just created to see your repository’s files.

## Build
Go back to the workspace directory.
Bulld using:
```
colcon build
```

# Usage
To use `soem` in your ROS2 package add the following to your `package.xml`and `CMakeLists.txt`, respectively.

In your `package.xml` add:

```xml
  <depend>soem</depend>
```

and in your `CMakeLists.txt`, add it to `find_package` as shown:

```CMake
find_package(soem REQUIRED)
```

Better yet, there is an example application ether_ros2 that is a simple ROS2-node in which some analog values are polled. It is advised to observe the code including the `package.xml`and `CMakeLists.txt` for your convenience.
