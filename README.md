# REPORT
Implementation of a generalized publisher and subscriber ROS package

## Setup
1. Create a catkin workspace in Melodic or Noetic installed computer and run the command 
	```
	https://github.com/konyalioglu/turan_konyalioglu_17012023.git
	```
2. Then make the project using catkin_make or catkin build.

3. In order to launch the project run the command below.
	```
	roslaunch composiv_tryouts composiv_tryouts.launch
	```

## Repository

A new repository is initialized regarding given format in the case document.

## Environment Setup

ROS official web site is analyzed and ROS Melodic is selected to implement the given case. 

To accomplish that a docker image is pulled using following command:

```
docker pull ros:melodic
```

Then, the image can be run by using following command:

```
docker run -it ros:melodic-robot
```

and in the parallel terminal the following command can be used.

```
docker exec -it {CONTAINER_ID} bash
```

{CONTAINER_ID} of running container can be retrieved by using the command

```
docker ps
```
 
## ROS Package

A catkin workspace called case_ws is created by using following commands:

```
mkdir case_ws
mkdir case_ws/src
cd case_ws
catkin_make
```

Then, a package called 'composiv_tryouts' is created with appropriate CMakeLists.txt and package.xml files. The following commands were used:
```
mkdir compsive_tryouts
touch compsive_tryouts/CMakelist.txt
touch compsive_tryouts/Package.xml
```

Because there should be two different node running parallel for subscriber-publisher application, two .cpp files are created under compsive_tryouts/src folder, named composiv_talker for publisher and composiv_listener for subscriber.

Lastly, a launch file is created under the composiv_tryouts/launch directory.


After launching the package, the nodes that will be run can be seen in the ROS Node Graph.

![ROS Node Graph](https://github.com/konyalioglu/turan_konyalioglu_17012023/blob/main/rosgraph.png)

## Code Explanation

### CMakeLists.txt
The CMakeLists.txt is a input file to CMake build system. It is mostly used for relatively large projects.

```
cmake_minimum_required(VERSION 2.8.3)
```

Defines the required CMake version.

```
project(case_tryouts)
```

This line defines the project name which must be same as the name in the package.xml file.

```
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
```

Finds the CMake or Catkin packages which are needed to build this ROS package.

```
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs)
```

Whereas the function catkin_package() specifies the build export information, the CATKIN_DEPENDS defines the dependent packages for the current project.

```
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})


add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

Finally, the add_executable() function declare the executables for the each node and target_link_libraries() specifies the library which the executables will be linked.  


### package.xml

The package.xml file defines the properties of the package.

Such as,
	- Package name
	- Version number
	- Authors
	- Dependencies

The parameters must be conformed with the CMakeLists.txt file.


### listener.cpp 

"listener.cpp" is a C++ code for a listener node in this ROS package. It subscribe to a topic defined by a ros param which is assigned in the launch file.

The further details are given in the file. 


### talker.cpp

"talker.cpp is a C++ code for a talker node in this ROS package. In this node, a timer and a publisher are created to perform some specific tasks. As the timer triggered in the time period which is defined in the launch file, the node publishes a message in the format of std_msgs::String. On the other hand, the node also retrieve the topic name and message data as the ROS param.

The further details are given in the file. 


### composiv_tryouts.launch

The "composiv_tryouts.launch" is a launch file for this project. In this, launch file talker and listener executables are run as it is described in the file itself. Also, ROS params can be defined in the launch file. 

The further details are given in the file. 



