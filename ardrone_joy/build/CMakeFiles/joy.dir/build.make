# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jara/electric_workspace/ardrone_joy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jara/electric_workspace/ardrone_joy/build

# Include any dependencies generated for this target.
include CMakeFiles/joy.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joy.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joy.dir/flags.make

CMakeFiles/joy.dir/src/Joystick.o: CMakeFiles/joy.dir/flags.make
CMakeFiles/joy.dir/src/Joystick.o: ../src/Joystick.cpp
CMakeFiles/joy.dir/src/Joystick.o: ../manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/rospy/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/tools/rosclean/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosgraph/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosparam/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosmaster/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosout/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/roslaunch/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/ros/tools/rosunit/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rostest/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/topic_tools/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosbag/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/rosbagmigration/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/common_msgs/geometry_msgs/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /home/jara/electric_workspace/ardrone_msgs/manifest.xml
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/ros_comm/tools/topic_tools/srv_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /opt/ros/electric/stacks/common_msgs/geometry_msgs/msg_gen/generated
CMakeFiles/joy.dir/src/Joystick.o: /home/jara/electric_workspace/ardrone_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jara/electric_workspace/ardrone_joy/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/joy.dir/src/Joystick.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/joy.dir/src/Joystick.o -c /home/jara/electric_workspace/ardrone_joy/src/Joystick.cpp

CMakeFiles/joy.dir/src/Joystick.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joy.dir/src/Joystick.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/jara/electric_workspace/ardrone_joy/src/Joystick.cpp > CMakeFiles/joy.dir/src/Joystick.i

CMakeFiles/joy.dir/src/Joystick.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joy.dir/src/Joystick.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/jara/electric_workspace/ardrone_joy/src/Joystick.cpp -o CMakeFiles/joy.dir/src/Joystick.s

CMakeFiles/joy.dir/src/Joystick.o.requires:
.PHONY : CMakeFiles/joy.dir/src/Joystick.o.requires

CMakeFiles/joy.dir/src/Joystick.o.provides: CMakeFiles/joy.dir/src/Joystick.o.requires
	$(MAKE) -f CMakeFiles/joy.dir/build.make CMakeFiles/joy.dir/src/Joystick.o.provides.build
.PHONY : CMakeFiles/joy.dir/src/Joystick.o.provides

CMakeFiles/joy.dir/src/Joystick.o.provides.build: CMakeFiles/joy.dir/src/Joystick.o

# Object files for target joy
joy_OBJECTS = \
"CMakeFiles/joy.dir/src/Joystick.o"

# External object files for target joy
joy_EXTERNAL_OBJECTS =

../bin/joy: CMakeFiles/joy.dir/src/Joystick.o
../bin/joy: CMakeFiles/joy.dir/build.make
../bin/joy: CMakeFiles/joy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/joy"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joy.dir/build: ../bin/joy
.PHONY : CMakeFiles/joy.dir/build

CMakeFiles/joy.dir/requires: CMakeFiles/joy.dir/src/Joystick.o.requires
.PHONY : CMakeFiles/joy.dir/requires

CMakeFiles/joy.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joy.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joy.dir/clean

CMakeFiles/joy.dir/depend:
	cd /home/jara/electric_workspace/ardrone_joy/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jara/electric_workspace/ardrone_joy /home/jara/electric_workspace/ardrone_joy /home/jara/electric_workspace/ardrone_joy/build /home/jara/electric_workspace/ardrone_joy/build /home/jara/electric_workspace/ardrone_joy/build/CMakeFiles/joy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joy.dir/depend

