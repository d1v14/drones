# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dmitry/drones/task5/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmitry/drones/task5/build

# Include any dependencies generated for this target.
include uav_controller/CMakeFiles/testing.dir/depend.make

# Include the progress variables for this target.
include uav_controller/CMakeFiles/testing.dir/progress.make

# Include the compile flags for this target's objects.
include uav_controller/CMakeFiles/testing.dir/flags.make

uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.o: uav_controller/CMakeFiles/testing.dir/flags.make
uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.o: /home/dmitry/drones/task5/src/uav_controller/src/uav_controller_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/task5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.o"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/uav_controller_node.cpp.o -c /home/dmitry/drones/task5/src/uav_controller/src/uav_controller_node.cpp

uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/uav_controller_node.cpp.i"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/task5/src/uav_controller/src/uav_controller_node.cpp > CMakeFiles/testing.dir/src/uav_controller_node.cpp.i

uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/uav_controller_node.cpp.s"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/task5/src/uav_controller/src/uav_controller_node.cpp -o CMakeFiles/testing.dir/src/uav_controller_node.cpp.s

uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.o: uav_controller/CMakeFiles/testing.dir/flags.make
uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.o: /home/dmitry/drones/task5/src/uav_controller/src/uav_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/task5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.o"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testing.dir/src/uav_controller.cpp.o -c /home/dmitry/drones/task5/src/uav_controller/src/uav_controller.cpp

uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testing.dir/src/uav_controller.cpp.i"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/task5/src/uav_controller/src/uav_controller.cpp > CMakeFiles/testing.dir/src/uav_controller.cpp.i

uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testing.dir/src/uav_controller.cpp.s"
	cd /home/dmitry/drones/task5/build/uav_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/task5/src/uav_controller/src/uav_controller.cpp -o CMakeFiles/testing.dir/src/uav_controller.cpp.s

# Object files for target testing
testing_OBJECTS = \
"CMakeFiles/testing.dir/src/uav_controller_node.cpp.o" \
"CMakeFiles/testing.dir/src/uav_controller.cpp.o"

# External object files for target testing
testing_EXTERNAL_OBJECTS =

/home/dmitry/drones/task5/devel/lib/uav_controller/testing: uav_controller/CMakeFiles/testing.dir/src/uav_controller_node.cpp.o
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: uav_controller/CMakeFiles/testing.dir/src/uav_controller.cpp.o
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: uav_controller/CMakeFiles/testing.dir/build.make
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/libroscpp.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/librosconsole.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/librostime.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /opt/ros/noetic/lib/libcpp_common.so
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/dmitry/drones/task5/devel/lib/uav_controller/testing: uav_controller/CMakeFiles/testing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dmitry/drones/task5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/dmitry/drones/task5/devel/lib/uav_controller/testing"
	cd /home/dmitry/drones/task5/build/uav_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
uav_controller/CMakeFiles/testing.dir/build: /home/dmitry/drones/task5/devel/lib/uav_controller/testing

.PHONY : uav_controller/CMakeFiles/testing.dir/build

uav_controller/CMakeFiles/testing.dir/clean:
	cd /home/dmitry/drones/task5/build/uav_controller && $(CMAKE_COMMAND) -P CMakeFiles/testing.dir/cmake_clean.cmake
.PHONY : uav_controller/CMakeFiles/testing.dir/clean

uav_controller/CMakeFiles/testing.dir/depend:
	cd /home/dmitry/drones/task5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmitry/drones/task5/src /home/dmitry/drones/task5/src/uav_controller /home/dmitry/drones/task5/build /home/dmitry/drones/task5/build/uav_controller /home/dmitry/drones/task5/build/uav_controller/CMakeFiles/testing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : uav_controller/CMakeFiles/testing.dir/depend

