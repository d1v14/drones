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
CMAKE_SOURCE_DIR = /home/dmitry/drones/avoidance/VFH/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmitry/drones/avoidance/VFH/build

# Include any dependencies generated for this target.
include VFH/CMakeFiles/flight_with_avoidance.dir/depend.make

# Include the progress variables for this target.
include VFH/CMakeFiles/flight_with_avoidance.dir/progress.make

# Include the compile flags for this target's objects.
include VFH/CMakeFiles/flight_with_avoidance.dir/flags.make

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o: VFH/CMakeFiles/flight_with_avoidance.dir/flags.make
VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o: /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightWithAvoidanceNode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/avoidance/VFH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o -c /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightWithAvoidanceNode.cpp

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.i"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightWithAvoidanceNode.cpp > CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.i

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.s"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightWithAvoidanceNode.cpp -o CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.s

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o: VFH/CMakeFiles/flight_with_avoidance.dir/flags.make
VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o: /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightCommander.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/avoidance/VFH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o -c /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightCommander.cpp

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.i"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightCommander.cpp > CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.i

VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.s"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/avoidance/VFH/src/VFH/src/FlightCommander.cpp -o CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.s

VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o: VFH/CMakeFiles/flight_with_avoidance.dir/flags.make
VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o: /home/dmitry/drones/avoidance/VFH/src/VFH/src/Histogram.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/avoidance/VFH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o -c /home/dmitry/drones/avoidance/VFH/src/VFH/src/Histogram.cpp

VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.i"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/avoidance/VFH/src/VFH/src/Histogram.cpp > CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.i

VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.s"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/avoidance/VFH/src/VFH/src/Histogram.cpp -o CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.s

VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o: VFH/CMakeFiles/flight_with_avoidance.dir/flags.make
VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o: /home/dmitry/drones/avoidance/VFH/src/VFH/src/Avoidance.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmitry/drones/avoidance/VFH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o -c /home/dmitry/drones/avoidance/VFH/src/VFH/src/Avoidance.cpp

VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.i"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmitry/drones/avoidance/VFH/src/VFH/src/Avoidance.cpp > CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.i

VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.s"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmitry/drones/avoidance/VFH/src/VFH/src/Avoidance.cpp -o CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.s

# Object files for target flight_with_avoidance
flight_with_avoidance_OBJECTS = \
"CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o" \
"CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o" \
"CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o" \
"CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o"

# External object files for target flight_with_avoidance
flight_with_avoidance_EXTERNAL_OBJECTS =

/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightWithAvoidanceNode.cpp.o
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/src/FlightCommander.cpp.o
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/src/Histogram.cpp.o
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/src/Avoidance.cpp.o
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/build.make
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libtf2_ros.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libactionlib.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libmessage_filters.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libtf2.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libroscpp.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/librosconsole.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/librostime.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /opt/ros/noetic/lib/libcpp_common.so
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance: VFH/CMakeFiles/flight_with_avoidance.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dmitry/drones/avoidance/VFH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable /home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance"
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flight_with_avoidance.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
VFH/CMakeFiles/flight_with_avoidance.dir/build: /home/dmitry/drones/avoidance/VFH/devel/lib/VFH/flight_with_avoidance

.PHONY : VFH/CMakeFiles/flight_with_avoidance.dir/build

VFH/CMakeFiles/flight_with_avoidance.dir/clean:
	cd /home/dmitry/drones/avoidance/VFH/build/VFH && $(CMAKE_COMMAND) -P CMakeFiles/flight_with_avoidance.dir/cmake_clean.cmake
.PHONY : VFH/CMakeFiles/flight_with_avoidance.dir/clean

VFH/CMakeFiles/flight_with_avoidance.dir/depend:
	cd /home/dmitry/drones/avoidance/VFH/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmitry/drones/avoidance/VFH/src /home/dmitry/drones/avoidance/VFH/src/VFH /home/dmitry/drones/avoidance/VFH/build /home/dmitry/drones/avoidance/VFH/build/VFH /home/dmitry/drones/avoidance/VFH/build/VFH/CMakeFiles/flight_with_avoidance.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : VFH/CMakeFiles/flight_with_avoidance.dir/depend
