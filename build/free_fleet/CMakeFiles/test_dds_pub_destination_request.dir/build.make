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
CMAKE_SOURCE_DIR = /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rmf-tester/ncs_ws/build/free_fleet

# Include any dependencies generated for this target.
include CMakeFiles/test_dds_pub_destination_request.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_dds_pub_destination_request.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_dds_pub_destination_request.dir/flags.make

CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o: CMakeFiles/test_dds_pub_destination_request.dir/flags.make
CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o: /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/tests/test_dds_pub_destination_request.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmf-tester/ncs_ws/build/free_fleet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o -c /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/tests/test_dds_pub_destination_request.cpp

CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/tests/test_dds_pub_destination_request.cpp > CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.i

CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/tests/test_dds_pub_destination_request.cpp -o CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.s

CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o: CMakeFiles/test_dds_pub_destination_request.dir/flags.make
CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o: /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/dds_utils/common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmf-tester/ncs_ws/build/free_fleet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o -c /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/dds_utils/common.cpp

CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/dds_utils/common.cpp > CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.i

CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/dds_utils/common.cpp -o CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.s

CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o: CMakeFiles/test_dds_pub_destination_request.dir/flags.make
CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o: /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/messages/FleetMessages.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmf-tester/ncs_ws/build/free_fleet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o   -c /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/messages/FleetMessages.c

CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/messages/FleetMessages.c > CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.i

CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet/src/messages/FleetMessages.c -o CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.s

# Object files for target test_dds_pub_destination_request
test_dds_pub_destination_request_OBJECTS = \
"CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o" \
"CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o" \
"CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o"

# External object files for target test_dds_pub_destination_request
test_dds_pub_destination_request_EXTERNAL_OBJECTS =

test_dds_pub_destination_request: CMakeFiles/test_dds_pub_destination_request.dir/src/tests/test_dds_pub_destination_request.cpp.o
test_dds_pub_destination_request: CMakeFiles/test_dds_pub_destination_request.dir/src/dds_utils/common.cpp.o
test_dds_pub_destination_request: CMakeFiles/test_dds_pub_destination_request.dir/src/messages/FleetMessages.c.o
test_dds_pub_destination_request: CMakeFiles/test_dds_pub_destination_request.dir/build.make
test_dds_pub_destination_request: /opt/ros/foxy/lib/x86_64-linux-gnu/libddsc.so.0.7.0
test_dds_pub_destination_request: CMakeFiles/test_dds_pub_destination_request.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmf-tester/ncs_ws/build/free_fleet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable test_dds_pub_destination_request"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_dds_pub_destination_request.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_dds_pub_destination_request.dir/build: test_dds_pub_destination_request

.PHONY : CMakeFiles/test_dds_pub_destination_request.dir/build

CMakeFiles/test_dds_pub_destination_request.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_dds_pub_destination_request.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_dds_pub_destination_request.dir/clean

CMakeFiles/test_dds_pub_destination_request.dir/depend:
	cd /home/rmf-tester/ncs_ws/build/free_fleet && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet /home/rmf-tester/ncs_ws/src/free_fleet/free_fleet /home/rmf-tester/ncs_ws/build/free_fleet /home/rmf-tester/ncs_ws/build/free_fleet /home/rmf-tester/ncs_ws/build/free_fleet/CMakeFiles/test_dds_pub_destination_request.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_dds_pub_destination_request.dir/depend

