# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build

# Include any dependencies generated for this target.
include CMakeFiles/test_linvel_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_linvel_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_linvel_controller.dir/flags.make

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o: CMakeFiles/test_linvel_controller.dir/flags.make
CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o: ../tests/test_linvel_controller.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o   -c /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/tests/test_linvel_controller.c

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/tests/test_linvel_controller.c > CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.i

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/tests/test_linvel_controller.c -o CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.s

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.requires:

.PHONY : CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.requires

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.provides: CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.requires
	$(MAKE) -f CMakeFiles/test_linvel_controller.dir/build.make CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.provides.build
.PHONY : CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.provides

CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.provides.build: CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o


CMakeFiles/test_linvel_controller.dir/src/client.c.o: CMakeFiles/test_linvel_controller.dir/flags.make
CMakeFiles/test_linvel_controller.dir/src/client.c.o: ../src/client.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/test_linvel_controller.dir/src/client.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_linvel_controller.dir/src/client.c.o   -c /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/client.c

CMakeFiles/test_linvel_controller.dir/src/client.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_linvel_controller.dir/src/client.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/client.c > CMakeFiles/test_linvel_controller.dir/src/client.c.i

CMakeFiles/test_linvel_controller.dir/src/client.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_linvel_controller.dir/src/client.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/client.c -o CMakeFiles/test_linvel_controller.dir/src/client.c.s

CMakeFiles/test_linvel_controller.dir/src/client.c.o.requires:

.PHONY : CMakeFiles/test_linvel_controller.dir/src/client.c.o.requires

CMakeFiles/test_linvel_controller.dir/src/client.c.o.provides: CMakeFiles/test_linvel_controller.dir/src/client.c.o.requires
	$(MAKE) -f CMakeFiles/test_linvel_controller.dir/build.make CMakeFiles/test_linvel_controller.dir/src/client.c.o.provides.build
.PHONY : CMakeFiles/test_linvel_controller.dir/src/client.c.o.provides

CMakeFiles/test_linvel_controller.dir/src/client.c.o.provides.build: CMakeFiles/test_linvel_controller.dir/src/client.c.o


CMakeFiles/test_linvel_controller.dir/src/simulator.c.o: CMakeFiles/test_linvel_controller.dir/flags.make
CMakeFiles/test_linvel_controller.dir/src/simulator.c.o: ../src/simulator.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/test_linvel_controller.dir/src/simulator.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_linvel_controller.dir/src/simulator.c.o   -c /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/simulator.c

CMakeFiles/test_linvel_controller.dir/src/simulator.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_linvel_controller.dir/src/simulator.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/simulator.c > CMakeFiles/test_linvel_controller.dir/src/simulator.c.i

CMakeFiles/test_linvel_controller.dir/src/simulator.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_linvel_controller.dir/src/simulator.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/simulator.c -o CMakeFiles/test_linvel_controller.dir/src/simulator.c.s

CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.requires:

.PHONY : CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.requires

CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.provides: CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.requires
	$(MAKE) -f CMakeFiles/test_linvel_controller.dir/build.make CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.provides.build
.PHONY : CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.provides

CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.provides.build: CMakeFiles/test_linvel_controller.dir/src/simulator.c.o


CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o: CMakeFiles/test_linvel_controller.dir/flags.make
CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o: ../src/vehicle.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o   -c /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/vehicle.c

CMakeFiles/test_linvel_controller.dir/src/vehicle.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_linvel_controller.dir/src/vehicle.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/vehicle.c > CMakeFiles/test_linvel_controller.dir/src/vehicle.c.i

CMakeFiles/test_linvel_controller.dir/src/vehicle.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_linvel_controller.dir/src/vehicle.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/vehicle.c -o CMakeFiles/test_linvel_controller.dir/src/vehicle.c.s

CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.requires:

.PHONY : CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.requires

CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.provides: CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.requires
	$(MAKE) -f CMakeFiles/test_linvel_controller.dir/build.make CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.provides.build
.PHONY : CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.provides

CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.provides.build: CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o


CMakeFiles/test_linvel_controller.dir/src/controller.c.o: CMakeFiles/test_linvel_controller.dir/flags.make
CMakeFiles/test_linvel_controller.dir/src/controller.c.o: ../src/controller.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/test_linvel_controller.dir/src/controller.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_linvel_controller.dir/src/controller.c.o   -c /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/controller.c

CMakeFiles/test_linvel_controller.dir/src/controller.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_linvel_controller.dir/src/controller.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/controller.c > CMakeFiles/test_linvel_controller.dir/src/controller.c.i

CMakeFiles/test_linvel_controller.dir/src/controller.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_linvel_controller.dir/src/controller.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/src/controller.c -o CMakeFiles/test_linvel_controller.dir/src/controller.c.s

CMakeFiles/test_linvel_controller.dir/src/controller.c.o.requires:

.PHONY : CMakeFiles/test_linvel_controller.dir/src/controller.c.o.requires

CMakeFiles/test_linvel_controller.dir/src/controller.c.o.provides: CMakeFiles/test_linvel_controller.dir/src/controller.c.o.requires
	$(MAKE) -f CMakeFiles/test_linvel_controller.dir/build.make CMakeFiles/test_linvel_controller.dir/src/controller.c.o.provides.build
.PHONY : CMakeFiles/test_linvel_controller.dir/src/controller.c.o.provides

CMakeFiles/test_linvel_controller.dir/src/controller.c.o.provides.build: CMakeFiles/test_linvel_controller.dir/src/controller.c.o


# Object files for target test_linvel_controller
test_linvel_controller_OBJECTS = \
"CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o" \
"CMakeFiles/test_linvel_controller.dir/src/client.c.o" \
"CMakeFiles/test_linvel_controller.dir/src/simulator.c.o" \
"CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o" \
"CMakeFiles/test_linvel_controller.dir/src/controller.c.o"

# External object files for target test_linvel_controller
test_linvel_controller_EXTERNAL_OBJECTS =

test_linvel_controller: CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/src/client.c.o
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/src/simulator.c.o
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/src/controller.c.o
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/build.make
test_linvel_controller: CMakeFiles/test_linvel_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C executable test_linvel_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_linvel_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_linvel_controller.dir/build: test_linvel_controller

.PHONY : CMakeFiles/test_linvel_controller.dir/build

CMakeFiles/test_linvel_controller.dir/requires: CMakeFiles/test_linvel_controller.dir/tests/test_linvel_controller.c.o.requires
CMakeFiles/test_linvel_controller.dir/requires: CMakeFiles/test_linvel_controller.dir/src/client.c.o.requires
CMakeFiles/test_linvel_controller.dir/requires: CMakeFiles/test_linvel_controller.dir/src/simulator.c.o.requires
CMakeFiles/test_linvel_controller.dir/requires: CMakeFiles/test_linvel_controller.dir/src/vehicle.c.o.requires
CMakeFiles/test_linvel_controller.dir/requires: CMakeFiles/test_linvel_controller.dir/src/controller.c.o.requires

.PHONY : CMakeFiles/test_linvel_controller.dir/requires

CMakeFiles/test_linvel_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_linvel_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_linvel_controller.dir/clean

CMakeFiles/test_linvel_controller.dir/depend:
	cd /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2 /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2 /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build /home/racecar/Documents/StudentRepo_ABadillo/Assignment_2/build/CMakeFiles/test_linvel_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_linvel_controller.dir/depend

