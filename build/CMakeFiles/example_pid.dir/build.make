# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/adrianguel/Documents/CodeControl/OpenPID

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adrianguel/Documents/CodeControl/OpenPID/build

# Include any dependencies generated for this target.
include CMakeFiles/example_pid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/example_pid.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/example_pid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_pid.dir/flags.make

CMakeFiles/example_pid.dir/examples/example_pid.cpp.o: CMakeFiles/example_pid.dir/flags.make
CMakeFiles/example_pid.dir/examples/example_pid.cpp.o: /home/adrianguel/Documents/CodeControl/OpenPID/examples/example_pid.cpp
CMakeFiles/example_pid.dir/examples/example_pid.cpp.o: CMakeFiles/example_pid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adrianguel/Documents/CodeControl/OpenPID/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_pid.dir/examples/example_pid.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/example_pid.dir/examples/example_pid.cpp.o -MF CMakeFiles/example_pid.dir/examples/example_pid.cpp.o.d -o CMakeFiles/example_pid.dir/examples/example_pid.cpp.o -c /home/adrianguel/Documents/CodeControl/OpenPID/examples/example_pid.cpp

CMakeFiles/example_pid.dir/examples/example_pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/example_pid.dir/examples/example_pid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adrianguel/Documents/CodeControl/OpenPID/examples/example_pid.cpp > CMakeFiles/example_pid.dir/examples/example_pid.cpp.i

CMakeFiles/example_pid.dir/examples/example_pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/example_pid.dir/examples/example_pid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adrianguel/Documents/CodeControl/OpenPID/examples/example_pid.cpp -o CMakeFiles/example_pid.dir/examples/example_pid.cpp.s

# Object files for target example_pid
example_pid_OBJECTS = \
"CMakeFiles/example_pid.dir/examples/example_pid.cpp.o"

# External object files for target example_pid
example_pid_EXTERNAL_OBJECTS =

example_pid: CMakeFiles/example_pid.dir/examples/example_pid.cpp.o
example_pid: CMakeFiles/example_pid.dir/build.make
example_pid: libpid_lib.a
example_pid: CMakeFiles/example_pid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/adrianguel/Documents/CodeControl/OpenPID/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_pid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_pid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_pid.dir/build: example_pid
.PHONY : CMakeFiles/example_pid.dir/build

CMakeFiles/example_pid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_pid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_pid.dir/clean

CMakeFiles/example_pid.dir/depend:
	cd /home/adrianguel/Documents/CodeControl/OpenPID/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrianguel/Documents/CodeControl/OpenPID /home/adrianguel/Documents/CodeControl/OpenPID /home/adrianguel/Documents/CodeControl/OpenPID/build /home/adrianguel/Documents/CodeControl/OpenPID/build /home/adrianguel/Documents/CodeControl/OpenPID/build/CMakeFiles/example_pid.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/example_pid.dir/depend

