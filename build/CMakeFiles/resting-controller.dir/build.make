# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/john/motioncontrolproject/motion-control-manipulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/john/motioncontrolproject/motion-control-manipulator/build

# Include any dependencies generated for this target.
include CMakeFiles/resting-controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/resting-controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/resting-controller.dir/flags.make

CMakeFiles/resting-controller.dir/resting-controller.cpp.o: CMakeFiles/resting-controller.dir/flags.make
CMakeFiles/resting-controller.dir/resting-controller.cpp.o: ../resting-controller.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/john/motioncontrolproject/motion-control-manipulator/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/resting-controller.dir/resting-controller.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/resting-controller.dir/resting-controller.cpp.o -c /home/john/motioncontrolproject/motion-control-manipulator/resting-controller.cpp

CMakeFiles/resting-controller.dir/resting-controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/resting-controller.dir/resting-controller.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/john/motioncontrolproject/motion-control-manipulator/resting-controller.cpp > CMakeFiles/resting-controller.dir/resting-controller.cpp.i

CMakeFiles/resting-controller.dir/resting-controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/resting-controller.dir/resting-controller.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/john/motioncontrolproject/motion-control-manipulator/resting-controller.cpp -o CMakeFiles/resting-controller.dir/resting-controller.cpp.s

CMakeFiles/resting-controller.dir/resting-controller.cpp.o.requires:
.PHONY : CMakeFiles/resting-controller.dir/resting-controller.cpp.o.requires

CMakeFiles/resting-controller.dir/resting-controller.cpp.o.provides: CMakeFiles/resting-controller.dir/resting-controller.cpp.o.requires
	$(MAKE) -f CMakeFiles/resting-controller.dir/build.make CMakeFiles/resting-controller.dir/resting-controller.cpp.o.provides.build
.PHONY : CMakeFiles/resting-controller.dir/resting-controller.cpp.o.provides

CMakeFiles/resting-controller.dir/resting-controller.cpp.o.provides.build: CMakeFiles/resting-controller.dir/resting-controller.cpp.o

# Object files for target resting-controller
resting__controller_OBJECTS = \
"CMakeFiles/resting-controller.dir/resting-controller.cpp.o"

# External object files for target resting-controller
resting__controller_EXTERNAL_OBJECTS =

libresting-controller.so: CMakeFiles/resting-controller.dir/resting-controller.cpp.o
libresting-controller.so: CMakeFiles/resting-controller.dir/build.make
libresting-controller.so: /usr/local/lib/libMoby.so
libresting-controller.so: /usr/local/lib/libRavelin.so
libresting-controller.so: CMakeFiles/resting-controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared module libresting-controller.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/resting-controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/resting-controller.dir/build: libresting-controller.so
.PHONY : CMakeFiles/resting-controller.dir/build

CMakeFiles/resting-controller.dir/requires: CMakeFiles/resting-controller.dir/resting-controller.cpp.o.requires
.PHONY : CMakeFiles/resting-controller.dir/requires

CMakeFiles/resting-controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/resting-controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/resting-controller.dir/clean

CMakeFiles/resting-controller.dir/depend:
	cd /home/john/motioncontrolproject/motion-control-manipulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/john/motioncontrolproject/motion-control-manipulator /home/john/motioncontrolproject/motion-control-manipulator /home/john/motioncontrolproject/motion-control-manipulator/build /home/john/motioncontrolproject/motion-control-manipulator/build /home/john/motioncontrolproject/motion-control-manipulator/build/CMakeFiles/resting-controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/resting-controller.dir/depend

