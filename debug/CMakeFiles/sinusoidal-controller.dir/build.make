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
CMAKE_SOURCE_DIR = /home/john/GearedUR10

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/john/GearedUR10/debug

# Include any dependencies generated for this target.
include CMakeFiles/sinusoidal-controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sinusoidal-controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sinusoidal-controller.dir/flags.make

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o: CMakeFiles/sinusoidal-controller.dir/flags.make
CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o: ../sinusoidal-controller.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/john/GearedUR10/debug/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o -c /home/john/GearedUR10/sinusoidal-controller.cpp

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/john/GearedUR10/sinusoidal-controller.cpp > CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.i

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/john/GearedUR10/sinusoidal-controller.cpp -o CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.s

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.requires:
.PHONY : CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.requires

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.provides: CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.requires
	$(MAKE) -f CMakeFiles/sinusoidal-controller.dir/build.make CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.provides.build
.PHONY : CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.provides

CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.provides.build: CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o

# Object files for target sinusoidal-controller
sinusoidal__controller_OBJECTS = \
"CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o"

# External object files for target sinusoidal-controller
sinusoidal__controller_EXTERNAL_OBJECTS =

libsinusoidal-controller.so: CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o
libsinusoidal-controller.so: CMakeFiles/sinusoidal-controller.dir/build.make
libsinusoidal-controller.so: /usr/local/lib/libMoby.so
libsinusoidal-controller.so: /usr/local/lib/libRavelin.so
libsinusoidal-controller.so: CMakeFiles/sinusoidal-controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared module libsinusoidal-controller.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sinusoidal-controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sinusoidal-controller.dir/build: libsinusoidal-controller.so
.PHONY : CMakeFiles/sinusoidal-controller.dir/build

CMakeFiles/sinusoidal-controller.dir/requires: CMakeFiles/sinusoidal-controller.dir/sinusoidal-controller.cpp.o.requires
.PHONY : CMakeFiles/sinusoidal-controller.dir/requires

CMakeFiles/sinusoidal-controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sinusoidal-controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sinusoidal-controller.dir/clean

CMakeFiles/sinusoidal-controller.dir/depend:
	cd /home/john/GearedUR10/debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/john/GearedUR10 /home/john/GearedUR10 /home/john/GearedUR10/debug /home/john/GearedUR10/debug /home/john/GearedUR10/debug/CMakeFiles/sinusoidal-controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sinusoidal-controller.dir/depend

