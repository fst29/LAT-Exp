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
CMAKE_SOURCE_DIR = /home/pi/git/Armstrong/SpringFriction

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/git/Armstrong/SpringFriction/build

# Include any dependencies generated for this target.
include CMakeFiles/example_pos.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_pos.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_pos.dir/flags.make

CMakeFiles/example_pos.dir/example_pos.cpp.o: CMakeFiles/example_pos.dir/flags.make
CMakeFiles/example_pos.dir/example_pos.cpp.o: ../example_pos.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/git/Armstrong/SpringFriction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_pos.dir/example_pos.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_pos.dir/example_pos.cpp.o -c /home/pi/git/Armstrong/SpringFriction/example_pos.cpp

CMakeFiles/example_pos.dir/example_pos.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_pos.dir/example_pos.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/git/Armstrong/SpringFriction/example_pos.cpp > CMakeFiles/example_pos.dir/example_pos.cpp.i

CMakeFiles/example_pos.dir/example_pos.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_pos.dir/example_pos.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/git/Armstrong/SpringFriction/example_pos.cpp -o CMakeFiles/example_pos.dir/example_pos.cpp.s

# Object files for target example_pos
example_pos_OBJECTS = \
"CMakeFiles/example_pos.dir/example_pos.cpp.o"

# External object files for target example_pos
example_pos_EXTERNAL_OBJECTS =

../bin/example_pos: CMakeFiles/example_pos.dir/example_pos.cpp.o
../bin/example_pos: CMakeFiles/example_pos.dir/build.make
../bin/example_pos: ../lib/raspberry/libCTRE_Phoenix.so
../bin/example_pos: ../lib/raspberry/libCTRE_PhoenixCCI.so
../bin/example_pos: /usr/lib/libwiringPi.so
../bin/example_pos: CMakeFiles/example_pos.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/git/Armstrong/SpringFriction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/example_pos"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_pos.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/cmake -E copy_directory /home/pi/git/Armstrong/SpringFriction/lib/raspberry /home/pi/git/Armstrong/SpringFriction/bin

# Rule to build all files generated by this target.
CMakeFiles/example_pos.dir/build: ../bin/example_pos

.PHONY : CMakeFiles/example_pos.dir/build

CMakeFiles/example_pos.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_pos.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_pos.dir/clean

CMakeFiles/example_pos.dir/depend:
	cd /home/pi/git/Armstrong/SpringFriction/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/git/Armstrong/SpringFriction /home/pi/git/Armstrong/SpringFriction /home/pi/git/Armstrong/SpringFriction/build /home/pi/git/Armstrong/SpringFriction/build /home/pi/git/Armstrong/SpringFriction/build/CMakeFiles/example_pos.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_pos.dir/depend

