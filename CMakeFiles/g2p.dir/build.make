# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.6.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.6.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing

# Include any dependencies generated for this target.
include CMakeFiles/g2p.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/g2p.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/g2p.dir/flags.make

CMakeFiles/g2p.dir/src/g2p.cpp.o: CMakeFiles/g2p.dir/flags.make
CMakeFiles/g2p.dir/src/g2p.cpp.o: src/g2p.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/g2p.dir/src/g2p.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g2p.dir/src/g2p.cpp.o -c /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/g2p.cpp

CMakeFiles/g2p.dir/src/g2p.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2p.dir/src/g2p.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/g2p.cpp > CMakeFiles/g2p.dir/src/g2p.cpp.i

CMakeFiles/g2p.dir/src/g2p.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2p.dir/src/g2p.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/g2p.cpp -o CMakeFiles/g2p.dir/src/g2p.cpp.s

CMakeFiles/g2p.dir/src/g2p.cpp.o.requires:

.PHONY : CMakeFiles/g2p.dir/src/g2p.cpp.o.requires

CMakeFiles/g2p.dir/src/g2p.cpp.o.provides: CMakeFiles/g2p.dir/src/g2p.cpp.o.requires
	$(MAKE) -f CMakeFiles/g2p.dir/build.make CMakeFiles/g2p.dir/src/g2p.cpp.o.provides.build
.PHONY : CMakeFiles/g2p.dir/src/g2p.cpp.o.provides

CMakeFiles/g2p.dir/src/g2p.cpp.o.provides.build: CMakeFiles/g2p.dir/src/g2p.cpp.o


# Object files for target g2p
g2p_OBJECTS = \
"CMakeFiles/g2p.dir/src/g2p.cpp.o"

# External object files for target g2p
g2p_EXTERNAL_OBJECTS =

g2p: CMakeFiles/g2p.dir/src/g2p.cpp.o
g2p: CMakeFiles/g2p.dir/build.make
g2p: CMakeFiles/g2p.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable g2p"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2p.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/g2p.dir/build: g2p

.PHONY : CMakeFiles/g2p.dir/build

CMakeFiles/g2p.dir/requires: CMakeFiles/g2p.dir/src/g2p.cpp.o.requires

.PHONY : CMakeFiles/g2p.dir/requires

CMakeFiles/g2p.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/g2p.dir/cmake_clean.cmake
.PHONY : CMakeFiles/g2p.dir/clean

CMakeFiles/g2p.dir/depend:
	cd /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles/g2p.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/g2p.dir/depend

