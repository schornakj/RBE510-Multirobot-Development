# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /Users/multirobot/Desktop/Team_3/RBE510-Multirobot-Development

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/multirobot/Desktop/Team_3/RBE510-Multirobot-Development

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/local/Cellar/cmake/3.6.2/bin/ccmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/local/Cellar/cmake/3.6.2/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/multirobot/Desktop/Team_3/RBE510-Multirobot-Development/CMakeFiles /Users/multirobot/Desktop/Team_3/RBE510-Multirobot-Development/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/multirobot/Desktop/Team_3/RBE510-Multirobot-Development/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named Assignment2

# Build rule for target.
Assignment2: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Assignment2
.PHONY : Assignment2

# fast build rule for target.
Assignment2/fast:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/build
.PHONY : Assignment2/fast

src/Assignment2.o: src/Assignment2.cpp.o

.PHONY : src/Assignment2.o

# target to build an object file
src/Assignment2.cpp.o:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o
.PHONY : src/Assignment2.cpp.o

src/Assignment2.i: src/Assignment2.cpp.i

.PHONY : src/Assignment2.i

# target to preprocess a source file
src/Assignment2.cpp.i:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/Assignment2.cpp.i
.PHONY : src/Assignment2.cpp.i

src/Assignment2.s: src/Assignment2.cpp.s

.PHONY : src/Assignment2.s

# target to generate assembly for a file
src/Assignment2.cpp.s:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/Assignment2.cpp.s
.PHONY : src/Assignment2.cpp.s

src/planner.o: src/planner.cpp.o

.PHONY : src/planner.o

# target to build an object file
src/planner.cpp.o:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/planner.cpp.o
.PHONY : src/planner.cpp.o

src/planner.i: src/planner.cpp.i

.PHONY : src/planner.i

# target to preprocess a source file
src/planner.cpp.i:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/planner.cpp.i
.PHONY : src/planner.cpp.i

src/planner.s: src/planner.cpp.s

.PHONY : src/planner.s

# target to generate assembly for a file
src/planner.cpp.s:
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/planner.cpp.s
.PHONY : src/planner.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... Assignment2"
	@echo "... src/Assignment2.o"
	@echo "... src/Assignment2.i"
	@echo "... src/Assignment2.s"
	@echo "... src/planner.o"
	@echo "... src/planner.i"
	@echo "... src/planner.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

