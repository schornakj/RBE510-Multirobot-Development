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
include CMakeFiles/Assignment1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Assignment1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Assignment1.dir/flags.make

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o: CMakeFiles/Assignment1.dir/flags.make
CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o: src/Assignment1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o -c /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment1.cpp

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Assignment1.dir/src/Assignment1.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment1.cpp > CMakeFiles/Assignment1.dir/src/Assignment1.cpp.i

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Assignment1.dir/src/Assignment1.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment1.cpp -o CMakeFiles/Assignment1.dir/src/Assignment1.cpp.s

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.requires:

.PHONY : CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.requires

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.provides: CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.requires
	$(MAKE) -f CMakeFiles/Assignment1.dir/build.make CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.provides.build
.PHONY : CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.provides

CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.provides.build: CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o


CMakeFiles/Assignment1.dir/src/planner.cpp.o: CMakeFiles/Assignment1.dir/flags.make
CMakeFiles/Assignment1.dir/src/planner.cpp.o: src/planner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Assignment1.dir/src/planner.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Assignment1.dir/src/planner.cpp.o -c /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/planner.cpp

CMakeFiles/Assignment1.dir/src/planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Assignment1.dir/src/planner.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/planner.cpp > CMakeFiles/Assignment1.dir/src/planner.cpp.i

CMakeFiles/Assignment1.dir/src/planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Assignment1.dir/src/planner.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/planner.cpp -o CMakeFiles/Assignment1.dir/src/planner.cpp.s

CMakeFiles/Assignment1.dir/src/planner.cpp.o.requires:

.PHONY : CMakeFiles/Assignment1.dir/src/planner.cpp.o.requires

CMakeFiles/Assignment1.dir/src/planner.cpp.o.provides: CMakeFiles/Assignment1.dir/src/planner.cpp.o.requires
	$(MAKE) -f CMakeFiles/Assignment1.dir/build.make CMakeFiles/Assignment1.dir/src/planner.cpp.o.provides.build
.PHONY : CMakeFiles/Assignment1.dir/src/planner.cpp.o.provides

CMakeFiles/Assignment1.dir/src/planner.cpp.o.provides.build: CMakeFiles/Assignment1.dir/src/planner.cpp.o


# Object files for target Assignment1
Assignment1_OBJECTS = \
"CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o" \
"CMakeFiles/Assignment1.dir/src/planner.cpp.o"

# External object files for target Assignment1
Assignment1_EXTERNAL_OBJECTS =

Assignment1: CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o
Assignment1: CMakeFiles/Assignment1.dir/src/planner.cpp.o
Assignment1: CMakeFiles/Assignment1.dir/build.make
Assignment1: /usr/local/lib/libopencv_stitching.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_superres.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_videostab.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_aruco.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_bgsegm.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_bioinspired.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_ccalib.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_dnn.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_dpm.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_fuzzy.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_line_descriptor.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_optflow.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_plot.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_reg.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_saliency.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_stereo.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_structured_light.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_surface_matching.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_tracking.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_xfeatures2d.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_ximgproc.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_xobjdetect.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_xphoto.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_shape.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_rgbd.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_calib3d.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_video.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_datasets.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_face.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_text.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_features2d.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_flann.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_objdetect.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_ml.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_highgui.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_photo.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_videoio.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_imgcodecs.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_imgproc.3.1.0.dylib
Assignment1: /usr/local/lib/libopencv_core.3.1.0.dylib
Assignment1: CMakeFiles/Assignment1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Assignment1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Assignment1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Assignment1.dir/build: Assignment1

.PHONY : CMakeFiles/Assignment1.dir/build

CMakeFiles/Assignment1.dir/requires: CMakeFiles/Assignment1.dir/src/Assignment1.cpp.o.requires
CMakeFiles/Assignment1.dir/requires: CMakeFiles/Assignment1.dir/src/planner.cpp.o.requires

.PHONY : CMakeFiles/Assignment1.dir/requires

CMakeFiles/Assignment1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Assignment1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Assignment1.dir/clean

CMakeFiles/Assignment1.dir/depend:
	cd /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles/Assignment1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Assignment1.dir/depend

