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
include CMakeFiles/Assignment2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Assignment2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Assignment2.dir/flags.make

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o: CMakeFiles/Assignment2.dir/flags.make
CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o: src/Assignment2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o -c /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment2.cpp

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Assignment2.dir/src/Assignment2.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment2.cpp > CMakeFiles/Assignment2.dir/src/Assignment2.cpp.i

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Assignment2.dir/src/Assignment2.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/src/Assignment2.cpp -o CMakeFiles/Assignment2.dir/src/Assignment2.cpp.s

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.requires:

.PHONY : CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.requires

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.provides: CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.requires
	$(MAKE) -f CMakeFiles/Assignment2.dir/build.make CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.provides.build
.PHONY : CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.provides

CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.provides.build: CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o


# Object files for target Assignment2
Assignment2_OBJECTS = \
"CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o"

# External object files for target Assignment2
Assignment2_EXTERNAL_OBJECTS =

Assignment2: CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o
Assignment2: CMakeFiles/Assignment2.dir/build.make
Assignment2: /usr/local/lib/libopencv_stitching.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_superres.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_videostab.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_aruco.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_bgsegm.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_bioinspired.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_ccalib.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_dnn.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_dpm.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_fuzzy.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_line_descriptor.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_optflow.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_plot.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_reg.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_saliency.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_stereo.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_structured_light.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_surface_matching.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_tracking.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_xfeatures2d.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_ximgproc.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_xobjdetect.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_xphoto.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_shape.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_rgbd.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_calib3d.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_video.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_datasets.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_face.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_text.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_features2d.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_flann.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_objdetect.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_ml.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_highgui.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_photo.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_videoio.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_imgcodecs.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_imgproc.3.1.0.dylib
Assignment2: /usr/local/lib/libopencv_core.3.1.0.dylib
Assignment2: CMakeFiles/Assignment2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Assignment2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Assignment2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Assignment2.dir/build: Assignment2

.PHONY : CMakeFiles/Assignment2.dir/build

CMakeFiles/Assignment2.dir/requires: CMakeFiles/Assignment2.dir/src/Assignment2.cpp.o.requires

.PHONY : CMakeFiles/Assignment2.dir/requires

CMakeFiles/Assignment2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Assignment2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Assignment2.dir/clean

CMakeFiles/Assignment2.dir/depend:
	cd /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing /Users/multirobot/Desktop/jgschornak/Multi_Robotics_F2016-transform_testing/CMakeFiles/Assignment2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Assignment2.dir/depend

