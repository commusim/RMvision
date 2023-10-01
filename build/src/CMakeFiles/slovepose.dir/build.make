# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/commusim/RM/vision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/commusim/RM/vision/build

# Include any dependencies generated for this target.
include src/CMakeFiles/slovepose.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/slovepose.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/slovepose.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/slovepose.dir/flags.make

src/CMakeFiles/slovepose.dir/PNP.cpp.o: src/CMakeFiles/slovepose.dir/flags.make
src/CMakeFiles/slovepose.dir/PNP.cpp.o: ../src/PNP.cpp
src/CMakeFiles/slovepose.dir/PNP.cpp.o: src/CMakeFiles/slovepose.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/commusim/RM/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/slovepose.dir/PNP.cpp.o"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/slovepose.dir/PNP.cpp.o -MF CMakeFiles/slovepose.dir/PNP.cpp.o.d -o CMakeFiles/slovepose.dir/PNP.cpp.o -c /home/commusim/RM/vision/src/PNP.cpp

src/CMakeFiles/slovepose.dir/PNP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slovepose.dir/PNP.cpp.i"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/commusim/RM/vision/src/PNP.cpp > CMakeFiles/slovepose.dir/PNP.cpp.i

src/CMakeFiles/slovepose.dir/PNP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slovepose.dir/PNP.cpp.s"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/commusim/RM/vision/src/PNP.cpp -o CMakeFiles/slovepose.dir/PNP.cpp.s

src/CMakeFiles/slovepose.dir/real3D.cpp.o: src/CMakeFiles/slovepose.dir/flags.make
src/CMakeFiles/slovepose.dir/real3D.cpp.o: ../src/real3D.cpp
src/CMakeFiles/slovepose.dir/real3D.cpp.o: src/CMakeFiles/slovepose.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/commusim/RM/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/slovepose.dir/real3D.cpp.o"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/slovepose.dir/real3D.cpp.o -MF CMakeFiles/slovepose.dir/real3D.cpp.o.d -o CMakeFiles/slovepose.dir/real3D.cpp.o -c /home/commusim/RM/vision/src/real3D.cpp

src/CMakeFiles/slovepose.dir/real3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slovepose.dir/real3D.cpp.i"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/commusim/RM/vision/src/real3D.cpp > CMakeFiles/slovepose.dir/real3D.cpp.i

src/CMakeFiles/slovepose.dir/real3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slovepose.dir/real3D.cpp.s"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/commusim/RM/vision/src/real3D.cpp -o CMakeFiles/slovepose.dir/real3D.cpp.s

src/CMakeFiles/slovepose.dir/recognition.cpp.o: src/CMakeFiles/slovepose.dir/flags.make
src/CMakeFiles/slovepose.dir/recognition.cpp.o: ../src/recognition.cpp
src/CMakeFiles/slovepose.dir/recognition.cpp.o: src/CMakeFiles/slovepose.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/commusim/RM/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/slovepose.dir/recognition.cpp.o"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/slovepose.dir/recognition.cpp.o -MF CMakeFiles/slovepose.dir/recognition.cpp.o.d -o CMakeFiles/slovepose.dir/recognition.cpp.o -c /home/commusim/RM/vision/src/recognition.cpp

src/CMakeFiles/slovepose.dir/recognition.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slovepose.dir/recognition.cpp.i"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/commusim/RM/vision/src/recognition.cpp > CMakeFiles/slovepose.dir/recognition.cpp.i

src/CMakeFiles/slovepose.dir/recognition.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slovepose.dir/recognition.cpp.s"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/commusim/RM/vision/src/recognition.cpp -o CMakeFiles/slovepose.dir/recognition.cpp.s

src/CMakeFiles/slovepose.dir/slovepose.cpp.o: src/CMakeFiles/slovepose.dir/flags.make
src/CMakeFiles/slovepose.dir/slovepose.cpp.o: ../src/slovepose.cpp
src/CMakeFiles/slovepose.dir/slovepose.cpp.o: src/CMakeFiles/slovepose.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/commusim/RM/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/slovepose.dir/slovepose.cpp.o"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/slovepose.dir/slovepose.cpp.o -MF CMakeFiles/slovepose.dir/slovepose.cpp.o.d -o CMakeFiles/slovepose.dir/slovepose.cpp.o -c /home/commusim/RM/vision/src/slovepose.cpp

src/CMakeFiles/slovepose.dir/slovepose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slovepose.dir/slovepose.cpp.i"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/commusim/RM/vision/src/slovepose.cpp > CMakeFiles/slovepose.dir/slovepose.cpp.i

src/CMakeFiles/slovepose.dir/slovepose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slovepose.dir/slovepose.cpp.s"
	cd /home/commusim/RM/vision/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/commusim/RM/vision/src/slovepose.cpp -o CMakeFiles/slovepose.dir/slovepose.cpp.s

# Object files for target slovepose
slovepose_OBJECTS = \
"CMakeFiles/slovepose.dir/PNP.cpp.o" \
"CMakeFiles/slovepose.dir/real3D.cpp.o" \
"CMakeFiles/slovepose.dir/recognition.cpp.o" \
"CMakeFiles/slovepose.dir/slovepose.cpp.o"

# External object files for target slovepose
slovepose_EXTERNAL_OBJECTS =

../bin/slovepose: src/CMakeFiles/slovepose.dir/PNP.cpp.o
../bin/slovepose: src/CMakeFiles/slovepose.dir/real3D.cpp.o
../bin/slovepose: src/CMakeFiles/slovepose.dir/recognition.cpp.o
../bin/slovepose: src/CMakeFiles/slovepose.dir/slovepose.cpp.o
../bin/slovepose: src/CMakeFiles/slovepose.dir/build.make
../bin/slovepose: /usr/local/lib/libopencv_gapi.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_stitching.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_alphamat.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_aruco.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_bgsegm.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_bioinspired.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_ccalib.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_dnn_objdetect.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_dnn_superres.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_dpm.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_face.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_freetype.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_fuzzy.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_hdf.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_hfs.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_img_hash.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_intensity_transform.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_line_descriptor.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_mcc.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_quality.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_rapid.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_reg.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_rgbd.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_saliency.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_stereo.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_structured_light.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_superres.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_surface_matching.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_tracking.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_videostab.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_viz.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_wechat_qrcode.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_xfeatures2d.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_xobjdetect.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_xphoto.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_shape.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_highgui.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_datasets.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_plot.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_text.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_ml.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_phase_unwrapping.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_optflow.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_ximgproc.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_video.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_videoio.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_objdetect.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_calib3d.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_dnn.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_features2d.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_flann.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_photo.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_imgproc.so.4.8.0
../bin/slovepose: /usr/local/lib/libopencv_core.so.4.8.0
../bin/slovepose: src/CMakeFiles/slovepose.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/commusim/RM/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../../bin/slovepose"
	cd /home/commusim/RM/vision/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slovepose.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/slovepose.dir/build: ../bin/slovepose
.PHONY : src/CMakeFiles/slovepose.dir/build

src/CMakeFiles/slovepose.dir/clean:
	cd /home/commusim/RM/vision/build/src && $(CMAKE_COMMAND) -P CMakeFiles/slovepose.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/slovepose.dir/clean

src/CMakeFiles/slovepose.dir/depend:
	cd /home/commusim/RM/vision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/commusim/RM/vision /home/commusim/RM/vision/src /home/commusim/RM/vision/build /home/commusim/RM/vision/build/src /home/commusim/RM/vision/build/src/CMakeFiles/slovepose.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/slovepose.dir/depend

