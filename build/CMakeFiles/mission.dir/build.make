# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_SOURCE_DIR = /home/local/mission

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/local/mission/build

# Include any dependencies generated for this target.
include CMakeFiles/mission.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mission.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mission.dir/flags.make

CMakeFiles/mission.dir/main.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mission.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/main.cpp.o -c /home/local/mission/main.cpp

CMakeFiles/mission.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/main.cpp > CMakeFiles/mission.dir/main.cpp.i

CMakeFiles/mission.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/main.cpp -o CMakeFiles/mission.dir/main.cpp.s

CMakeFiles/mission.dir/src/ubridge.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/ubridge.cpp.o: ../src/ubridge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mission.dir/src/ubridge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/ubridge.cpp.o -c /home/local/mission/src/ubridge.cpp

CMakeFiles/mission.dir/src/ubridge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/ubridge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/ubridge.cpp > CMakeFiles/mission.dir/src/ubridge.cpp.i

CMakeFiles/mission.dir/src/ubridge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/ubridge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/ubridge.cpp -o CMakeFiles/mission.dir/src/ubridge.cpp.s

CMakeFiles/mission.dir/src/upose.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/upose.cpp.o: ../src/upose.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mission.dir/src/upose.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/upose.cpp.o -c /home/local/mission/src/upose.cpp

CMakeFiles/mission.dir/src/upose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/upose.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/upose.cpp > CMakeFiles/mission.dir/src/upose.cpp.i

CMakeFiles/mission.dir/src/upose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/upose.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/upose.cpp -o CMakeFiles/mission.dir/src/upose.cpp.s

CMakeFiles/mission.dir/src/ucomment.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/ucomment.cpp.o: ../src/ucomment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/mission.dir/src/ucomment.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/ucomment.cpp.o -c /home/local/mission/src/ucomment.cpp

CMakeFiles/mission.dir/src/ucomment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/ucomment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/ucomment.cpp > CMakeFiles/mission.dir/src/ucomment.cpp.i

CMakeFiles/mission.dir/src/ucomment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/ucomment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/ucomment.cpp -o CMakeFiles/mission.dir/src/ucomment.cpp.s

CMakeFiles/mission.dir/src/ustate.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/ustate.cpp.o: ../src/ustate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/mission.dir/src/ustate.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/ustate.cpp.o -c /home/local/mission/src/ustate.cpp

CMakeFiles/mission.dir/src/ustate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/ustate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/ustate.cpp > CMakeFiles/mission.dir/src/ustate.cpp.i

CMakeFiles/mission.dir/src/ustate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/ustate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/ustate.cpp -o CMakeFiles/mission.dir/src/ustate.cpp.s

CMakeFiles/mission.dir/src/uvision.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/uvision.cpp.o: ../src/uvision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/mission.dir/src/uvision.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/uvision.cpp.o -c /home/local/mission/src/uvision.cpp

CMakeFiles/mission.dir/src/uvision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/uvision.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/uvision.cpp > CMakeFiles/mission.dir/src/uvision.cpp.i

CMakeFiles/mission.dir/src/uvision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/uvision.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/uvision.cpp -o CMakeFiles/mission.dir/src/uvision.cpp.s

CMakeFiles/mission.dir/src/utime.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/utime.cpp.o: ../src/utime.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/mission.dir/src/utime.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/utime.cpp.o -c /home/local/mission/src/utime.cpp

CMakeFiles/mission.dir/src/utime.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/utime.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/utime.cpp > CMakeFiles/mission.dir/src/utime.cpp.i

CMakeFiles/mission.dir/src/utime.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/utime.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/utime.cpp -o CMakeFiles/mission.dir/src/utime.cpp.s

CMakeFiles/mission.dir/src/uplay.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/uplay.cpp.o: ../src/uplay.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/mission.dir/src/uplay.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/uplay.cpp.o -c /home/local/mission/src/uplay.cpp

CMakeFiles/mission.dir/src/uplay.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/uplay.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/uplay.cpp > CMakeFiles/mission.dir/src/uplay.cpp.i

CMakeFiles/mission.dir/src/uplay.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/uplay.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/uplay.cpp -o CMakeFiles/mission.dir/src/uplay.cpp.s

CMakeFiles/mission.dir/src/uevent.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/uevent.cpp.o: ../src/uevent.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/mission.dir/src/uevent.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/uevent.cpp.o -c /home/local/mission/src/uevent.cpp

CMakeFiles/mission.dir/src/uevent.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/uevent.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/uevent.cpp > CMakeFiles/mission.dir/src/uevent.cpp.i

CMakeFiles/mission.dir/src/uevent.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/uevent.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/uevent.cpp -o CMakeFiles/mission.dir/src/uevent.cpp.s

CMakeFiles/mission.dir/src/uencoder.cpp.o: CMakeFiles/mission.dir/flags.make
CMakeFiles/mission.dir/src/uencoder.cpp.o: ../src/uencoder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/mission.dir/src/uencoder.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission.dir/src/uencoder.cpp.o -c /home/local/mission/src/uencoder.cpp

CMakeFiles/mission.dir/src/uencoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission.dir/src/uencoder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/local/mission/src/uencoder.cpp > CMakeFiles/mission.dir/src/uencoder.cpp.i

CMakeFiles/mission.dir/src/uencoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission.dir/src/uencoder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/local/mission/src/uencoder.cpp -o CMakeFiles/mission.dir/src/uencoder.cpp.s

# Object files for target mission
mission_OBJECTS = \
"CMakeFiles/mission.dir/main.cpp.o" \
"CMakeFiles/mission.dir/src/ubridge.cpp.o" \
"CMakeFiles/mission.dir/src/upose.cpp.o" \
"CMakeFiles/mission.dir/src/ucomment.cpp.o" \
"CMakeFiles/mission.dir/src/ustate.cpp.o" \
"CMakeFiles/mission.dir/src/uvision.cpp.o" \
"CMakeFiles/mission.dir/src/utime.cpp.o" \
"CMakeFiles/mission.dir/src/uplay.cpp.o" \
"CMakeFiles/mission.dir/src/uevent.cpp.o" \
"CMakeFiles/mission.dir/src/uencoder.cpp.o"

# External object files for target mission
mission_EXTERNAL_OBJECTS =

mission: CMakeFiles/mission.dir/main.cpp.o
mission: CMakeFiles/mission.dir/src/ubridge.cpp.o
mission: CMakeFiles/mission.dir/src/upose.cpp.o
mission: CMakeFiles/mission.dir/src/ucomment.cpp.o
mission: CMakeFiles/mission.dir/src/ustate.cpp.o
mission: CMakeFiles/mission.dir/src/uvision.cpp.o
mission: CMakeFiles/mission.dir/src/utime.cpp.o
mission: CMakeFiles/mission.dir/src/uplay.cpp.o
mission: CMakeFiles/mission.dir/src/uevent.cpp.o
mission: CMakeFiles/mission.dir/src/uencoder.cpp.o
mission: CMakeFiles/mission.dir/build.make
mission: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_alphamat.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_aruco.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_bgsegm.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_bioinspired.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_ccalib.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_dnn_objdetect.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_dnn_superres.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_dpm.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_face.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_freetype.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_fuzzy.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_hdf.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_hfs.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_img_hash.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_intensity_transform.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_line_descriptor.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_mcc.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_quality.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_rapid.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_reg.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_rgbd.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_saliency.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_shape.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_stereo.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_structured_light.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_surface_matching.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_tracking.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_viz.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_xobjdetect.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_xphoto.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_datasets.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_plot.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_text.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_phase_unwrapping.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_optflow.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_ximgproc.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_dnn.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_videoio.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_imgcodecs.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.4.5.1
mission: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.4.5.1
mission: CMakeFiles/mission.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/local/mission/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable mission"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mission.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mission.dir/build: mission

.PHONY : CMakeFiles/mission.dir/build

CMakeFiles/mission.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mission.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mission.dir/clean

CMakeFiles/mission.dir/depend:
	cd /home/local/mission/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/local/mission /home/local/mission /home/local/mission/build /home/local/mission/build /home/local/mission/build/CMakeFiles/mission.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mission.dir/depend

