# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/amaan/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amaan/catkin_ws/build

# Include any dependencies generated for this target.
include ros_deep_learning/CMakeFiles/segnet.dir/depend.make

# Include the progress variables for this target.
include ros_deep_learning/CMakeFiles/segnet.dir/progress.make

# Include the compile flags for this target's objects.
include ros_deep_learning/CMakeFiles/segnet.dir/flags.make

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o: ros_deep_learning/CMakeFiles/segnet.dir/flags.make
ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o: /home/amaan/catkin_ws/src/ros_deep_learning/src/node_segnet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amaan/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segnet.dir/src/node_segnet.cpp.o -c /home/amaan/catkin_ws/src/ros_deep_learning/src/node_segnet.cpp

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segnet.dir/src/node_segnet.cpp.i"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amaan/catkin_ws/src/ros_deep_learning/src/node_segnet.cpp > CMakeFiles/segnet.dir/src/node_segnet.cpp.i

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segnet.dir/src/node_segnet.cpp.s"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amaan/catkin_ws/src/ros_deep_learning/src/node_segnet.cpp -o CMakeFiles/segnet.dir/src/node_segnet.cpp.s

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.requires:

.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.requires

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.provides: ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.requires
	$(MAKE) -f ros_deep_learning/CMakeFiles/segnet.dir/build.make ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.provides.build
.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.provides

ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.provides.build: ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o


ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o: ros_deep_learning/CMakeFiles/segnet.dir/flags.make
ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o: /home/amaan/catkin_ws/src/ros_deep_learning/src/image_converter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amaan/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segnet.dir/src/image_converter.cpp.o -c /home/amaan/catkin_ws/src/ros_deep_learning/src/image_converter.cpp

ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segnet.dir/src/image_converter.cpp.i"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amaan/catkin_ws/src/ros_deep_learning/src/image_converter.cpp > CMakeFiles/segnet.dir/src/image_converter.cpp.i

ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segnet.dir/src/image_converter.cpp.s"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amaan/catkin_ws/src/ros_deep_learning/src/image_converter.cpp -o CMakeFiles/segnet.dir/src/image_converter.cpp.s

ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.requires:

.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.requires

ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.provides: ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.requires
	$(MAKE) -f ros_deep_learning/CMakeFiles/segnet.dir/build.make ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.provides.build
.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.provides

ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.provides.build: ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o


ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o: ros_deep_learning/CMakeFiles/segnet.dir/flags.make
ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o: /home/amaan/catkin_ws/src/ros_deep_learning/src/ros_compat.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amaan/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segnet.dir/src/ros_compat.cpp.o -c /home/amaan/catkin_ws/src/ros_deep_learning/src/ros_compat.cpp

ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segnet.dir/src/ros_compat.cpp.i"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amaan/catkin_ws/src/ros_deep_learning/src/ros_compat.cpp > CMakeFiles/segnet.dir/src/ros_compat.cpp.i

ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segnet.dir/src/ros_compat.cpp.s"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amaan/catkin_ws/src/ros_deep_learning/src/ros_compat.cpp -o CMakeFiles/segnet.dir/src/ros_compat.cpp.s

ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.requires:

.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.requires

ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.provides: ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.requires
	$(MAKE) -f ros_deep_learning/CMakeFiles/segnet.dir/build.make ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.provides.build
.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.provides

ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.provides.build: ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o


# Object files for target segnet
segnet_OBJECTS = \
"CMakeFiles/segnet.dir/src/node_segnet.cpp.o" \
"CMakeFiles/segnet.dir/src/image_converter.cpp.o" \
"CMakeFiles/segnet.dir/src/ros_compat.cpp.o"

# External object files for target segnet
segnet_EXTERNAL_OBJECTS =

/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: ros_deep_learning/CMakeFiles/segnet.dir/build.make
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libimage_transport.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libmessage_filters.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libclass_loader.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/libPocoFoundation.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libdl.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libroslib.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/librospack.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libroscpp.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/librosconsole.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/librostime.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /opt/ros/melodic/lib/libcpp_common.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/local/lib/libjetson-inference.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/local/lib/libjetson-utils.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/local/cuda/lib64/libcudart_static.a
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/lib/aarch64-linux-gnu/librt.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: /usr/local/cuda/lib64/libnppicc.so
/home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet: ros_deep_learning/CMakeFiles/segnet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amaan/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet"
	cd /home/amaan/catkin_ws/build/ros_deep_learning && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segnet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ros_deep_learning/CMakeFiles/segnet.dir/build: /home/amaan/catkin_ws/devel/lib/ros_deep_learning/segnet

.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/build

ros_deep_learning/CMakeFiles/segnet.dir/requires: ros_deep_learning/CMakeFiles/segnet.dir/src/node_segnet.cpp.o.requires
ros_deep_learning/CMakeFiles/segnet.dir/requires: ros_deep_learning/CMakeFiles/segnet.dir/src/image_converter.cpp.o.requires
ros_deep_learning/CMakeFiles/segnet.dir/requires: ros_deep_learning/CMakeFiles/segnet.dir/src/ros_compat.cpp.o.requires

.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/requires

ros_deep_learning/CMakeFiles/segnet.dir/clean:
	cd /home/amaan/catkin_ws/build/ros_deep_learning && $(CMAKE_COMMAND) -P CMakeFiles/segnet.dir/cmake_clean.cmake
.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/clean

ros_deep_learning/CMakeFiles/segnet.dir/depend:
	cd /home/amaan/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amaan/catkin_ws/src /home/amaan/catkin_ws/src/ros_deep_learning /home/amaan/catkin_ws/build /home/amaan/catkin_ws/build/ros_deep_learning /home/amaan/catkin_ws/build/ros_deep_learning/CMakeFiles/segnet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros_deep_learning/CMakeFiles/segnet.dir/depend

