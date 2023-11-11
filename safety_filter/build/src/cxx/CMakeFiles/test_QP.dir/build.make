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
CMAKE_SOURCE_DIR = /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build

# Include any dependencies generated for this target.
include src/cxx/CMakeFiles/test_QP.dir/depend.make

# Include the progress variables for this target.
include src/cxx/CMakeFiles/test_QP.dir/progress.make

# Include the compile flags for this target's objects.
include src/cxx/CMakeFiles/test_QP.dir/flags.make

src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.o: src/cxx/CMakeFiles/test_QP.dir/flags.make
src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.o: ../src/cxx/test/test_QP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.o"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_QP.dir/test/test_QP.cpp.o -c /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/test/test_QP.cpp

src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_QP.dir/test/test_QP.cpp.i"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/test/test_QP.cpp > CMakeFiles/test_QP.dir/test/test_QP.cpp.i

src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_QP.dir/test/test_QP.cpp.s"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/test/test_QP.cpp -o CMakeFiles/test_QP.dir/test/test_QP.cpp.s

# Object files for target test_QP
test_QP_OBJECTS = \
"CMakeFiles/test_QP.dir/test/test_QP.cpp.o"

# External object files for target test_QP
test_QP_EXTERNAL_OBJECTS =

src/cxx/test_QP: src/cxx/CMakeFiles/test_QP.dir/test/test_QP.cpp.o
src/cxx/test_QP: src/cxx/CMakeFiles/test_QP.dir/build.make
src/cxx/test_QP: src/cxx/libSafetyFilter.so
src/cxx/test_QP: src/cxx/libPolyhedron.so
src/cxx/test_QP: /usr/local/lib/libcdd.so
src/cxx/test_QP: /usr/local/lib/libOsqpEigen.so.0.8.0
src/cxx/test_QP: /usr/local/lib/libosqp.so
src/cxx/test_QP: src/cxx/CMakeFiles/test_QP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_QP"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_QP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/cxx/CMakeFiles/test_QP.dir/build: src/cxx/test_QP

.PHONY : src/cxx/CMakeFiles/test_QP.dir/build

src/cxx/CMakeFiles/test_QP.dir/clean:
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && $(CMAKE_COMMAND) -P CMakeFiles/test_QP.dir/cmake_clean.cmake
.PHONY : src/cxx/CMakeFiles/test_QP.dir/clean

src/cxx/CMakeFiles/test_QP.dir/depend:
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx/CMakeFiles/test_QP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/cxx/CMakeFiles/test_QP.dir/depend

