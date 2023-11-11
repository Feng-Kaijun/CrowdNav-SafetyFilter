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
include src/cxx/CMakeFiles/Polyhedron.dir/depend.make

# Include the progress variables for this target.
include src/cxx/CMakeFiles/Polyhedron.dir/progress.make

# Include the compile flags for this target's objects.
include src/cxx/CMakeFiles/Polyhedron.dir/flags.make

src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o: src/cxx/CMakeFiles/Polyhedron.dir/flags.make
src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o: ../src/cxx/Polyhedron.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o -c /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/Polyhedron.cpp

src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Polyhedron.dir/Polyhedron.cpp.i"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/Polyhedron.cpp > CMakeFiles/Polyhedron.dir/Polyhedron.cpp.i

src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Polyhedron.dir/Polyhedron.cpp.s"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx/Polyhedron.cpp -o CMakeFiles/Polyhedron.dir/Polyhedron.cpp.s

# Object files for target Polyhedron
Polyhedron_OBJECTS = \
"CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o"

# External object files for target Polyhedron
Polyhedron_EXTERNAL_OBJECTS =

src/cxx/libPolyhedron.so: src/cxx/CMakeFiles/Polyhedron.dir/Polyhedron.cpp.o
src/cxx/libPolyhedron.so: src/cxx/CMakeFiles/Polyhedron.dir/build.make
src/cxx/libPolyhedron.so: /usr/local/lib/libcdd.so
src/cxx/libPolyhedron.so: src/cxx/CMakeFiles/Polyhedron.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libPolyhedron.so"
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Polyhedron.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/cxx/CMakeFiles/Polyhedron.dir/build: src/cxx/libPolyhedron.so

.PHONY : src/cxx/CMakeFiles/Polyhedron.dir/build

src/cxx/CMakeFiles/Polyhedron.dir/clean:
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx && $(CMAKE_COMMAND) -P CMakeFiles/Polyhedron.dir/cmake_clean.cmake
.PHONY : src/cxx/CMakeFiles/Polyhedron.dir/clean

src/cxx/CMakeFiles/Polyhedron.dir/depend:
	cd /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx /home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/build/src/cxx/CMakeFiles/Polyhedron.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/cxx/CMakeFiles/Polyhedron.dir/depend
