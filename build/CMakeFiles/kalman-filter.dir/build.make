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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.18.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.18.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/administrator/Documents/Projects/kalman_filter_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/administrator/Documents/Projects/kalman_filter_cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/kalman-filter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kalman-filter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kalman-filter.dir/flags.make

CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o: CMakeFiles/kalman-filter.dir/flags.make
CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o: ../src/BasicKalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/administrator/Documents/Projects/kalman_filter_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o -c /Users/administrator/Documents/Projects/kalman_filter_cpp/src/BasicKalman.cpp

CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/administrator/Documents/Projects/kalman_filter_cpp/src/BasicKalman.cpp > CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.i

CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/administrator/Documents/Projects/kalman_filter_cpp/src/BasicKalman.cpp -o CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.s

CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o: CMakeFiles/kalman-filter.dir/flags.make
CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o: ../src/Kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/administrator/Documents/Projects/kalman_filter_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o -c /Users/administrator/Documents/Projects/kalman_filter_cpp/src/Kalman.cpp

CMakeFiles/kalman-filter.dir/src/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman-filter.dir/src/Kalman.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/administrator/Documents/Projects/kalman_filter_cpp/src/Kalman.cpp > CMakeFiles/kalman-filter.dir/src/Kalman.cpp.i

CMakeFiles/kalman-filter.dir/src/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman-filter.dir/src/Kalman.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/administrator/Documents/Projects/kalman_filter_cpp/src/Kalman.cpp -o CMakeFiles/kalman-filter.dir/src/Kalman.cpp.s

CMakeFiles/kalman-filter.dir/src/main.cpp.o: CMakeFiles/kalman-filter.dir/flags.make
CMakeFiles/kalman-filter.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/administrator/Documents/Projects/kalman_filter_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/kalman-filter.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kalman-filter.dir/src/main.cpp.o -c /Users/administrator/Documents/Projects/kalman_filter_cpp/src/main.cpp

CMakeFiles/kalman-filter.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalman-filter.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/administrator/Documents/Projects/kalman_filter_cpp/src/main.cpp > CMakeFiles/kalman-filter.dir/src/main.cpp.i

CMakeFiles/kalman-filter.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalman-filter.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/administrator/Documents/Projects/kalman_filter_cpp/src/main.cpp -o CMakeFiles/kalman-filter.dir/src/main.cpp.s

# Object files for target kalman-filter
kalman__filter_OBJECTS = \
"CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o" \
"CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o" \
"CMakeFiles/kalman-filter.dir/src/main.cpp.o"

# External object files for target kalman-filter
kalman__filter_EXTERNAL_OBJECTS =

kalman-filter: CMakeFiles/kalman-filter.dir/src/BasicKalman.cpp.o
kalman-filter: CMakeFiles/kalman-filter.dir/src/Kalman.cpp.o
kalman-filter: CMakeFiles/kalman-filter.dir/src/main.cpp.o
kalman-filter: CMakeFiles/kalman-filter.dir/build.make
kalman-filter: CMakeFiles/kalman-filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/administrator/Documents/Projects/kalman_filter_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable kalman-filter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kalman-filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kalman-filter.dir/build: kalman-filter

.PHONY : CMakeFiles/kalman-filter.dir/build

CMakeFiles/kalman-filter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kalman-filter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kalman-filter.dir/clean

CMakeFiles/kalman-filter.dir/depend:
	cd /Users/administrator/Documents/Projects/kalman_filter_cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/administrator/Documents/Projects/kalman_filter_cpp /Users/administrator/Documents/Projects/kalman_filter_cpp /Users/administrator/Documents/Projects/kalman_filter_cpp/build /Users/administrator/Documents/Projects/kalman_filter_cpp/build /Users/administrator/Documents/Projects/kalman_filter_cpp/build/CMakeFiles/kalman-filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kalman-filter.dir/depend
