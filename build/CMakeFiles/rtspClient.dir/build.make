# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.29.6/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.29.6/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/sunday/Desktop/Code/rtspClient

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/sunday/Desktop/Code/rtspClient/build

# Include any dependencies generated for this target.
include CMakeFiles/rtspClient.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/rtspClient.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rtspClient.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rtspClient.dir/flags.make

CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o: CMakeFiles/rtspClient.dir/flags.make
CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o: /Users/sunday/Desktop/Code/rtspClient/src/TestLive264.cpp
CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o: CMakeFiles/rtspClient.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/sunday/Desktop/Code/rtspClient/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o -MF CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o.d -o CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o -c /Users/sunday/Desktop/Code/rtspClient/src/TestLive264.cpp

CMakeFiles/rtspClient.dir/src/TestLive264.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/rtspClient.dir/src/TestLive264.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/sunday/Desktop/Code/rtspClient/src/TestLive264.cpp > CMakeFiles/rtspClient.dir/src/TestLive264.cpp.i

CMakeFiles/rtspClient.dir/src/TestLive264.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/rtspClient.dir/src/TestLive264.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/sunday/Desktop/Code/rtspClient/src/TestLive264.cpp -o CMakeFiles/rtspClient.dir/src/TestLive264.cpp.s

# Object files for target rtspClient
rtspClient_OBJECTS = \
"CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o"

# External object files for target rtspClient
rtspClient_EXTERNAL_OBJECTS =

/Users/sunday/Desktop/Code/rtspClient/bin/rtspClient: CMakeFiles/rtspClient.dir/src/TestLive264.cpp.o
/Users/sunday/Desktop/Code/rtspClient/bin/rtspClient: CMakeFiles/rtspClient.dir/build.make
/Users/sunday/Desktop/Code/rtspClient/bin/rtspClient: CMakeFiles/rtspClient.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/sunday/Desktop/Code/rtspClient/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /Users/sunday/Desktop/Code/rtspClient/bin/rtspClient"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtspClient.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rtspClient.dir/build: /Users/sunday/Desktop/Code/rtspClient/bin/rtspClient
.PHONY : CMakeFiles/rtspClient.dir/build

CMakeFiles/rtspClient.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rtspClient.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rtspClient.dir/clean

CMakeFiles/rtspClient.dir/depend:
	cd /Users/sunday/Desktop/Code/rtspClient/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/sunday/Desktop/Code/rtspClient /Users/sunday/Desktop/Code/rtspClient /Users/sunday/Desktop/Code/rtspClient/build /Users/sunday/Desktop/Code/rtspClient/build /Users/sunday/Desktop/Code/rtspClient/build/CMakeFiles/rtspClient.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/rtspClient.dir/depend

