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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF

# Include any dependencies generated for this target.
include test/CMakeFiles/all_tests.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/all_tests.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/all_tests.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/all_tests.dir/flags.make

test/CMakeFiles/all_tests.dir/simple.cpp.o: test/CMakeFiles/all_tests.dir/flags.make
test/CMakeFiles/all_tests.dir/simple.cpp.o: test/simple.cpp
test/CMakeFiles/all_tests.dir/simple.cpp.o: test/CMakeFiles/all_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/all_tests.dir/simple.cpp.o"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/all_tests.dir/simple.cpp.o -MF CMakeFiles/all_tests.dir/simple.cpp.o.d -o CMakeFiles/all_tests.dir/simple.cpp.o -c /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test/simple.cpp

test/CMakeFiles/all_tests.dir/simple.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/all_tests.dir/simple.cpp.i"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test/simple.cpp > CMakeFiles/all_tests.dir/simple.cpp.i

test/CMakeFiles/all_tests.dir/simple.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/all_tests.dir/simple.cpp.s"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test/simple.cpp -o CMakeFiles/all_tests.dir/simple.cpp.s

test/CMakeFiles/all_tests.dir/__/q.cpp.o: test/CMakeFiles/all_tests.dir/flags.make
test/CMakeFiles/all_tests.dir/__/q.cpp.o: q.cpp
test/CMakeFiles/all_tests.dir/__/q.cpp.o: test/CMakeFiles/all_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/all_tests.dir/__/q.cpp.o"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/all_tests.dir/__/q.cpp.o -MF CMakeFiles/all_tests.dir/__/q.cpp.o.d -o CMakeFiles/all_tests.dir/__/q.cpp.o -c /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/q.cpp

test/CMakeFiles/all_tests.dir/__/q.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/all_tests.dir/__/q.cpp.i"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/q.cpp > CMakeFiles/all_tests.dir/__/q.cpp.i

test/CMakeFiles/all_tests.dir/__/q.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/all_tests.dir/__/q.cpp.s"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/q.cpp -o CMakeFiles/all_tests.dir/__/q.cpp.s

test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o: test/CMakeFiles/all_tests.dir/flags.make
test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o: calcDynamics.cpp
test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o: test/CMakeFiles/all_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o -MF CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o.d -o CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o -c /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/calcDynamics.cpp

test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/all_tests.dir/__/calcDynamics.cpp.i"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/calcDynamics.cpp > CMakeFiles/all_tests.dir/__/calcDynamics.cpp.i

test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/all_tests.dir/__/calcDynamics.cpp.s"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/calcDynamics.cpp -o CMakeFiles/all_tests.dir/__/calcDynamics.cpp.s

test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o: test/CMakeFiles/all_tests.dir/flags.make
test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o: aeroData.cpp
test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o: test/CMakeFiles/all_tests.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o -MF CMakeFiles/all_tests.dir/__/aeroData.cpp.o.d -o CMakeFiles/all_tests.dir/__/aeroData.cpp.o -c /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/aeroData.cpp

test/CMakeFiles/all_tests.dir/__/aeroData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/all_tests.dir/__/aeroData.cpp.i"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/aeroData.cpp > CMakeFiles/all_tests.dir/__/aeroData.cpp.i

test/CMakeFiles/all_tests.dir/__/aeroData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/all_tests.dir/__/aeroData.cpp.s"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/aeroData.cpp -o CMakeFiles/all_tests.dir/__/aeroData.cpp.s

# Object files for target all_tests
all_tests_OBJECTS = \
"CMakeFiles/all_tests.dir/simple.cpp.o" \
"CMakeFiles/all_tests.dir/__/q.cpp.o" \
"CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o" \
"CMakeFiles/all_tests.dir/__/aeroData.cpp.o"

# External object files for target all_tests
all_tests_EXTERNAL_OBJECTS =

test/all_tests: test/CMakeFiles/all_tests.dir/simple.cpp.o
test/all_tests: test/CMakeFiles/all_tests.dir/__/q.cpp.o
test/all_tests: test/CMakeFiles/all_tests.dir/__/calcDynamics.cpp.o
test/all_tests: test/CMakeFiles/all_tests.dir/__/aeroData.cpp.o
test/all_tests: test/CMakeFiles/all_tests.dir/build.make
test/all_tests: lib/libgtest.a
test/all_tests: lib/libgtest_main.a
test/all_tests: lib/libgtest.a
test/all_tests: test/CMakeFiles/all_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable all_tests"
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/all_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/all_tests.dir/build: test/all_tests
.PHONY : test/CMakeFiles/all_tests.dir/build

test/CMakeFiles/all_tests.dir/clean:
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test && $(CMAKE_COMMAND) -P CMakeFiles/all_tests.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/all_tests.dir/clean

test/CMakeFiles/all_tests.dir/depend:
	cd /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test /Users/tarasaxena/Documents/NUSTARS/ADS/6DOF/test/CMakeFiles/all_tests.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : test/CMakeFiles/all_tests.dir/depend

