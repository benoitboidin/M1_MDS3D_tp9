# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.25.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.25.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build

# Include any dependencies generated for this target.
include ext/SOIL/CMakeFiles/SOIL.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ext/SOIL/CMakeFiles/SOIL.dir/compiler_depend.make

# Include the progress variables for this target.
include ext/SOIL/CMakeFiles/SOIL.dir/progress.make

# Include the compile flags for this target's objects.
include ext/SOIL/CMakeFiles/SOIL.dir/flags.make

ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o: ext/SOIL/CMakeFiles/SOIL.dir/flags.make
ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o: /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_helper.c
ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o: ext/SOIL/CMakeFiles/SOIL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o -MF CMakeFiles/SOIL.dir/src/image_helper.c.o.d -o CMakeFiles/SOIL.dir/src/image_helper.c.o -c /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_helper.c

ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SOIL.dir/src/image_helper.c.i"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_helper.c > CMakeFiles/SOIL.dir/src/image_helper.c.i

ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SOIL.dir/src/image_helper.c.s"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_helper.c -o CMakeFiles/SOIL.dir/src/image_helper.c.s

ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o: ext/SOIL/CMakeFiles/SOIL.dir/flags.make
ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o: /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_DXT.c
ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o: ext/SOIL/CMakeFiles/SOIL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o -MF CMakeFiles/SOIL.dir/src/image_DXT.c.o.d -o CMakeFiles/SOIL.dir/src/image_DXT.c.o -c /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_DXT.c

ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SOIL.dir/src/image_DXT.c.i"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_DXT.c > CMakeFiles/SOIL.dir/src/image_DXT.c.i

ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SOIL.dir/src/image_DXT.c.s"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/image_DXT.c -o CMakeFiles/SOIL.dir/src/image_DXT.c.s

ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o: ext/SOIL/CMakeFiles/SOIL.dir/flags.make
ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o: /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/SOIL2.c
ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o: ext/SOIL/CMakeFiles/SOIL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o -MF CMakeFiles/SOIL.dir/src/SOIL2.c.o.d -o CMakeFiles/SOIL.dir/src/SOIL2.c.o -c /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/SOIL2.c

ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SOIL.dir/src/SOIL2.c.i"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/SOIL2.c > CMakeFiles/SOIL.dir/src/SOIL2.c.i

ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SOIL.dir/src/SOIL2.c.s"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/SOIL2.c -o CMakeFiles/SOIL.dir/src/SOIL2.c.s

ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o: ext/SOIL/CMakeFiles/SOIL.dir/flags.make
ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o: /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/etc1_utils.c
ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o: ext/SOIL/CMakeFiles/SOIL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o -MF CMakeFiles/SOIL.dir/src/etc1_utils.c.o.d -o CMakeFiles/SOIL.dir/src/etc1_utils.c.o -c /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/etc1_utils.c

ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SOIL.dir/src/etc1_utils.c.i"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/etc1_utils.c > CMakeFiles/SOIL.dir/src/etc1_utils.c.i

ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SOIL.dir/src/etc1_utils.c.s"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL/src/etc1_utils.c -o CMakeFiles/SOIL.dir/src/etc1_utils.c.s

# Object files for target SOIL
SOIL_OBJECTS = \
"CMakeFiles/SOIL.dir/src/image_helper.c.o" \
"CMakeFiles/SOIL.dir/src/image_DXT.c.o" \
"CMakeFiles/SOIL.dir/src/SOIL2.c.o" \
"CMakeFiles/SOIL.dir/src/etc1_utils.c.o"

# External object files for target SOIL
SOIL_EXTERNAL_OBJECTS =

ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/src/image_helper.c.o
ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/src/image_DXT.c.o
ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/src/SOIL2.c.o
ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/src/etc1_utils.c.o
ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/build.make
ext/SOIL/libSOIL.a: ext/SOIL/CMakeFiles/SOIL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libSOIL.a"
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && $(CMAKE_COMMAND) -P CMakeFiles/SOIL.dir/cmake_clean_target.cmake
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SOIL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/SOIL/CMakeFiles/SOIL.dir/build: ext/SOIL/libSOIL.a
.PHONY : ext/SOIL/CMakeFiles/SOIL.dir/build

ext/SOIL/CMakeFiles/SOIL.dir/clean:
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL && $(CMAKE_COMMAND) -P CMakeFiles/SOIL.dir/cmake_clean.cmake
.PHONY : ext/SOIL/CMakeFiles/SOIL.dir/clean

ext/SOIL/CMakeFiles/SOIL.dir/depend:
	cd /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9 /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/ext/SOIL /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL /Users/benoitboidin/Documents/professionnel/études/master_informatique/s8_info/3d/tp9/mds3d_td9/build/ext/SOIL/CMakeFiles/SOIL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/SOIL/CMakeFiles/SOIL.dir/depend

