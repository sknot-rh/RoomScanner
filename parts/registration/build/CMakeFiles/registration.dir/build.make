# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/stanly/kinect/DP/registration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stanly/kinect/DP/registration/build

# Include any dependencies generated for this target.
include CMakeFiles/registration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/registration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/registration.dir/flags.make

CMakeFiles/registration.dir/registration.cpp.o: CMakeFiles/registration.dir/flags.make
CMakeFiles/registration.dir/registration.cpp.o: ../registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stanly/kinect/DP/registration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/registration.dir/registration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/registration.dir/registration.cpp.o -c /home/stanly/kinect/DP/registration/registration.cpp

CMakeFiles/registration.dir/registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/registration.dir/registration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stanly/kinect/DP/registration/registration.cpp > CMakeFiles/registration.dir/registration.cpp.i

CMakeFiles/registration.dir/registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/registration.dir/registration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stanly/kinect/DP/registration/registration.cpp -o CMakeFiles/registration.dir/registration.cpp.s

CMakeFiles/registration.dir/registration.cpp.o.requires:

.PHONY : CMakeFiles/registration.dir/registration.cpp.o.requires

CMakeFiles/registration.dir/registration.cpp.o.provides: CMakeFiles/registration.dir/registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/registration.dir/build.make CMakeFiles/registration.dir/registration.cpp.o.provides.build
.PHONY : CMakeFiles/registration.dir/registration.cpp.o.provides

CMakeFiles/registration.dir/registration.cpp.o.provides.build: CMakeFiles/registration.dir/registration.cpp.o


# Object files for target registration
registration_OBJECTS = \
"CMakeFiles/registration.dir/registration.cpp.o"

# External object files for target registration
registration_EXTERNAL_OBJECTS =

registration: CMakeFiles/registration.dir/registration.cpp.o
registration: CMakeFiles/registration.dir/build.make
registration: /usr/local/lib/libboost_system.so
registration: /usr/local/lib/libboost_filesystem.so
registration: /usr/local/lib/libboost_thread.so
registration: /usr/local/lib/libboost_date_time.so
registration: /usr/local/lib/libboost_iostreams.so
registration: /usr/local/lib/libboost_serialization.so
registration: /usr/local/lib/libboost_chrono.so
registration: /usr/local/lib/libboost_atomic.so
registration: /usr/local/lib/libboost_regex.so
registration: /usr/lib/x86_64-linux-gnu/libpthread.so
registration: /usr/local/lib/libpcl_common.so
registration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
registration: /usr/local/lib/libpcl_kdtree.so
registration: /usr/local/lib/libpcl_octree.so
registration: /usr/local/lib/libpcl_search.so
registration: /usr/local/lib/libpcl_sample_consensus.so
registration: /usr/local/lib/libpcl_filters.so
registration: /usr/lib/libOpenNI.so
registration: /usr/lib/libOpenNI2.so
registration: /usr/local/lib/libpcl_io.so
registration: /usr/local/lib/libpcl_tracking.so
registration: /usr/local/lib/libpcl_visualization.so
registration: /usr/local/lib/libpcl_outofcore.so
registration: /usr/local/lib/libpcl_features.so
registration: /usr/local/lib/libpcl_ml.so
registration: /usr/local/lib/libpcl_segmentation.so
registration: /usr/lib/x86_64-linux-gnu/libqhull.so
registration: /usr/local/lib/libpcl_surface.so
registration: /usr/local/lib/libpcl_registration.so
registration: /usr/local/lib/libpcl_recognition.so
registration: /usr/local/lib/libpcl_keypoints.so
registration: /usr/local/lib/libpcl_people.so
registration: /usr/local/lib/libpcl_stereo.so
registration: /usr/local/lib/libboost_system.so
registration: /usr/local/lib/libboost_filesystem.so
registration: /usr/local/lib/libboost_thread.so
registration: /usr/local/lib/libboost_date_time.so
registration: /usr/local/lib/libboost_iostreams.so
registration: /usr/local/lib/libboost_serialization.so
registration: /usr/local/lib/libboost_chrono.so
registration: /usr/local/lib/libboost_atomic.so
registration: /usr/local/lib/libboost_regex.so
registration: /usr/lib/x86_64-linux-gnu/libpthread.so
registration: /usr/lib/x86_64-linux-gnu/libqhull.so
registration: /usr/lib/libOpenNI.so
registration: /usr/lib/libOpenNI2.so
registration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
registration: /usr/local/lib/libvtkIOExport-6.3.so.1
registration: /usr/local/lib/libvtkRenderingGL2PS-6.3.so.1
registration: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.so.1
registration: /usr/local/lib/libvtkgl2ps-6.3.so.1
registration: /usr/local/lib/libvtkImagingStatistics-6.3.so.1
registration: /usr/local/lib/libvtkRenderingLOD-6.3.so.1
registration: /usr/local/lib/libvtkTestingIOSQL-6.3.so.1
registration: /usr/local/lib/libvtkIOParallel-6.3.so.1
registration: /usr/local/lib/libvtkIONetCDF-6.3.so.1
registration: /usr/local/lib/libvtkjsoncpp-6.3.so.1
registration: /usr/local/lib/libvtkFiltersFlowPaths-6.3.so.1
registration: /usr/local/lib/libvtkDomainsChemistry-6.3.so.1
registration: /usr/local/lib/libvtkRenderingLIC-6.3.so.1
registration: /usr/local/lib/libvtkImagingMorphological-6.3.so.1
registration: /usr/local/lib/libvtkImagingMath-6.3.so.1
registration: /usr/local/lib/libvtkRenderingImage-6.3.so.1
registration: /usr/local/lib/libvtkFiltersHyperTree-6.3.so.1
registration: /usr/local/lib/libvtkFiltersParallelImaging-6.3.so.1
registration: /usr/local/lib/libvtkFiltersVerdict-6.3.so.1
registration: /usr/local/lib/libvtkverdict-6.3.so.1
registration: /usr/local/lib/libvtkIOInfovis-6.3.so.1
registration: /usr/local/lib/libvtklibxml2-6.3.so.1
registration: /usr/local/lib/libvtkTestingGenericBridge-6.3.so.1
registration: /usr/local/lib/libvtkRenderingQt-6.3.so.1
registration: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.so.1
registration: /usr/local/lib/libvtkTestingRendering-6.3.so.1
registration: /usr/local/lib/libvtkGUISupportQtSQL-6.3.so.1
registration: /usr/local/lib/libvtkIOLSDyna-6.3.so.1
registration: /usr/local/lib/libvtkIOAMR-6.3.so.1
registration: /usr/local/lib/libvtkGUISupportQtWebkit-6.3.so.1
registration: /usr/local/lib/libvtkViewsQt-6.3.so.1
registration: /usr/local/lib/libvtkViewsInfovis-6.3.so.1
registration: /usr/local/lib/libvtkGUISupportQtOpenGL-6.3.so.1
registration: /usr/local/lib/libvtkViewsContext2D-6.3.so.1
registration: /usr/local/lib/libvtkIOImport-6.3.so.1
registration: /usr/local/lib/libvtkFiltersSelection-6.3.so.1
registration: /usr/local/lib/libvtkInteractionImage-6.3.so.1
registration: /usr/local/lib/libvtkGeovisCore-6.3.so.1
registration: /usr/local/lib/libvtkIOPLY-6.3.so.1
registration: /usr/local/lib/libvtkIOEnSight-6.3.so.1
registration: /usr/local/lib/libvtkImagingStencil-6.3.so.1
registration: /usr/local/lib/libvtkFiltersGeneric-6.3.so.1
registration: /usr/local/lib/libvtkIOVideo-6.3.so.1
registration: /usr/local/lib/libvtkFiltersProgrammable-6.3.so.1
registration: /usr/local/lib/libvtkIOExodus-6.3.so.1
registration: /usr/local/lib/libvtkIOMovie-6.3.so.1
registration: /usr/local/lib/libvtkFiltersSMP-6.3.so.1
registration: /usr/local/lib/libvtkIOMINC-6.3.so.1
registration: /usr/local/lib/libvtkIOParallelXML-6.3.so.1
registration: /usr/local/lib/libpcl_common.so
registration: /usr/local/lib/libpcl_kdtree.so
registration: /usr/local/lib/libpcl_octree.so
registration: /usr/local/lib/libpcl_search.so
registration: /usr/local/lib/libpcl_sample_consensus.so
registration: /usr/local/lib/libpcl_filters.so
registration: /usr/local/lib/libpcl_io.so
registration: /usr/local/lib/libpcl_tracking.so
registration: /usr/local/lib/libpcl_visualization.so
registration: /usr/local/lib/libpcl_outofcore.so
registration: /usr/local/lib/libpcl_features.so
registration: /usr/local/lib/libpcl_ml.so
registration: /usr/local/lib/libpcl_segmentation.so
registration: /usr/local/lib/libpcl_surface.so
registration: /usr/local/lib/libpcl_registration.so
registration: /usr/local/lib/libpcl_recognition.so
registration: /usr/local/lib/libpcl_keypoints.so
registration: /usr/local/lib/libpcl_people.so
registration: /usr/local/lib/libpcl_stereo.so
registration: /usr/local/lib/libvtkFiltersParallel-6.3.so.1
registration: /usr/local/lib/libvtkFiltersTexture-6.3.so.1
registration: /usr/local/lib/libvtkIOSQL-6.3.so.1
registration: /usr/local/lib/libvtksqlite-6.3.so.1
registration: /usr/local/lib/libvtkFiltersAMR-6.3.so.1
registration: /usr/local/lib/libvtkRenderingLabel-6.3.so.1
registration: /usr/local/lib/libvtkFiltersImaging-6.3.so.1
registration: /usr/local/lib/libvtkChartsCore-6.3.so.1
registration: /usr/local/lib/libvtkGUISupportQt-6.3.so.1
registration: /usr/local/lib/libvtkRenderingOpenGL-6.3.so.1
registration: /usr/lib/x86_64-linux-gnu/libGLU.so
registration: /usr/lib/x86_64-linux-gnu/libSM.so
registration: /usr/lib/x86_64-linux-gnu/libICE.so
registration: /usr/lib/x86_64-linux-gnu/libX11.so
registration: /usr/lib/x86_64-linux-gnu/libXext.so
registration: /usr/lib/x86_64-linux-gnu/libXt.so
registration: /home/stanly/Qt/5.7/gcc_64/lib/libQt5Widgets.so.5.7.0
registration: /home/stanly/Qt/5.7/gcc_64/lib/libQt5Gui.so.5.7.0
registration: /home/stanly/Qt/5.7/gcc_64/lib/libQt5Core.so.5.7.0
registration: /usr/local/lib/libvtkRenderingContext2D-6.3.so.1
registration: /usr/local/lib/libvtkInfovisLayout-6.3.so.1
registration: /usr/local/lib/libvtkInfovisCore-6.3.so.1
registration: /usr/local/lib/libvtkproj4-6.3.so.1
registration: /usr/local/lib/libvtkViewsCore-6.3.so.1
registration: /usr/local/lib/libvtkInteractionWidgets-6.3.so.1
registration: /usr/local/lib/libvtkRenderingAnnotation-6.3.so.1
registration: /usr/local/lib/libvtkImagingColor-6.3.so.1
registration: /usr/local/lib/libvtkRenderingFreeType-6.3.so.1
registration: /usr/local/lib/libvtkftgl-6.3.so.1
registration: /usr/local/lib/libvtkfreetype-6.3.so.1
registration: /usr/lib/x86_64-linux-gnu/libGL.so
registration: /usr/local/lib/libvtkImagingHybrid-6.3.so.1
registration: /usr/local/lib/libvtkFiltersModeling-6.3.so.1
registration: /usr/local/lib/libvtkImagingGeneral-6.3.so.1
registration: /usr/local/lib/libvtkInteractionStyle-6.3.so.1
registration: /usr/local/lib/libvtkRenderingVolume-6.3.so.1
registration: /usr/local/lib/libvtkexoIIc-6.3.so.1
registration: /usr/local/lib/libvtkoggtheora-6.3.so.1
registration: /usr/local/lib/libvtkIOImage-6.3.so.1
registration: /usr/local/lib/libvtkDICOMParser-6.3.so.1
registration: /usr/local/lib/libvtkmetaio-6.3.so.1
registration: /usr/local/lib/libvtkpng-6.3.so.1
registration: /usr/local/lib/libvtktiff-6.3.so.1
registration: /usr/local/lib/libvtkjpeg-6.3.so.1
registration: /usr/local/lib/libvtkNetCDF_cxx-6.3.so.1
registration: /usr/local/lib/libvtkNetCDF-6.3.so.1
registration: /usr/local/lib/libvtkhdf5_hl-6.3.so.1
registration: /usr/local/lib/libvtkhdf5-6.3.so.1
registration: /usr/local/lib/libvtkFiltersHybrid-6.3.so.1
registration: /usr/local/lib/libvtkRenderingCore-6.3.so.1
registration: /usr/local/lib/libvtkFiltersGeometry-6.3.so.1
registration: /usr/local/lib/libvtkFiltersSources-6.3.so.1
registration: /usr/local/lib/libvtkCommonColor-6.3.so.1
registration: /usr/local/lib/libvtkFiltersExtraction-6.3.so.1
registration: /usr/local/lib/libvtkFiltersGeneral-6.3.so.1
registration: /usr/local/lib/libvtkFiltersCore-6.3.so.1
registration: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.so.1
registration: /usr/local/lib/libvtkFiltersStatistics-6.3.so.1
registration: /usr/local/lib/libvtkImagingFourier-6.3.so.1
registration: /usr/local/lib/libvtkalglib-6.3.so.1
registration: /usr/local/lib/libvtkImagingSources-6.3.so.1
registration: /usr/local/lib/libvtkImagingCore-6.3.so.1
registration: /usr/local/lib/libvtkParallelCore-6.3.so.1
registration: /usr/local/lib/libvtkIOLegacy-6.3.so.1
registration: /usr/local/lib/libvtkIOXML-6.3.so.1
registration: /usr/local/lib/libvtkIOGeometry-6.3.so.1
registration: /usr/local/lib/libvtkIOXMLParser-6.3.so.1
registration: /usr/local/lib/libvtkIOCore-6.3.so.1
registration: /usr/local/lib/libvtkCommonExecutionModel-6.3.so.1
registration: /usr/local/lib/libvtkCommonDataModel-6.3.so.1
registration: /usr/local/lib/libvtkCommonSystem-6.3.so.1
registration: /usr/local/lib/libvtksys-6.3.so.1
registration: /usr/local/lib/libvtkCommonTransforms-6.3.so.1
registration: /usr/local/lib/libvtkCommonMisc-6.3.so.1
registration: /usr/local/lib/libvtkCommonMath-6.3.so.1
registration: /usr/local/lib/libvtkCommonCore-6.3.so.1
registration: /usr/local/lib/libvtkzlib-6.3.so.1
registration: /usr/local/lib/libvtkexpat-6.3.so.1
registration: CMakeFiles/registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/stanly/kinect/DP/registration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable registration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/registration.dir/build: registration

.PHONY : CMakeFiles/registration.dir/build

CMakeFiles/registration.dir/requires: CMakeFiles/registration.dir/registration.cpp.o.requires

.PHONY : CMakeFiles/registration.dir/requires

CMakeFiles/registration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/registration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/registration.dir/clean

CMakeFiles/registration.dir/depend:
	cd /home/stanly/kinect/DP/registration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stanly/kinect/DP/registration /home/stanly/kinect/DP/registration /home/stanly/kinect/DP/registration/build /home/stanly/kinect/DP/registration/build /home/stanly/kinect/DP/registration/build/CMakeFiles/registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/registration.dir/depend

