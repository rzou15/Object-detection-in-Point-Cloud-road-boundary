cmake_minimum_required( VERSION 2.8 )
# Create Project
project( cloud_viewer )
add_executable( cloud_viewer cloud_viewer.cpp )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "cloud_viewer" )

# Find Packages
# Find PCL
find_package( PCL 1.8 REQUIRED )

# Find OpenCV
set( OpenCV_DIR "C:/Program Files/opencv/build" )
find_package( OpenCV 3 REQUIRED )

if( PCL_FOUND AND OpenCV_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( cloud_viewer ${PCL_LIBRARIES} )
  target_link_libraries( cloud_viewer ${OpenCV_LIBS} )
endif()