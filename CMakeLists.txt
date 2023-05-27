cmake_minimum_required( VERSION 3.10 )

project( visual_frontend )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add vio stereo orb slam3.
if ( NOT TARGET lib_vio_stereo_orb_slam3 )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_orb_slam3 )
endif()

# Add opencv.
find_package( OpenCV REQUIRED )

# Add test_vio_stereo_orb_slam3_on_euroc
add_executable( test_vio_stereo_orb_slam3_on_euroc
    test/test_vio_stereo_orb_slam3_on_euroc.cpp
)
target_link_libraries( test_vio_stereo_orb_slam3_on_euroc
    lib_vio_stereo_orb_slam3
    ${OpenCV_LIBS}
)
