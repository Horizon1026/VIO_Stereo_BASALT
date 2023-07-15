cmake_minimum_required( VERSION 3.10 )

project( visual_frontend )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -lpthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add vio stereo basalt
if ( NOT TARGET lib_vio_stereo_basalt )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt )
endif()

# Add opencv.
find_package( OpenCV REQUIRED )

# Add test_vio_stereo_basalt_on_euroc
add_executable( test_vio_stereo_basalt_on_euroc
    test/test_vio_stereo_basalt_on_euroc.cpp
)
target_link_libraries( test_vio_stereo_basalt_on_euroc
    lib_vio_stereo_basalt
    ${OpenCV_LIBS}
)

# Add test_dataloader
add_executable( test_dataloader
    test/test_dataloader.cpp
)
target_link_libraries( test_dataloader
    lib_vio_stereo_basalt
    ${OpenCV_LIBS}
)
