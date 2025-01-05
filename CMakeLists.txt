cmake_minimum_required( VERSION 3.10 )

project( test_vio_stereo_basalt )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -g -Wall -pthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 20 )

# Add stack backward.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_stack_backward )
    add_subdirectory( ${SLAM_UTILITY_PATH}/3rd_libraries/backward ${PROJECT_SOURCE_DIR}/build/lib_stack_backward )
endif()

# Add vio stereo basalt
if ( NOT TARGET lib_vio_stereo_basalt )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt )
endif()

# Add test_dataloader
add_executable( test_dataloader
    test/test_dataloader.cpp
)
target_link_libraries( test_dataloader
    lib_vio_stereo_basalt
)

# Add test_vio
add_executable( test_vio
    test/test_vio.cpp
)
target_link_libraries( test_vio
    lib_vio_stereo_basalt
    lib_stack_backward
)
