aux_source_directory( . AUX_SRC_VIO_STEREO_BASALT_BACKEND )

# Add all components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility/src )
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ${SLAM_UTILITY_PATH}/datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_slam_utility_tick_tock )
    add_subdirectory( ${SLAM_UTILITY_PATH}/tick_tock ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_tick_tock )
endif()

# Add binary data logger.
set( BINARY_DATA_LOG_PATH ${PROJECT_SOURCE_DIR}/../Binary_Data_Log/src )
if ( NOT TARGET lib_binary_data_log )
    add_subdirectory( ${BINARY_DATA_LOG_PATH} ${PROJECT_SOURCE_DIR}/build/lib_binary_data_log )
endif()

# Add data manager.
set( DATA_MANAGER_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_BASALT/src )
if ( NOT TARGET lib_vio_stereo_basalt_data_manager )
    add_subdirectory( ${DATA_MANAGER_PATH}/data_manager ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt_data_manager )
endif()

# Add slam solver : general_graph_optimizor.
set( SLAM_SOLVER_PATH ${PROJECT_SOURCE_DIR}/../Slam_Solver/src )
if ( NOT TARGET lib_general_graph_optimizor )
    add_subdirectory( ${SLAM_SOLVER_PATH}/general_graph_optimizor ${PROJECT_SOURCE_DIR}/build/lib_general_graph_optimizor )
endif()

# Create library.
add_library( lib_vio_stereo_basalt_backend ${AUX_SRC_VIO_STEREO_BASALT_BACKEND} )
target_include_directories( lib_vio_stereo_basalt_backend PUBLIC . )
target_link_libraries( lib_vio_stereo_basalt_backend
    lib_slam_utility_datatype
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_vio_stereo_basalt_data_manager

    lib_binary_data_log
    lib_general_graph_optimizor
)
