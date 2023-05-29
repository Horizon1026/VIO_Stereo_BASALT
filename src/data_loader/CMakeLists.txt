aux_source_directory( . AUX_SRC_VIO_STEREO_ORB_SLAM3_DATA_LOADER )

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

# Add imu model.
set( SENSOR_MODEL ${PROJECT_SOURCE_DIR}/../Sensor_Model/src )
if ( NOT TARGET lib_imu_model )
	add_subdirectory( ${SENSOR_MODEL}/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()

# Create library.
add_library( lib_vio_stereo_orb_slam3_data_loader ${AUX_SRC_VIO_STEREO_ORB_SLAM3_DATA_LOADER} )
target_include_directories( lib_vio_stereo_orb_slam3_data_loader PUBLIC . )
target_link_libraries( lib_vio_stereo_orb_slam3_data_loader
    lib_slam_utility_datatype
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_imu_model
)
