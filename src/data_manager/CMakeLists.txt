aux_source_directory( . AUX_SRC_VIO_STEREO_BASALT_DATA_MANAGER )

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

# Add all components of slam utility data struct.
if ( NOT TARGET lib_covisible_graph )
    add_subdirectory( ${SLAM_UTILITY_PATH}/data_struct/covisible_graph ${PROJECT_SOURCE_DIR}/build/lib_covisible_graph )
endif()
if ( NOT TARGET lib_object_pool )
    add_subdirectory( ${SLAM_UTILITY_PATH}/data_struct/object_pool ${PROJECT_SOURCE_DIR}/build/lib_object_pool )
endif()

# Add data loader.
set( DATA_LOADER_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_BASALT/src/data_loader )
if ( NOT TARGET lib_vio_stereo_basalt_data_loader )
    add_subdirectory( ${DATA_LOADER_PATH} ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt_data_loader )
endif()

# Add visual frontend.
set( VISUAL_FRONTEND_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_BASALT/src/frontend )
if ( NOT TARGET lib_vio_stereo_basalt_frontend )
    add_subdirectory( ${VISUAL_FRONTEND_PATH} ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt_frontend )
endif()

# Add imu model.
set( SENSOR_MODEL_PATH ${PROJECT_SOURCE_DIR}/../Sensor_Model/src )
if ( NOT TARGET lib_imu_model )
    add_subdirectory( ${SENSOR_MODEL_PATH}/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()

# Create library.
add_library( lib_vio_stereo_basalt_data_manager ${AUX_SRC_VIO_STEREO_BASALT_DATA_MANAGER} )
target_include_directories( lib_vio_stereo_basalt_data_manager PUBLIC . )
target_link_libraries( lib_vio_stereo_basalt_data_manager
    lib_slam_utility_datatype
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_vio_stereo_basalt_data_loader
    lib_vio_stereo_basalt_frontend

    lib_covisible_graph
    lib_object_pool
    lib_imu_model
)
