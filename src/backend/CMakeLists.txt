aux_source_directory( . AUX_SRC_VIO_STEREO_BASALT_BACKEND )
aux_source_directory( initializor AUX_SRC_VIO_STEREO_BASALT_BACKEND_INITIALIZOR )
aux_source_directory( estimator AUX_SRC_VIO_STEREO_BASALT_BACKEND_ESTIMATOR )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_slam_utility_tick_tock )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/tick_tock ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_tick_tock )
endif()

# Add binary data logger.
set( BINARY_DATA_LOG_PATH ${PROJECT_SOURCE_DIR}/../Binary_Data_Log/src )
if ( NOT TARGET lib_binary_data_log )
    add_subdirectory( ${BINARY_DATA_LOG_PATH} ${PROJECT_SOURCE_DIR}/build/lib_binary_data_log )
endif()

# Add data manager.
set( DATA_MANAGER_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_BASALT/src/data_manager )
if ( NOT TARGET lib_vio_stereo_basalt_data_manager )
    add_subdirectory( ${DATA_MANAGER_PATH} ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt_data_manager )
endif()

# Add all relative components of vision geometry.
set( VISION_GEOMETRY_PATH ${PROJECT_SOURCE_DIR}/../Vision_Geometry )
if ( NOT TARGET lib_vision_geometry_epipolar )
    add_subdirectory( ${VISION_GEOMETRY_PATH}/src/epipolar ${PROJECT_SOURCE_DIR}/build/lib_vision_geometry_epipolar )
endif()
if ( NOT TARGET lib_vision_geometry_pnp )
    add_subdirectory( ${VISION_GEOMETRY_PATH}/src/perspective_n_point ${PROJECT_SOURCE_DIR}/build/lib_vision_geometry_pnp )
endif()
if ( NOT TARGET lib_vision_geometry_point_triangulator )
    add_subdirectory( ${VISION_GEOMETRY_PATH}/src/point_triangulator ${PROJECT_SOURCE_DIR}/build/lib_vision_geometry_point_triangulator )
endif()
if ( NOT TARGET lib_vision_geometry_relative_rotation )
    add_subdirectory( ${VISION_GEOMETRY_PATH}/src/relative_rotation ${PROJECT_SOURCE_DIR}/build/lib_vision_geometry_relative_rotation )
endif()

# Add components of slam solver.
set( SLAM_SOLVER_PATH ${PROJECT_SOURCE_DIR}/../Slam_Solver )
if ( NOT TARGET lib_general_graph_optimizor )
    add_subdirectory( ${SLAM_SOLVER_PATH}/src/general_graph_optimizor ${PROJECT_SOURCE_DIR}/build/lib_general_graph_optimizor )
endif()
if ( NOT TARGET lib_polynomial_solver )
    add_subdirectory( ${SLAM_SOLVER_PATH}/src/polynomial_solver ${PROJECT_SOURCE_DIR}/build/lib_polynomial_solver )
endif()

# Add image painter.
set( IMAGE_PAINTER_PATH ${PROJECT_SOURCE_DIR}/../Image_Painter )
if ( NOT TARGET lib_image_painter )
    add_subdirectory( ${IMAGE_PAINTER_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_image_painter )
endif()

# Add visualizor.
set( VISUALIZOR_2D_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D )
if ( NOT TARGET lib_2d_visualizor )
    add_subdirectory( ${VISUALIZOR_2D_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_2d_visualizor )
endif()
set( VISUALIZOR_3D_PATH ${PROJECT_SOURCE_DIR}/../Visualizor3D )
if ( NOT TARGET lib_3d_visualizor )
    add_subdirectory( ${VISUALIZOR_3D_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_3d_visualizor )
endif()

# Add visual frontend.
set( VISUAL_FRONTEND_PATH ${PROJECT_SOURCE_DIR}/../VIO_Stereo_BASALT/src/frontend )
if ( NOT TARGET lib_vio_stereo_basalt_frontend )
    add_subdirectory( ${VISUAL_FRONTEND_PATH} ${PROJECT_SOURCE_DIR}/build/lib_vio_stereo_basalt_frontend )
endif()

# Create library.
add_library( lib_vio_stereo_basalt_backend
    ${AUX_SRC_VIO_STEREO_BASALT_BACKEND}
    ${AUX_SRC_VIO_STEREO_BASALT_BACKEND_INITIALIZOR}
    ${AUX_SRC_VIO_STEREO_BASALT_BACKEND_ESTIMATOR}
)
target_include_directories( lib_vio_stereo_basalt_backend
    PUBLIC . estimator
)
target_link_libraries( lib_vio_stereo_basalt_backend
    lib_slam_utility_basic_type
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_vio_stereo_basalt_frontend
    lib_vio_stereo_basalt_data_manager

    lib_vision_geometry_epipolar
    lib_vision_geometry_pnp
    lib_vision_geometry_point_triangulator
    lib_vision_geometry_relative_rotation

    lib_binary_data_log
    lib_image_painter
    lib_2d_visualizor
    lib_3d_visualizor

    lib_general_graph_optimizor
    lib_polynomial_solver
)
