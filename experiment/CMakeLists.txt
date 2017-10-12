set( build_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    rgbd_tutor_lib
    ${PROJECT_SOURCE_DIR}/Thirdparty/orbslam_modified/lib/libORB_SLAM2.so
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
    ${BOOST_LIBRARIES}
)

add_executable( match_orb match_orbfeature_tum.cpp )
target_link_libraries( match_orb
    ${build_libs}
)
