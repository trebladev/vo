aux_source_directory(. DIR_LIB_SRCS)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)
message(STATUS "project dir ${PROJECT_SOURCE_DIR}")

add_executable(readimg readimg.cpp)
target_link_libraries(readimg
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_NAME}
        ${realsense2_LIBRARY}
        )

add_executable(test_orbex test_orbextractor.cpp ../src/ORBextractor.cpp)
target_link_libraries(test_orbex
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${realsense2_LIBRARY}
        ${PROJECT_NAME}
        )

add_executable(test_f2f_track test_f2f_track.cpp)
target_link_libraries(test_f2f_track
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${realsense2_LIBRARY}
        ${PROJECT_NAME}
        )