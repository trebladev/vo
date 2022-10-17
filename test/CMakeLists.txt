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
        )