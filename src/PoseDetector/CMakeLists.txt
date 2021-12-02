find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(${PROJECT_NAME} src/poseDetector.cpp )
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen ${realsense2_LIBRARY} )
