cmake_minimum_required(VERSION 3.10)

project(MeshKinematics VERSION 1.0)

find_package(iDynTree REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(${PROJECT_NAME} src/meshKinematics.cpp )
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} Eigen3::Eigen)
