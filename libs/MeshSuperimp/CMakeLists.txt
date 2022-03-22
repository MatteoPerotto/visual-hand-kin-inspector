set(LIBRARY_TARGET_NAME meshSuperimposer)

# List of CPP (source) library files.
set(${LIBRARY_TARGET_NAME}_SRC
        src/meshSuperimposer.cpp
)

# List of HPP (header) library files.
set(${LIBRARY_TARGET_NAME}_HDR
        include/meshSuperimposer.h
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(SuperimposeMesh REQUIRED)
find_package(realsense2 REQUIRED)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_link_libraries(${LIBRARY_TARGET_NAME} ${OpenCV_LIBS} Eigen3::Eigen SI::SuperimposeMesh ${realsense2_LIBRARY})
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC include)

# add_subdirectory(test)
