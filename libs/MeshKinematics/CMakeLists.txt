set(LIBRARY_TARGET_NAME meshKinematics)

# List of CPP (source) library files.
set(${LIBRARY_TARGET_NAME}_SRC
        src/meshKinematics.cpp
)

# List of HPP (header) library files.
set(${LIBRARY_TARGET_NAME}_HDR
        include/meshKinematics.h
)

find_package(iDynTree REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_link_libraries(${LIBRARY_TARGET_NAME} ${iDynTree_LIBRARIES} Eigen3::Eigen)
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC include)

add_subdirectory(test)
