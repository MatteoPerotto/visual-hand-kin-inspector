cmake_minimum_required(VERSION 3.10)
project(VisualKinInspector VERSION 1.0)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(libs)

# Build the application

find_package(YARP REQUIRED COMPONENTS cv dev eigen os sig)
find_package(ICUB REQUIRED)

# List of CPP (source) library files.
set(${TARGET_NAME}_SRC
        src/main.cpp
        src/mk2EncoderReader.cpp
)

# List of HPP (header) library files.
set(${TARGET_NAME}_HDR
        src/mk2EncoderReader.h        
)
                 
add_executable(${PROJECT_NAME} ${${TARGET_NAME}_SRC} ${${TARGET_NAME}_HDR})
target_include_directories(${PROJECT_NAME} PUBLIC "src")
target_link_libraries(${PROJECT_NAME} PUBLIC 
			YARP::YARP_cv 
                          YARP::YARP_dev
                          YARP::YARP_eigen
                          YARP::YARP_init
                          YARP::YARP_os
                          YARP::YARP_sig
                          meshKinematics
                          poseDetector
                          meshSuperimposer
                          ICUB::iKin
                     )
