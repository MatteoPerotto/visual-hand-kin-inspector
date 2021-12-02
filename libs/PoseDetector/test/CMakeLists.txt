set(TEST_TARGET_NAME testPoseDetector)

set(${TEST_TARGET_NAME}_SRC
    testPoseDetector.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})
target_link_libraries(${TEST_TARGET_NAME} poseDetector)
