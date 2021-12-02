set(TEST_TARGET_NAME testMeshKinematics)

set(${TEST_TARGET_NAME}_SRC
	testMeshKinematics.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})
target_link_libraries(${TEST_TARGET_NAME} meshKinematics)
