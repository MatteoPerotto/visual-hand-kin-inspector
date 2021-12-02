set(TEST_TARGET_NAME testMeshSuperimposer)

set(${TEST_TARGET_NAME}_SRC
    testMeshSuperimposer.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})
target_link_libraries(${TEST_TARGET_NAME} meshSuperimposer)
