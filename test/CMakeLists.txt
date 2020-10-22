find_package(rostest REQUIRED)
find_package(roslib REQUIRED)

# client + server, client gets config from a file using environment variable
add_rostest_gtest(test_client client.test client.cpp)
target_link_libraries(test_client ${catkin_LIBRARIES} ${roslib_LIBRARIES})

# server not started, client gets config from a file using environment variable
add_rostest(client_no_server.test DEPENDENCIES test_client)

# environment variable is not set, client gets config from a topic published by server
add_rostest(client_no_env.test DEPENDENCIES test_client)

# no environment variable, no server, config is published with rostopic
add_rostest(client_no_env_no_server.test DEPENDENCIES test_client)

# replay with bag file
add_rostest(client_replay0.test DEPENDENCIES test_client)
add_rostest(client_replay1.test DEPENDENCIES test_client)
add_rostest(client_replay2.test DEPENDENCIES test_client)
add_rostest(client_replay3.test DEPENDENCIES test_client)
