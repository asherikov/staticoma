/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License,
    Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "staticoma_server");
        ros::NodeHandle nh("~");

        ros::Publisher config_publisher;
        {
            const char *config_file_name_ptr = std::getenv("STATICOMA_CONFIG_FILE");
            const std::string config_file_name = nullptr == config_file_name_ptr ? "" : config_file_name_ptr;

            if (boost::filesystem::is_regular(config_file_name))
            {
                std_msgs::String config_msg;
                std::ifstream ifs(config_file_name);
                config_msg.data.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

                config_publisher = nh.advertise<std_msgs::String>("/staticoma/config", /*queue_size=*/1, /*latch=*/true);
                config_publisher.publish(config_msg);
            }
            else
            {
                ROS_ERROR("STATICOMA_CONFIG_FILE='%s' is not a regular file.", config_file_name.c_str());
            }
        }

        ros::Publisher parameter_server_publisher;
        {
            std_msgs::String parameter_server_msg;

            {
                FILE *pipe_fd = popen("rosparam dump", "r"); // NOLINT
                if (nullptr != pipe_fd)
                {
                    const std::size_t max_read_size = 1024 * 1024 * 10;
                    const std::size_t read_buffer_size = 1024 * 10;
                    std::size_t total_counter = 0;
                    std::array<char, read_buffer_size> read_buffer;

                    std::vector<std::string> clear_keys{ "rosversion", "rosdistro", "run_id", "roslaunch" };
                    bool clear_key = false;


                    for (;;)
                    {
                        if (nullptr == fgets(read_buffer.data(), read_buffer.size(), pipe_fd))
                        {
                            break;
                        }

                        const std::size_t bytes_read = strnlen(read_buffer.data(), read_buffer.size());
                        if (bytes_read == read_buffer.size())
                        {
                            throw std::runtime_error("Input line from `rosparam dump` is too long.");
                        }

                        total_counter += bytes_read;
                        if (total_counter >= max_read_size)
                        {
                            throw std::runtime_error("Too much input from `rosparam dump`.");
                        }


                        if (clear_key)
                        {
                            // this is crap
                            if (bytes_read > 1 and isalpha(read_buffer[0]) > 0)
                            {
                                clear_key = false;
                            }
                        }

                        if (not clear_key)
                        {
                            for (const std::string &key : clear_keys)
                            {
                                if (bytes_read >= key.size())
                                {
                                    if (0 == strncmp(read_buffer.data(), key.c_str(), key.size()))
                                    {
                                        clear_key = true;
                                        break;
                                    }
                                }
                            }
                        }

                        if (not clear_key)
                        {
                            parameter_server_msg.data.insert(
                                    parameter_server_msg.data.end(), read_buffer.data(), read_buffer.data() + bytes_read);
                        }
                    }
                    pclose(pipe_fd);
                }
            }

            parameter_server_publisher =
                    nh.advertise<std_msgs::String>("/staticoma/parameter_server", /*queue_size=*/1, /*latch=*/true);
            parameter_server_publisher.publish(parameter_server_msg);
        }

        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return (EXIT_FAILURE);
    }

    return (EXIT_SUCCESS);
}
