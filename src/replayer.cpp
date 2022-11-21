/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License,
    Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>

#include <future>
#include <fstream>

#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <ros/param.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>


#include "popl.hpp"


namespace
{
    void load(const std::string &parameters)
    {
        FILE *pipe_fd = popen("rosparam load -", "w");  // NOLINT
        if (nullptr != pipe_fd)
        {
            if (parameters.size() != fwrite(parameters.data(), sizeof(char), parameters.size(), pipe_fd))
            {
                throw std::runtime_error("Could not load parameter server contents.");
            }
            pclose(pipe_fd);
        }
    }
}  // namespace


int main(int argc, char **argv)
{
    std::vector<std::string> bag_files;
    bool publish_config = false;
    bool dump = false;
    try
    {
        popl::OptionParser op("Usage: 'replayer [options] [bag files]'\nOptions");
        auto help_option = op.add<popl::Switch>("h", "help", "produce help message");
        auto publish_config_option = op.add<popl::Switch>("c", "config", "publish staticoma config message");
        auto bagfile_option = op.add<popl::Switch>("b", "bagfile", "read topics from bag file(s)");
        auto stdout_option =
                op.add<popl::Switch>("o", "stdout", "write parameter server to stdout (to be used in <param ... />)");

        op.parse(argc, argv);

        if (help_option->count() > 0)
        {
            std::cout << op << std::endl;
            return (EXIT_SUCCESS);
        }

        if (bagfile_option->count() > 0)
        {
            std::vector<std::string> non_option_args = op.non_option_args();

            for (const std::string &arg : non_option_args)
            {
                if (boost::filesystem::is_regular(arg))
                {
                    bag_files.emplace_back(arg);
                }
            }
        }

        publish_config = publish_config_option->count() > 0;
        dump = stdout_option->count() > 0;

        if (publish_config and dump)
        {
            std::cerr << "Both -o (--stdout) and -c (--config) options cannot be used at the same time." << std::endl;
            return (EXIT_FAILURE);
        }
    }
    catch (const std::exception &e)
    {
        return (EXIT_FAILURE);
    }


    // load parameter server contents from a bag file
    try
    {
        if (not bag_files.empty())
        {
            std::vector<std::string> topics;
            topics.emplace_back("/staticoma/parameter_server");
            rosbag::TopicQuery query(topics);

            bool done = false;

            for (const std::string &bag_file : bag_files)
            {
                try
                {
                    rosbag::Bag bag;
                    bag.open(bag_file, rosbag::bagmode::Read);

                    for (const rosbag::MessageInstance &msg : rosbag::View(bag, query))
                    {
                        std_msgs::String::ConstPtr msg_ptr = msg.instantiate<std_msgs::String>();

                        if (nullptr != msg_ptr)
                        {
                            if (not msg_ptr->data.empty())
                            {
                                if (dump)
                                {
                                    std::cout << msg_ptr->data << std::endl;
                                }
                                else
                                {
                                    load(msg_ptr->data);
                                    std::cout << "Loaded parameter server data from '" << bag_file << "'.";
                                }
                            }
                            done = true;
                            break;
                        }
                    }
                }
                catch (...)
                {
                    // ignore all exceptions
                }

                if (done)
                {
                    break;
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what();
        return (EXIT_FAILURE);
    }


    if (bag_files.empty() or publish_config)
    {
        try
        {
            ros::init(argc, argv, "staticoma_replayer");
            ros::NodeHandle nh("~");


            if (bag_files.empty())  // no bag files, load parameter server contents from a message.
            {
                const ros::WallDuration &clock_wait = ros::WallDuration(10.0);
                const ros::Duration &msg_wait = ros::Duration(10.0);

                if (ros::Time::waitForValid(clock_wait))
                {
                    std_msgs::String::ConstPtr msg_ptr =
                            ros::topic::waitForMessage<std_msgs::String>("/staticoma/parameter_server", msg_wait);

                    if (nullptr != msg_ptr and not msg_ptr->data.empty())
                    {
                        load(msg_ptr->data);
                        std::cout << "Loaded parameter server data from a topic.";
                    }
                }
            }

            if (not bag_files.empty() and publish_config)  // get staticoma config from a bag file and publish it
            {
                ros::Publisher config_publisher;

                std::vector<std::string> topics;
                topics.emplace_back("/staticoma/config");
                rosbag::TopicQuery query(topics);

                bool done = false;

                for (const std::string &bag_file : bag_files)
                {
                    try
                    {
                        rosbag::Bag bag;
                        bag.open(bag_file, rosbag::bagmode::Read);

                        for (const rosbag::MessageInstance &msg : rosbag::View(bag, query))
                        {
                            std_msgs::String::ConstPtr msg_ptr = msg.instantiate<std_msgs::String>();

                            if (nullptr != msg_ptr)
                            {
                                config_publisher = nh.advertise<std_msgs::String>(
                                        "/staticoma/config", /*queue_size=*/1, /*latch=*/true);
                                config_publisher.publish(*msg_ptr);
                                done = true;
                                break;
                            }
                        }
                    }
                    catch (...)
                    {
                        // ignore all exceptions
                    }

                    if (done)
                    {
                        break;
                    }
                }

                ros::spin();
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
            return (EXIT_FAILURE);
        }
    }

    return (EXIT_SUCCESS);
}
