/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License,
    Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <cstdlib>
#include <fstream>
#include <sstream>

#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace staticoma
{
    class Client
    {
    protected:
        std::string config_file_name_;
        std::stringstream config_content_;
        bool config_fetched_;

        std::stringstream robot_description_content_;
        bool robot_description_fetched_;

    public:
        Client()
        {
            config_fetched_ = false;
            robot_description_fetched_ = false;
        }


        bool fetchConfig(const std::string &given_config_file_name = "",
                         const ros::Duration &msg_wait = ros::Duration(10.0),
                         const ros::WallDuration &clock_wait = ros::WallDuration(10.0))
        {
            config_fetched_ = false;
            config_content_.str("");


            if (true == given_config_file_name.empty())
            {
                const char *config_file_name_ptr = std::getenv("STATICOMA_CONFIG_FILE");
                config_file_name_ = nullptr == config_file_name_ptr ? "" : config_file_name_ptr;
            }
            else
            {
                config_file_name_ = given_config_file_name;
            }

            if (true == config_file_name_.empty())
            {
                if (true == ros::Time::waitForValid(clock_wait))
                {
                    const std_msgs::String::ConstPtr msg_ptr =
                            ros::topic::waitForMessage<std_msgs::String>("/staticoma/config", msg_wait);

                    if (nullptr != msg_ptr)
                    {
                        config_content_.str(msg_ptr->data);
                        config_fetched_ = true;
                    }
                }
            }
            else
            {
                if (true == boost::filesystem::is_regular(config_file_name_))
                {
                    std::ifstream ifs(config_file_name_);
                    config_content_.str(std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()));
                    config_fetched_ = true;
                }
                else
                {
                    ROS_ERROR("Staticoma configuration file '%s' is not a regular file.", config_file_name_.c_str());
                }
            }

            return (config_fetched_);
        }


        bool fetchRobotDescription(const ros::Duration &msg_wait = ros::Duration(10.0),
                                   const ros::WallDuration &clock_wait = ros::WallDuration(10.0))
        {
            robot_description_fetched_ = false;
            std::string model_param;
            ros::NodeHandle nh("~");
            nh.param<std::string>("robot_description", model_param);
            robot_description_content_.str(model_param);

            if (true == model_param.empty())
            {
                if (true == ros::Time::waitForValid(clock_wait))
                {
                    const std_msgs::String::ConstPtr msg_ptr =
                            ros::topic::waitForMessage<std_msgs::String>("/staticoma/robot_description", msg_wait);

                    if (nullptr != msg_ptr)
                    {
                        robot_description_content_.str(msg_ptr->data);
                        robot_description_fetched_ = true;
                    }
                }
            }
            else
            {
                robot_description_fetched_ = true;
            }

            return (robot_description_fetched_);
        }


        template <class... t_Args>
        std::string getConfigString(t_Args &&... args)
        {
            if (false == config_fetched_)
            {
                config_fetched_ = fetchConfig(std::forward<t_Args>(args)...);
            }
            return (config_content_.str());
        }


        template <class... t_Args>
        std::istream &getConfigStream(t_Args &&... args)
        {
            if (false == config_fetched_)
            {
                config_fetched_ = fetchConfig(std::forward<t_Args>(args)...);
            }
            config_content_.seekg(0);
            return (config_content_);
        }


        template <class... t_Args>
        std::string getRobotDescriptionString(t_Args &&... args)
        {
            if (false == robot_description_fetched_)
            {
                robot_description_fetched_ = fetchRobotDescription(std::forward<t_Args>(args)...);
            }
            return (robot_description_content_.str());
        }


        template <class... t_Args>
        std::istream &getRobotDescriptionStream(t_Args &&... args)
        {
            if (false == robot_description_fetched_)
            {
                robot_description_fetched_ = fetchRobotDescription(std::forward<t_Args>(args)...);
            }
            robot_description_content_.seekg(0);
            return (robot_description_content_);
        }
    };
}  // namespace staticoma
