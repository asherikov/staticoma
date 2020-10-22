/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License,
    Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "staticoma/client.h"

#include <ros/package.h>
#include <ros/param.h>

#include <gtest/gtest.h>


TEST(client, client)
{
    ROS_ERROR("STATICOMA_CONFIG_FILE='%s'", std::getenv("STATICOMA_CONFIG_FILE"));

    staticoma::Client staticoma_client;

    EXPECT_TRUE(staticoma_client.fetchConfig());
    EXPECT_EQ(staticoma_client.getConfigString(), "client: true\n");
    std::string config;
    std::getline(staticoma_client.getConfigStream(), config);
    EXPECT_EQ(config, "client: true");

    EXPECT_FALSE(staticoma_client.fetchRobotDescription(ros::Duration(1.0)));

    double param_val = 0.0;
    if (not ros::param::get("/something/something_else", param_val))
    {
        ros::WallDuration(3.0).sleep();
        ASSERT_TRUE(ros::param::get("/something/something_else", param_val));
        EXPECT_EQ(param_val, 1.0);
    }
}

TEST(client, client_no_file)
{
    staticoma::Client staticoma_client;
    EXPECT_FALSE(staticoma_client.fetchConfig("nonexistent.yaml"));
}

TEST(client, client_file)
{
    staticoma::Client staticoma_client;
    EXPECT_TRUE(staticoma_client.fetchConfig(ros::package::getPath("staticoma") + "/test/client.yaml"));
    EXPECT_EQ(staticoma_client.getConfigString(), "client: true\n");
    std::string config;
    std::getline(staticoma_client.getConfigStream(), config);
    EXPECT_EQ(config, "client: true");

    EXPECT_FALSE(staticoma_client.fetchRobotDescription(ros::Duration(1.0)));
}

TEST(client, client_implicit_fetch)
{
    ROS_ERROR("STATICOMA_CONFIG_FILE='%s'", std::getenv("STATICOMA_CONFIG_FILE"));

    staticoma::Client staticoma_client;

    EXPECT_EQ(staticoma_client.getConfigString(), "client: true\n");
    std::string config;
    std::getline(staticoma_client.getConfigStream(), config);
    EXPECT_EQ(config, "client: true");

    EXPECT_FALSE(staticoma_client.fetchRobotDescription(ros::Duration(1.0)));
}

TEST(client, client_no_file_implicit_fetch)
{
    staticoma::Client staticoma_client;
    EXPECT_EQ(staticoma_client.getConfigString("nonexistent.yaml"), "");

    EXPECT_FALSE(staticoma_client.fetchRobotDescription(ros::Duration(1.0)));
}

TEST(client_file, client_file_implicit_fetch)
{
    staticoma::Client staticoma_client;
    EXPECT_EQ(staticoma_client.getConfigString(ros::package::getPath("staticoma") + "/test/client.yaml"), "client: true\n");
    std::string config;
    std::getline(staticoma_client.getConfigStream(), config);
    EXPECT_EQ(config, "client: true");

    EXPECT_FALSE(staticoma_client.fetchRobotDescription(ros::Duration(1.0)));
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if (true == ros::Time::waitForValid(ros::WallDuration(10.0)))
    {
        return (RUN_ALL_TESTS());
    }
    else
    {
        ROS_ERROR("No valid time.");
        return (EXIT_FAILURE);
    }
}
