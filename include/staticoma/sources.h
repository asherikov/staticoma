/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License,
    Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <sstream>

#include <ariles2/adapters/basic.h>
#include <ariles2/adapters/std_map.h>
#include <ariles2/ariles.h>
#include <ariles2/extra.h>

#include <ros/ros.h>


namespace staticoma
{
    class Provider : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, topic, std::string)                                                                        \
    ARILES2_TYPED_ENTRY_(v, type, std::string)
#include ARILES2_INITIALIZE

    public:
        typedef std::map<std::string, Provider> Map;
        typedef Map::value_type Named;
        typedef std::vector<Named> NamedList;

    public:
        static const std::string &getName(const Named &named)
        {
            return (named.first);
        }

        static const std::string &getType(const Named &named)
        {
            return (named.second.type_);
        }

        static const std::string &getTopic(const Named &named)
        {
            return (named.second.topic_);
        }


    public:
        template <class t_Message>
        static typename t_Message::ConstPtr getMessage(const Named &named, const ros::Duration &wait_time)
        {
            if (ros::message_traits::DataType<t_Message>::value() != getType(named))
            {
                ROS_ERROR("Message type mismatch %s/%s",
                          ros::message_traits::DataType<t_Message>::value(),
                          getType(named).c_str());
                throw "Message type mismatch";
            }
            return(ros::topic::waitForMessage<t_Message>(getTopic(named), wait_time));
        }

        template <class t_Data>
        static bool readMessage(t_Data *data, const Named &named, const ros::Duration &wait_time)
        {
            const std_msgs::String::ConstPtr msg_ptr = getMessage<std_msgs::String>(named, wait_time);
            if (nullptr != msg_ptr)
            {
                std::stringstream strstream;
                strstream.str(msg_ptr->data);
                ariles2::apply<ariles2::yaml_cpp::Reader>(strstream, *data);
                return (true);
            }
            return (false);
        }
    };


    class Sources : public ariles2::SloppyBase
    {
    public:
        typedef std::map<std::string, Provider::Map> SourcesMap;

#define ARILES2_DEFAULT_ID "staticoma"
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, sources, SourcesMap)
#include ARILES2_INITIALIZE


    public:
        template <class t_Message>
        Provider::NamedList select(const std::string &source_id) const
        {
            Provider::NamedList result;

            const SourcesMap::const_iterator source_it = sources_.find(source_id);
            if (sources_.end() != source_it)
            {
                for (const Provider::Named &provider : source_it->second)
                {
                    if (ros::message_traits::DataType<t_Message>::value() == Provider::getType(provider))
                    {
                        result.push_back(provider);
                    }
                }
            }

            return (result);
        }

        Provider::NamedList select(const std::string &source_id) const
        {
            return (select<std_msgs::String>(source_id));
        }
    };
}  // namespace staticoma
