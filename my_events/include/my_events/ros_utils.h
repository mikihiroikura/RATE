#ifndef MY_EVENTS_ROS_UTILS_H
#define MY_EVENTS_ROS_UTILS_H


#include "ros/ros.h"


template<class T>
inline void printOption(bool user_defined, std::string field, T value)
{
    std::stringstream stream;
    if(user_defined)
    {
        stream << "[Param] User defined value for " << field << " is " << value;
    }
    else
    {
        stream << "[Param] Default value for " << field << " is " << value;
    }
    ROS_INFO("%s",stream.str().c_str());
}

inline void printOptionError(std::string field)
{
    std::stringstream stream;
    stream << "[Param] User defined value for " << field << " is not valid";
    ROS_ERROR("%s",stream.str().c_str());
    throw std::invalid_argument("Invalid parameter");
}


template<class T>
inline T readField(ros::NodeHandle& n, std::string field, T default_value)
{   
    bool user_defined = false;
    T output;
    if(n.hasParam(field))
    {
        user_defined = true;
        n.getParam(field, output);
    }
    else
    {
        output = default_value;
    }
    printOption(user_defined, field, output);
    return output;
}

template<class T>
inline T readRequiredField(ros::NodeHandle& n, std::string field)
{
    T output;
    if(n.hasParam(field))
    {
        n.getParam(field, output);
    }
    else
    {
        std::stringstream stream;
        stream << "[Param] It seems that the parameter " << field << " is not provided";
        ROS_ERROR("%s",stream.str().c_str());
        throw std::invalid_argument("Invalid parameter");
    }
    printOption(true, field, output);
    return output;
}


#endif