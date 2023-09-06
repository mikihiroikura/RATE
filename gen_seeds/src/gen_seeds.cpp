#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <fstream>

ros::Subscriber event_sub;
ros::Publisher seeds_pub;
bool isTimeInitialized = false;
double initTime;
double time_firstseed, time_event;
std::vector<std::string> seeds;

auto splitString(const std::string& s, char delim) -> std::vector<std::string> {
  std::vector<std::string> ret;
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) { ret.emplace_back(item); }
  return ret;
}

void EventMsgCallback(const dvs_msgs::EventArray::ConstPtr &event_msg){
    const int n_event = event_msg->events.size();
    if (n_event == 0){return;}
    if (!isTimeInitialized){
        initTime = event_msg->events[0].ts.toSec();
        isTimeInitialized = true;
    }

    for (const auto &e : event_msg->events){
        time_event = e.ts.toSec() - initTime;
        if (time_event > time_firstseed){
            // Publish seeds
            std_msgs::String msg;
            msg.data = seeds[0];
            seeds_pub.publish(msg);
            ROS_INFO("Publish seed: %s", seeds[0].c_str());
            // Update remaining seeds
            seeds.erase(seeds.begin());
            if (seeds.empty()){
                ROS_INFO("All seeds are published.");
                event_sub.shutdown();
                return;
            }
            else{
                auto seed_str = splitString(seeds[0], ',');
                time_firstseed = std::stod(seed_str[0]);
            }
        }
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gen_seeds");
    ros::NodeHandle nh;

    // Set seeds
    std::string seed_txt;
    if(!nh.getParam("/gen_seeds/seed_txt", seed_txt)){
        seed_txt = "/app/rosbag/scaramuzza_lab/shapes_rotation_corner_classification.txt";
    };
    std::fstream seed_txt_file;
    seed_txt_file.open(seed_txt, std::ios::in);
    if(seed_txt_file.is_open()){
        std::string line;
        while(getline(seed_txt_file, line)){
            std::cout << line << std::endl;
            seeds.push_back(line);
        }
    }
    auto seed_str = splitString(seeds[0], ',');
    time_firstseed = std::stod(seed_str[0]);


    // Set publisher
    seeds_pub = nh.advertise<std_msgs::String>("seeds", 1);

    // Set subscriber
    event_sub = nh.subscribe("/dvs/events", 0, &EventMsgCallback);


    while(ros::ok()){ros::spinOnce();}

    return 0;
}