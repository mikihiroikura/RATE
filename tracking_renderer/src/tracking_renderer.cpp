#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>

#include <opencv2/core/core.hpp>

// Subscribe:   dvs_rendering_events
//              haste_track
// Publish  :   dvs_rendering_events_with_tracked
ros::Subscriber haste_sub;
image_transport::Subscriber dvs_render_sub;
image_transport::Publisher tracking_render_sub;
cv_bridge::CvImagePtr cv_ptr;
std::vector<std_msgs::Float32MultiArray> haste_track_results;


void RenderedEventImgMsgCallback(const sensor_msgs::Image::ConstPtr &rendered_event_msgs){
    cv_ptr = cv_bridge::toCvCopy(rendered_event_msgs);

    // Add tracked dot into accumulated event image
    for (auto itr = haste_track_results.begin(); itr!=haste_track_results.end(); itr++){
        cv::circle(cv_ptr->image, cv::Point(static_cast<int>(itr->data[2]), static_cast<int>(itr->data[3])), 5, cv::Scalar(0,255,0), 1);
    }    
    haste_track_results.clear();

    //Publish event images with tracking results
    tracking_render_sub.publish(cv_ptr->toImageMsg());
}

void HasteMsgCallback(const std_msgs::Float32MultiArray::ConstPtr &haste_track_msgs){
    std_msgs::Float32MultiArray haste_track_result;
    haste_track_result.data = haste_track_msgs->data;
    haste_track_results.push_back(haste_track_result);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "traking_renderer");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);


    // Set publisher
    dvs_render_sub = it.subscribe("/dvs_rendering_events", 0, &RenderedEventImgMsgCallback);
    haste_sub = nh.subscribe("/haste_track", 0, &HasteMsgCallback);

    // Set subscriber
    tracking_render_sub = it.advertise("tracking_renderer", 1);

    ros::spin();

    return 0;
}