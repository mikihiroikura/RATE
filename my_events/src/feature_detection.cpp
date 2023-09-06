#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dvs_msgs/EventArray.h"
#include "my_events/ros_utils.h"
#include "my_events/types.h"

#include "my_events/sae.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

const double kVisTime = 0.5;

double fx, fy, cx, cy, p1, p2, k1, k2, k3;
double filter_thr;
int width, height;


// Undistortion lookup table
std::shared_ptr<cv::Mat> undist_table;

bool first = true;

image_transport::Publisher pub_sae;
image_transport::Publisher pub_of;
image_transport::Publisher pub_debug;
ros::Publisher pub_seeds;


std::vector<myev::EventPtr> events;

double visualisation_period = 0.1;

// Debug variables
ros::Time t0;


std::pair<double, double> undistortPixel(double u, double v)
{
    double x = (u - cx)/fx;
    double y = (v - cy)/fy;

    double x0 = x;
    double y0 = y;

    for(int i = 0; i < 3; ++i)
    {
        double r2 = (x*x) + (y*y);
        double k_inv = 1.0 / (1 + (k1*r2) + (k2*r2*r2) + (k3*r2*r2*r2));
        double delta_x = (2*p1*x*y) + (p2*(r2+(2*x*x)));
        double delta_y = (p1*(r2+(2*y*y))) + (2*p2*x*y);
        x = (x0 - delta_x) * k_inv;
        y = (y0 - delta_y) * k_inv;
    }

    x = x*fx + cx;
    y = y*fy + cy;
    return {x,y};
}





myev::SAE img_sae;
unsigned long long int counter = 0;


void evCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
    if(first)
    {

        // Fill the undistortion lookup table
        undist_table = std::shared_ptr<cv::Mat>(new cv::Mat(msg->height, msg->width, CV_64FC2));
        for(int r = 0; r < msg->height; ++r)
        {
            for(int c = 0; c < msg->width; ++c)
            {
                auto undist = undistortPixel(c,r);
                undist_table->at<cv::Vec2d>(r,c) = cv::Vec2d(undist.first, undist.second);
            }
        }
        first = false;

        t0 = msg->events[0].ts;

        img_sae.init(msg->width, msg->height, undist_table, filter_thr);
    }


    // Insert the events in the SAE
    for(int i = 0; i < msg->events.size(); ++i)
    {
        img_sae.addEvent(msg->events[i].x, msg->events[i].y, msg->events[i].ts);

        ++counter;
    }
    

    // Publish the surface of active events
    if(pub_sae.getNumSubscribers() > 0)
    {
        cv::Mat sae_mat = img_sae.getSae();
        double max_val;
        cv::minMaxLoc(sae_mat, NULL, &max_val);
        // Convert to a 32FC1 image
        cv::Mat img_to_send = cv::Mat::zeros(sae_mat.rows, sae_mat.cols, CV_32FC1);
        for(int r = 0; r < sae_mat.rows; ++r)
        {
            for(int c = 0; c < sae_mat.cols; ++c)
            {
                if(max_val - sae_mat.at<double>(r,c) < visualisation_period)
                    img_to_send.at<float>(r,c) = sae_mat.at<double>(r,c);
                else
                    img_to_send.at<float>(r,c) = max_val-visualisation_period;
            }
        }
        // Normalise to 1
        cv::normalize(img_to_send, img_to_send, 0, 1, cv::NORM_MINMAX, CV_32FC1);
        pub_sae.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, img_to_send).toImageMsg());
    }


    // Publish the filtered SAE and potential patch seed on the debug topic
    if(pub_debug.getNumSubscribers() > 0 || pub_seeds.getNumSubscribers() > 0)
    {
        cv::Mat img_to_send;
        img_sae.getLastSlice(img_to_send);
        // Get patche seeds
        auto [seeds, scores] = img_sae.getPatchSeed(img_to_send);
    
        // Overlay the seeds on the image
        cv::Mat img_to_send_overlay;
        myev::overlaySeeds(img_to_send, img_to_send_overlay, seeds, scores);

        // Publish seeds
        if (pub_seeds.getNumSubscribers() > 0){
            double latest_sae_time = img_sae.getTimeOfLatestEvents();
            for (const auto &seed : seeds){
                std_msgs::String msg;
                msg.data = "";
                msg.data.append(std::to_string(latest_sae_time)).append(",").append(std::to_string(static_cast<int>(seed.x))).append(",");
                msg.data.append(std::to_string(static_cast<int>(seed.y))).append(",").append("0,0");
                pub_seeds.publish(msg);
            }
        }

        // Publish the image
        if (pub_debug.getNumSubscribers() > 0){
            pub_debug.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, img_to_send_overlay).toImageMsg());
        }
    }


}
    



int main(int argc, char **argv)
{

    ros::init(argc, argv, "ev_feature_detection");
    ros::NodeHandle n("~");
    image_transport::ImageTransport it_sae(n);
    pub_sae = it_sae.advertise("sae", 10);
    image_transport::ImageTransport it_of(n);
    image_transport::ImageTransport it_debug(n);
    pub_debug = it_debug.advertise("debug", 10);
    fx = readRequiredField<double>(n, "fx");
    fy = readRequiredField<double>(n, "fy");
    cx = readRequiredField<double>(n, "cx");
    cy = readRequiredField<double>(n, "cy");
    p1 = readRequiredField<double>(n, "p1");
    p2 = readRequiredField<double>(n, "p2");
    k1 = readRequiredField<double>(n, "k1");
    k2 = readRequiredField<double>(n, "k2");
    k3 = readRequiredField<double>(n, "k3");
    filter_thr = readField<double>(n, "filter_thr", 0.0005);
    ros::Subscriber sub = n.subscribe("/dvs/events", 1000, evCallback);

    // Set publisher
    pub_seeds = n.advertise<std_msgs::String>("seeds", 1);

    ros::spin();
    return 0;
}
        
