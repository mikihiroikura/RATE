#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include "haste/haste_tracker.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <utility>

struct MultipleTracker
{
    unsigned int tracker_id;
    uint16_t position_id;
    HasteTracker tracker;
    double track_start_time;
};

HasteTracker haste_tracker;
std::vector<MultipleTracker> haste_trackers;
bool isTimeInitialized = false;
double initTime;
constexpr auto kRemovalMargin = haste::app::Tracker::kPatchSizeHalf;
haste::app::Camera camera;
const auto stoppingCondition = [&](const haste::app::Tracker& tracker) -> bool {
    return !((tracker.x() >= kRemovalMargin) && (tracker.y() >= kRemovalMargin)
                && ((tracker.x() + kRemovalMargin) < camera.width) && ((tracker.y() + kRemovalMargin) < camera.height));
};
ros::Subscriber event_sub, seed_sub;
ros::Publisher haste_pub;
std_msgs::Float32MultiArray haste_result;
bool finishTracking = false;
bool poschange_flg = false;
std::string camera_size, seed_data, camera_calib, best_tracker;
unsigned int tracker_id_cnt = 1;
uint16_t position_id_old = 0, position_id_tmp = 0, position_id_tmp_seed = 0, position_id_read_seed = 0;
uint16_t subimage_size = 30;
unsigned int** tracker_distribution;
unsigned int* base_tracker_distribution;
unsigned int num_distribution, max_duplication = 9;
int distribution_cnt = 0, distribution_cnt_addseed = 0, max_hypothesis_score_itr_dist_subimg = 0;
float max_hypothesis_score = 0;
float bad_tracking_threshold;
float timestamp, tracking_duration_time = 0;

template<typename A, size_t N, typename T>
void fillArray(A (&array)[N], const T &val){
    std::fill( (T*)array, (T*)(array+N), val );
}

uint16_t ReadSeed(std::string seed_data){
    auto seed_str = haste::splitString(seed_data, ',');
    if (seed_str.size() != 5){
        LOG(FATAL) << "Read Seed: Unexpected number of tokens.";
    }
    auto x = (unsigned int) std::stod(seed_str[1]);
    auto y = (unsigned int) std::stod(seed_str[2]);
    position_id_read_seed = static_cast<uint16_t>(y / subimage_size) * static_cast<uint16_t>(camera.width / subimage_size) + static_cast<uint16_t>(x / subimage_size);
    return position_id_read_seed;
}

double ReadTimestamp(std::string seed_data){
    auto seed_str = haste::splitString(seed_data, ',');
    if (seed_str.size() != 5){
        LOG(FATAL) << "Read Seed: Unexpected number of tokens.";
    }
    auto t = (double) std::stod(seed_str[0]);
    return t;
}


void EventMsgCallback(const dvs_msgs::EventArray::ConstPtr &event_msg){
    const int n_event = event_msg->events.size();
    if (n_event == 0) {return;}
    if (!isTimeInitialized){
        initTime = event_msg->events[0].ts.toSec();
        isTimeInitialized = true;
    }

    for (const auto& e : event_msg->events) {
        poschange_flg = false;
        int itr_haste_size = haste_trackers.size();
        for (int itr_haste = 0; itr_haste < itr_haste_size; itr_haste++){
            auto haste_tracker = haste_trackers[itr_haste].tracker;
            auto result = haste_tracker.track(e.ts.toSec() - initTime, e.x, e.y);
            if (result == haste::app::Tracker::EventUpdate::kStateEvent){
                haste::app::TrackerPtr tracker = haste_tracker.returnTracker();
                // Update position id
                position_id_old = haste_trackers[itr_haste].position_id;
                position_id_tmp = static_cast<uint16_t>(tracker->y() / subimage_size) * static_cast<uint16_t>(camera.width / subimage_size) + static_cast<uint16_t>(tracker->x() / subimage_size);
                if (position_id_old != position_id_tmp){
                    // Position id is changed
                    poschange_flg = true;
                    // Delete old distribution
                    for (int itr_distr = 0; itr_distr < max_duplication; itr_distr++){
                        if(tracker_distribution[position_id_old][itr_distr] == haste_trackers[itr_haste].tracker_id){
                            tracker_distribution[position_id_old][itr_distr] = 0;
                            break;
                        }
                    }
                    // Create new distribution
                    haste_trackers[itr_haste].position_id = position_id_tmp;
                    for (int itr_distr = 0; itr_distr < max_duplication; itr_distr++){
                        if(tracker_distribution[position_id_tmp][itr_distr] == 0){
                            tracker_distribution[position_id_tmp][itr_distr] = haste_trackers[itr_haste].tracker_id;
                            break;
                        }
                    }
                    // Check hypothesis score and evaluate if tracking fails
                    bad_tracking_threshold = (tracker->max_hypothesis_score() - tracker->min_hypothesis_score()) / tracker->max_hypothesis_score();
                    if (bad_tracking_threshold < 0.1){
                        ROS_INFO("Tracking failure... Tracker ID: %d, threshold: %f", haste_trackers[itr_haste].tracker_id, bad_tracking_threshold);
                        haste_trackers[itr_haste] = haste_trackers.back();
                        haste_trackers.pop_back();
                        itr_haste--;
                        itr_haste_size--;
                        continue;
                    }
                }
                ROS_INFO("Tracker ID: %d, Position ID: %d, Tracking_time: %lf, state updated to: {t=%lf, x=%.0lf, y=%.0lf, theta=%lf}", haste_trackers[itr_haste].tracker_id, haste_trackers[itr_haste].position_id, tracker->t() - haste_trackers[itr_haste].track_start_time, tracker->t(), tracker->x(), tracker->y(), tracker->theta());
                haste_result.data[0] = haste_trackers[itr_haste].tracker_id; // ID
                haste_result.data[1] = tracker->t();    // Time
                haste_result.data[2] = tracker->x();    // x
                haste_result.data[3] = tracker->y();    // y
                haste_result.data[4] = tracker->theta();// theta
                // Publish tracked data
                haste_pub.publish(haste_result);

                // Out of range. Stop tracker
                if (stoppingCondition(*tracker)){
                    ROS_INFO("Tracked point is out of range. Exit.");
                    // Delete distribution
                    int position_id = haste_trackers[itr_haste].position_id;
                    for (int itr_distr = 0; itr_distr < max_duplication; itr_distr++){
                        if(tracker_distribution[position_id][itr_distr] == haste_trackers[itr_haste].tracker_id){
                            tracker_distribution[position_id][itr_distr] = 0;
                            break;
                        }
                    }
                    // Delete tracker
                    haste_trackers[itr_haste] = haste_trackers.back();
                    haste_trackers.pop_back();
                    itr_haste--;
                    itr_haste_size--;
                }
            }
        }

        // Check duplication in one subimage
        if (poschange_flg){
            for (int itr_dist = 0; itr_dist < num_distribution; itr_dist++){
                distribution_cnt = 0;
                for (int itr_dist_subimg = 0; itr_dist_subimg < max_duplication; itr_dist_subimg++){
                    if (tracker_distribution[itr_dist][itr_dist_subimg] != 0){
                        distribution_cnt++;
                    }
                }
                if(distribution_cnt > 1){
                    // Select one seed by using hypothesis score
                    max_hypothesis_score = 0;
                    max_hypothesis_score_itr_dist_subimg = 0;
                    for (int itr_dist_subimg = 0; itr_dist_subimg < max_duplication; itr_dist_subimg++){
                        if (tracker_distribution[itr_dist][itr_dist_subimg] != 0){
                            // Delete tracker
                            for (int itr_haste = 0; itr_haste < haste_trackers.size(); itr_haste++){
                                if (haste_trackers[itr_haste].tracker_id == tracker_distribution[itr_dist][itr_dist_subimg]){
                                    // Highest alignment score
                                    if (best_tracker == "alignment_score" || (best_tracker != "lifespan" && best_tracker != "hybrid")) {
                                        if (max_hypothesis_score != 0 && max_hypothesis_score > haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score()){
                                            // Erase current haste_tracker
                                            haste_trackers[itr_haste] = haste_trackers.back();
                                            haste_trackers.pop_back();
                                            // Delete distribution
                                            tracker_distribution[itr_dist][itr_dist_subimg] = 0;
                                            break;
                                        }
                                        else if (max_hypothesis_score != 0 && haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score() >= max_hypothesis_score){
                                            // Erase old haste tracker
                                            for (int itr_haste_j = 0; itr_haste_j < haste_trackers.size(); itr_haste_j++){
                                                if (haste_trackers[itr_haste_j].tracker_id == tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg]){
                                                    haste_trackers[itr_haste_j] = haste_trackers.back();
                                                    haste_trackers.pop_back();
                                                    break;
                                                }
                                            }
                                            // Delete distribution
                                            tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg] = 0;
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score = haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score();
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            break;
                                        }
                                        else{
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            max_hypothesis_score = haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score();
                                        }
                                    }
                                    // Longest lifespan
                                    else if (best_tracker == "lifespan") {
                                        auto tracker = haste_trackers[itr_haste].tracker.returnTracker();
                                        tracking_duration_time = tracker->t() - haste_trackers[itr_haste].track_start_time;
                                        if (max_hypothesis_score != 0 && max_hypothesis_score > (float)tracking_duration_time){
                                            // Erase current haste_tracker
                                            haste_trackers[itr_haste] = haste_trackers.back();
                                            haste_trackers.pop_back();
                                            // Delete distribution
                                            tracker_distribution[itr_dist][itr_dist_subimg] = 0;
                                            break;
                                        }
                                        else if (max_hypothesis_score != 0 && (float)tracking_duration_time >= max_hypothesis_score){
                                            // Erase old haste tracker
                                            for (int itr_haste_j = 0; itr_haste_j < haste_trackers.size(); itr_haste_j++){
                                                if (haste_trackers[itr_haste_j].tracker_id == tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg]){
                                                    haste_trackers[itr_haste_j] = haste_trackers.back();
                                                    haste_trackers.pop_back();
                                                    break;
                                                }
                                            }
                                            // Delete distribution
                                            tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg] = 0;
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score = (float)tracking_duration_time;
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            break;
                                        }
                                        else{
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            max_hypothesis_score = (float)tracking_duration_time;
                                        }
                                    }
                                    // Hybrid = Higher alignment score and longer lifespan
                                    else if (best_tracker == "hybrid") {
                                        auto tracker = haste_trackers[itr_haste].tracker.returnTracker();
                                        tracking_duration_time = tracker->t() - haste_trackers[itr_haste].track_start_time;
                                        if (max_hypothesis_score != 0 && max_hypothesis_score > haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score() * (float)tracking_duration_time){
                                            // Erase current haste_tracker
                                            haste_trackers[itr_haste] = haste_trackers.back();
                                            haste_trackers.pop_back();
                                            // Delete distribution
                                            tracker_distribution[itr_dist][itr_dist_subimg] = 0;
                                            break;
                                        }
                                        else if (max_hypothesis_score != 0 && haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score() * (float)tracking_duration_time >= max_hypothesis_score){
                                            // Erase old haste tracker
                                            for (int itr_haste_j = 0; itr_haste_j < haste_trackers.size(); itr_haste_j++){
                                                if (haste_trackers[itr_haste_j].tracker_id == tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg]){
                                                    haste_trackers[itr_haste_j] = haste_trackers.back();
                                                    haste_trackers.pop_back();
                                                    break;
                                                }
                                            }
                                            // Delete distribution
                                            tracker_distribution[itr_dist][max_hypothesis_score_itr_dist_subimg] = 0;
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score = haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score() * (float)tracking_duration_time;
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            break;
                                        }
                                        else{
                                            // Update max hypothesis score and its iteration
                                            max_hypothesis_score_itr_dist_subimg = itr_dist_subimg;
                                            max_hypothesis_score = haste_trackers[itr_haste].tracker.returnTracker()->max_hypothesis_score() * (float)tracking_duration_time;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void SeedMsgCallback(const std_msgs::String::ConstPtr& seed_msg){
    seed_data = seed_msg->data;
    position_id_tmp_seed = ReadSeed(seed_data);
    timestamp = ReadTimestamp(seed_data);
    // Add new seed when no seed is in subimage
    distribution_cnt_addseed = 0;
    for (int itr_dist_subimg = 0; itr_dist_subimg < max_duplication; itr_dist_subimg++){
        if (tracker_distribution[position_id_tmp_seed][itr_dist_subimg] != 0){
            distribution_cnt_addseed++;
        }
    }
    if (distribution_cnt_addseed == 0){
        // Add tracker
        haste_tracker = HasteTracker(camera_size, seed_data, camera_calib);
        MultipleTracker multiTracker;
        multiTracker.tracker = haste_tracker;
        multiTracker.tracker_id = tracker_id_cnt;
        multiTracker.position_id = position_id_tmp_seed;
        multiTracker.track_start_time = timestamp;
        haste_trackers.emplace_back(multiTracker);

        // Add distribution
        tracker_distribution[position_id_tmp_seed][0] = tracker_id_cnt;
        tracker_id_cnt++;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "haste_ros");
    ros::NodeHandle nh;

    // set Tracker
    nh.getParam("/haste_ros/camerasize", camera_size);
    nh.getParam("/haste_ros/seed_data", seed_data);
    nh.getParam("/haste_ros/calib_file", camera_calib);
    nh.getParam("/haste_ros/best_tracker_func", best_tracker);
    haste_tracker = HasteTracker(camera_size, seed_data, camera_calib);
    camera = haste_tracker.returnCamera();

    // Create trackers distribution
    num_distribution = static_cast<unsigned int>(camera.width / subimage_size) * static_cast<unsigned int>(camera.height / subimage_size);
    tracker_distribution = (unsigned int**)malloc(sizeof(unsigned int*) * num_distribution);
    base_tracker_distribution = (unsigned int*)malloc(sizeof(unsigned int) * num_distribution * max_duplication);
    for (int i = 0; i < num_distribution; i++){
        tracker_distribution[i] = base_tracker_distribution + i * max_duplication;
    }
    for (int i = 0; i < num_distribution; i++) {
        for (int j = 0; j < max_duplication; j++) {
            tracker_distribution[i][j] = 0;
        }
    }

    // Reserve haste_trackers
    haste_trackers.reserve(100);

    // set Subscriber
    event_sub = nh.subscribe("events", 0, &EventMsgCallback);
    seed_sub = nh.subscribe("seeds", 0, &SeedMsgCallback);

    // Set publisher
    haste_pub = nh.advertise<std_msgs::Float32MultiArray>("haste_track", 1);
    haste_result.data.resize(5);

    while(ros::ok() && !finishTracking){ros::spinOnce();}

    // Free memory
    free(base_tracker_distribution);
    free(tracker_distribution);
    return 0;
}