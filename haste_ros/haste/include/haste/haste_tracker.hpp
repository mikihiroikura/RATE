#ifndef HASTE_TRACKER_H
#define HASTE_TRACKER_H
#include "haste/app/tracking.hpp"
#include "haste/app/command_parser.hpp"

class HasteTracker{
public:
    HasteTracker();
    HasteTracker(std::string camera_size, std::string seed, std::string calib_file);
    ~HasteTracker();
    void ChangeSeed(std::string seed_data);

    haste::HypothesisPatchTracker::EventUpdate track(double et, int ex, int ey);
    haste::app::TrackerPtr returnTracker();
    haste::app::Camera returnCamera();
private:
    haste::app::Camera camera;
    haste::app::TrackerState seed;

    haste::app::TrackerPtr tracker;
};

HasteTracker::HasteTracker(){}

HasteTracker::HasteTracker(std::string camera_size, std::string seed_data, std::string calib_file){
    // Set camera width and height
    auto camera_size_str = haste::splitString(camera_size, 'x');
    if (camera_size_str.size() != 2) {
        LOG(FATAL) << "Camera size is specified, but incorrectly formatted. Please use format WIDTHxHEIGHT.";
    }
    size_t camera_width = std::stoul(camera_size_str[0]), camera_height = std::stoul(camera_size_str[1]);
    camera.width = camera_width;
    camera.height = camera_height;
    LOG(INFO) << "camera_width: " << camera_width << ", " << "camera_height: " << camera_height;
    haste::RpgDataset::loadCalibration(calib_file, camera);

    // Set seed manually (--seed=0.009098,194,283,0,0)
    // seed from shape_translation.bag 0.6,125.0,52.0,0.0,0
    auto seed_str = haste::splitString(seed_data, ',');
    if (seed_str.size() != 5){
        LOG(FATAL) << "Unexpected number of tokens.";
    }
    auto t = (haste::app::TrackerState::Scalar) std::stod(seed_str[0]);
    auto x = (haste::app::TrackerState::Scalar) std::stod(seed_str[1]);
    auto y = (haste::app::TrackerState::Scalar) std::stod(seed_str[2]);
    auto theta = (haste::app::TrackerState::Scalar) std::stod(seed_str[3]);
    auto id = (haste::app::TrackerState::ID) std::stoul(seed_str[4]);
    seed.t = t;
    seed.x = x;
    seed.y = y;
    seed.theta = theta;
    seed.id = id;
    LOG(INFO) << "seed.t: " << t << ", " << "seed.x: " << x << ", " << "seed.y: " << y << ", " << "seed.theta: " << theta;


    // Initialize tracker
    FLAGS_tracker_type = "haste_correlation_star";
    tracker = haste::app::createTracker(FLAGS_tracker_type, seed);
}

void HasteTracker::ChangeSeed(std::string seed_data){
    auto seed_str = haste::splitString(seed_data, ',');
    if (seed_str.size() != 5){
        LOG(FATAL) << "Unexpected number of tokens.";
    }
    auto t = (haste::app::TrackerState::Scalar) std::stod(seed_str[0]);
    auto x = (haste::app::TrackerState::Scalar) std::stod(seed_str[1]);
    auto y = (haste::app::TrackerState::Scalar) std::stod(seed_str[2]);
    auto theta = (haste::app::TrackerState::Scalar) std::stod(seed_str[3]);
    auto id = (haste::app::TrackerState::ID) std::stoul(seed_str[4]);
    seed.t = t;
    seed.x = x;
    seed.y = y;
    seed.theta = theta;
    seed.id = id;
    //LOG(INFO) << "seed.t: " << t << ", " << "seed.x: " << x << ", " << "seed.y: " << y << ", " << "seed.theta: " << theta;

    // Initialize tracker
    FLAGS_tracker_type = "haste_correlation_star";
    tracker = haste::app::createTracker(FLAGS_tracker_type, seed);
}

HasteTracker::~HasteTracker(){}

haste::HypothesisPatchTracker::EventUpdate HasteTracker::track(double et, int ex, int ey){
    // If not initialized, initialize tracker
    if(tracker->status() == haste::app::Tracker::kUninitialized){
        if (et < tracker->t()){
            return haste::HypothesisPatchTracker::EventUpdate::kOutOfRange;
        }
    }

    const auto& update_type = tracker->pushEvent(et, ex, ey);

    return update_type;
}

haste::app::TrackerPtr HasteTracker::returnTracker(){
    return tracker;
};

haste::app::Camera HasteTracker::returnCamera(){
    return camera;
}

#endif