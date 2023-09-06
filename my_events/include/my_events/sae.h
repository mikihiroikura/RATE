#ifndef MY_EVENTS_SAE_H
#define MY_EVENTS_SAE_H


#include "my_events/types.h"
#include "my_events/math_utils.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


namespace myev
{


// Get neighbors of a pixel
inline std::tuple<std::array<std::array<double,3>, 3>, unsigned int> getNeighbors(const cv::Mat &img, const int x, const int y, const double time_thr = std::numeric_limits<double>::lowest())
{
    std::array<std::array<double,3>, 3> neighbors;
    unsigned int n_neighbors = 0;
    for(int i=-1; i <= 1; ++i)
    {
        for(int j=-1; j <= 1; ++j)
        {
            if((x+j >= 0) && (x+j < img.cols) && (y+i >= 0) && (y+i < img.rows))
            {
                if(img.at<double>(y+i, x+j) > time_thr)
                {
                    neighbors[i+1][j+1] = img.at<double>(y+i, x+j);
                    ++n_neighbors;
                }
                else
                {
                    neighbors[i+1][j+1] = std::numeric_limits<double>::lowest();
                }
            }
            else
            {
                neighbors[i+1][j+1] = std::numeric_limits<double>::lowest();
            }
        }
    }
    return {neighbors, n_neighbors};
}

// Get minimum time difference with neighbors
inline double getMinTimeDiff(const cv::Mat& img, const int x, const int y)
{
    double min_diff = std::numeric_limits<double>::lowest();
    for(int i=-1; i <= 1; ++i)
    {
        for(int j=-1; j <= 1; ++j)
        {
            if((i != 0) || (j != 0))
            {
                if((x+j >= 0) && (x+j < img.cols) && (y+i >= 0) && (y+i < img.rows))
                {
                    if(img.at<double>(y+i, x+j) > 0)
                    {
                        double diff = std::abs(img.at<double>(y+i, x+j) - img.at<double>(y, x));
                        if(diff < min_diff || min_diff < 0)
                        {
                            min_diff = diff;
                        }
                    }
                }
            }
        }
    }
    return min_diff;
}

// Simple difference from neighbors
inline std::tuple<bool, double, double> getSimpleGrad3Neighbor(const std::array<std::array<double,3>, 3>& neighbors, const int x, const int y, const cv::Mat& undist_table, const double time_thr = std::numeric_limits<double>::lowest())
{
    double dx = 0.0;
    double dy = 0.0;
    bool x_valid = false;
    bool y_valid = false;

    if( (neighbors[0][1] > time_thr) && (neighbors[2][1] < neighbors[0][1]) )
    {
        double y0 = undist_table.at<cv::Vec2d>(y-1, x).val[1];
        double y_curr = undist_table.at<cv::Vec2d>(y, x).val[1];
        double diff_t = neighbors[1][1] - neighbors[0][1];
        if(diff_t != 0)
        {
            dy = (y_curr-y0)/diff_t;
            y_valid = true;
            //dy = 1;
        }
    }
    else if( (neighbors[2][1] > time_thr) && (neighbors[0][1] < neighbors[2][1]) )
    {
        double y_curr = undist_table.at<cv::Vec2d>(y, x).val[1];
        double y2 = undist_table.at<cv::Vec2d>(y+1, x).val[1];
        double diff_t = neighbors[2][1] - neighbors[1][1];
        if(diff_t != 0)
        {
            dy = (y2-y_curr)/diff_t;
            y_valid = true;
            //dy = -1;
        }
    }
    if( (neighbors[1][0] > time_thr) && (neighbors[1][2] < neighbors[1][0]) )
    {
        double x0 = undist_table.at<cv::Vec2d>(y, x-1).val[0];
        double x_curr = undist_table.at<cv::Vec2d>(y, x).val[0];
        double diff_t = neighbors[1][1] - neighbors[1][0];
        if(diff_t != 0)
        {
            dx = (x_curr-x0)/diff_t;
            x_valid = true;
            //dx = 1;
        }
    }
    else if( (neighbors[1][2] > time_thr) && (neighbors[1][0] < neighbors[1][2]) )
    {
        double x_curr = undist_table.at<cv::Vec2d>(y, x).val[0];
        double x2 = undist_table.at<cv::Vec2d>(y, x+1).val[0];
        double diff_t = neighbors[1][2] - neighbors[1][1];
        if(diff_t != 0)
        {
            dx = (x2-x_curr)/diff_t;
            x_valid = true;
            //dx = -1;
        }
    }

    return {x_valid && y_valid, dx, dy};
}

// Get angle from gradient image
cv::Mat getAngleFromGrad(const cv::Mat& grad)
{
    cv::Mat angle(grad.size(), CV_64F);
    for(int y=0; y < grad.rows; ++y)
    {
        for(int x=0; x < grad.cols; ++x)
        {
            double dx = grad.at<cv::Vec2d>(y, x).val[0];
            double dy = grad.at<cv::Vec2d>(y, x).val[1];
            double ang = std::atan2(dy, dx);
            angle.at<double>(y, x) = ang;
        }
    }
    return angle;
}



class SAE
{

    private:
        cv::Mat sae_;
        cv::Mat sae_diff_;
        //cv::Mat sae_diff_ang_;
        cv::Mat sae_diff_ang_active_;
        cv::Mat grad_;
        cv::Mat grad_filter_;
        double rebound_thr_;
        double close_thr_ = 0.1;
        double max_val_ = 0.0;
        bool first_ = true;
        bool init_ = false;
        ros::Time first_time_;
        std::shared_ptr<cv::Mat> undist_table_;
        bool diff_cal_ = false;
        double diff_thr_ = -1;
        double cut_freq_ = 1.0/20000.0;
        double filter_beta_ = 2.0*M_PI*cut_freq_/(1.0 + 2.0*M_PI*cut_freq_);

    public:

        // Initialise the SAE with the image size
        // Input:
        //      width: width of the image
        //      height: height of the image
        void init(int width, int height, std::shared_ptr<cv::Mat> undist_table, double rebound_thr = 0.0005)
        {
            undist_table_ = undist_table;
            sae_ = cv::Mat::zeros(height, width, CV_64F);
            rebound_thr_ = rebound_thr;
            init_ = true;
        }

        // Add an event to the SAE
        // Input:
        //      event: event to be added
        void addEvent(const int x, const int y, const ros::Time &t)
        {
            if(!init_)
            {
                ROS_ERROR("SAE not initialised");
                return;
            }
            if(first_)
            {
                first_time_ = t;
                first_ = false;
            }
            
            double temp_t = (t - first_time_).toSec();
            double temp_diff = temp_t - sae_.at<double>(y, x);

            if(temp_diff > rebound_thr_ )
            {
                sae_.at<double>(y, x) = temp_t;
                max_val_ = temp_t;
            }

            // Get the smallest difference in the neighborhood
            // Update the difference threshold with a low pass filter
            double min_diff = getMinTimeDiff(sae_, x, y);
            if(min_diff > 0)
            {
                if(diff_thr_ < 0)
                    diff_thr_ = min_diff;
                else
                    diff_thr_ = filter_beta_*min_diff + (1.0- filter_beta_)*diff_thr_;
            }
            //diff_thr_ = 0.05;


            // If enough difference with the previous value do the differentation computations
            double proximity_thr = 2*diff_thr_;
            if( diff_cal_ && (temp_t-sae_diff_.at<double>(y,x) > 8.0*diff_thr_))
            {
                sae_diff_.at<double>(y, x) = temp_t;

                if((x > 0) && (x < (sae_.cols-1)) && (y > 0) && (y < (sae_.rows-1)))
                {
                    // Check how many neighbors have a difference smaller than the threshold
                    auto [neighbors, n_neighbors] = getNeighbors(sae_diff_, x, y, temp_t-proximity_thr);

                    // If there are enough neighbors
                    if(n_neighbors >= 5 &&  n_neighbors <= 8)
                    {
                        // Compute the gradient
                        auto [valid, dx, dy] = getSimpleGrad3Neighbor(neighbors, x, y, *undist_table_, temp_t-proximity_thr);
                        if(valid)
                        {
                            grad_.at<cv::Vec2d>(y, x).val[0] = dx;
                            grad_.at<cv::Vec2d>(y, x).val[1] = dy;
                            sae_diff_ang_active_.at<double>(y, x) = temp_t;
                            // Compute the average gradient from the neighbors
                            double sum_dx = dx;
                            double sum_dy = dy;
                            unsigned int n_valid = 1;
                            auto [grad_neighbors, n_grad_neighbors] = getNeighbors(sae_diff_ang_active_, x, y, temp_t-proximity_thr);
                            for(int i=-1; i <= 1; ++i)
                            {
                                for(int j=-1; j <= 1; ++j)
                                {
                                    if(i != 0 || j != 0)
                                    {
                                        if(grad_neighbors[j+1][i+1] > 0)
                                        {
                                            sum_dx += grad_.at<cv::Vec2d>(y+j, x+i).val[0];
                                            sum_dy += grad_.at<cv::Vec2d>(y+j, x+i).val[1];
                                            n_valid++;
                                        }
                                    }
                                }
                            }
                            sum_dx /= (double)n_valid;
                            sum_dy /= (double)n_valid;
                            grad_filter_.at<cv::Vec2d>(y, x).val[0] = sum_dx;
                            grad_filter_.at<cv::Vec2d>(y, x).val[1] = sum_dy;
                        }
                    }
                }


            }
        }

        // Get median difference in the neighborhood
        // Input:
        //      duration: duration from the last event backward to compute the median
        //      mask: mask to restrict the search area
        // Output:
        //      out: median difference in the neighborhood
        double getMedianDiffTime(const double duration, const cv::Mat &mask = cv::Mat())
        {
            double time_thr = max_val_ - duration;
            std::vector<double> diff_vec;
            for(int r=1; r < sae_.rows-1; ++r)
            {
                for(int c=1; c < sae_.cols-1; ++c)
                {
                    if (mask.empty() || mask.at<uchar>(r, c) > 0)
                    {
                        std::vector<double> temp_diff_vec;
                        if(sae_.at<double>(r, c) > time_thr && sae_.at<double>(r, c) >=0)
                        {
                            for(int i=-1; i <= 1; ++i)
                            {
                                for(int j=-1; j <= 1; ++j)
                                {
                                    if((i != 0) || (j != 0))
                                    {
                                        if(sae_.at<double>(r+i, c+j) > time_thr)
                                        {
                                            double diff = std::abs(sae_.at<double>(r, c) - sae_.at<double>(r+i, c+j));
                                            temp_diff_vec.push_back(diff);
                                        }
                                    }
                                }
                            }
                            if (temp_diff_vec.size() > 2)
                            {
                                // Get the 2nd smallest difference
                                std::sort(temp_diff_vec.begin(), temp_diff_vec.end());
                                diff_vec.push_back(temp_diff_vec[2]);
                                //max_val = std::max(max_val, sae.at<double>(r, c));
                            }

                        }
                    }
                }
            }

            // Get the median difference
            if(diff_vec.size() > 100)
            {
                std::sort(diff_vec.begin(), diff_vec.end());
                double median_diff = diff_vec[diff_vec.size()/2];

                return median_diff;
            }
            else
            {
                return -1.0;
            }
        }


        // Initialise the diff calculation
        void initDiffCalc()
        {
            diff_cal_ = true;
        
            sae_diff_ = cv::Mat::zeros(sae_.rows, sae_.cols, CV_64F);
            sae_diff_ang_active_ = cv::Mat::zeros(sae_.rows, sae_.cols, CV_64F);
            grad_ = cv::Mat::zeros(sae_.rows, sae_.cols, CV_64FC2);
            grad_filter_ = cv::Mat::zeros(sae_.rows, sae_.cols, CV_64FC2);

        }

        // Get the SAE
        // Output:
        //      out: SAE
        cv::Mat getSae()
        {
            std::cout << "Get SAE diff_thr_: " << diff_thr_ << std::endl;
            return sae_;
        }

        // Get the SAE diff
        // Output:
        //      out: SAE diff
        cv::Mat getSaeDiff()
        {
            std::cout << "Get SAE diff diff_thr_: " << diff_thr_ << std::endl;
            return sae_diff_ang_active_;
        }

        // Get the SAE diff ang
        // Output:
        //      out: SAE diff ang
        cv::Mat getSaeDiffAng()
        {
            std::cout << "Get SAE diff ang diff_thr_: " << diff_thr_ << std::endl;
            return getAngleFromGrad(grad_);
        }


        // Get last slice of the SAE
        // Input:
        //      mask: mask to restrict the search area
        // Output:
        //      out: last slice of the SAE
        void getLastSlice(cv::Mat &out, const cv::Mat &mask = cv::Mat())
        {
            // Get the median difference in the neighborhood
            double median_diff = getMedianDiffTime(std::min(25*close_thr_, 0.1), mask);
            if(median_diff > 0.0) close_thr_ = 2*median_diff;



            out = cv::Mat::zeros(sae_.rows, sae_.cols, CV_32F);

            // Get the newest events
            double time_thr = max_val_ - close_thr_;
            for(int r=0; r < sae_.rows; ++r)
            {
                for(int c=0; c < sae_.cols; ++c)
                {
                    if(mask.empty() || mask.at<uchar>(r, c) > 0)
                    {
                        if(sae_.at<double>(r, c) > time_thr)
                        {
                            out.at<float>(r, c) = 1.0;
                        }
                    }
                }
            }
            cv::Mat res_init = out.clone();
            // Remove alone pixels
            for(int r=1; r < sae_.rows-1; ++r)
            {
                for(int c=1; c < sae_.cols-1; ++c)
                {
                    if(mask.empty() || mask.at<uchar>(r, c) > 0)
                    {
                        if(res_init.at<float>(r, c) == 1.0)
                        {
                            int count = 0;
                            for(int i=-1; i <= 1; ++i)
                            {
                                for(int j=-1; j <= 1; ++j)
                                {
                                    if((i != 0) || (j != 0))
                                    {
                                        if(res_init.at<float>(r+i, c+j) == 1.0)
                                        {
                                            ++count;
                                        }
                                    }
                                }
                            }
                            if(count == 0)
                            {
                                out.at<float>(r, c) = 0.0;
                            }
                        }
                    }
                }
            }

            // Pixel blocs
            res_init = out.clone();
            int size = 3;
            int max_count = 1+size;
            for(int r=size; r < sae_.rows-size; ++r)
            {
                for(int c=size; c < sae_.cols-size; ++c)
                {
                    if(res_init.at<float>(r, c) == 1.0)
                    {
                        if(res_init.at<float>(r, c) == 1.0)
                        {
                            int count = 0;
                            for(int i=-size; i <= size; ++i)
                            {
                                for(int j=-size; j <= size; ++j)
                                {
                                    if(res_init.at<float>(r+i, c+j) == 1.0)
                                    {
                                        ++count;
                                    }
                                }
                            }
                            if(count <= max_count)
                            {
                                out.at<float>(r, c) = 0.0;
                            }
                        }
                    }
                }
            }
        }

        // Get time of latest events
        // Input:
        // Output:
        //      time of last slice of SAE
        double getTimeOfLatestEvents(){
            return max_val_;
        }


        // Get patch seed from the SAE using only the last slice of events
        // ( using corner detection, minimum of 2 corners in vicinity of the seed,
        // and score is based on the spread of Events in the SAE)
        // Input:
        //      img: input image
        //      mask: mask to restrict the search area
        //      patch_size: size of the patch to be extracted
        //      max_num: maximum number of corners to be detected
        // Output:
        //      std tuple of vector of seed points and vector of scores
        std::tuple<std::vector<cv::Point2f>, std::vector<double> > getPatchSeed(const cv::Mat& img, const cv::Mat &mask = cv::Mat(), const int patch_size = 20, const int max_num = 100, const double spread_thr = 0.3)
        {
            // Apply gaussian blur
            cv::Mat img_blur;
            cv::GaussianBlur(img, img_blur, cv::Size(5, 5), 0, 0);


            // Parameters
            const double kQualityLevel = 0.1;
            const double kMinDistance = patch_size/4.0;
            const int kBlockSize = 7;
            const bool kUseHarrisDetector = false;
            const double kK = 0.04;

            // Detect corners
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(img_blur, corners, max_num, kQualityLevel, kMinDistance, mask, kBlockSize, kUseHarrisDetector, kK);



            // For each corner find the closest neighbours in radius of patch_size/3
            std::vector<cv::Point2f> corners_final;
            std::vector<double> scores;
            for(int i=0; i < corners.size(); ++i)
            {
                // Get the neighbours
                std::vector<cv::Point2f> neighbours;
                for(int j=0; j < corners.size(); ++j)
                {
                    double dist = std::sqrt((corners[i].x-corners[j].x)*(corners[i].x-corners[j].x) + (corners[i].y-corners[j].y)*(corners[i].y-corners[j].y));
                    if(dist < patch_size/3.0)
                    {
                        neighbours.push_back(corners[j]);
                    }
                }

                // Check if the neighbours are spread enough
                if(neighbours.size() > 1)
                {
                    // Get the events in the patch around the corner
                    std::vector<cv::Point2f> events;
                    for(int r=-patch_size/2; r < patch_size/2; ++r)
                    {
                        for(int c=-patch_size/2; c < patch_size/2; ++c)
                        {
                            if(corners[i].x+c >= 0 && corners[i].x+c < img.cols && corners[i].y+r >= 0 && corners[i].y+r < img.rows)
                            {
                                if(img.at<float>(corners[i].y+r, corners[i].x+c) > 0.5)
                                {
                                    events.push_back(cv::Point2f(corners[i].x+c, corners[i].y+r));
                                }
                            }
                        }
                    }

                    // Turn the vector of events into a matrix
                    cv::Mat events_mat(events.size(), 2, CV_64F);
                    for(int j=0; j < events.size(); ++j)
                    {
                        events_mat.at<double>(j, 0) = events[j].x;
                        events_mat.at<double>(j, 1) = events[j].y;
                    }

                    // Compute the PCA of the neighbours
                    cv::PCA pca(events_mat, cv::noArray(), 0);

                    double min_eig_val = std::min(pca.eigenvalues.at<double>(0), pca.eigenvalues.at<double>(1));
                    double max_eig_val = std::max(pca.eigenvalues.at<double>(0), pca.eigenvalues.at<double>(1));

                    if(min_eig_val/max_eig_val > spread_thr)
                    {
                        corners_final.push_back(cv::Point2f(pca.mean.at<double>(0), pca.mean.at<double>(1)));
                        scores.push_back(min_eig_val/max_eig_val);
                    }
                }
            }

            return {corners_final, scores};
        }

};


class PatchSAE
{
    private:
        std::vector<std::vector<std::vector<EventPtr> > > sae_;

        double slice_thr_;

        std::vector<EventPtr> opt_flow_events_;
        std::vector<Vec2> opt_flow_vecs_;

        ros::Time first_time_;
        ros::Time last_time_;


    public:

        // Constructor
        // Input:
        //      events_: vector of events
        PatchSAE(const std::vector<EventPtr>& events, bool get_optical_flow = false)
        {
            //// Debug
            //std::cout << "Debug code in Patch SAE..." << std::endl;
            //std::vector<Vec3> plane_eqs;

            // Get first and last time stamp
            first_time_ = events[0]->t;
            last_time_ = events.back()->t;


            // Get the dimensions of the SAE
            double max_x = 0;
            double max_y = 0;
            double min_x = 1e10;
            double min_y = 1e10;
            for(int i=0; i < events.size(); ++i)
            {
                max_x = std::max(max_x, events[i]->x);
                max_y = std::max(max_y, events[i]->y);
                min_x = std::min(min_x, events[i]->x);
                min_y = std::min(min_y, events[i]->y);
            }

            // Initialize the SAE
            sae_.resize(max_x-min_x+1);
            for(int i=0; i < sae_.size(); ++i)
            {
                sae_[i].resize(max_y-min_y+1);
            }

            // Add the events to the SAE
            for(int i=0; i < events.size(); ++i)
            {
                sae_[events[i]->x-min_x][events[i]->y-min_y].push_back(events[i]);
            }

            // Get the median difference of time
            std::vector<double> dt;
            // For each pixel
            for(int i=0; i < sae_.size(); ++i)
            {
                for(int j=0; j < sae_[i].size(); ++j)
                {
                    if(sae_[i][j].size() > 0)
                    {   
                        // For each neighbour
                        for(int r=-1; r < 1; ++r)
                        {
                            for(int c=-1; c < 1; ++c)
                            {
                                if( (c!=0 || r!=0) && (i+r >= 0) && (i+r < sae_.size()) && (j+c >= 0) && (j+c < sae_[i].size()))
                                {
                                    if(sae_[i+r][j+c].size() > 0)
                                    {
                                        dt.push_back(std::abs((sae_[i][j].back()->t - sae_[i+r][j+c].back()->t).toSec()));
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Sort the vector
            std::sort(dt.begin(), dt.end());

            // Get the median
            slice_thr_ = dt[dt.size()/2];

            if(get_optical_flow) computeOpticalFlow();

            //// Debug
            //MatX to_csv(plane_eqs.size(),6);
            //double scale = 400.0;
            //for(int i=0; i < plane_eqs.size(); ++i)
            //{
            //    to_csv(i,0) = opt_flow_events_[i]->x / scale;
            //    to_csv(i,1) = opt_flow_events_[i]->y / scale;
            //    to_csv(i,2) = opt_flow_events_[i]->t.toSec()/ scale;
            //    to_csv(i,3) = plane_eqs[i](0);
            //    to_csv(i,4) = plane_eqs[i](1);
            //    to_csv(i,5) = plane_eqs[i](2);
            //}
            //Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n");
            //std::ofstream file("/home/ced/Desktop/tempplane_eqs.xyz");
            //file << to_csv.format(CSVFormat);
            //file.close();

            //std::cout << "/////////////////////////////// File saved with " << plane_eqs.size() << " events" << std::endl;
        }


        // Compute the optical flow 
        void computeOpticalFlow()
        {
            // Compute the optical flow for each pixel
            double proximity_thr = 1.5*slice_thr_;
            for(int i=0; i < sae_.size(); ++i)
            {
                for(int j=0; j < sae_[i].size(); ++j)
                {
                    // For each event in the pixel
                    for(int k=0; k < sae_[i][j].size(); ++k)
                    {
                        // If the event is not at the beginning nor the end of the batch
                        if( ( (sae_[i][j][k]->t - first_time_).toSec() > slice_thr_) && ((last_time_ - sae_[i][j][k]->t).toSec() > slice_thr_) )
                        {
                            std::vector<EventPtr> plane_events;
                            unsigned int pixel_count = 0;

                            // For each neighbour
                            for(int r=-1; r <= 1; ++r)
                            {
                                for(int c=-1; c <= 1; ++c)
                                {
                                    if( (i+r >= 0) && (i+r < sae_.size()) && (j+c >= 0) && (j+c < sae_[i].size()))
                                    {
                                        // Get all the events in proximity threshold
                                        bool found = false;
                                        for(int l=0; l < sae_[i+r][j+c].size(); ++l)
                                        {
                                            double dt = std::abs((sae_[i][j][k]->t - sae_[i+r][j+c][l]->t).toSec());
                                            if(dt < proximity_thr)
                                            {
                                                plane_events.push_back(sae_[i+r][j+c][l]);
                                                found = true;
                                            }
                                        }
                                        if(found) ++pixel_count;
                                    }
                                }
                            }
                            //std::cout << "Plane events size: " << plane_events.size() << std::endl;
                            if(pixel_count >= 8)
                            {
                                // Compute the optical flow
                                MatX ev_mat = evVecToMat(plane_events);
                                auto [plane, avg_res] = computePlaneEq(ev_mat);

                                if(avg_res < 1.0*slice_thr_)
                                {
                                    auto [dx, dy] = fromPlaneEq2OF(plane);

                                    opt_flow_events_.push_back(sae_[i][j][k]);
                                    opt_flow_vecs_.push_back(Vec2(dx, dy));

                                    //// Debug
                                    //plane_eqs.push_back((plane.segment<3>(0)).normalized());
                                }
                            }
                        }
                    }
                }
            }

        }


        // Get the first slice
        std::vector<EventPtr> getFirstSlice()
        {
            std::vector<EventPtr> slice;
            for(int i=0; i < sae_.size(); ++i)
            {
                for(int j=0; j < sae_[i].size(); ++j)
                {
                    for(int k=0; k < sae_[i][j].size(); ++k)
                    {
                        if( (sae_[i][j][k]->t - first_time_).toSec() < slice_thr_)
                        {
                            slice.push_back(sae_[i][j][k]);
                        }
                    }
                }
            }
            return slice;
        }

        // Get the last slice
        std::vector<EventPtr> getLastSlice()
        {
            std::vector<EventPtr> slice;
            for(int i=0; i < sae_.size(); ++i)
            {
                for(int j=0; j < sae_[i].size(); ++j)
                {
                    for(int k=0; k < sae_[i][j].size(); ++k)
                    {
                        if( (last_time_ - sae_[i][j][k]->t).toSec() < slice_thr_)
                        {
                            slice.push_back(sae_[i][j][k]);
                        }
                    }
                }
            }
            return slice;
        }
};



// Overlay the seeds on the image
// Input:
//      src: input image (single channel 0-1)
//      seeds: vector of seeds
//      scores: vector of scores (0-1)
// Output:
//      dist: output image (3 channel 0-255)
inline void overlaySeeds(const cv::Mat &src, cv::Mat &dist, const std::vector<cv::Point2f> &seeds, const std::vector<double> &scores)
{
    // Convert the image to 3 channels 
    cv::Mat img_temp = src.clone();
    img_temp = img_temp*255;
    img_temp.convertTo(img_temp, CV_8U);
    cv::Mat in[] = {img_temp, img_temp, img_temp};
    cv::merge(in, 3, dist);

    // Add the corners to the image
    for(int i=0; i < seeds.size(); ++i)
    {
        cv::circle(dist, seeds[i], 5, cv::Scalar(0, 255*scores[i], 255*(1-scores[i])), 2, 8, 0);
    }
}








// Code not used, saving here just in case
// Apply a laplcian-like filter
//for(int r=kSize; r < sae.rows-kSize; ++r)
//{
//    for(int c=kSize; c < sae.cols-kSize; ++c)
//    {
//        int close_count = 0;
//        int far_count = 0;

//        for(int i=-kSize; i <= kSize; ++i)
//        {
//            for(int j=-kSize; j <= kSize; ++j)
//            {
//                if((i != 0) || (j != 0))
//                {
//                    double diff = sae.at<double>(r, c) - sae.at<double>(r+i, c+j);
//                    if(diff >= 0 && diff < kCloseThr)
//                    {
//                        ++close_count;
//                    }
//                    else if(diff > kFarThr)
//                    {
//                        ++far_count;
//                    }
//                }
//            }
//        }
//        if((close_count >= kMinClose) && (far_count >= kMinFar))
//        {
//            res.at<float>(r, c) = 1.0;
//        }
//    }
//}
//// Remove fancy blobs
//cv::Mat labels, stats, centroids;
//cv::Mat res_8u;
//res.convertTo(res_8u, CV_8U);
//res_8u = 255*res_8u;
//int nLabels = cv::connectedComponentsWithStats(res_8u, labels, stats, centroids, 8, CV_32S);
//for(int i=1; i < nLabels; ++i)
//{
//    int area = stats.at<int>(i, cv::CC_STAT_AREA);
//    if(area < 2)
//    {
//        for(int r=0; r < res.rows; ++r)
//        {
//            for(int c=0; c < res.cols; ++c)
//            {
//                if(labels.at<int>(r, c) == i)
//                {
//                    res.at<float>(r, c) = 0.0;
//                }
//            }
//        }
//    }
//}


}   // namespace
# endif