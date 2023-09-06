#ifndef MY_EVENTS_MATH_UTILS_H
#define MY_EVENTS_MATH_UTILS_H

#include "my_events/types.h"
#include <tuple>

#include "nanoflann/nanoflann.hpp"


namespace myev
{



// Function to get plane equation from N points in a N*3 eigen matrix using the z = ax + by + c plane equation
inline std::tuple<Vec4, double> computePlaneEq(const MatX &points)
{
    myev::MatX X(points.rows(), 3);
    X.col(0) = points.col(0);
    X.col(1) = points.col(1);
    X.col(2) = myev::VecX::Ones(points.rows());
    myev::VecX Y = points.col(2);

    myev::Mat3 inv_X = (X.transpose()*X).inverse();
    myev::Vec3 abc = inv_X*X.transpose()*Y;
    myev::Vec4 plane(abc(0), abc(1), -1, abc(2));

    double avg_res = std::sqrt((X*abc - Y).squaredNorm()/points.rows());

    return {plane, avg_res};
}

// Function to get optical flow (dx, dy) from plane equation
std::tuple<double, double> fromPlaneEq2OF(const Vec4 &plane)
{
    if(plane(0) == 0 || plane(1) == 0)
    {
        std::cout << "Error: plane equation is not valid" << std::endl;
    }
    double dx = - plane(2)/plane(0);
    double dy = - plane(2)/plane(1);
    return {dx, dy};
}


// Create a wrapper for nanoFLANN from Eigen
class KDTreeNeighborSearch
{
    private:
        // Data
        MatX data_;
        int dim_;

        // NanoFLANN
        typedef nanoflann::KDTreeEigenMatrixAdaptor<MatX> KDTree;
        std::shared_ptr<KDTree> kdtree_;

    public:

        KDTreeNeighborSearch(const MatX &data)
        {
            dim_ = data.cols();
            data_ = data;
            kdtree_ = std::shared_ptr<KDTree>(new KDTree(dim_, data_, 10));
            kdtree_->index_->buildIndex();
        }

        // Get the k nearest neighbors
        std::tuple<MatX, VecX> getKNN(const VecX &query, const int k)
        {
            // Get the k nearest neighbors
            std::vector<Eigen::Index> indices(k);
            std::vector<double> dists(k);
            kdtree_->index_->knnSearch(&query[0], k, &indices[0], &dists[0]);

            // Get the k nearest neighbors
            MatX knn(k, dim_);
            VecX dists_vec(k);
            for(int i = 0; i < k; i++)
            {
                knn.row(i) = data_.row(indices[i]);
                dists_vec(i) = std::sqrt(dists[i]);
            }

            return {knn, dists_vec};

        }

        // Get the k nearest neighbors indexes and distances
        std::tuple<std::vector<size_t>, VecX> getKNNIdx(const VecX &query, const int k)
        {
            // Get the k nearest neighbors
            std::vector<Eigen::Index> indices(k);
            std::vector<double> dists(k);
            kdtree_->index_->knnSearch(&query[0], k, &indices[0], &dists[0]);

            // Get the k nearest neighbors
            VecX dists_vec(k);
            std::vector<size_t> indices_vec(k);
            for(int i = 0; i < k; i++)
            {
                indices_vec[i] = indices[i];
                dists_vec(i) = std::sqrt(dists[i]);
            }

            return {indices_vec, dists_vec};
        }

        // Get the neighbors within a radius
        std::tuple<MatX, VecX> getRadiusNN(const VecX &query, const double radius)
        {
            // Get the neighbors within a radius
            std::vector<nanoflann::ResultItem<long int, double>> ret_matches;
            kdtree_->index_->radiusSearch(&query[0], radius, ret_matches);

            // Get the neighbors within a radius
            MatX radius_nn(ret_matches.size(), dim_);
            VecX dists_vec(ret_matches.size());
            for(int i = 0; i < ret_matches.size(); i++)
            {
                radius_nn.row(i) = data_.row(ret_matches[i].first);
                dists_vec(i) = std::sqrt(ret_matches[i].second);
            }
            return {radius_nn, dists_vec};
        }

        // Get the neighbors indexes within a radius
        std::tuple<std::vector<size_t>, VecX> getRadiusNNIdx(const VecX &query, const double radius)
        {
            // Get the neighbors within a radius
            std::vector<nanoflann::ResultItem<long int, double>> ret_matches;
            kdtree_->index_->radiusSearch(&query[0], radius, ret_matches);

            // Get the neighbors within a radius
            std::vector<size_t> indices_vec(ret_matches.size());
            VecX dists_vec(ret_matches.size());
            for(int i = 0; i < ret_matches.size(); i++)
            {
                indices_vec[i] = ret_matches[i].first;
                dists_vec(i) = std::sqrt(ret_matches[i].second);
            }
            return {indices_vec, dists_vec};
        }

};


// Create a distance field image using a kd tree
std::tuple<MatX, XYRowColConverter> createImgDistanceFieldKDTree(const MatX &points, const double padding, const double res=1)
{
    // Get the min and max of the points
    VecX min = points.colwise().minCoeff();
    VecX max = points.colwise().maxCoeff();

    // Create the image
    XYRowColConverter converter(min(0) - padding, max(0) + padding, min(1) - padding, max(1) + padding, res);
    MatX img(converter.rows, converter.cols);
    img.setConstant(std::numeric_limits<double>::max());

    // Create the kd tree
    KDTreeNeighborSearch kdtree(points);

    // Get the distance field
    for(int i = 0; i < converter.rows; i++)
    {
        for(int j = 0; j < converter.cols; j++)
        {
            VecX query = converter.pixToReal(i, j);
            auto [knn, dists] = kdtree.getKNN(query, 1);
            img(i, j) = dists(0);
        }
    }
    return {img, converter};
}

}

#endif // MY_EVENTS_MATH_UTILS_H