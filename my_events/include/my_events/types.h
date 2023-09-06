#ifndef MY_EVENTS_TYPES_H
#define MY_EVENTS_TYPES_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <iostream>

namespace myev
{
    typedef Eigen::Matrix<double, 2, 2> Mat2;
    typedef Eigen::Matrix<double, 3, 3> Mat3;
    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef Eigen::Matrix<double, 4, 1> Vec4;
    typedef Eigen::Matrix<double, 1, 3> Row3;
    typedef Eigen::Matrix<double, 1, 4> Row4;
    typedef Eigen::Matrix<double, 1, 2> Row2;
    typedef Eigen::Matrix<double, 6, 6> Mat6;
    typedef Eigen::Matrix<double, 4, 4> Mat4;
    typedef Eigen::Matrix<double, 9, 9> Mat9;
    typedef Eigen::Matrix<double, 12, 12> Mat12;
    typedef Eigen::Matrix<double, 2, 1> Vec2;
    typedef Eigen::Matrix<double, 6, 1> Vec6;
    typedef Eigen::Matrix<double, 8, 1> Vec8;
    typedef Eigen::Matrix<double, 1, 8> Row8;
    typedef Eigen::Matrix<double, 9, 1> Vec9;
    typedef Eigen::Matrix<double, 1, 9> Row9;
    typedef Eigen::Matrix<double, 1, 12> Row12;
    typedef Eigen::Matrix<double, 3, 6> Mat3_6;
    typedef Eigen::Matrix<double, 3, 2> Mat3_2;
    typedef Eigen::Matrix<double, 3, 4> Mat3_4;
    typedef Eigen::Matrix<double, 2, 6> Mat2_6;
    typedef Eigen::Matrix<double, 9, 6> Mat9_6;
    typedef Eigen::Matrix<double, 3, 9> Mat3_9;
    typedef Eigen::Matrix<double, 2, 8> Mat2_8;
    typedef Eigen::Matrix<double, 2, 9> Mat2_9;
    typedef Eigen::Matrix<double, 2, 3> Mat2_3;
    typedef Eigen::Matrix<double, 9, 3> Mat9_3;
    typedef Eigen::Matrix<double, 9, 8> Mat9_8;
    typedef Eigen::Matrix<double, 3, 12> Mat3_12;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowX;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatX;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatRX;
    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;


    // Data structure to store individual event measurements
    struct Event
    {
        ros::Time t;
        double x;
        double y;
        bool pol;

        Vec2 pixVec()
        {
            Vec2 output(x, y);
            return output;
        }
        Vec3 pixVec3(const ros::Time t_ref = ros::Time(0))
        {
            Vec3 output(x, y, (t - t_ref).toSec());
            return output;
        }

        void print()
        {
            std::cout << "Event a t = " << t << "    : x = " << x << "    y : " << y << "    pol: " << pol << std::endl;
        }

        Event(){}

        Event(Vec2 xy)
        {
            x = xy(0);
            y = xy(1);
            t = ros::Time(0);
            pol = false;
        }
        Event(Vec2 xy, ros::Time _t)
        {
            x = xy(0);
            y = xy(1);
            t = _t;
            pol = false;
        }

        Event(Vec3 xyt)
        {
            x = xyt(0);
            y = xyt(1);
            t = ros::Time(xyt(2));
            pol = false;
        }
        Event(double _x, double _y, ros::Time _t)
        {
            x = _x;
            y = _y;
            t = _t;
            pol = false;
        }
    };
    typedef std::shared_ptr<Event> EventPtr;

    // Function to convert a vector of events to a vector of MatX
    inline MatX evVecToMat(const std::vector<EventPtr>& ev_vec, const bool use_time = true)
    {
        MatX output(ev_vec.size(), use_time ? 3 : 2);
        for (int i = 0; i < ev_vec.size(); i++)
        {   
            if (use_time)
                output.row(i) = (ev_vec[i]->pixVec3(ev_vec[0]->t)).transpose();
            else
                output.row(i) = (ev_vec[i]->pixVec()).transpose();
        }
        return output;
    }


    // Data structure to convert real coordinates to pixel coordinates
    struct XYRowColConverter
    {
        double x_min;
        double x_max;
        double y_min;
        double y_max;
        unsigned int rows;
        unsigned int cols;

        XYRowColConverter(){}

        XYRowColConverter(double _x_min, double _x_max, double _y_min, double _y_max, double res)
        {
            x_min = _x_min;
            x_max = _x_max;
            y_min = _y_min;
            y_max = _y_max;
            rows = ceil((_x_max - _x_min) / res);
            cols = ceil((_y_max - _y_min) / res);
        }

        template <typename T=unsigned int, typename U=double >
        std::tuple<T, T> realToPix(const U& x, const U& y) const
        {
            // Throw error if real is out of bounds
            if (x < U(x_min) || x > U(x_max) || y < U(y_min) || y > U(y_max))
            {
                std::cout << "Error: realToPix: real is out of bounds" << std::endl;
                std::cout << "x: " << x << " y: " << y << std::endl;
                std::cout << "x_min: " << x_min << " x_max: " << x_max << " y_min: " << y_min << " y_max: " << y_max << std::endl;
                throw std::runtime_error("Error: realToPix: real is out of bounds");
            }
            T row = T((x - U(x_min)) / U(x_max - x_min) * U(rows));
            T col = T((y - U(y_min)) / U(y_max - y_min) * U(cols));
            return std::make_tuple(row, col);
        }


        Vec2 pixToReal(unsigned int row, unsigned int col) const
        {
            Vec2 real;
            real(0) = (double)(row) / (double)(rows) * (x_max - x_min) + x_min;
            real(1) = (double)(col) / (double)(cols) * (y_max - y_min) + y_min;
            return real;
        }

    };
}


#endif