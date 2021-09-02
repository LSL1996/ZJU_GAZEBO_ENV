/*
 * @FilePath: /src/DynamicMap/include/kalman_filter.h
 * @Brief: 
 * @Version: 1.0
 * @Date: 2021-01-20 10:51:32
 * @Author: your name
 * @Copyright: your copyright description
 * @LastEditors: your name
 * @LastEditTime: 2021-02-08 23:23:22
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <ros/ros.h>

using Eigen::MatrixXf;
using namespace std;

class kalman_filter
{
    public:
    // members
    bool is_initialized;
    MatrixXf states;
    MatrixXf A; // state matrix
    MatrixXf B; // input matrix
    MatrixXf H; // observation matrix
    MatrixXf P; // uncertianty
    MatrixXf Q; // process noise
    MatrixXf R; // obsevation noise
    MatrixXf maxScale;
    MatrixXf currentScale;
    ros::Time global_start_time_;

    double max_vel_ = 1.0;      // wait to add in param

    int liveNum;
    int forwardNum;

    // constructor
    kalman_filter();

    // set up the filter
    void setup( MatrixXf states,
                MatrixXf A,
                MatrixXf B,
                MatrixXf H,
                MatrixXf P,
                MatrixXf Q,
                MatrixXf R,
                ros::Time global_start_time);

    // set A (sometimes sampling time will differ)
    void setA(MatrixXf A);

    // state estimate
    void estimate(MatrixXf z, MatrixXf u, ros::Time global_start_time);

    // read output from the state
    const float&  output(int state_index);

    void forward();
    
    bool dead();

    void clearForward();

    Eigen::Vector3d evaluateConstVel(double t);

    bool ifSetup();

};

#endif