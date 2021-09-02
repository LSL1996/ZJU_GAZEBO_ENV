/*
 * @FilePath: /src/DynamicMap/src/kalman_filter.cpp
 * @Brief: 
 * @Version: 1.0
 * @Date: 2020-11-02 16:51:11
 * @Author: your name
 * @Copyright: your copyright description
 * @LastEditors: your name
 * @LastEditTime: 2021-02-08 23:24:01
 */
#include <Eigen/Dense>
#include <iostream>
#include <kalman_filter.h>

using Eigen::MatrixXf;
using namespace std;

kalman_filter::kalman_filter()
{
    this->is_initialized = false;
}

void kalman_filter::setup(MatrixXf states, MatrixXf A, MatrixXf B, MatrixXf H, MatrixXf P, MatrixXf Q, MatrixXf R, ros::Time global_start_time)
{
    this->states = states;
    this->A = A;
    this->B = B;
    this->H = H;
    this->P = P;
    this->Q = Q;
    this->R = R;
    this->global_start_time_ = global_start_time;
    this->is_initialized = true;
}

void kalman_filter::setA(MatrixXf A)
{
    this->A = A;
}

void kalman_filter::estimate(MatrixXf z, MatrixXf u, ros::Time global_start_time)
{
    // predict
    // cout << "state" << this->states << endl;
    // cout << "A" << this->A << endl;

    this->states = this->A * this->states + this->B * u;
    this->P = this->A * this->P * this->A.transpose() + this->Q;
    this->global_start_time_ = global_start_time;

    // update
    MatrixXf S = this->R + this->H * this->P * this->H.transpose(); // innovation matrix
    MatrixXf K = this->P * this->H.transpose() * S.inverse(); // kalman gain
    // cout << "state" << this->states << endl;
    // cout << "K" << K << endl;
    // cout << "z" << z << endl;
    // cout << "H" <<  this->H << endl;


    this->states = this->states + K * (z - this->H * this->states);

    this->P = (MatrixXf::Identity(this->P.rows(),this->P.cols()) - K * this->H) * this->P;
    // maxScale(0) = max(maxScale(0), z(3));
    // maxScale(1) = max(maxScale(1), z(4));
    // maxScale(2) = max(maxScale(2), z(5));

    // currentScale = z.block<3,1>(3,0);
}

const float& kalman_filter::output(int state_index)
{
    if(this->is_initialized)
    {
        return this->states(state_index, 0);
    }
    else
    {
        return 0;
    }
}

void kalman_filter::forward()
{
    this->forwardNum  = this->forwardNum + 1;
}

bool kalman_filter::dead()
{
    return this->forwardNum > this->liveNum;
}

void kalman_filter::clearForward()
{
    this->forwardNum = 0;
}

Eigen::Vector3d kalman_filter::evaluateConstVel(double t){
    double dt = pow(t - this->global_start_time_.toSec(), 1);

    Eigen::Vector3d pt, vel;
    vel << this->states(3), this->states(4), this->states(5);
    
    // vel constraints
    if ( vel.norm() >= this->max_vel_){
        vel = vel / vel.norm() * this->max_vel_;   
    }
        
    pt(0) = this->states(0) + dt * vel(0);
    pt(1) = this->states(1) + dt * vel(1);
    pt(2) = 1.2;

    return pt;
}

bool kalman_filter::ifSetup(){
    return is_initialized;
}