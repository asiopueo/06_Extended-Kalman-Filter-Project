#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() 
{
    F_ = MatrixXd(4, 4);
    Q_ = MatrixXd(4, 4);
    P_ = MatrixXd(4, 4);
    H_ = MatrixXd(3, 4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init_Update(MatrixXd &H_in, MatrixXd &R_in) 
{
    H_ = H_in;
    R_ = R_in;
}

void KalmanFilter::Predict() 
{
    // TODO: predict the state
    x_ = F_ * x_ ;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
    // TODO: update the state by using Kalman Filter equations

    // KF Measurement update step

    VectorXd y = VectorXd(2);

    MatrixXd S, K, Ht, Si;
    y = z - H_ * x_;
    Ht = H_.transpose();
    S = H_ * P_ * Ht + R_;
    Si = S.inverse();

    K = P_ * Ht * Si;
    // new state
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
    // TODO: update the state by using Extended Kalman Filter equations
    const double PI = 4*atan(1);

    // KF Measurement update step
    VectorXd y = VectorXd(3);
    MatrixXd S, K, Ht, Si;

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    double sq = sqrt(px*px+py*py);
    VectorXd h_ = VectorXd(3);
    MatrixXd I = MatrixXd::Identity(4, 4);

    //h_ << sq, atan2(py, px), (px*vx+py*vy) / sq;
    h_(0) = sq;

    if (fabs(px)<0.0001 || fabs(py)<0.0001) {
        h_(1) = 0.0;
        h_(2) = 0.0;
    }
    else {
        h_(1) = atan2(py, px); // atan2 returns values between -pi and +pi
        h_(2) = (px*vx+py*vy)/sq;
    }

    y = z - h_;
       
    
    // Normalizes y(1) to values between -PI and PI:
    if (y(1) > PI)
        y(1) = y(1) - 2*PI;
    else if (y(1) < -PI)
        y(1) = y(1) + 2*PI;


    Ht = H_.transpose();
    S = H_ * P_ * Ht + R_;
    Si = S.inverse();
    K = P_ * Ht * Si;

    // new state
    x_ = x_ + K * y;
    P_ = (I - K * H_) * P_;

}
