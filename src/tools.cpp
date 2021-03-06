#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
{
    // TODO: Calculate the RMSE here.
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // Check
    if ((estimations.size() == 0) || (estimations.size() != ground_truth.size()))
        return rmse;

    VectorXd resDiff(4);
    
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        resDiff = estimations[i]-ground_truth[i];
        rmse.array() += resDiff.array()*resDiff.array();
    }

    rmse = rmse.array()/estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}



MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
    // Calculate a Jacobian here.
    MatrixXd Hj(3,4);

    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //check division by zero
    //if (px<0.000001 && py < 0.000001)
    //    return Hj;  // Returns zero Jacobian

    double sq = sqrt(px*px+py*py);
    
    Hj << px/sq,  py/sq, 0, 0,
          -py/(sq*sq), px/(sq*sq), 0, 0,
          py*(vx*py-vy*px)/(sq*sq*sq), px*(vy*px-vx*py)/(sq*sq*sq), px/sq, py/sq;
            
    return Hj;
}



