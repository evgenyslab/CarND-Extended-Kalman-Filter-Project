#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE
    float px2py2 = pow(px,2) + pow(py,2);
    float vxpy = vx*py;
    float vypx = vy*px;
    //check division by zero
    if (fabs(px2py2)<1e-6){
        cout << "Error\n";
        Hj << 0,0,0,0,0,0,0,0,0,0,0,0;
    }
    else{
        //compute the Jacobian matrix
        Hj <<   px/(sqrt(px2py2)),
                py/(sqrt(px2py2)),
                0,
                0,
                -py/px2py2,
                px/px2py2,
                0,
                0,
                py*(vxpy-vypx)/sqrt(pow(px2py2,3)),
                px*(vypx-vxpy)/sqrt(pow(px2py2,3)),
                px/(sqrt(px2py2)),
                py/(sqrt(px2py2));
    }
    return Hj;
}
