#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  // MatrixXd Ht = H_.transpose();
  // MatrixXd S = H_ * P_ * Ht + R_; // TODO: MUST USE R_laser
  // MatrixXd K = (P_ * Ht) * S.inverse();

  MatrixXd K = CalculateKalmanGain();
  Estimate(y,K); // Use 1 function to reduce lines.
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    z is polar coordinates rho, theta, rho-dot
  */
  // TODO: Convert x_ to polar coordinates FIRST!
    VectorXd Hx = Tools::CalculatePolarMap(x_); // this should convert -pi,pi
    VectorXd zm = VectorXd(3);
    float rho = z(0);
    float theta = z(1);
    float rhodot = z(2);
    // Here, wrap theta between -pi,pi
    if (theta>PI){
        theta = -PI + (theta-PI);
    }else if (theta < -PI){
        theta = PI - (-theta-PI);
    }
    zm << rho , theta , rhodot;// seems like theta can be > pi...

    // This code sets the innovation term (or error term) of the radar measurement to correctly wrap around -pi.pi limits:
    VectorXd y = VectorXd(3);
    auto dtheta = zm[1]-Hx[1];
    if (dtheta > PI){
        dtheta = -(2*PI-fabs(dtheta));
    }else if(dtheta <-PI){
        dtheta = (2*PI-fabs(dtheta));
    }
    y << zm[0]-Hx[0],dtheta, zm[2] - Hx[2];


  MatrixXd K = CalculateKalmanGain();
  Estimate(y,K);
}

MatrixXd KalmanFilter::CalculateKalmanGain(){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // TODO: MUST USE R_laser
  MatrixXd K = (P_ * Ht) * S.inverse();

  return K;
}

void KalmanFilter::Estimate(const VectorXd &y, const MatrixXd &K){
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
