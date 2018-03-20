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
  // innovation:
  VectorXd y = z - H_ * x_;
  // calculate Kalman gain using single function:
  MatrixXd K = CalculateKalmanGain();
  // run state estimate using single function:
  Estimate(y,K);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // get state components:
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  // create vector for h(x'):
  VectorXd Hx(3);
  // populate h(x') with [rho, theta, tho_dot]:
  Hx << sqrt( px*px + py*py ), atan2( py, px ), ( px*vx + py*vy )/sqrt( px*px + py*py );
  // Innovation term:
  VectorXd y = z - Hx;
  // check error distance on theta and wrap between -pi,pi:
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;
  // calculate Kalman gain using single function:
  MatrixXd K = CalculateKalmanGain();
  // run state estimate using single function:
  Estimate(y,K);
}

// common function for kalman gain calculation
MatrixXd KalmanFilter::CalculateKalmanGain(){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = (P_ * Ht) * S.inverse();
  return K;
}

// common function for state update and covariance update
void KalmanFilter::Estimate(const VectorXd &y, const MatrixXd &K){
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
