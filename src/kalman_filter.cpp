#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_; // Q - Process Covariance Matrix
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // R - Measurement noise/measurement covariance
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float ro = sqrt(px * px + py * py);
  float theta;
  
  // Check division by zero
  if (fabs(ro) < 0.0001)
  {
    cout << "ro close to zero, Not Update" << endl;
    return; 
  }

  if (fabs(px) > 0.0001)
  { 
    theta = atan2(py, px); // atan2(y,x) requires 2 arguments
  } else if (py > 0) {
    cout << "px close to zero, py >0: " << px << endl;
    theta = M_PI / 2;
  } else {
    cout << "px close to zero, py <0: " << px << endl;
    theta = - M_PI/2;
  }
  
  float ro_dot = (px * vx + py * vy) / ro;

  VectorXd z_pred(3, 1); // declare z_pred
  z_pred << ro, theta, ro_dot;
  VectorXd y = z - z_pred; // measurement difference to what we predicted
  
  // Normalize angles to be in range of -PI to PI 
  // becauese KF expecting small angle values
  if (y(1) < -M_PI)
  {
    cout << "UpdateEKF: y out of range:" << y(1) << endl;
    y(1) += 2 * M_PI;
    cout << "y(1)" << y(1) << endl;
  }
  else if (y(1) > M_PI)
  {
    cout << "UpdateEKF: y out of range:" << y(1) << endl;
    y(1) -= 2 * M_PI;
    cout << "y(1)" << y(1) << endl;
  }
 
  // same as Kalman Filter
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // R - Measurement noise/measurement covariance
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
