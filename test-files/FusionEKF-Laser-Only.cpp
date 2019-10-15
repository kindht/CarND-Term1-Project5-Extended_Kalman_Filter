#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
  
  // TODO:  Initialize Fusion EKF

  // create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  // initial state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // initializing matrices  H , R
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // measurement matrix  - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

// Process a single measurement
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "FusionEKF: First measurement - Initialization " << endl;
    
    float px;
    float py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      /*
      float ro;
      float theta;   
      ro    = measurement_pack.raw_measurements_[0];
      theta = measurement_pack.raw_measurements_[1];
     
      px = ro * cos(theta);  // convert coordinates
      py = ro * sin(theta);
      ekf_.x_ << px, py, 0, 0;
      */
      

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];   
      ekf_.x_ << px, py, 0, 0;
      
    }
    
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1,  0,
             0, 0, 0,  1;
  
  // pre-calculate some values
  float dt2 = pow(dt,2);
  float dt3 = pow(dt,3);
  float dt4 = pow(dt,4);
  
  float c_dt4 = dt4/4;
  float c_dt3_ax = dt3/2 * noise_ax;
  float c_dt3_ay = dt3/2 * noise_ay;
  
  // set process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << c_dt4 * noise_ax,    0,     c_dt3_ax, 0,
             0,        c_dt4 * noise_ay, 0, c_dt3_ay,
             c_dt3_ax,     0,      dt2 * noise_ax, 0,
             0,        c_dt3_ay,  0,  dt2 * noise_ay;

  // use Kalman filter to predict x_', P_'
  ekf_.Predict();
  
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    /*
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;       // MatrixXd(3, 4)
    ekf_.R_ = R_radar_;  // MatrixXd(3, 3)
    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    */

  } else {
    // TODO: Laser updates
    
    ekf_.H_ = H_laser_;  //  MatrixXd(2, 4);
    ekf_.R_ = R_laser_;  //  MatrixXd(2, 2);
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
