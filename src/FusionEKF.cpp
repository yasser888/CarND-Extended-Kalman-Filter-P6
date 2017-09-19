#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  
  // Set flags to test using only one sensor type for comparison
  radar_only_ = false;
  laser_only_ = false;

  // Dummy initial previous timestamp, will be updated with first measurement
  previous_timestamp_ = 0;

  // Initialize R and H matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_radar_ = MatrixXd(3, 4);

  // Measurement noise covariance matrix R for LASER
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement noise covariance matrix R for RADAR
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // Measurement matrix H_laser_ for mapping state x = [px, py, vx, vy] to
  // the measured [px, py] vector
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Can't set Hj_radar_ in construction of object because it needs to be
  // calculated after the state x is determined using the
  // tools.CalculateJacobian(ekf_.x_) function.
  
  // Set process noise to be used in calculating process noise covariance
  // matrix Q.  These are values for treating acceleration as a noise.
  noise_ax_ = 9.;
  noise_ay_ = 9.;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

/**
 * Main fusion Kalman filter process loop.
 *
 * If first measurement, just initialize parameters based on type of 
 * measurement.
 *
 * If already initialized, perform linear Kalman filter Predict step with 
 * the current dt time step.
 *
 * Next, perform Update step using linear Kalman filter if LASER or nonlinear 
 * Extended Kalman filter if RADAR measurement.
 */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  
  // Reset filter if timestamp resets (restart dataset) or
  // has a gap > 1000 sec (switch dataset)
  if ((measurement_pack.timestamp_ < previous_timestamp_) ||
      (abs(previous_timestamp_ - measurement_pack.timestamp_) > 1000000000.0)) {
    is_initialized_ = false;
  }
  
  // Initialize filter if needed
  if (!is_initialized_) {
    if (((laser_only_ == false) && (radar_only_ == false)) ||
        ((laser_only_ == true) &&
         (measurement_pack.sensor_type_ == MeasurementPackage::LASER)) ||
        ((radar_only_ == true) &&
         (measurement_pack.sensor_type_ == MeasurementPackage::RADAR))) {
      
      // Initialize previous timestamp with first timestamp for next dt calc
      previous_timestamp_ = measurement_pack.timestamp_;

      // Initialize filter matrices
      VectorXd x_ = VectorXd(4);
      MatrixXd P_ = MatrixXd(4, 4);
      MatrixXd F_ = MatrixXd(4, 4);
      MatrixXd Q_ = MatrixXd(4, 4);
      
      // State covariance matrix P set with high uncertainty for velocities
      P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
      
      // State transition matrix F set for linear motion equations, but will
      // be updated by dt in the predict step
      F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
      
      // Process noise covariance matrix Q values will be set in the predict
      // step based on dt

      // Set initial state x values based on measurement type and init ekf_
      if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert RADAR from polar to cartesian coordinates
        // px = rho * cos(phi)
        // py = rho * sin(phi)
        // vx = vy = 0 (unknown)
        float rho = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        x_ << (rho * cos(phi)), (rho * sin(phi)), 0, 0;
        cout << "RADAR initial x_ = \n" << x_ << endl;

        // Initialize ekf_ object with current matrices
        ekf_.Init(x_, P_, F_, Hj_radar_, R_radar_, Q_);
      }
      else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state directly from measured LASER px, py.
        // px = measured px
        // py = measured py
        // vx = vy = 0 (unknown)
        float px = measurement_pack.raw_measurements_[0];
        float py = measurement_pack.raw_measurements_[1];
        x_ << px, py, 0, 0;
        cout << "LIDAR initial x_ = \n" << x_ << endl;

        // Initialize ekf_ object with current matrices
        ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
      }

      // Done initializing
      is_initialized_ = true;
    }
    else {
      // Using only LASER or RADAR and the first measurement was the other type
      // so ignore it and initialize x as a dummy with all zeros.
      VectorXd x_ = VectorXd(4);
      ekf_.x_ = x_;
      ekf_.x_.setZero();
    }
    // No need to predict or update for initialization step
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  if (((laser_only_ == false) && (radar_only_ == false)) ||
      ((laser_only_ == true) &&
       (measurement_pack.sensor_type_ == MeasurementPackage::LASER)) ||
      ((radar_only_ == true) &&
       (measurement_pack.sensor_type_ == MeasurementPackage::RADAR))) {

    // Compute the time elapsed dt between current and previous measurements.
    // dt unit is seconds.
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // Update the state transition matrix F using dt for this timestep.
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    // Update the process noise covariance matrix Q using dt for this timestep.
    // These equations are derived from treating acceleration as a noise.
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    ekf_.Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
                0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
                dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
                0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
    
    // Linear Kalman Filter prediction step for either measurement type
    ekf_.Predict();
  
    /***************************************************************************
     *  Update
     **************************************************************************/
   
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // RADAR update using nonlinear Extended Kalman Filter equations
      Hj_radar_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_radar_;
      ekf_.R_ = R_radar_;

      // Get RADAR measurement values [rho, phi, rhodot]
      VectorXd z_ = VectorXd(3);
      z_[0] = measurement_pack.raw_measurements_[0];
      z_[1] = measurement_pack.raw_measurements_[1];
      z_[2] = measurement_pack.raw_measurements_[2];

      // Extended Kalman Filter update step
      ekf_.UpdateEKF(z_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // LASER update using linear Kalman Filter equations
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      
      // Get LASER measurement values [px, py]
      VectorXd z_ = VectorXd(2);
      z_[0] = measurement_pack.raw_measurements_[0];
      z_[1] = measurement_pack.raw_measurements_[1];

      // Linear Kalman Filter update step
      ekf_.Update(z_);
    }
  }
}
