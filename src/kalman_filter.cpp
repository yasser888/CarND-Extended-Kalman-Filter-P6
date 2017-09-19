#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/**
 * Initialize Kalman filter object with the required matrices.
 */
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * Predict the state x and state covariance matrix P using Linear Kalman Filter
 * equations.
 *
 * The state transition matrix F and process noise covariance matrix Q should
 * already be updated with the latest dt timestep.
 */
void KalmanFilter::Predict() {
  
  // Standard linear Kalman filter equations
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Update the state x by using Linear Kalman Filter equations with the
 * received measurement z.  
 *
 * Measurement matrix H maps the state to the same format as measurement z 
 * to make the predicted measurement. Error y is the difference between actual 
 * measurement and predicted measurement.  Pre-fit covariance matrix S maps 
 * the state covariance matrix P to the measurement space and adds measurement
 * noise covariance matrix R.  Kalman Gain K is calculated from these results.
 * The state x is updated using the Kalman Gain K and the error y.
 * The state covariance matrix P is also updated with the Kalman Gain K.
 * R is the measurement noise covariance matrix R_laser_.
 */
void KalmanFilter::Update(const VectorXd &z) {
  
  // Standard linear Kalman filter equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  
  // Calculate Kalman Gain K based on new measurement
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // Update state x and state covariance matrix P using Kalman Gain K
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  // Log output for debugging
  cout << "z_pred:\n" << z_pred << "\n" << endl;
  cout << "z:\n" << z << "\n" << endl;
  cout << "y:\n" << y << "\n" << endl;
  cout << "H:\n" << H_ << "\n" << endl;
  cout << "R:\n" << R_ << "\n" << endl;
  cout << "K:\n" << K << "\n" << endl;
}

/**
 * Update the state by using Extended Kalman Filter equations for nonlinear
 * RADAR measurements in polar coordinates.
 *
 * H is set as Jacobian Hj calculated for the current state.
 * R is the measurement noise covariance matrix R_radar_.
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  // First check if the Jacobian Hj could be calculated.  In case of divide by
  // zero, Hj will be returned as all zero so skip the update step.
  if (!H_.isZero()) {
    
    // Calculate predicted measurement z_pred using nonlinear measurement
    // equations h(x') for mapping cartesian state variables to polar
    // measurement coordinates:
    // z_pred[0] = rho = sqrt(px^2 + py^2)
    // z_pred[1] = phi = arctan(py/px)
    // z_pred[2] = rhodot = (px*vx + py*vy) / (sqrt(px^2 + py^2))
    VectorXd z_pred = VectorXd(3);
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];
    z_pred[0] = sqrt(px*px + py*py);
    z_pred[1] = atan2(py, px);
    z_pred[2] = (px*vx + py*vy) / (sqrt(px*px + py*py));
    
    // Raw error between actual measurement and predicted measurement
    VectorXd y = z - z_pred;
    
    // Normalize error angle for phi (y[1]) to be between [-pi,+pi]
    while (y[1] > M_PI) { y[1] -= 2*M_PI; }
    while (y[1] < -M_PI) { y[1] += 2*M_PI; }
    // y[1] = atan2(sin(y[1]), cos(y[1]));
    
    // Calculate Kalman Gain K based on new measurement
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // Update state x and state covariance matrix P using Kalman Gain K
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  
    // Log output for debugging
    cout << "z_pred:\n" << z_pred << "\n" << endl;
    cout << "z:\n" << z << "\n" << endl;
    cout << "y:\n" << y << "\n" << endl;
    cout << "H:\n" << H_ << "\n" << endl;
    cout << "R:\n" << R_ << "\n" << endl;
    cout << "K:\n" << K << "\n" << endl;
  }
  else {
    cout << "Hj not calculated due to divide by zero, update skipped." << endl;
  }
}
