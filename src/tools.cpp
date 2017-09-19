#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate RMSE between accumulated state estimations and ground truth.
 *
 * Inputs are vectors of accumulated estimations and corresponding ground truth.
 * Returns a vector of RMSE values for [px, py, vx, vy].
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ( (estimations.size() != ground_truth.size()) ||
       (estimations.size() == 0) ) {
    cout << "Invalid estimation or ground_truth data size" << endl;
    return rmse;
  }
  
  // Accumulate squared error (residual)
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // Coefficient-wise multiplication to square the error
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  // Calculate the mean of squared error
  rmse = rmse / estimations.size();
  
  // Calculate the Root of Mean Squared Error
  rmse = rmse.array().sqrt();
  
  return rmse;
}

/**
 * Calculate Jacobian matrix Hj to linearize RADAR polar coordinate measurements
 * to cartesian coordinates at the current state.
 *
 * Input is current state vector.
 * Returns 3x4 Jacobian matrix Hj.
 */
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  
  MatrixXd Hj(3,4);
  
  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Pre-compute terms to avoid repeated calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;
  
  // Check division by zero, return zero matrix if detected
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    Hj.setZero();
    return Hj;
  }
  
  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        (-py/c1), (px/c1), 0, 0,
        (py*(vx*py - vy*px)/c3), (px*(px*vy - py*vx)/c3), (px/c2), (py/c2);
  
  return Hj;
}
