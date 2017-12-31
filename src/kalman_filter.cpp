#include "kalman_filter.h"
#include <iostream>
#define M_PI 3.14159265358979323846

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  VectorXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ = x_ + (K*y);
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  // Check division by zero
  if (fabs(px*px + py*py) < 0.0001) {
    std::cout << "UpdateEKF(): CalculateJacobian() - Error - Division by Zero" << std::endl;
    return;
  }

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py,px);
  double rho_dot = 0;
  
  if(fabs(rho)>=0.0001){
    rho_dot =  (px*vx + py*vy)/rho;
  }


  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  y(1) = atan2(sin(y(1)),cos(y(1)));

  if(y(1) < -M_PI){
    y(1) += 2*M_PI;
  }

  if(y(1) > M_PI){
    y(1) -= 2*M_PI;
  }

  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();

  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  x_ = x_+ (K*y);
  P_ = (I - K*H_)*P_;
}