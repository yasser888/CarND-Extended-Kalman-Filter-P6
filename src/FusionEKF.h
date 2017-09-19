#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  // Previous timestamp
  long long previous_timestamp_;

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter object, update and prediction math lives in here.
  */
  KalmanFilter ekf_;
  
private:
  // Check whether initialized or not (first measurement)
  bool is_initialized_;
  
  // Flags to test using only one sensor type for comparison
  bool radar_only_;
  bool laser_only_;
  
  // Process noise
  float noise_ax_;
  float noise_ay_;

  // Measurement-related matrices
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_radar_;

  /**
   * Tool object used to compute Jacobian and RMSE
   */
  Tools tools;
};

#endif /* FusionEKF_H_ */
