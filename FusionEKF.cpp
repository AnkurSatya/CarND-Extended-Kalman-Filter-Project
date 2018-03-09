#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  // state_velocity_init = 1;

  previous_timestamp_ = 0;
  deltaT=0.0;

  // initializing matrices
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //Edited   
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0; // Random velocity given at starting.
   
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<   1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // tools.pi_range(measurement_pack.raw_measurements_);
      ekf_.x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]), 
                 measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
                 0,
                 0;
                 // measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]),
                 // measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
      ekf_.x_[2] = 0;
      ekf_.x_[3] = 0;
      // state_velocity_init = 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_= measurement_pack.timestamp_;
    ekf_.F_ = MatrixXd(4,4);
    ekf_.DT = MatrixXd(4,2);
    ekf_.Q_ = MatrixXd(4,4);
    // ekf_.DT = MatrixXd(4,2);
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /*
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  deltaT = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, deltaT, 0,
             0, 1, 0, deltaT,
             0, 0, 1, 0,
             0, 0, 0, 1;

  
  ekf_.DT << 0.5 * (deltaT*deltaT), 0,
             0, 0.5 * (deltaT*deltaT),
             deltaT, 0,
             0, deltaT;

  // float t_=deltaT*deltaT;
  // float t_cube=t_*deltaT;
  // float t_quad=t_*t_;

  // ekf_.Q_ <<  t_quad*ekf_.u[0]/4, 0, t_cube*ekf_.u[0]/2,0,
  //             0,t_quad*ekf_.u[1]/4,0,t_cube*ekf_.u[1]/2,
  //             t_cube*ekf_.u[0]/2,0,t_*ekf_.u[0],0,
  //             0,t_cube*ekf_.u[1]/2,0,t_*ekf_.u[1];

  ekf_.Q_ << (ekf_.DT * ekf_.u) * ((ekf_.DT * ekf_.u).transpose());
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // return;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    // cout<<"Laser"<<endl;
    // Laser updates
    // return;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout<<"Hj_ - "<<endl<<Hj_<<endl;
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
