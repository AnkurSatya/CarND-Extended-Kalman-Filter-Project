#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() 
{
  //Taking noise_ax=9 and noise_ay=9;
  u = VectorXd(2); 
  u << 9, 9;
}

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
  x_ = F_ * x_;//Removed (DT * u)
  P_ = F_ * P_ * (F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y(2);// Error vector
  MatrixXd S(2,2);//Helper Matrix - error in prediction + error in signal measurement
  MatrixXd K(4,2);//Kalman Gain Matrix
  MatrixXd I = MatrixXd::Identity(4,4);//Identity Matrix

  // Update Equations
  y = z - H_ * x_;
  S = H_ * P_ * (H_.transpose()) + R_;
  K = P_ * (H_.transpose()) * (S.inverse());
  x_ = x_ + (K * y);
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  if (x_[0]!=0)
  {
    VectorXd H_trans(3);
    float p1 = sqrt(pow(x_[0], 2) + pow(x_[1], 2));

    H_trans << p1,
               atan2(x_[1], x_[0]),
               (x_[0] * x_[2] + x_[1] * x_[3])/p1;

    VectorXd y(3);
    MatrixXd S(3,3);
    MatrixXd K(4,3);
    MatrixXd I = MatrixXd::Identity(4,4);

    //Update Equations
    y = z - H_trans;
    tools.pi_range(y);
    S = H_ * P_ * (H_. transpose()) + R_;
    K = P_ * (H_.transpose()) * (S.inverse());
    x_ = x_ + (K * y);
    P_ = (I - (K * H_)) * P_;
  }
}
