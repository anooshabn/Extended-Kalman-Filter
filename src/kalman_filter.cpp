#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd ft = F_.transpose();
  P_ = F_ * P_ * ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  updateState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py,px);
  double rho_dot = (px * vx + py * vy)/(rho);
  
  if (fabs(px) < 0.0001 || fabs(py) < 0.0001){
  	if (fabs(px) < 0.0001){
  		px = 0.0001;
  	}
  	if (fabs(py) < 0.0001){
  		py = 0.0001;
  	}
    phi = 0;
    rho_dot = 0;
  }
  
  VectorXd z_pre = VectorXd(3);
  z_pre << rho, phi, rho_dot;
  VectorXd y = z - z_pre;
  while (y(1) > M_PI || y(1) < -M_PI){
    if (y(1) > M_PI){
      y(1) -=  2 * M_PI;
    }else{
      y(1) +=  2 * M_PI;
    }
  }
  updateState(y);
}

void KalmanFilter::updateState(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
