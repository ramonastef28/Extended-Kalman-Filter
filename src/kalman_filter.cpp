#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
//using std::vector;

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
   VectorXd y_ = VectorXd::Zero(z.rows());
   MatrixXd I = MatrixXd::Identity(x_.rows(), x_.rows());   
   y_ = z - H_*x_;
   MatrixXd S_ = H_*P_*H_.transpose() + R_;
   MatrixXd K_ = P_*H_.transpose()*S_.inverse();
   // new state
   x_ = x_ + K_* y_;
   P_ = (I - K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd y_ = VectorXd::Zero(z.rows()); 
  VectorXd z_pred(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];



  float ro = sqrt(px * px + py *py);
  float phi = atan (py/px);
  float ro_dot = (px*vx + py *vy)/ro;  
  //z_pred << sqrt(x_[0]*x_[0]+x_[1]*x_[1]), atan(x_[1]/x_[0]), (x_[0]*x_[2] + x_[1]*x_[3])/sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
  z_pred << ro, phi, ro_dot;
  y_ = z - z_pred;
  MatrixXd I = MatrixXd::Identity(x_.rows(), x_.rows());
  //y_ = z - H_*x_;
  
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  
  // new state
   x_ = x_ + K_* y_;
   P_ = (I - K_*H_)*P_;
}
