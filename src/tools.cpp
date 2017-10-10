#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse.fill(0);

  for (int k = 0; k < estimations.size(); ++k){
    VectorXd diff = estimations[k] - ground_truth[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

VectorXd Tools::cartesian2polar(const VectorXd& x) {
  VectorXd result(3);
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  result(0) = sqrt(px*px + py*py);
  result(1) = atan2(py, px);
  result(2) = abs(result(0))>0.0001 ? (px*vx + py*vy)/result(0) : 0.0;

  return result;
}

VectorXd Tools::polar2cartesian(VectorXd pos) {
  VectorXd result(4);

  double px = pos(0) * cos(pos(1));
  double py = pos(0) * sin(pos(1));
  double vx = pos(2) * cos(pos(1));
  double vy = pos(2) * sin(pos(1));

  result << px, py, vx, vy;
  return result;
}

MatrixXd Tools::jacobian(const VectorXd &x_state) {
  MatrixXd result = MatrixXd::Zero(3, 4);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian - Error - Division by Zero" << endl;
    return result;
  }

  result << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return result;

}
