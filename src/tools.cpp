#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd error(4);
	MatrixXd for_square(4,4);

	error = ground_truth - estimations;
	for_square << error[0], 0, 0, 0,
				  0, error[1], 0, 0,
				  0, 0, error[2], 0,
				  0, 0, 0, error[3];
	error = sqrt(for_square * error);
	return error;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	float p1 = pow(x_state[0], 2) + pow(x_state[1], 2);
	float p2 = x_state[2]*x_state[1] - x_state[0]*x_state[3];
	MatrixXd Jacobian(3,4);

	Jacobian << x_state[0]/pow(p1, 1/2), x_state[1]/pow(p1, 1/2), 0, 0,
				-x_state[1]/p1, x_state[0]/p1, 0, 0,
				x_state[1]*p2/pow(p1, 3/2), x_state[0]*(-p2)/pow(p1, 3/2), x_state[0]/pow(p1, 1/2), x_state[1]/pow(p1, 1/2)	 
	return Jacobian;
}
