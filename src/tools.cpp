#include <iostream>
#include "tools.h"
#include <cmath>

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
	VectorXd rmse(4);
	rmse<<0,0,0,0;

	for(int i=0; i<estimations.size(); i++)
	{
		error = ground_truth[i] - estimations[i];
		error = error.array() * error.array(); // Array conversion allows element-wise multiplication.
		rmse+= error;
	}
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();
	
	return rmse;
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
				x_state[1]*p2/pow(p1, 3/2), x_state[0]*(-p2)/pow(p1, 3/2), x_state[0]/pow(p1, 1/2), x_state[1]/pow(p1, 1/2);	 
	return Jacobian;
}

void Tools::pi_range(VectorXd &value)
{
	int sign = value[1]/abs(value[1]);
	while(value[1] > M_PI || value[1] < -M_PI)
	{
		value[1]+= -sign*2*M_PI;
	}
}
