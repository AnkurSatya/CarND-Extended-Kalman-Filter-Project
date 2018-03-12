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
	VectorXd rmse(4);
	rmse<<0,0,0,0;

	for(unsigned int i=0; i<estimations.size(); ++i)
	{
		VectorXd error = ground_truth[i] - estimations[i];
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
	MatrixXd Jacobian(3,4);

	float px = x_state[0];
	float py = x_state[1];
	float vx = x_state[2];
	float vy = x_state[3];

	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);
	

	if(fabs(c1)<0.0001)
	{
		cout<<"CalculateJacobian() - Error - Division By zero"<<endl;
		return Jacobian;
	}

	Jacobian <<(px/c2), (py/c2), 0, 0,
			   -(py/c1), (px/c1), 0, 0,
			   py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
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
