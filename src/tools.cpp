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
 VectorXd rmse(4);
 rmse << 0,0,0,0;
 if(estimations.size() == ground_truth.size() && estimations.size()!=0){
  for(int i=0;i<estimations.size();i++){
	  VectorXd residual = estimations[i]-ground_truth[i];
	  residual = residual.array()*residual.array();
	  rmse += residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
}
return rmse;
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
    float pxys = px*px + py*py;
	//check division by zero
	if(px!=0 || py!=0){
	    Hj<< px/sqrt(pxys),py/sqrt(pxys),0,0,
	         -py/pxys,px/pxys,0,0,
	         py*(vx*py-vy*px)/sqrt(pxys*pxys*pxys),px*(vy*px-vx*py)/sqrt(pxys*pxys*pxys),px/sqrt(pxys),py/sqrt(pxys);
	}
	
	//compute the Jacobian matrix

	return Hj;
}
