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
    if(estimations.size() != ground_truth.size() || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    for(unsigned int i = 0; i < estimations.size(); ++i){
        VectorXd temp(4);
        temp = estimations[i] - ground_truth[i];
        temp = temp.array()*temp.array();
        rmse += temp;
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
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    if(fabs(px) < 0.0001)
      px = 0.0001;
    if(fabs(py) < 0.0001)
      py = 0.0001;

    float t_1 = px*px + py*py;
    float t_1_2 = sqrt(t_1);
    float t_3_2 = t_1 * t_1_2;

    MatrixXd Hj(3,4);
    if(t_1 < 0.0001 * 0.0001){
        std::cout<<"Error! check the denominator is zero or not."<<std::endl;
        //return Hj;
        t_1 = 0.0001 * 0.0001;
      }

    Hj << px/t_1_2, py/t_1_2, 0, 0,
          -py/t_1, px/t_1, 0, 0,
          py*(vx*py - vy*px)/t_3_2, px*(vy*px - vx*py)/t_3_2, px/t_1_2, py/t_1_2;

    return Hj;
}
