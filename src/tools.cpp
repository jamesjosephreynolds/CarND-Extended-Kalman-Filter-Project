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
    rmse << 0,0,0,0;
    
    // From Lesson 21
    
    if (estimations.size() != ground_truth.size()){
        std::cout << "Array sizes must match!";
        return rmse;
    }
    else if (estimations.size() == 0){
        std::cout << "Arrays must be non-empty!";
        return rmse;
    }
    
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd tmp = estimations[i]-ground_truth[i];
        tmp = tmp.array()*tmp.array();
        rmse += tmp;
        
    }
    
    //calculate the mean
    rmse = rmse/estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Jacobian(3,4);
    Jacobian << 0,0,0,0,
                0,0,0,0,
                0,0,0,0;
    
    // From Lesson 17
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float c2 = px*px+py*py;
    float c1 = sqrt(c2);
    float c3 = c1*c2;
    
    if (c2 == 0){
        std::cout << "CalculateJacobian() Error - Division by zero";
        return Jacobian;
    }
    else {
        Jacobian << px/c1, py/c1, 0, 0,
        -py/c2, px/c2, 0, 0,
        py*(vx*py-vy*px)/c3, px*(-vx*py+vy*px)/c3, px/c1, py/c1;
    }
    
    return Jacobian;
}

VectorXd Tools::Cartesian2Polar(const VectorXd& x_state) {
    /**
     * Convert cartesian coordinates to polar.
     */
    VectorXd h_x(3,1);
    h_x << 1,0,0;
    
    // From Lesson 14
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float c1 = sqrt(px*px+py*py);
    
    if ((px == 0)||(c1==0)){
        std::cout << "Cartesian2Polar Error - Division by zero";
        return h_x;
    }
    else {
        h_x << c1,
               atan2(py, px),
               (vx*px+vy*py)/c1;
    }
    
    return h_x;
}

VectorXd Tools::Polar2Cartesian(const VectorXd& radar_meas) {
    /**
     * Convert polar coordinates to Cartesian.
     */
    VectorXd f_x(4,1);
    f_x << 1,1,0,0;
    
    // From Lesson 14
    
    float rho = radar_meas(0);
    float phi = radar_meas(1);
    //float rho_dot = radar_meas(2);
    
    f_x << rho*cos(phi),
           -rho*sin(phi),
           0, //unknown how to break rho_dot into vx and vy components
           0;
    
    return f_x;
}
