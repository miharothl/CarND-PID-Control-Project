#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0.;
  this->d_error = 0.;
  this->i_error = 0.;
}

void PID::UpdateError(double cte) {

//    prev_cte = robot.y
//    int_cte = 0
//    for i in range(n):
//        cte = robot.y
//        diff_cte = cte - prev_cte
//        prev_cte = cte
//        int_cte += cte
//        steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
//        robot.move(steer, speed)
//        x_trajectory.append(robot.x)
//        y_trajectory.append(robot.y)
//    return x_trajectory, y_trajectory

  this -> d_error = cte - this->p_error;
  this -> p_error = cte;
  this -> i_error =+ cte;
}

double PID::TotalError() {
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}

