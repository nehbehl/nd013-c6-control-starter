/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_maxi = 0.0;
  output_lim_mini = 0.0;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  d_error = Kd * (cte - this->cte)/delta_time;
  p_error = Kp * cte;
  i_error += Ki * cte * delta_time;
  this->cte = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   
    double control= p_error + i_error + d_error;
    if(control > output_lim_maxi)
      control = output_lim_maxi;
    else
      control = output_lim_mini;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time=new_delta_time;
}
