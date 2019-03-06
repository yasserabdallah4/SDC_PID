#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

#define DEBUG 0
PID::PID()
{
}

PID::~PID() {}

// NOTE: Changed variable names to avoid any chance of incorrect scoping
void PID::Init(double lKp, double lKi, double lKd, double mn, double mx)
{
  // Initialize Gains
  // This could also be done as a copy constructor but effectively makes no difference

  Kp = lKp;
  Kd = lKd;
  Ki = lKi;

  //Initialize Errors
  int_val = 0.00;
  last_int_val = 0.00;
  last_error = 0.00;

  // Initialize Errors
  val = 0.00;
  integral = 0.00;
  derivative = 0.00;
  y = 0.00;

  min_val = mn;
  max_val = mx;
}

double PID::output(double error, double sample_time)
{

  last_int_val = int_val;

  integral = int_val + error * sample_time;
  derivative = (error - last_error) / sample_time;

  val = -1.0 * (Kp * error + Ki * int_val + Kd * derivative);

  if (val > max_val)
    val = max_val;
  else if (val < min_val)
    val = min_val;
  else
    int_val = integral;

  last_error = error;

  return val;
}