#ifndef PID_H
#define PID_H

class PID
{
public:
  /*
	 * Errors
	 */
  double int_val;
  double last_int_val;
  double last_error;

  /*
	 * Coefficients
	 */
  double Kp;
  double Ki;
  double Kd;

  // current Value of total Error
  double val;
  double integral;
  double derivative;
  double y;

  double min_val;
  double max_val;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double lKp, double lKi, double lKd, double mn, double mx);

  /*
	 * Output Value for PID instantiation
	 */
  double output(double error, double sample_time);
};

#endif // PID_H