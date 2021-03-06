#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double cap;

  int counter;
  double twiddle_error;

public:

  void doRecalc(int times, double threshold);
  /*
  * Constructor
  */
  PID( double Kp, double Ki, double Kd, double cap);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
