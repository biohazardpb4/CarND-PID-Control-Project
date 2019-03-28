#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Constructor
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  PID(double Kp_, double Ki_, double Kd_);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Process a given cross track error.
   * @param cte The current cross track error
   */
  void ProcessError(double cte);

  /**
   * Calculate the output of the PID controller.
   * @output PID controller output
   */
  double Output();

  /**
   * Calculate the total squared CTE since construction.
   * @output The total CTE since construction
   */
  double TotalSquaredCTE();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  const static int CTE_HIST_SIZE = 100;
  double cte_hist[CTE_HIST_SIZE];
  int cte_hist_index = 0;
  double total_squared_cte = 0;
};

#endif  // PID_H