#ifndef TWIDDLER_H
#define TWIDDLER_H

enum State { UP, DOWN };

class Twiddler {
 public:
  /**
   * Constructor
   */
  Twiddler();

  /**
   * Constructor
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  Twiddler(double Kp, double Ki, double Kd, double Dp, double Di, double Dd);

  /**
   * Destructor.
   */
  virtual ~Twiddler();

  /**
   * Process the total squared error from the most recent run.
   * @param cte Squared CTE from most recent PID run.
   */
  void ProcessTotalSquaredCTE(double cte);

  double GetKp();

  double GetKi();

  double GetKd();

 private:
  double Dp;
  double Di;
  double Dd;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  State state;
  int iK;
  double minTotalSquaredCTE = -1.0;
  
};

#endif  // TWIDDLER_H