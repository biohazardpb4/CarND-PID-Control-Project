#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::PID(double Kp_, double Ki_, double Kd_) : Kp(Kp_), Ki(Ki_), Kd(Kd_) {
  p_error = i_error = d_error = 0;
  for (int i = 0; i < PID::CTE_HIST_SIZE; i++) {
    this->cte_hist[i] = 0;
  }
}

PID::~PID() {}

void PID::ProcessError(double cte) {
  this->p_error = cte;
  this->i_error = 0;
  for (int i = 0; i < PID::CTE_HIST_SIZE; i++) {
    this->i_error += cte_hist[i];
  }
  this->d_error = cte - this->cte_hist[this->cte_hist_index];
  this->cte_hist_index = (this->cte_hist_index+1)%CTE_HIST_SIZE;
  this->cte_hist[this->cte_hist_index] = cte;
  this->total_squared_cte += cte * cte;
}

double PID::Output() {
  return -1.0*this->Kp*this->p_error - this->Ki*this->i_error - this->Kd*this->d_error;
}

double PID::TotalSquaredCTE() {
  return this->total_squared_cte;
}