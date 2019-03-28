 #include "twiddler.h"
 #include <iostream>
 
 Twiddler::Twiddler() {};

  /**
   * Constructor
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  Twiddler::Twiddler(double Kp, double Ki, double Kd, double Dp, double Di, double Dd):
    Kp(Kp), Ki(Ki), Kd(Kd), Dp(Dp), Di(Di), Dd(Dd) {
        // start in the up state
        this->state = UP;
        this->iK = 0;
        this->Kp += this->Dp;
    };

  /**
   * Destructor.
   */
  Twiddler::~Twiddler() {};

  /**
   * Process the total squared error from the most recent run.
   * @param cte Squared CTE from most recent PID run.
   */
  void Twiddler::ProcessTotalSquaredCTE(double cte) {
    std::cout << "processing cte: " << cte << ", iK: " << this->iK << ", state: " <<
      (this->state == UP ? "UP" : "DOWN") << std::endl;
    if (this->iK == 0) {
        this->Kp += this->Dp;
    }
    if (this->iK == 1) {
        this->Ki += this->Di;
    }
    if (this->iK == 2) {
        this->Kd += this->Dd;
    }
      switch(this->state) {
        case UP:
          if (cte < this->minTotalSquaredCTE || this->minTotalSquaredCTE < 0) {
              std::cout << "found min, increasing delta" << std::endl;
              if (this->iK == 0) {
                  this->Dp *= 1.1;
              }
              if (this->iK == 1) {
                  this->Di *= 1.1;
              }
              if (this->iK == 2) {
                  this->Dd *= 1.1;
              }
              this->minTotalSquaredCTE = cte;
              this->iK = (this->iK+1)%3;
          } else {
              std::cout << "flipping delta" << std::endl;
              if (this->iK == 0) {
                  this->Kp -= 2.0*this->Dp;
              }
              if (this->iK == 1) {
                  this->Ki -= 2.0*this->Di;
              }
              if (this->iK == 2) {
                  this->Kd -= 2.0*this->Dd;
              }
              this->state = DOWN;
          }
          break;
        case DOWN:
        if (cte < this->minTotalSquaredCTE || this->minTotalSquaredCTE < 0) {
              std::cout << "found min, increasing delta" << std::endl;
              if (this->iK == 0) {
                  this->Dp *= 1.1;
              }
              if (this->iK == 1) {
                  this->Di *= 1.1;
              }
              if (this->iK == 2) {
                  this->Dd *= 1.1;
              }
              this->minTotalSquaredCTE = cte;
              this->iK = (this->iK+1)%3;
        } else {
            std::cout << "decreasing delta" << std::endl;
            if (this->iK == 0) {
                this->Dp *= 0.9;
            }
            if (this->iK == 1) {
                this->Di *= 0.9;
            }
            if (this->iK == 2) {
                this->Dd *= 0.9;
            }
        }
        this->state = UP;
        this->iK = (this->iK+1)%3;
      }
  };

  double Twiddler::GetKp() {
      return this->Kp;
  };

  double Twiddler::GetKi() {
      return this->Ki;
  };

  double Twiddler::GetKd() {
      return this->Kd;
  };
