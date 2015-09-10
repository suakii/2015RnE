
class PIDController {
  
  float kp;
  float ki;
  float kd;
  
  float integral;
  float preError;
  float maxControlValue;
  boolean reCalculate;
  
  PIDController(float kp, float ki, float kd, float maxControlValue) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    preError = 0.0;
    integral = 0.0;
    this.maxControlValue = maxControlValue;
    reCalculate = false;
  }
  
  float estimate(float currentValue, float setValue) {
    float error = setValue - currentValue;
    
    if (reCalculate == true) {
      preError = error;
      integral = 0.0;
      reCalculate = false;
    }
    
    float derivative = error - preError;
    integral = integral + error;
    float output = kp*error + ki*integral + kd*derivative;
    preError = error;
    
    if (output > maxControlValue) {
      output = maxControlValue;
      //integral = 0;
    }
    else if (output < -maxControlValue) {
      output = -maxControlValue;
      //integral = 0;
    }
      
    return output * 100;
  }

  void reSet() {
    reCalculate = true;
  }
}
