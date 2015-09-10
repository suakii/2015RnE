
class PIDController {
  
  float kp;
  float ki;
  float kd;
  
  float integral;
  float preError;
  float dt;
  float maxControlValue;
  boolean reCalculate;
  
  PIDController(float kp, float ki, float kd, float maxControlValue) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.dt = 0.01;
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
    
    float derivative = (error - preError)/dt;
    integral = integral + error*dt;
    float output = kp*error + ki*integral + kd*derivative;
    //float output = kp*error + ki*integral + kd*derivative;
    //float output = kp*error + ki*integral + kd*derivative;
    //float output = kp*error;// + ki*integral + kd*derivative;

    preError = error;
    
    if (output > maxControlValue) {
      output = maxControlValue;
      
    }
    
    else if (output < -maxControlValue) {
      output = -maxControlValue;
    }

    
    return output;
  }

  void reSet() {
    reCalculate = true;
  }
}