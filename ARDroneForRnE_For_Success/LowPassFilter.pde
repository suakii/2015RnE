
class LowPassFilter {

  float alpha;
  float previous;
  boolean reCalculate;


  LowPassFilter(float a) {
    alpha = a;
    previous = 0.0;
    reCalculate=false;
  }


  float estimate(float input) {
    float value;

      if (reCalculate==true) {
        previous=input;
        reCalculate=false;
      }

    value= alpha * previous + (1.0 - alpha) * input;
    previous = value;
    return value;
  }

 
  void reSet() {
    reCalculate=true;
  }
}

