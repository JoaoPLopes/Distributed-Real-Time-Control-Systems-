#ifndef PiC_H
#define PiC_H

class PiC {
  float kp, ki, T;
  float k1, k2;
  float ep, yp, ip;
  public:
    PiC(float kp, float ki, float T);
    float calc(float ref, float y);
    void updat(float ref, float y, float ff);
    void reset();
};

#endif
