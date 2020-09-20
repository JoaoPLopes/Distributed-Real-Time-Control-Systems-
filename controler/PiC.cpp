#include "PiC.h"
#include <math.h>
#include <arduino.h>

using namespace std;

PiC::PiC(float p, float i, float _T){
  ep=yp=ip=0.0f;
  k1=kp=p; ki=i; T=_T;
  k2=kp*ki*T/2000;
}

void PiC::reset(){
  ep=yp=ip=0.0f;
}

float PiC::calc(float ref, float y){
  float e=ref-y;
  if (abs(e) < 1) {
    e = 0;
  }
  float p=k1*e;
  float i=ip+k2*(e+ep);
  return p+i;
}

void PiC::updat(float ref, float y, float ff){
  float e=ref-y;
  if (abs(e) < 1) {
    e = 0;
  }
  float p=k1*e;
  float i=ip+k2*(e+ep);
  
  // Satura termo integral - Anti-windup
  if ((p+i+ff)>255){
    i=255-ff-p;
  }else if((p+i+ff)<0){
    i=0-ff-p;
  }
  yp=y;ip=i;ep=e;
}
