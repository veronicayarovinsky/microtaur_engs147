#include "math.h"
// for PWM function:
#define A 2.507
#define B 0.384
#define C -68.2
#define D 0.9572
#define E -0.459
#define F 84.27

//function for PWM mapping
int pwmproducer (float controleffort){
  int pulsewidth;
  float volt=controleffort;
  if (volt<0.001) {
    pulsewidth=(-A*(exp((-B)*volt)))+C;
    if (pulsewidth<-400){//cutoffs
      pulsewidth=-400;
    }
  } else if (volt>0.001){
    pulsewidth=(D*(exp((-E)*volt)))+F;
    if (pulsewidth>400){
      pulsewidth=400;
    }
  } else{
    pulsewidth=0;
  }
    return pulsewidth;
}
