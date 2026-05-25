#include "AxisEncoderShield3.h"
#include "ArduinoMotorShieldR3.h"
#include "math.h"
ArduinoMotorShieldR3 md;

#define RECORD_TIME 10000000 //10 seconds
#define ONELOOP 10000 //will equate to 10 milliseconds
#define N 1000 //array length
/*#define ONELOOP 1000
#define N 100*/ //for 0.1Sec Ts
#define TWO_PI 6.283831 
#define COUNTS (12*100.37) //number of counts
#define REF 20

// for PWM functiob:
#define A 2.507
#define B 0.384
#define C -68.2
#define D 0.9572
#define E -0.459
#define F 84.27

//for Gc
#define V1 1
#define E 0.68337
#define E1 0.5167

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


void setup() {
  Serial.begin(74880);
  initEncoderShield(); //initializing Encoder Shiels
  md.init(); //initializing Motor Shield
  delay(100);
}

void loop() {
  long encoder1Value; //x

  // arrays for storing motor transient behaviour
  float timevals[N];
  float encoderposition[N];
  float speed[N];
  int pulsewidthvals[N];
  float voltagevals[N];

  unsigned long curtime, progstart, prevloopstart;
  bool running = false;
  int i=0;
  float current_speed=0.0;
  float error[N];
  int pulsewidth;
  float volt;

  //starting motor
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(2000);

  // first value for speed assumed zero since there is no dt
  speed[0]=0;

  curtime=micros(); // current time in microseconds
  progstart=curtime;
  prevloopstart=curtime;

  while(curtime <= progstart+RECORD_TIME && (i < N)){ //10 second while loop

    curtime=micros();
    if(!running || (curtime - prevloopstart) >= ONELOOP){ // 10 (oneloop) microseconds if loop
      running = true;
      prevloopstart = curtime; 

      timevals[i]=(float)((curtime-progstart))/1000000.0; //from when loop started to now

      encoder1Value=getEncoderValue(1);
      encoderposition[i]=((float)(encoder1Value)/COUNTS)*TWO_PI; //position of shaft in radians 

      if (i>0){ // speed calc
        speed[i]=(encoderposition[i]-encoderposition[i-1])/((timevals[i]-timevals[i-1]))*-1.0;//multiply by -1 to match volt directionality to speed values
      }
      current_speed=speed[i]; // feedback sensing
      
      error[i]=REF-current_speed;//summation junction

      if (i==0){
        voltagevals[i]=E*error[i]; // similar to E*error[i], will decide which version later
      } else {
        voltagevals[i]=(V1*voltagevals[i-1])+(E*error[i])-(E1*error[i-1]); //controller box
      }
      if (voltagevals[i]>12.9){
        voltagevals[i]=12.9;
      } else if (voltagevals[i]<-12.9){
        voltagevals[i]=-12.9;
      }
      
      pulsewidth=pwmproducer(voltagevals[i]);
      md.setM1Speed(-pulsewidth);
      md.setM2Speed(pulsewidth);

      pulsewidthvals[i]=pulsewidth;
      ++i; //next index
      }
    
  }

  /*for (i=0; i<N; ++i) {
    Serial.print(timevals[i],4); 
    Serial.print(',');    
    Serial.print(speed[i],4);
    Serial.print(','); 
    Serial.print(voltagevals[i],4); 
    Serial.print(',');    
    Serial.print(pulsewidthvals[i]);
    Serial.println();
  }  */                   

}