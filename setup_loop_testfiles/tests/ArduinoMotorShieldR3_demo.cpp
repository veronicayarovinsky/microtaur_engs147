
#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

ArduinoMotorShieldR3 md;

void setup()
{
  SerialUSB.begin(115200);
  SerialUSB.println("Arduino Motor Shield R3");
  md.init();
}

void loop()
{
  SerialUSB.println("M1 Speed 100% Forward");
  md.setM1Speed(400);
  SerialUSB.println("M2 Speed 100% Forward");
  md.setM2Speed(400);
  SerialUSB.print("M1 current: ");
  SerialUSB.println(md.getM1CurrentMilliamps());
  SerialUSB.print("M2 current: ");
  SerialUSB.println(md.getM2CurrentMilliamps());
  delay(2000);

  SerialUSB.println("M1 Speed 100% Backward");
  md.setM1Speed(-400);
  SerialUSB.println("M2 Speed 100% Backward");
  md.setM2Speed(-400);
  SerialUSB.print("M1 current: ");
  SerialUSB.println(md.getM1CurrentMilliamps());
  SerialUSB.print("M2 current: ");
  SerialUSB.println(md.getM2CurrentMilliamps());
  delay(2000);
  
  SerialUSB.println("M1 Speed 50% Forward");
  md.setM1Speed(200);
  SerialUSB.println("M2 Speed 50% Forward");
  md.setM2Speed(200);
  SerialUSB.print("M1 current: ");
  SerialUSB.println(md.getM1CurrentMilliamps());
  SerialUSB.print("M2 current: ");
  SerialUSB.println(md.getM2CurrentMilliamps());
  delay(2000);

  SerialUSB.println("M1 Speed 50% Backward");
  md.setM1Speed(-200);
  SerialUSB.println("M2 Speed 50% Backward");
  md.setM2Speed(-200);
  SerialUSB.print("M1 current: ");
  SerialUSB.println(md.getM1CurrentMilliamps());
  SerialUSB.print("M2 current: ");
  SerialUSB.println(md.getM2CurrentMilliamps());
  delay(2000);
  
  SerialUSB.println("M1 Speed 0%");
  md.setM1Speed(0);
  SerialUSB.println("M2 Speed 0%");
  md.setM2Speed(0);
  SerialUSB.print("M1 current: ");
  SerialUSB.println(md.getM1CurrentMilliamps());
  SerialUSB.print("M2 current: ");
  SerialUSB.println(md.getM2CurrentMilliamps());
  delay(2000);
}
