// Robogaia.com
// Refactored by M. Kokko
// Modified 27-Mar-2027

# include <Arduino.h>
# include <AxisEncoderShield3.h>


//*****************************************************
void setup() 
//*****************************************************
{
  SerialUSB.begin(115200);
  initEncoderShield();
}

//*****************************************************
void loop() 
//*****************************************************
{
        long encoder1Value;
        long encoder2Value;
        long encoder3Value; 
        
        encoder1Value = getEncoderValue(1);  
        SerialUSB.print("Encoder X= ");
        SerialUSB.print(encoder1Value);
        
        encoder2Value = getEncoderValue(2);  
        SerialUSB.print(" Encoder Y= ");
        SerialUSB.print(encoder2Value);
        
        encoder3Value = getEncoderValue(3);  
        SerialUSB.print(" Encoder Z= ");
        SerialUSB.print(encoder3Value);
 
        SerialUSB.print("\r\n");

     delay(100); 
 
}//end loop

