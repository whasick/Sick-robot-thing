
#include <SoftwareSerial.h>
const unsigned int ci_Light_Sensor = 7;
char irvalue[4]={'A','E','I','O'};
bool ir_true=true;
char data=NULL;

SoftwareSerial mySerial(7, 7); // RX, TX
const int ci_switch=2;//pin 10 for switch (use switch 3)
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("MSE 2202 IR tester");
  
digitalWrite(13,LOW);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
  //mySerial.println("Hello, world?");
}

 

void loop() { // run over and over
if(digitalRead(ci_switch)==HIGH)//AE
{
  ir_true=1;
  Serial.println("Looking for AE");
}
if(digitalRead(ci_switch)==LOW)//IO
{
  ir_true=0;
  Serial.println("Looking for IO");
}
  if (mySerial.available())
  {
    //Serial.write(mySerial.read());
    data =mySerial.read();
    Serial.println(data);
  int val1;
  int val2; 
  if(ir_true)//looking for AE
  {
    val1=0;
    val2=1;
  }
  else
  {
    val1=2;
    val2=3;
  }
  if(data==irvalue[val1] || data==irvalue[val2])
  {
    digitalWrite(13,HIGH);
    Serial.print("FOUND");
  }
  if(!(data==irvalue[val1] || data==irvalue[val2]))
  {
    Serial.print("NOT FOUND");
    Serial.write(0);
  }

  }
}
