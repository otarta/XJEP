#include <Servo.h>

int pinUT = 2;
int pinUE = 3;
int pinLT = 4;
int pinLE = 5;

Servo UT;
Servo UE;
Servo LT;
Servo LE;



void setup() {
connect();
Serial.begin(9600);
}

void loop() {

withMap(1023);

}



void connect(){
  UT.attach(pinUT);
  UE.attach(pinUE);
  LT.attach(pinLT);
  LE.attach(pinLE);
}

void withMap(int val){
  int angle = map(val, 0, 1023, 0, 60);
  LE.write(angle);
}






