#include <Servo.h>

Servo ser_0;
Servo ser_1;
Servo ser_2;
Servo ser_3;
Servo ser_4;

void setup() {
 
  ser_0.attach(5);
  ser_1.attach(6); // angle 90 real = 45
  ser_2.attach(9); // 120 parallel to ser_1
  ser_3.attach(10);
  ser_4.attach(11);
  Serial.begin(9600);

}

void loop() {
  unsigned char servo_num;
  unsigned char angle;
  if (Serial.available() > 0) {
    servo_num = Serial.read();
    Serial.print(servo_num);
    Serial.print(" ");
    delay(10);
    if (Serial.available() > 0) {
      angle = Serial.read();
      Serial.println(angle);
    }
  }
  servo(servo_num, angle);
}

void servo(unsigned char servo_num, unsigned char angle){
  switch(servo_num){
    case 0:
        ser_0.write(angle);
        break;
    case 1:
        ser_1.write(angle);
        break;
    case 2:
        ser_2.write(angle);
        break;
    case 3:
        ser_3.write(angle);
        break;
    case 4:
        ser_4.write(angle);
        break;
    default:
        break;
  }
}
