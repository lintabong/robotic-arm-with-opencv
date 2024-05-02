#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int servo0pin = 11;
const int servo1pin = 10;
const int servo2pin = 9;
const int servo3pin = 6;
const int servo4pin = 5;
 
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int servo0angle = 90;
int servo1angle = 90;
int servo2angle = 168;
int servo3angle = 150;
int servo4angle = 90;

int servo0angleOFF = 90;
int servo1angleOFF = 90;
int servo2angleOFF = 168;
int servo3angleOFF = 150;
int servo4angleOFF = 90;

String data;

void setup() {
  Serial.begin(115200);

  lcd.begin();
  lcd.backlight();
  lcd.clear();

  pinMode(servo0pin, OUTPUT);
  pinMode(servo1pin, OUTPUT);
  pinMode(servo2pin, OUTPUT);
  pinMode(servo3pin, OUTPUT);
  pinMode(servo4pin, OUTPUT);
  
  servo0.attach(servo0pin);
  servo1.attach(servo1pin);
  servo2.attach(servo2pin);
  servo3.attach(servo3pin);
  servo4.attach(servo4pin);

  servo0.write(servo0angleOFF);
  servo1.write(servo1angleOFF);
  servo2.write(servo2angleOFF);
  servo3.write(servo3angleOFF);
  servo4.write(servo4angleOFF);
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print(millis());

  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');

    servo0angle = getValue(data, '.', 0).toInt();
    servo1angle = getValue(data, '.', 1).toInt();
    servo2angle = getValue(data, '.', 2).toInt();
    servo3angle = getValue(data, '.', 3).toInt();
    servo4angle = getValue(data, '.', 4).toInt();

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(data);

    if (data){
      data = String(servo0angle) + " " + String(servo1angle)+ " " + String(servo2angle) + 
            " " + String(servo3angle) + " " + String(servo4angle);

      servo0.write(servo0angle);
      servo1.write(servo1angle);
      servo2.write(servo2angle);
      servo3.write(servo3angle);
      servo4.write(servo4angle);      
    }
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
