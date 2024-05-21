#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);


// config servo
const int servo0pin = 11;
const int servo1pin = 10;
const int servo2pin = 9;

// config pump
const int pumpPin = 8;

// config motor
const int EN1 = 3;
const int motor1a = 2;
const int motor1b = 4;

const int EN2 = 5;
const int motor2a = 6;
const int motor2b = 7;

const int EN3 = 8;
const int motor3a = 9;
const int motor3b = 10;

const int EN4 = 11;
const int motor4a = 12;
const int motor4b = 13;

int motor1direction = 0;
int motor2direction = 0;
int motor3direction = 0;
int motor4direction = 0;
int motor1speed = 0;
int motor2speed = 0;
int motor3speed = 0;
int motor4speed = 0;

// config hc-sr04, diganti dengan pin sensor jarak
const int triggerPin = 9;
const int echoPin = 10;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int servo0angle = 90;
int servo1angle = 90;
int servo2angle = 168;

int servo0angleOFF = 90;
int servo1angleOFF = 90;
int servo2angleOFF = 168;

String direction = "";
int speed = 0;

int pick = 0;
int pumpValue = 0;

String data;

long duration;
int distance;

void setup() {
  Serial.begin(115200);

  lcd.begin();
  lcd.backlight();
  lcd.clear();

  pinMode(servo0pin, OUTPUT);
  pinMode(servo1pin, OUTPUT);
  pinMode(servo2pin, OUTPUT);

  pinMode(EN1, OUTPUT);
  pinMode(motor1a, OUTPUT);
  pinMode(motor1b, OUTPUT);

  pinMode(EN2, OUTPUT);
  pinMode(motor2a, OUTPUT);
  pinMode(motor2b, OUTPUT);

  pinMode(EN3, OUTPUT);
  pinMode(motor3a, OUTPUT);
  pinMode(motor3b, OUTPUT);

  pinMode(EN4, OUTPUT);
  pinMode(motor4a, OUTPUT);
  pinMode(motor4b, OUTPUT);

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  servo0.attach(servo0pin);
  servo1.attach(servo1pin);
  servo2.attach(servo2pin);

  servo0.write(servo0angleOFF);
  servo1.write(servo1angleOFF);
  servo2.write(servo2angleOFF);

  pinMode(pumpPin, OUTPUT);
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print(millis());

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.println(distance);

  lcd.setCursor(0, 3);
  lcd.print("dis: ");
  lcd.setCursor(5, 3);
  lcd.print(distance);

  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');

    servo0angle = getValue(data, '.', 0).toInt();
    servo1angle = getValue(data, '.', 1).toInt();
    servo2angle = getValue(data, '.', 2).toInt();
    motor1direction = getValue(data, '.', 3).toInt();
    motor1speed = getValue(data, '.', 4).toInt();
    motor2direction = getValue(data, '.', 5).toInt();
    motor2speed = getValue(data, '.', 6).toInt();
    motor3direction = getValue(data, '.', 7).toInt();
    motor3speed = getValue(data, '.', 8).toInt();
    motor4direction = getValue(data, '.', 9).toInt();
    motor4speed = getValue(data, '.', 10).toInt();
    pick = getValue(data, '.', 11).toInt();

    data = String(servo0angle) + " " + String(servo1angle)+ " " + String(servo2angle);

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(data);

    data = direction + " " + String(speed);
    lcd.setCursor(0, 2);
    lcd.print(data);

    lcd.setCursor(0, 3);
    lcd.print("dis: ");
    lcd.setCursor(5, 3);
    lcd.print(distance);

    analogWrite(EN1, motor1speed);
    if (motor1direction == 1){
      digitalWrite(motor1a, LOW);
      digitalWrite(motor1b, HIGH);
    } else {
      digitalWrite(motor1a, HIGH);
      digitalWrite(motor1b, LOW);
    }

    analogWrite(EN2, motor2speed);
    if (motor2direction == 1){
      digitalWrite(motor2a, LOW);
      digitalWrite(motor2b, HIGH);
    } else {
      digitalWrite(motor2a, HIGH);
      digitalWrite(motor2b, LOW);
    }

    analogWrite(EN3, motor3speed);
    if (motor3direction == 1){
      digitalWrite(motor3a, LOW);
      digitalWrite(motor3b, HIGH);
    } else {
      digitalWrite(motor3a, HIGH);
      digitalWrite(motor3b, LOW);
    }

    analogWrite(EN4, motor4speed);
    if (motor4direction == 1){
      digitalWrite(motor4a, LOW);
      digitalWrite(motor4b, HIGH);
    } else {
      digitalWrite(motor4a, HIGH);
      digitalWrite(motor4b, LOW);
    }

    servo0.write(servo0angle);
    servo1.write(servo1angle);
    servo2.write(servo2angle); 
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
