#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define RightSensorPin 36 //A1
#define LeftSensorPin 32  //A0
#define FrontSensorPin 34  //A2
#define pumpPin 5
#define buzzerPin 6
#define in1 9
#define in2 10
#define in3 11
#define in4 12
#define ENA 8      // enable pin for the right motor
#define ENB 13     // enable pin for the left motor
#define servoPin 44

int pos = 90;  // Initialize servo position to 90 degrees
int servoStep = 1;  // Step size for servo movement
int flag = 0;

void setup() {
  pinMode(in4, OUTPUT);  // Left motors forward
  pinMode(in3, OUTPUT);  // Left motors reverse
  pinMode(in2, OUTPUT);  // Right motors forward
  pinMode(in1, OUTPUT);  // Right motors reverse
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  myservo.attach(servoPin);
  myservo.write(pos);  // Set initial position to 90 degrees

  pinMode(RightSensorPin, INPUT);
  pinMode(LeftSensorPin, INPUT);
  pinMode(FrontSensorPin, INPUT);

  Serial.begin(9600);
   lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Fire Detection");
}
void buzzerOn() {
  digitalWrite(buzzerPin, HIGH); 
}

void buzzerOff() {
  digitalWrite(buzzerPin, LOW); 
}

void lcdPrint(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}

void moveForward() {
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveBackward() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  analogWrite(ENA, 250);
  analogWrite(ENB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveRight() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 250);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopp() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void sweepServoRight() {
  for (int angle = pos; angle >= 20; angle -= servoStep) {
    myservo.write(angle);
    delay(15); // Adjust delay as needed for the servo to reach the desired position
  }
  for (int angle = 20; angle <= pos; angle += servoStep) {
    myservo.write(angle);
    delay(15);
  }
}

void sweepServoLeft() {
  for (int angle = pos; angle <= 160; angle += servoStep) {
    myservo.write(angle);
    delay(15);
  }
  for (int angle = 160; angle >= pos; angle -= servoStep) {
    myservo.write(angle);
    delay(15);
  }
}

void sweepServoRightToLeft() {
  for (int angle = 20; angle <= 160; angle += servoStep) {
    myservo.write(angle);
    delay(15);
  }
  for (int angle = 160; angle >= 20; angle -= servoStep) {
    myservo.write(angle);
    delay(15);
  }
}

void activatePump() {
  digitalWrite(pumpPin, HIGH);
  delay(500);
  digitalWrite(pumpPin, LOW);
}

void turnRightUntilFrontSensor() {
  while (digitalRead(FrontSensorPin) == 1) {
    moveRight(); // Keep turning right until the front sensor becomes 0
  }
  stopp();
}

void turnLeftUntilFrontSensor() {
  while (digitalRead(FrontSensorPin) == 1) {
    turnLeft(); // Keep turningleft until the front sensor becomes 0
  }
  stopp();
}

void moveForwardUntilRightSensor() {
  while (digitalRead(RightSensorPin) == 1) {
    moveForward(); // Keep moving forward until the right sensor becomes 0
  }
  stopp();
}

void AutocheckFire() {
  Serial.println("in AAAutoo");
  int rightSensor = digitalRead(RightSensorPin);
  int leftSensor = digitalRead(LeftSensorPin);
  int frontSensor = digitalRead(FrontSensorPin);

  if (rightSensor == 0 && frontSensor == 1) {
    turnRightUntilFrontSensor();
    lcdPrint("There is fire in right side !! ");
    buzzerOn();
    activatePump();
    sweepServoRight();
    buzzerOff();
   lcdPrint("We are in safe Now <3 !!");
  } else if (leftSensor == 0 && frontSensor == 1) {
    turnLeftUntilFrontSensor();
    lcdPrint("There is fire in left side !!");
    buzzerOn();
    activatePump();
    sweepServoLeft();
    buzzerOff();
    lcdPrint("We are in Safe Now <3 !!");
  } else if (rightSensor == 0 && leftSensor == 0 && frontSensor == 0) {
    stopp();
    lcdPrint("HEEELP HELPPP SOS ): !!");
    buzzerOn();
    activatePump();
    sweepServoRightToLeft();
    buzzerOff();
  } else if (frontSensor == 0) {
    moveForwardUntilRightSensor();
    lcdPrint("There is fire in Front !! ");
    buzzerOn();
    activatePump();
    sweepServoRightToLeft();
    buzzerOff();
    lcdPrint("We are in safe Now <3 !!");
  } else if (Serial.available() > 0 && Serial.read() == 'M') {
    flag = 0;
    lcdPrint("Manual mode !!");
  } else {
    digitalWrite(pumpPin, LOW); // Turn off the pump
    myservo.write(pos); // Set servo to 90 degrees
  }
}

void loop() {
  if (Serial.available()) {
    if (flag == 0) {
      switch (Serial.read()) {
        case 'F':
        lcdPrint("Move forward");
          moveForward();
          break;
        case 'B':
        lcdPrint("Move Back");
          moveBackward();
          break;
        case 'L':
        lcdPrint("Move left");
          turnLeft();
          break;
        case 'R':
        lcdPrint("Move right ");
          moveRight();
          break;
        case 'S':
        lcdPrint("Stop");
          stopp();
          break;
        case 'r':
        lcdPrint("Move servo Right");
          sweepServoRight();
          break;
        case 'l':
        lcdPrint("Move servo left");
          sweepServoLeft();
          break;
        case 'P':
        lcdPrint("Fire Fire Activate The PUMP Now...!!");
          buzzerOn();
          activatePump();
          buzzerOff();
          break;
        case 'A':
        lcdPrint("Automatic Mode!!");
          flag = 1;
          break;
        default:
          break;
      }
    }
  }
  if (flag == 1) {
    AutocheckFire();
  }
}
