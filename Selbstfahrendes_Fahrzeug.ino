#include <IRremote.h>

const int IR_RECEIVER_PIN = A5;
IRrecv irReceiver(IR_RECEIVER_PIN);
decode_results irResults;

#define PIN_MOTOR_ENABLE_A 10
#define PIN_MOTOR_ENABLE_B 5
#define PIN_MOTOR_IN1 9 
#define PIN_MOTOR_IN2 8
#define PIN_MOTOR_IN3 7
#define PIN_MOTOR_IN4 6
#define PIN_SERVO_CONTROL A4
#define PIN_IR_SENSOR_RIGHT A0
#define PIN_IR_SENSOR_LEFT A1
#define PIN_ULTRASONIC_ECHO A2
#define PIN_ULTRASONIC_TRIGGER A3
#define PIN_HEADLIGHT 11
#define PIN_BRAKE_LIGHT 12
#define PIN_HORN 4

enum operationMode { MANUAL_CONTROL = 0, LINE_FOLLOWING = 1, OBSTACLE_AVOIDANCE = 2 };
operationMode currentMode = MANUAL_CONTROL;

int i_leftDistance, i_rightDistance;
int i_frontDistance = 30;
long l_ultrasonicDistance;

int i_obstacleThreshold = 20;
int i_remoteData;
int i_infraredData;
int i_motorSpeed = 100;

//.............................................................................................


void setup() {
  pinMode(PIN_IR_SENSOR_RIGHT, INPUT);
  pinMode(PIN_IR_SENSOR_LEFT, INPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_ULTRASONIC_TRIGGER, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE_A, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE_B, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);
  pinMode(PIN_SERVO_CONTROL, OUTPUT);
  pinMode(PIN_HEADLIGHT, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  pinMode(PIN_HORN, OUTPUT);

  irReceiver.enableIRIn();
  irReceiver.blink13(true);
  Serial.begin(9600);
  
  calibrateServo();

  digitalWrite(PIN_HORN,HIGH);
  delay(1000);
  digitalWrite(PIN_HORN,LOW);
}

//.............................................................................................

void loop(){  

  digitalWrite(PIN_BRAKE_LIGHT, LOW);

  if (irReceiver.decode(&irResults)) {
    Serial.println(irResults.value, HEX);
    i_remoteData = getIRRemoteData();
    irReceiver.resume();
    delay(100);
    processRemoteData(i_remoteData);
  }

  analogWrite(PIN_MOTOR_ENABLE_A, i_motorSpeed);
  analogWrite(PIN_MOTOR_ENABLE_B, i_motorSpeed);

  switch(currentMode) {
    case MANUAL_CONTROL:  // Modus 0: Tastensteuerbefehl 
      manageUserInputs();
      break;
    case LINE_FOLLOWING:  // Modus 1: Linienverfolgungssteuerung 
      trackLine();
      break;
    case OBSTACLE_AVOIDANCE:  // Modus 2: Hindernisvermeidung 
      avoidObstacles();
      break;
  }

  delay(10);
}

//Funktionen..............................................................................................

void processRemoteData(int data) {
  switch(data) {
    case 8:  // (Taste 1) -> Manuelle Fernbedienungssteuerung
      currentMode = MANUAL_CONTROL;
      stop();
      break;
    case 9:  // (Taste 2) -> Automatischer Linienverfolgungsbefehl
      currentMode = LINE_FOLLOWING; 
      i_motorSpeed = 100;
      break;
    case 10:  // (Taste 3) -> Automatischer Hindernisvermeidungsbefehl
      currentMode = OBSTACLE_AVOIDANCE;
      i_motorSpeed = 120;
      break;
    case 11:
      digitalWrite(PIN_HEADLIGHT,HIGH);
      break;
    case 12:
      digitalWrite(PIN_HEADLIGHT,LOW);
      break;
    case 13:
      digitalWrite(PIN_HORN,HIGH);
      delay(1000);
      digitalWrite(PIN_HORN,LOW);
      break;
    default:
      break;
  }
}

void manageUserInputs() {
  if (i_remoteData >= 1 && i_remoteData <= 5) {
    switch (i_remoteData) {
      case 1: moveForward(); break;
      case 2: moveBackward(); break;
      case 3: turnLeft(); break;
      case 4: turnRight(); break;
      case 5: stop(); break;
    }
  }
}

void trackLine() {
  int rightSensor = digitalRead(PIN_IR_SENSOR_RIGHT);
  int leftSensor = digitalRead(PIN_IR_SENSOR_LEFT);

  if (rightSensor == LOW && leftSensor == LOW) {
    moveForward();
  } else if (rightSensor == HIGH && leftSensor == LOW) {
    turnRight();
  } else if (rightSensor == LOW && leftSensor == HIGH) {
    turnLeft();
  } else if (rightSensor == HIGH && leftSensor == HIGH) {
    stop();
  }
}

void avoidObstacles() {
  i_frontDistance = ultrasonicRead();

  if (i_frontDistance > i_obstacleThreshold) {
    moveForward();
  } else {
    checkSides();
  }
}

long getIRRemoteData() {
  switch (irResults.value) {
    case 0xFF18E7: i_infraredData = 1; break; //rauf
    case 0xFF4AB5: i_infraredData = 2; break; //runter
    case 0xFF10EF: i_infraredData = 3; break; //links
    case 0xFF5AA5: i_infraredData = 4; break; //rechts
    case 0xFF38C7: i_infraredData = 5; break; //stop();
    case 0xFFA25D: i_infraredData = 8; break; //Taste 1
    case 0xFF629D: i_infraredData = 9; break; //Taste 2
    case 0xFFE21D: i_infraredData = 10; break; //Taste 3
    case 0xFF6897: i_infraredData = 11; break; //Taste *
    case 0xFFB04F: i_infraredData = 12; break; //Taste #
    case 0xFF9867: i_infraredData = 13; break; //Taste 0
    default: break;
  }
  return i_infraredData;
}

long ultrasonicRead() {
  digitalWrite(PIN_ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  l_ultrasonicDistance = pulseIn(PIN_ULTRASONIC_ECHO, HIGH);
  return l_ultrasonicDistance / 29 / 2;
}

void checkSides() {
  stop();
  delay(100);

  i_leftDistance = scanDistance(70, 140, 5);
  i_rightDistance = scanDistance(140, 0, -5);

  resetServo();
  compareDistances();
}

long scanDistance(int start, int end, int step) {
  for (int angle = start; angle != end; angle += step) {
    servoPulse(PIN_SERVO_CONTROL, angle);
  }
  return ultrasonicRead();
}

void resetServo() {
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(PIN_SERVO_CONTROL, angle);
  }
  delay(300);
}

void compareDistances() {
  if (i_leftDistance > i_rightDistance) {
    turnLeft();
    delay(350);
  } else if (i_rightDistance > i_leftDistance) {
    turnRight();
    delay(350);
  } else {
    moveBackward();
    delay(300);
    turnRight();
    delay(600);
  }
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);
}

void calibrateServo() {
  calibrateServoSegment(70, 140, 5);
  calibrateServoSegment(140, 0, -5);
  calibrateServoSegment(0, 70, 5);
  delay(500);
}

void calibrateServoSegment(int start, int end, int step) {
  for (int angle = start; angle != end; angle += step) {
    servoPulse(PIN_SERVO_CONTROL, angle);
  }
  servoPulse(PIN_SERVO_CONTROL, end);
}

void moveForward() {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
}
void moveBackward() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_IN3, HIGH);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}
void turnRight() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
}
void turnLeft() {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW); 
  digitalWrite(PIN_MOTOR_IN3, HIGH); 
  digitalWrite(PIN_MOTOR_IN4, LOW);
}
void stop() {
  digitalWrite(PIN_BRAKE_LIGHT, HIGH);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}
