// PID Line Follower with Ultrasonic Obstacle Detection 
//  For L298N + TCRT5000 Sensor Array (5 Sensors Analog Sensor Just remove ADC chip),(Auto-Calibration) 

#define enA 6
#define in1 2
#define in2 3

#define enB 5
#define in3 4
#define in4 7

// ====== Ultrasonic Sensor Pins ======

#define trigPin 8
#define echoPin 9

// ====== IR Sensor Pins (Analog) ======
#define ir1 A1
#define ir2 A2
#define ir3 A3
#define ir4 A4
#define ir5 A5

// ====== PID Constants ======

float Kp = 120.0;   // Adjust the Kp vlue according to your robot . Used for sharp Turn.
float Ki = 0.00;
float Kd = 12.0;    // used to smooth the turn and to minimize the oscillation

// ====== Motor Speed Settings ======

int baseSpeed = 200;                    
int leftMotorSpeed, rightMotorSpeed;

// ====== PID Variables ======
int lastError = 0;
int errorVal = 0;
long Pterm, Iterm, Dterm;

// ====== Ultrasonic Variables ======
long duration;
int distance;

// ====== Auto-Calibration Variables ======
int irPins[5] = {ir1, ir2, ir3, ir4, ir5};
int irMin[5] = {1023, 1023, 1023, 1023, 1023};
int irMax[5] = {0, 0, 0, 0, 0};
int irThreshold[5];

// ====== Function Prototypes ======
void stopMotors();
void moveMotors(int leftSpeed, int rightSpeed);
int getDistance();
void autoCalibrate();  // UPDATED

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // IR sensors
  for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);

  // Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  stopMotors(); // safe start
  Serial.println("System Ready...");
  delay(1000);

  // ====== Auto Calibration (Robot rotates itself) ======
  autoCalibrate();
}

void loop() {

  // Measure obstacle distance
  distance = getDistance();
  if (distance > 0 && distance < 15) {
    stopMotors();
    Serial.println("Obstacle detected!");
    delay(200);
    return;
  }

  // ====== Read Calibrated IR Sensors ======
  int raw[5], d[5];
  for (int i = 0; i < 5; i++) {
    raw[i] = analogRead(irPins[i]);
    d[i] = (raw[i] < irThreshold[i]) ? 1 : 0; // 1 = black line
  }

  // ====== PID Error Calculation ======
  int position = (-4*d[0]) + (-2*d[1]) + (0*d[2]) + (2*d[3]) + (4*d[4]);
  errorVal = position;

  // ====== PID Calculation ======
  Pterm = errorVal;
  Iterm += errorVal;
  Dterm = errorVal - lastError;
  lastError = errorVal;

  long correction = (Kp*Pterm) + (Ki*Iterm) + (Kd*Dterm);

  leftMotorSpeed  = baseSpeed - correction;
  rightMotorSpeed = baseSpeed + correction;

  leftMotorSpeed  = constrain(leftMotorSpeed, -150, 200);
  rightMotorSpeed = constrain(rightMotorSpeed, -150, 200);

  moveMotors(leftMotorSpeed, rightMotorSpeed);
  delay(10);
}

// ====== Ultrasonic Distance Function ======
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 20000UL);
  if (duration == 0) return -1;
  int dist = (int)(duration * 0.034 / 2);
  return dist;
}

// ====== Move Motors ======
void moveMotors(int leftSpeed, int rightSpeed) {

  // LEFT MOTOR
  if (leftSpeed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, leftSpeed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -leftSpeed);
  }

  // RIGHT MOTOR
  if (rightSpeed >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, rightSpeed);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -rightSpeed);
  }
}

// ====== Stop Motors ======
void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// ====== AUTO-ROTATION CALIBRATION (TCRT5000) ======
void autoCalibrate() {

  Serial.println("Auto Calibration Starting...");
  Serial.println("Robot rotating in place...");

  unsigned long start = millis();
  unsigned long duration = 3500;

  // reset min/max
  for (int i = 0; i < 5; i++) {
    irMin[i] = 1023;
    irMax[i] = 0;
  }

  // ---- Robot rotates to scan black & white ----
  while (millis() - start < duration) {

    moveMotors(160, -160); // rotate left wheel forward, right backward

    for (int s = 0; s < 5; s++) {
      int val = analogRead(irPins[s]);

      if (val < irMin[s]) irMin[s] = val;
      if (val > irMax[s]) irMax[s] = val;
    }

    delay(5);
  }

  stopMotors();
  delay(300);

  Serial.println("Calibration Completed.\nThresholds:");

  for (int i = 0; i < 5; i++) {
    irThreshold[i] = (irMin[i] + irMax[i]) / 2;

    Serial.print("S");
    Serial.print(i+1);
    Serial.print(" Min:");
    Serial.print(irMin[i]);
    Serial.print(" Max:");
    Serial.print(irMax[i]);
    Serial.print(" Th:");
    Serial.println(irThreshold[i]);
  }

  delay(800);
}

