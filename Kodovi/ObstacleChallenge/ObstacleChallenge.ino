#include <ESP32Servo.h>

// --- PINOVI ---
#define TRIG 4
#define ECHO_FL 35
#define ECHO_FM 33
#define ECHO_FR 32
#define SERVO_PIN 22

#define IN1 25
#define IN2 26
#define ENA 23

// --- KONSTANTE ---
#define SERVO_CENTER 96
#define SERVO_LEFT 65
#define SERVO_RIGHT 125

Servo steering;

// --- PARAMETRI ---
float Kp = 0.45;      
float Kd = 0.02; 

float lastControl = 0;
float lastError = 0;

float lastFL = 20;
float lastFR = 20;
float lastSteer = SERVO_CENTER;

unsigned long startTime;

// --- FUNKCIJA ZA SENZOR ---
long readDistance(int echoPin) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 200; 
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(115200);

  // MOTOR STOP
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, LOW);

  // SENZORI
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO_FL, INPUT);
  pinMode(ECHO_FM, INPUT);
  pinMode(ECHO_FR, INPUT);

  startTime = millis();

  // SERVO
  steering.setPeriodHertz(50);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steering.attach(SERVO_PIN, 500, 2400);

  steering.write(SERVO_CENTER);
  delay(500);

  // MOTOR PWM
  ledcAttach(ENA, 1000, 8);
  ledcWrite(ENA, 0);

  delay(5000);
}

void loop() {

  // --- START FAZA ---
  if (millis() - startTime < 1500) {
    steering.write(SERVO_CENTER);
    ledcWrite(ENA, 180);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    return;
  }

  // --- ČITANJE SENZORA ---
  float dFM = readDistance(ECHO_FM);
  delay(20);

  float dFL = readDistance(ECHO_FL);
  delay(20);

  float dFR = readDistance(ECHO_FR);

  // --- FILTER ---
  if (dFL > 150) dFL = lastFL;
  if (dFR > 150) dFR = lastFR;

  dFL = 0.7 * lastFL + 0.3 * dFL;
  dFR = 0.7 * lastFR + 0.3 * dFR;

  lastFL = dFL;
  lastFR = dFR;

  // --- ERROR ---
  float error = dFR - dFL;

  if (abs(error) < 15) error = 0;

  // --- PD ---
  float control = (Kp * error) + (Kd * (error - lastError));

  // --- HISTERESIS ---
  if (abs(control) < 10 && abs(lastControl) < 10) {
    control = 0;
  }
  lastControl = control;

  // --- LIMIT ---
  control = constrain(control, -18, 18);

  float steer = SERVO_CENTER + control;

  // --- PREPREKA ---
  if (dFM < 80) {
    if (dFL > dFR) steer = SERVO_RIGHT;
    else steer = SERVO_LEFT;
  }

  steer = constrain(steer, SERVO_LEFT, SERVO_RIGHT);

  // --- SERVO CONTROL ---
  if (dFM < 80) {
    steering.write((int)steer);
    lastSteer = steer;
  } else {
    float factor = 0.2;
    float smoothSteer = lastSteer + factor * (steer - lastSteer);

    steering.write((int)smoothSteer);
    lastSteer = smoothSteer;
  }

  lastError = error;

  // --- BRZINA ---
  if (dFM < 80) {
    ledcWrite(ENA, 120);
  } 
  else if (dFL < 15 && dFR < 15) {
    ledcWrite(ENA, 150);
  } 
  else {
    ledcWrite(ENA, 200);
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // --- DEBUG ---
  Serial.print("L: "); Serial.print(dFL);
  Serial.print(" | R: "); Serial.print(dFR);
  Serial.print(" | M: "); Serial.print(dFM);
  Serial.print(" | Servo: "); Serial.println(lastSteer);

  delay(30);
}