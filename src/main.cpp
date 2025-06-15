#include <Arduino.h>

// RSSI Analog Pins
#define rssiTopPin A0
#define rssiBottomPin A1
#define rssiLeftPin A2
#define rssiRightPin A3

int rssiPins[4] = {A0, A1, A2, A3}; 

float rssiTop;
float rssiBottom;
float rssiLeft;
float rssiRight;

// PID Gains
float kp = 0;
float ki = 0;
float kd = 0;

// PID Struct
struct PIDController {
  float error = 0;
  float prevError = 0;
  float integral = 0;
  float output = 0;
};

PIDController verticalPID;
PIDController horizontalPID;

// Timing Variables
unsigned long lastTime = 0;
float deltaTime = 0;

void setup() {
  Serial.begin(9600);
}


void readRSSI(int rssiTop, int rssiBottom, int rssiLeft, int rssiRight) {
  int* rssi_array[4] = {&rssiTop, &rssiBottom, &rssiLeft, &rssiRight};

  for (int i=0; i<4; i++) {
    *rssi_array[i] = analogRead(rssiPins[i]);
  }
}


void loop() {
  // Time Management
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Read RSSI Voltages
  readRSSI(rssiTop, rssiBottom, rssiLeft, rssiRight);

  // Compute Errors
  verticalPID.error = rssiTop - rssiBottom;
  horizontalPID.error = rssiRight - rssiLeft;

  // Update Vertical PID
  verticalPID.integral += verticalPID.error * deltaTime;
  float verticalDerivative = (verticalPID.error - verticalPID.prevError) / deltaTime;
  verticalPID.output = kp * verticalPID.error + ki * verticalPID.integral + kd * verticalDerivative;
  verticalPID.prevError = verticalPID.error;

  // Update Horizontal PID
  horizontalPID.integral += horizontalPID.error * deltaTime;
  float horizontalDerivative = (horizontalPID.error - horizontalPID.prevError) / deltaTime;
  horizontalPID.output = kp * horizontalPID.error + ki * horizontalPID.integral + kd * horizontalDerivative;
  horizontalPID.prevError = horizontalPID.error;

  delay(10);
}
