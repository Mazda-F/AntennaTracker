#include <Arduino.h>

//RSSI Analog Pins
#define rssiTopPin A6
#define rssiBottomPin A7
#define rssiLeftPin A8
#define rssiRightPin A9

int rssiPins[4] = {rssiTopPin, rssiBottomPin, rssiLeftPin, rssiRightPin};
float rssiValues[4] = {0, 0, 0, 0};

//RSSI Kalman Filter State
struct KalmanFilter {
  float estimate = 0.0;
  float error = 1.0;
  //process noise
  const float q = 0.01;
  //measurement noise
  const float r = 0.1;
};

//One Kalman filter per RSSI input
KalmanFilter kalman[4];  

//PID Struct
struct PIDController {
  float error = 0;
  float prevError = 0;
  float integral = 0;
  float output = 0;
};

PIDController verticalPID, horizontalPID;

//PID Gains (needs tuning)
float kp = 1.0;
float ki = 1.0;
float kd = 0.0;

//Timing Variables used in the PID loop
unsigned long lastTime = 0;
float deltaTime = 0.01;

//Servo PWM Output Pins
const int panServoPin = 36;
const int tiltServoPin = 37;
//Need to adjust the initial PWM duty cycle or angle of the servos using the variables below
float lastPanServoPWM = 0.0;
float lastTiltServoPWM = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(panServoPin, OUTPUT);
  pinMode(tiltServoPin, OUTPUT);
}

//Linear Kalman Filter
float kalmanUpdate(KalmanFilter &kf, float measurement) {
  //Prediction update
  kf.error += kf.q;

  //Measurement update
  float gain = kf.error / (kf.error + kf.r);
  kf.estimate += gain * (measurement - kf.estimate);
  kf.error *= (1 - gain);

  return kf.estimate;
}

//RSSI Reading Function with Kalman Filtering
void readRSSI() {
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(rssiPins[i]);
    //Teensy uses 3.3V ADC reference
    //RTC6715 outputs a voltage between 0.5V-1.1V for RSSI
    //0.5V-1.1V corresponds to signal strength from -91 dBm to +5 dBm
    float voltage = (raw / 1023.0) * 3.3;  
    rssiValues[i] = kalmanUpdate(kalman[i], voltage);
  }
}

//PID Calculation Loop
void PIDloop() {
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  readRSSI();

  float top = rssiValues[0];
  float bottom = rssiValues[1];
  float left = rssiValues[2];
  float right = rssiValues[3];

  //Vertical PID
  verticalPID.error = top - bottom;
  verticalPID.integral += verticalPID.error * deltaTime;
  float dVert = (verticalPID.error - verticalPID.prevError) / deltaTime;
  verticalPID.output = kp * verticalPID.error + ki * verticalPID.integral + kd * dVert;
  verticalPID.prevError = verticalPID.error;

  //Horizontal PID
  horizontalPID.error = right - left;
  horizontalPID.integral += horizontalPID.error * deltaTime;
  float dHorz = (horizontalPID.error - horizontalPID.prevError) / deltaTime;
  horizontalPID.output = kp * horizontalPID.error + ki * horizontalPID.integral + kd * dHorz;
  horizontalPID.prevError = horizontalPID.error;
}

void driveMotors(float panDelta, float tiltDelta) {
  // Apply PID output to PWM duty cycle
  lastPanServoPWM += panDelta;
  lastTiltServoPWM += tiltDelta;

  //Constrain PWM to 0–2.5ms pulse width which is the range for Miuzei 20kg sersvos (0–32 duty at 50Hz, 8-bit)
  // 0ms pulse width is analogWrite(servoPin, 0), and 2.5ms pulse width is analogWrite(servoPin, 32) for a 50Hz PWM signal
  //Arduino analogWrite resolution is 8bits by default
  lastPanServoPWM = constrain(lastPanServoPWM, 0, 32);
  lastTiltServoPWM = constrain(lastTiltServoPWM, 0, 32);

  analogWrite(panServoPin, (int)lastPanServoPWM);
  analogWrite(tiltServoPin, (int)lastTiltServoPWM);
}


void loop() {
  PIDloop();
  driveMotors(horizontalPID.output, verticalPID.output);
}
