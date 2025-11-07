#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0; // Encoder position
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Variables for RPM calculation
volatile long lastPos = 0; // Last encoder position for RPM calculation
long lastRPMTime = 0;      // Last time RPM was calculated
float rpm = 0;             // RPM value
const int pulsesPerRevolution = 400; // Adjust based on your encoder's specs

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  Serial.println("target pos");
}

void loop() {
  //1 meter is 442.47788
  // set target position
  int target = 3950;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position atomically
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // Calculate RPM every 100ms (adjust interval as needed)
  if (millis() - lastRPMTime >= 100) {
    calculateRPM();
    lastRPMTime = millis();
  }

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // store previous error
  eprev = e;

  // Debugging output
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(" Pos: ");
  Serial.print(pos);
  Serial.print(" RPM: ");
  Serial.println(rpm);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }  
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void calculateRPM() {
  long currentPos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentPos = posi;
  }

  long deltaPos = currentPos - lastPos;
  lastPos = currentPos;

  // Calculate RPM (pulses / pulsesPerRevolution) * (60 seconds / time interval in seconds)
  rpm = (deltaPos / (float)pulsesPerRevolution) * (60000.0 / (millis() - lastRPMTime));
}
