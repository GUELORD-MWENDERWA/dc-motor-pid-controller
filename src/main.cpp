// Closed-loop DC motor speed control.
//
// A quadrature encoder measures shaft speed; a PID controller running at a
// fixed 100 Hz drives an H-bridge (L298N or TB6612) with PWM. Gains and the
// setpoint are tuned live over the serial port, and telemetry is printed in a
// format the Arduino Serial Plotter understands.

#include <Arduino.h>
#include <Pid.h>

// ----- Wiring (Arduino Uno / Nano) -----
constexpr uint8_t ENC_A = 2;    // interrupt pin
constexpr uint8_t ENC_B = 4;
constexpr uint8_t PWM_PIN = 5;  // H-bridge enable (ENA), 980 Hz PWM on pin 5
constexpr uint8_t IN1 = 7;
constexpr uint8_t IN2 = 8;

// ----- Motor and loop parameters -----
// Encoder: 11 pulses per motor revolution, 30:1 gearbox, x1 decoding on channel A
// (one interrupt per pulse), so 330 counts per output shaft revolution.
constexpr float COUNTS_PER_REV = 11.0f * 30.0f;
constexpr float LOOP_HZ = 100.0f;
constexpr unsigned long LOOP_US = 1000000UL / static_cast<unsigned long>(LOOP_HZ);
constexpr float MAX_RPM = 330.0f;

volatile long encoderCount = 0;

void onEncoderA() {
  // x1 decoding on channel A rising edges: direction from channel B.
  encoderCount += digitalRead(ENC_B) ? -1 : 1;
}

Pid pid(0.6f, 4.0f, 0.005f, 1.0f / LOOP_HZ, -255.0f, 255.0f);
float setpointRpm = 120.0f;
bool enabled = true;
bool telemetry = true;

void driveMotor(float command) {
  const int pwm = constrain(static_cast<int>(fabs(command)), 0, 255);
  digitalWrite(IN1, command >= 0 ? HIGH : LOW);
  digitalWrite(IN2, command >= 0 ? LOW : HIGH);
  analogWrite(PWM_PIN, pwm);
}

float readRpm() {
  static long last = 0;
  noInterrupts();
  const long now = encoderCount;
  interrupts();
  const long delta = now - last;
  last = now;
  return delta * LOOP_HZ * 60.0f / COUNTS_PER_REV;
}

void printHelp() {
  Serial.println(F("Commands: s<rpm> set speed | p<kp> i<ki> d<kd> gains | f<alpha> D filter"));
  Serial.println(F("          x stop | g go | t toggle telemetry | ? show state"));
}

void printState() {
  Serial.print(F("setpoint=")); Serial.print(setpointRpm);
  Serial.print(F(" enabled=")); Serial.print(enabled);
  Serial.print(F(" integral=")); Serial.println(pid.integral());
}

void handleSerial() {
  if (!Serial.available()) return;
  const char cmd = Serial.read();
  const float value = Serial.parseFloat();
  static float kp = 0.6f, ki = 4.0f, kd = 0.005f;
  switch (cmd) {
    case 's': setpointRpm = constrain(value, -MAX_RPM, MAX_RPM); break;
    case 'p': kp = value; pid.setGains(kp, ki, kd); break;
    case 'i': ki = value; pid.setGains(kp, ki, kd); break;
    case 'd': kd = value; pid.setGains(kp, ki, kd); break;
    case 'f': pid.setDerivativeFilter(constrain(value, 0.01f, 1.0f)); break;
    case 'x': enabled = false; driveMotor(0); break;
    case 'g': enabled = true; pid.reset(readRpm()); break;
    case 't': telemetry = !telemetry; break;
    case '?': printState(); break;
    case 'h': printHelp(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderA, RISING);
  pid.setDerivativeFilter(0.3f);
  printHelp();
}

void loop() {
  static unsigned long next = micros();
  handleSerial();
  if (static_cast<long>(micros() - next) < 0) return;
  next += LOOP_US;

  const float rpm = readRpm();
  const float command = enabled ? pid.update(setpointRpm, rpm) : 0.0f;
  if (enabled) driveMotor(command);

  static uint8_t divider = 0;
  if (telemetry && ++divider >= 5) {  // 20 Hz telemetry for the Serial Plotter
    divider = 0;
    Serial.print(F("setpoint:")); Serial.print(setpointRpm);
    Serial.print(F(",rpm:")); Serial.print(rpm);
    Serial.print(F(",pwm:")); Serial.println(command);
  }
}
