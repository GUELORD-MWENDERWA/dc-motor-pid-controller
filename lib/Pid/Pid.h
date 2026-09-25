#pragma once
// Discrete PID controller for fixed-rate control loops.
//
// Features that matter on real hardware:
//  - derivative on measurement, so a setpoint step does not cause a derivative kick
//  - first-order low-pass filter on the derivative term to limit noise amplification
//  - output clamping with conditional-integration anti-windup
//  - bumpless reset
//
// Hardware independent (no Arduino includes) so it can be unit-tested on a PC.

class Pid {
 public:
  Pid(float kp, float ki, float kd, float dt, float outMin, float outMax)
      : kp_(kp), ki_(ki), kd_(kd), dt_(dt), outMin_(outMin), outMax_(outMax) {}

  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  // alpha in (0, 1]: 1 disables filtering, smaller values filter more.
  void setDerivativeFilter(float alpha) { alpha_ = alpha; }

  void setLimits(float outMin, float outMax) {
    outMin_ = outMin;
    outMax_ = outMax;
  }

  void reset(float measurement = 0.0f) {
    integral_ = 0.0f;
    dFiltered_ = 0.0f;
    prevMeasurement_ = measurement;
    first_ = true;
  }

  float update(float setpoint, float measurement) {
    const float error = setpoint - measurement;

    // Derivative on measurement: d(error)/dt = -d(measurement)/dt when the setpoint is constant.
    float dRaw = 0.0f;
    if (!first_) dRaw = -(measurement - prevMeasurement_) / dt_;
    first_ = false;
    prevMeasurement_ = measurement;
    dFiltered_ += alpha_ * (dRaw - dFiltered_);

    const float p = kp_ * error;
    const float d = kd_ * dFiltered_;
    const float candidateIntegral = integral_ + ki_ * error * dt_;
    float out = p + candidateIntegral + d;

    // Anti-windup: only accept the new integral if it does not push further into saturation.
    if (out > outMax_) {
      out = outMax_;
      if (error < 0) integral_ = candidateIntegral;
    } else if (out < outMin_) {
      out = outMin_;
      if (error > 0) integral_ = candidateIntegral;
    } else {
      integral_ = candidateIntegral;
    }
    lastP_ = p;
    lastD_ = d;
    return out;
  }

  float integral() const { return integral_; }
  float lastP() const { return lastP_; }
  float lastD() const { return lastD_; }

 private:
  float kp_, ki_, kd_, dt_;
  float outMin_, outMax_;
  float alpha_ = 1.0f;
  float integral_ = 0.0f;
  float dFiltered_ = 0.0f;
  float prevMeasurement_ = 0.0f;
  float lastP_ = 0.0f, lastD_ = 0.0f;
  bool first_ = true;
};
