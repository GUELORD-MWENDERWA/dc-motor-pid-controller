// Closed-loop simulation of lib/Pid/Pid.h against the first-order motor model used
// in the unit tests. Prints CSV: time, then speed and PWM for each configuration.
//   g++ -std=c++17 -O2 -Ilib/Pid docs/step_response.cpp -o step && ./step > step.csv

#include <Pid.h>
#include <math.h>
#include <stdio.h>

struct Motor {
  float tau = 0.15f, gain = 1.3f, rpm = 0.0f;
  void step(float pwm, float dt) { rpm += dt / tau * (gain * pwm - rpm); }
};

int main() {
  const float dt = 0.01f;
  struct Config { const char* name; float kp, ki, kd, outMax; } cfg[] = {
      {"P", 0.5f, 0.0f, 0.0f, 255}, {"PI", 0.5f, 5.0f, 0.0f, 255},
      {"PID", 1.2f, 8.0f, 0.02f, 255}, {"PI_no_antiwindup", 0.5f, 5.0f, 0.0f, 1e9f},
  };
  const int n = sizeof cfg / sizeof cfg[0];
  Pid* pid[n];
  Motor motor[n];
  for (int k = 0; k < n; k++) {
    pid[k] = new Pid(cfg[k].kp, cfg[k].ki, cfg[k].kd, dt, -cfg[k].outMax, cfg[k].outMax);
    pid[k]->setDerivativeFilter(0.3f);
  }
  printf("time");
  for (int k = 0; k < n; k++) printf(",%s_rpm,%s_pwm", cfg[k].name, cfg[k].name);
  printf("\n");
  for (int i = 0; i <= 400; i++) {
    const float t = i * dt;
    // Setpoint 100 rpm, a large 400 rpm request between 1 s and 2 s, then back to 150 rpm.
    const float sp = t < 1.0f ? 100.0f : (t < 2.0f ? 400.0f : 150.0f);
    printf("%.2f", t);
    for (int k = 0; k < n; k++) {
      float u = pid[k]->update(sp, motor[k].rpm);
      u = fmaxf(-255.0f, fminf(255.0f, u));  // the PWM stage always clips
      motor[k].step(u, dt);
      printf(",%.3f,%.2f", motor[k].rpm, u);
    }
    printf("\n");
  }
}
