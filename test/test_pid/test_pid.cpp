// Unit tests for the PID controller, run on the host with `pio test -e native`.
// A first-order motor model (time constant tau, static gain K) closes the loop.

#include <Pid.h>
#include <math.h>
#include <unity.h>

namespace {

struct Motor {
  float tau = 0.15f;  // s
  float gain = 1.3f;  // rpm per PWM unit
  float rpm = 0.0f;
  float step(float pwm, float dt) {
    rpm += dt / tau * (gain * pwm - rpm);
    return rpm;
  }
};

constexpr float DT = 0.01f;

float simulate(Pid& pid, Motor& m, float setpoint, int steps, float* maxRpm = nullptr) {
  float peak = 0.0f;
  for (int i = 0; i < steps; i++) {
    const float u = pid.update(setpoint, m.rpm);
    m.step(u, DT);
    if (m.rpm > peak) peak = m.rpm;
  }
  if (maxRpm) *maxRpm = peak;
  return m.rpm;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_proportional_only_leaves_steady_state_error() {
  Pid pid(0.5f, 0.0f, 0.0f, DT, -255, 255);
  Motor m;
  const float final = simulate(pid, m, 100.0f, 500);
  // Closed-loop steady state of a P controller: K Kp / (1 + K Kp) of the setpoint.
  const float expected = 100.0f * 1.3f * 0.5f / (1.0f + 1.3f * 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, final);
}

void test_integral_action_removes_steady_state_error() {
  Pid pid(0.5f, 5.0f, 0.0f, DT, -255, 255);
  Motor m;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, simulate(pid, m, 100.0f, 1000));
}

void test_output_is_clamped() {
  Pid pid(100.0f, 0.0f, 0.0f, DT, -255, 255);
  TEST_ASSERT_EQUAL_FLOAT(255.0f, pid.update(1000.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(-255.0f, pid.update(-1000.0f, 0.0f));
}

// Saturate the loop with an unreachable setpoint for 3 s, then return the
// number of steps needed to settle within 2 % of a reachable one. The PWM
// actuator always clips at +/-255, whatever the controller asks for.
int recoverySteps(Pid& pid, float* integralAfterSaturation) {
  Motor m;
  auto actuate = [&](float setpoint) {
    const float u = fmaxf(-255.0f, fminf(255.0f, pid.update(setpoint, m.rpm)));
    m.step(u, DT);
  };
  for (int i = 0; i < 300; i++) actuate(400.0f);
  *integralAfterSaturation = pid.integral();
  for (int i = 1; i <= 2000; i++) {
    actuate(150.0f);
    if (fabsf(m.rpm - 150.0f) < 3.0f) return i;
  }
  return 2000;
}

void test_anti_windup_speeds_up_recovery() {
  Pid withLimits(0.5f, 5.0f, 0.0f, DT, -255, 255);
  // Same gains but limits the controller never reaches: no anti-windup.
  Pid withoutLimits(0.5f, 5.0f, 0.0f, DT, -1e9, 1e9);
  float iWith = 0.0f, iWithout = 0.0f;
  const int fast = recoverySteps(withLimits, &iWith);
  const int slow = recoverySteps(withoutLimits, &iWithout);
  TEST_ASSERT_TRUE(iWith < 255.0f);      // integral stops growing at saturation
  TEST_ASSERT_TRUE(iWithout > 1000.0f);  // without anti-windup it keeps growing
  TEST_ASSERT_TRUE(fast * 3 < slow);     // measured: 35 vs 168 steps
}

void test_no_derivative_kick_on_setpoint_step() {
  Pid pid(0.0f, 0.0f, 1.0f, DT, -1000, 1000);
  pid.update(0.0f, 50.0f);
  // Same measurement, large setpoint change: derivative on measurement stays zero.
  pid.update(500.0f, 50.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastD());
}

void test_derivative_filter_smooths_noise() {
  Pid raw(0.0f, 0.0f, 1.0f, DT, -1e6, 1e6);
  Pid filtered(0.0f, 0.0f, 1.0f, DT, -1e6, 1e6);
  filtered.setDerivativeFilter(0.1f);
  float maxRaw = 0.0f, maxFiltered = 0.0f;
  for (int i = 0; i < 100; i++) {
    const float noisy = 100.0f + ((i % 2) ? 1.0f : -1.0f);
    raw.update(100.0f, noisy);
    filtered.update(100.0f, noisy);
    if (i > 50) {
      maxRaw = fmaxf(maxRaw, fabsf(raw.lastD()));
      maxFiltered = fmaxf(maxFiltered, fabsf(filtered.lastD()));
    }
  }
  TEST_ASSERT_TRUE(maxFiltered < maxRaw / 5.0f);
}

void test_reset_clears_state() {
  Pid pid(1.0f, 10.0f, 0.0f, DT, -255, 255);
  for (int i = 0; i < 10; i++) pid.update(10.0f, 0.0f);
  TEST_ASSERT_TRUE(pid.integral() > 0.0f);
  pid.reset(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_proportional_only_leaves_steady_state_error);
  RUN_TEST(test_integral_action_removes_steady_state_error);
  RUN_TEST(test_output_is_clamped);
  RUN_TEST(test_anti_windup_speeds_up_recovery);
  RUN_TEST(test_no_derivative_kick_on_setpoint_step);
  RUN_TEST(test_derivative_filter_smooths_noise);
  RUN_TEST(test_reset_clears_state);
  return UNITY_END();
}
