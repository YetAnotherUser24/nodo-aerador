/**
 * @file test_motor_bus.cpp
 * @brief Host conformance test for the motor bus device model.
 *
 * Runs the DECLARATIVE simulator (`src/motor_sim.cpp`) on a deterministic clock and asserts
 * the contract the node relies on: arming, spin-up toward the commanded RPM, load/current
 * scaling, bus sag, fault disarming, and clearErrors recovery.
 *
 * Build (no Arduino needed — the sim is Arduino-free):
 *   g++ -std=c++17 -I src -I test/host test/host/test_motor_bus.cpp src/motor_sim.cpp -o /tmp/test_motor_bus
 */

#include "motor_sim.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static void advance(MotorSim& sim, uint32_t& t, uint32_t ms, uint32_t stepMs = 10) {
  for (uint32_t elapsed = 0; elapsed < ms; elapsed += stepMs) {
    t += stepMs;
    sim.tick(t);
  }
}

int main() {
  MotorSimSpec spec;
  spec.rpmMax = 600.0f;
  spec.spinUpTauS = 0.35f;
  spec.nominalVbus = 24.0f;
  spec.idleIbus = 0.15f;
  spec.maxLoadIbus = 9.5f;

  MotorSim sim(spec);
  uint32_t t = 1000;
  sim.tick(t);

  // 1. Boots disarmed and idle.
  assert(sim.state() == 1);
  assert(!sim.isArmed());
  assert(near(sim.actualRpm(), 0.0f, 0.001f));

  // 2. Unarmed velocity commands are ignored.
  sim.requestVelocityRpm(400.0f);
  advance(sim, t, 500);
  assert(near(sim.actualRpm(), 0.0f, 0.5f));

  // 3. Arming settles after armDelayMs and spins up toward the setpoint.
  sim.requestState(8);
  advance(sim, t, 200);                 // pass armDelayMs
  sim.requestVelocityRpm(600.0f);
  advance(sim, t, 3000);                // ~9 time constants
  assert(sim.isArmed());
  assert(sim.state() == 8);
  assert(sim.actualRpm() > 590.0f);     // reached envelope
  assert(sim.actualRpm() <= spec.rpmMax + 0.001f);

  // 4. Full-speed load, bus sag and thermal rise are consistent.
  assert(near(sim.ibus(), spec.idleIbus + spec.maxLoadIbus, 0.5f));
  assert(sim.vbus() < spec.nominalVbus);
  assert(near(sim.vbus(), spec.nominalVbus - sim.ibus() * spec.busResistance, 0.01f));
  assert(sim.power() > 0.0f);
  assert(sim.fetTemp() > spec.ambientTempC);

  // 5. Velocity is clamped to the envelope.
  sim.requestVelocityRpm(9999.0f);
  assert(sim.targetRpm() <= spec.rpmMax + 0.001f);

  // 5b. Torque and iq must fall to zero smoothly as the shaft stops: no blow-up from
  //     dividing the idle bus power by a near-zero omega.
  sim.requestState(1);
  bool torqueSettled = false;
  bool iqSettled = false;
  for (int i = 0; i < 500; ++i) {
    advance(sim, t, 10);
    if (sim.actualRpm() < 2.0f) {
      assert(sim.torque() <= 1.0f);   // never a spike
      assert(sim.iq() <= 15.0f);      // iq = torque/0.09, so bounded
      if (sim.actualRpm() < 0.5f) {
        assert(near(sim.torque(), 0.0f, 0.01f));
        assert(near(sim.iq(), 0.0f, 0.05f));
        torqueSettled = true;
        iqSettled = true;
        break;
      }
    }
  }
  assert(torqueSettled && iqSettled);

  // 6. Idle command coasts down.
  sim.requestState(1);
  advance(sim, t, 3000);
  assert(!sim.isArmed());
  assert(sim.actualRpm() < 1.0f);

  // 7. A fault disarms the drive and is cleared explicitly.
  sim.requestState(8);
  advance(sim, t, 200);
  sim.injectFault(0x00000001u);
  assert(!sim.isArmed());
  assert(sim.state() == 1);
  assert(sim.axisError() == 0x00000001u);
  sim.clearErrors();
  assert(sim.axisError() == 0u);

  // 8. Data-driven fault schedule fires once at its timestamp.
  MotorSim scheduled(spec);
  scheduled.addFaultEvent({5000u, 0x00000004u});
  uint32_t ts = 0;
  scheduled.tick(ts);
  advance(scheduled, ts, 4000);
  assert(scheduled.axisError() == 0u);
  advance(scheduled, ts, 2000);
  assert(scheduled.axisError() == 0x00000004u);

  std::printf("test_motor_bus: all assertions passed\n");
  return 0;
}
