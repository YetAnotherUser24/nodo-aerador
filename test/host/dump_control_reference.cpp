/**
 * Dumper de referencia del MOTOR DE CONTROL REAL.
 *
 * Compila `src/control_engine.cpp` (el de producción, sin copias) con los stubs de `test/host` y
 * emite una trayectoria determinista en CSV: `t_ms,do_mg_l,rpm`.
 *
 * Sirve para dos cosas:
 *  1. Verificar que el port a JavaScript (`scripts/virtual-bench/models/controlEngine.js`) es fiel:
 *     si difiere en algo, el test de fidelidad lo detecta.
 *  2. Probar en el host el comportamiento real del PID, cosa que el test existente NO hacía porque
 *     reimplementaba la matemática en vez de enlazar el motor.
 *
 * Compilación (desde Codigos/odrive-controller):
 *   g++ -O1 -std=c++17 -I test/host -I src -I include \
 *       test/host/dump_control_reference.cpp src/control_engine.cpp -o test/host/dump.exe
 */
#include <stdio.h>
#include <math.h>
#include "control_engine.h"
#include "config.h"

unsigned long g_fakeMillis = 0;

/* Escenario de referencia. DEBE coincidir paso a paso con el del verificador en JS. */
static const unsigned long T0_MS = 1000;
static const unsigned long DT_MS = 50;          // 20 Hz, como CONTROL_LOOP_INTERVAL_MS
static const int STEP_COUNT = 1080;             // ~54 s de simulación

/** Perfil de OD: meseta, escalón descendente, recuperación y finalmente silencio (watchdog). */
static float doAtStep(int step) {
  const unsigned long t = T0_MS + (unsigned long)step * DT_MS;
  if (t < 3000) return 8.0f;        // error negativo -> salida saturada en 0
  if (t < 12000) return 3.0f;       // escalón fuerte: aquí se nota el término derivativo
  if (t < 14000) return 5.0f;       // escalón inverso
  return -1.0f;                     // -1 => no llega paquete (watchdog)
}

int main() {
  ControlEngine engine;

  ControlConfig cfg = engine.getConfig();
  cfg.mode = MODE_PID;
  cfg.target_do_mg_l = 7.5f;
  cfg.kp = DEFAULT_KP;
  cfg.ki = DEFAULT_KI;
  cfg.kd = DEFAULT_KD;
  cfg.max_rpm = MAX_MOTOR_RPM;
  cfg.failsafe_rpm = FAILSAFE_MOTOR_RPM;
  engine.configure(cfg);

  printf("t_ms,do_mg_l,rpm\n");

  for (int step = 0; step < STEP_COUNT; step++) {
    const unsigned long t = T0_MS + (unsigned long)step * DT_MS;
    g_fakeMillis = t;

    const float doVal = doAtStep(step);
    if (doVal >= 0.0f) {
      engine.updateProcessVariable(doVal, 24.0f);
    }

    const float rpm = engine.computeOutputRpm(t);
    printf("%lu,%.3f,%.6f\n", t, doVal, rpm);
  }

  return 0;
}
