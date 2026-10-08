/**
 * Stubs mínimos de Arduino para compilar el firmware REAL en el host (test de fidelidad).
 *
 * Objetivo: poder enlazar `src/control_engine.cpp` tal cual, sin tocar el código de producción.
 * Todo lo que el motor usa de Arduino se resuelve aquí:
 *   - `constrain()`       (usado en configure() y computeOutputRpm())
 *   - `millis()`          (tiempo inyectable, para que el escenario sea determinista)
 *   - `portMUX_*`         (los spinlocks son no-ops en un solo hilo)
 *
 * NADA de esto entra en el binario del ESP32: sólo se usa con `-I test/host`.
 */
#pragma once

#include <stdint.h>
#include <math.h>
#include <stddef.h>

/** Reloj falso: el dumper lo fija antes de cada paso. */
extern unsigned long g_fakeMillis;
inline unsigned long millis() { return g_fakeMillis; }
inline void delay(unsigned long) {}

/** Equivalente exacto del macro `constrain` de Arduino. */
template <typename T>
inline T constrain(T amt, T low, T high) {
  return amt < low ? low : (amt > high ? high : amt);
}

/* Core 1 es el único dueño del motor: el mutex es un no-op en el host. */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}

/* Identificadores referenciados por los `#define` de board.h. */
#define VSPI 3
#define HSPI 2
#define GPIO_NUM_34 34
