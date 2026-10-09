# 🌊 Nodo aerador — ESP32-S3 + ODrive S1 (`nodo-aerador`)

> **Repositorio:** [github.com/YetAnotherUser24/nodo-aerador](https://github.com/YetAnotherUser24/nodo-aerador)

Sistema de control embebido en tiempo real para aeración en lazo cerrado basado en **ESP32** y controlador de motor **ODrive S1**. Este firmware regula automáticamente la velocidad de giro (RPM) del motor de aeración en función del nivel de **Oxígeno Disuelto (DO)** y temperatura del agua medidos en campo, además de sincronizarse bidireccionalmente con la plataforma en la nube (Dashboard Web).

---

## 📋 Tabla de Contenidos

- [Características Principales](#-características-principales)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Requisitos Previos](#-requisitos-previos)
- [Guía Rápida: Cómo Subir el Código (Nuevo Usuario)](#-guía-rápida-cómo-subir-el-código-nuevo-usuario)
- [Conexiones y Cableado de Hardware](#-conexiones-y-cableado-de-hardware)
- [Entornos de Compilación (PlatformIO)](#-entornos-de-compilación-platformio)
- [Modos de Control](#-modos-de-control)
- [Modo Simulación Virtual (ODrive Virtual)](#-modo-simulación-virtual-odrive-virtual)
- [Consola de Ingeniería y Calibración UART](#-consola-de-ingeniería-y-calibración-uart)
- [Configuración de Parámetros (`config.h`)](#-configuración-de-parámetros-configh)
- [Telemetría y Comandos en la Nube](#-telemetría-y-comandos-en-la-nube)

---

## 🚀 Características Principales

1. **Arquitectura Dual-Core Asíncrona (FreeRTOS)**:
   - **Core 1 (Control Crítico)**: Bucle determinístico de 20 Hz (50 ms). Dueño exclusivo del puerto UART del ODrive S1 para eliminar contención de bus y latencias.
   - **Core 0 (Comunicaciones y Almacenamiento)**: Cliente WiFi + HTTPS hacia la nube (Supabase / Vercel), sondeo de comandos remotos, persistencia NVS y escritura no bloqueante en MicroSD.
2. **Algoritmos de Control en Lazo Cerrado**:
   - **Controlador PID**: Con anti-windup clamping en el integrador y filtrado derivativo.
   - **Controlador Difuso (Fuzzy Logic)**: rama heredada del controlador de banco, **fuera del alcance de la tesis** (AGENTS.md regla 14). Se conserva por compatibilidad de comandos; el algoritmo declarado y sintonizado es el PID. El único término derivativo vivo está en el PID.
   - **Control Manual Remoto**: Ajuste de acelerador porcentual (0–100%) directo desde la interfaz web.
3. **Recepción Inalámbrica P2P (ESP-NOW)**:
   - Enlace directo con la boya o nodo sensor (`nodo-sensor-od`) sin requerir router WiFi para recibir lecturas de DO (mg/L), temperatura (°C) y sincronización temporal RTC.
4. **Failsafe y Parada de Emergencia (E-Stop)**:
   - Perro guardián (watchdog) de 35 segundos: si la boya pierde comunicación, el motor conmuta automáticamente a una velocidad de seguridad predefinida.
   - E-Stop por software con prioridad absoluta: apaga el motor inmediatamente y transiciona el ODrive a estado *IDLE* (desarmado).
5. **Datalogger MicroSD de Alta Velocidad (10 Hz)**:
   - Cola de memoria RAM FreeRTOS para encolar muestras en < 5 µs desde el Core 1 y escritura asíncrona por bloques en el Core 0 (evita congelar el lazo de control durante los borrados de flash de la SD).
6. **Simulador ODrive Virtual Incorporado**:
   - Permite compilar y validar toda la pila de red, dashboard web y lógica de control sin necesidad de conectar hardware físico (motor, fuente ni ODrive).

---

## 🏛 Arquitectura del Sistema

```
                        [ Boya / Nodo Sensor (OD-Logger) ]
                                       │
                                (ESP-NOW P2P)
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ESP32 (AquaControl)                              │
│                                                                             │
│  [ CORE 1 - TIEMPO REAL (20 Hz) ]          [ CORE 0 - COMUNICACIONES ]      │
│  ┌───────────────────────────────┐        ┌──────────────────────────────┐  │
│  │       espNowRxCallback        │        │      WiFi & HTTPS Client     │  │
│  │ (DO mg/L, Temp °C, RTC sync)  │        │  (Vercel Cloud REST API)     │  │
│  └──────────────┬────────────────┘        └──────────────┬───────────────┘  │
│                 │                                        │                  │
│                 ▼                                        ▼                  │
│  ┌───────────────────────────────┐ Inter-Core  ┌──────────────────────────┐  │
│  │         ControlEngine         │◄───────────►│  Configuración y E-Stop  │  │
│  │    (PID / Fuzzy / Manual)     │   Mutex     │   (Persistencia en NVS)  │  │
│  └──────────────┬────────────────┘             └──────────────────────────┘  │
│                 │                                                           │
│                 ▼                                                           │
│  ┌───────────────────────────────┐ FreeRTOS    ┌──────────────────────────┐  │
│  │     Driver ODriveUART         │──Queue────► │      sdWriterTask        │  │
│  │ (Dueño único del puerto UART) │ (< 5 µs)    │ (MicroSD Logger a 10 Hz) │  │
│  └──────────────┬────────────────┘             └──────────────┬───────────┘  │
└─────────────────┼─────────────────────────────────────────────┼─────────────┘
                  │ UART (115200 / 460800 baud)                 │ SPI (HSPI)
                  ▼                                             ▼
        [ ODrive S1 Driver ]                             [ Tarjeta MicroSD ]
                  │
                  ▼
          [ Motor Brushless ]
```

---

## 📂 Estructura del Proyecto

```text
nodo-aerador/
├── include/
│   └── esp_now_packet.h        # Estructura del paquete de telemetría P2P (Boya -> Controlador)
├── src/
│   ├── board.h                 # Mapeo de pines según la tarjeta (LilyGO vs ESP32 genérico vs S3)
│   ├── config.h                # Constantes globales: WiFi, URLs nube, PID, límites de RPM
│   ├── control_engine.h/.cpp   # Lógica de control PID, Fuzzy, Failsafe, E-Stop y Manual
│   ├── cloud_worker.h/.cpp     # Tarea de fondo en Core 0: REST API, telemetría y comandos
│   ├── esp_now_rx.h/.cpp       # Receptor inalámbrico ESP-NOW para lecturas de la boya
│   ├── main.cpp                # Punto de entrada, inicialización y bucle determinístico Core 1
│   ├── motor_bus.h             # Interfaz del bus del motor (única costura hardware<->control)
│   ├── motor_bus_real.cpp      # Implementación real: adaptador fino sobre ODriveUART
│   ├── motor_bus_virtual.cpp   # Implementación virtual: enlaza el simulador al bus
│   ├── motor_sim.h/.cpp        # Modelo declarativo del motor + driver (sin Arduino, testeable)
│   ├── odrive_sd_logger.h/.cpp # Logger CSV asíncrono no bloqueante a MicroSD
│   ├── odrive_uart.h/.cpp      # Driver ASCII UART para ODrive S1 con telemetría escalonada
│   └── test_odrive_uart.cpp    # Consola interactiva de ingeniería para calibración y pruebas
├── test/
│   ├── test_control_engine.cpp # Pruebas unitarias de cálculo PID y Lógica Difusa
│   └── host/test_motor_bus.cpp # Conformance del modelo del motor (host, sin Arduino)
├── platformio.ini              # Configuración de entornos de compilación de PlatformIO
└── README.md                   # Esta guía
```

---

## 🔧 Requisitos Previos

1. **Visual Studio Code (VS Code)** instalado en la computadora.
2. **Extensión PlatformIO IDE**:
   - Abrir el panel de extensiones de VS Code (`Ctrl + Shift + X`).
   - Buscar e instalar **PlatformIO IDE**.
3. **Drivers USB-Serial** (según tu placa):
   - Para ESP32 DevKit clásico: Drivers **CP210x** o **CH340**.
   - Para ESP32-S3: Controlador nativo USB CDC (suele detectarse automáticamente en Windows 10/11).
4. **Git** (opcional, para clonar el repositorio).

---

## ⚡ Guía Rápida: Cómo Subir el Código (Nuevo Usuario)

Sigue estos pasos ordenados para dejar tu ESP32 programado y funcionando:

### Paso 1: Abrir el Proyecto en VS Code
1. Abre VS Code.
2. Ve a `File` > `Open Folder...` y selecciona la carpeta del proyecto `nodo-aerador`.
3. Espera a que PlatformIO inicialice el entorno (verás el ícono de la hormiga de PlatformIO en la barra lateral izquierda).

### Paso 2: Configurar las Credenciales WiFi y Parámetros
Abre el archivo [src/config.h](src/config.h) y edita las siguientes líneas según tu red local:

```cpp
/* ----------------------------- WiFi ----------------------------- */
#define WIFI_SSID               "TuNombreDeRedWiFi"
#define WIFI_PASSWORD           "TuPasswordWiFi"

/* ----------------------- AquaControl Cloud ----------------------- */
#define AQUACONTROL_HOST        "tesisutec.vercel.app"
#define AQUACONTROL_DEVICE_KEY  "ESP32_ODRIVE"
```

### Paso 3: Seleccionar el Entorno de Compilación
`default_envs = esp32-s3`, así que el selector ya apunta a la placa de la tesis. Elige otro solo si tu hardware es distinto:

| Tu Hardware | Entorno a Seleccionar | Descripción |
| :--- | :--- | :--- |
| **ESP32-S3 SuperMini / DevKitC-1** (Placa de la tesis) | `esp32-s3` **(por defecto)** | Entorno por defecto; 4 MB de flash y USB-CDC |
| **Mismo ESP32-S3, por WiFi** | `esp32-s3-ota` | Flasheo inalámbrico en `aquacontrol-odrive.local` |
| **ESP32-S3 sin ODrive** | `esp32-s3-virtual` | Simula motor y física para probar la web |
| **ESP32 DevKit V1 / NodeMCU** | `esp32dev` | Conexión física por UART al ODrive S1 |
| **ESP32 DevKit, por WiFi** | `esp32dev-ota` | Flasheo inalámbrico en `aquacontrol-odrive.local` |
| **ESP32 DevKit, sin ODrive** | `esp32dev-virtual` | Simulador sobre la placa clásica |
| **LilyGO T3 LoRa32 + HAT** | `lora32` | Variante con HAT de tesis y MicroSD |
| **LilyGO T3 LoRa32, por WiFi** | `lora32-ota` | Flasheo inalámbrico en `aquacontrol-odrive.local` |
| **LilyGO T3 LoRa32, sin ODrive** | `lora32-virtual` | LilyGO en modo simulador |
| **Verificación de telemetría por USB** | `verify` | Lee las 6 variables del ODrive y las imprime como CSV, sin WiFi ni nube |
| **Consola de calibración de banco** | `uart-test` | Consola interactiva UART para configurar el ODrive |

> Los entornos `verify` y `uart-test` sobreescriben `build_src_filter`: **no** son el firmware del nodo.

### Paso 4: Compilar y Subir el Firmware
1. Conecta el ESP32 a tu computadora mediante un cable USB de datos de buena calidad.
2. En la barra inferior de PlatformIO, haz clic en:
   - **Build** (ícono de verificación `✓`) para compilar y comprobar que no haya errores de sintaxis.
   - **Upload** (ícono de flecha `→`) para transferir el firmware a la memoria flash del ESP32.
   *(Alternativa por terminal: `pio run -t upload`; para la placa clásica, `pio run -e esp32dev -t upload`)*.
3. Si el ESP32 no entra en modo bootloader automáticamente durante la subida, mantén presionado el botón **BOOT** (o **IO0**) en la placa cuando aparezca `Connecting........_____`, y suéltalo cuando empiece el flasheo.

### Paso 5: Abrir el Monitor Serial
1. Haz clic en el ícono del enchufe o monitor (`🔌`) en la barra inferior (o ejecuta `pio device monitor -b 115200`).
2. La velocidad configurada es **115200 baud**.
3. Deberás ver el banner de inicio:
   ```text
   =======================================================
     AquaControl: ESP32 ODrive S1 Controller Starting...  
     Board: Generic ESP32 DevKit
   =======================================================
   [Init] Connecting to ODrive S1 via UART...
   [Init] Launching Cloud Worker on Core 0...
   [Init] System ready. Deterministic control loop active on Core 1.
   ```

---

## 🔌 Conexiones y Cableado de Hardware

### 1. Conexión ESP32 a ODrive S1 (UART)

> [!CAUTION]
> **TIERRA COMÚN OBLIGATORIA (GND)**:
> Es indispensable unir el pin **GND del ESP32** con el pin **GND del ODrive S1**. Sin esta referencia común, las señales lógicas UART se corrompen y puede disparar fallos en el bus.

| Pin ESP32 DevKit | Pin ODrive S1 | Función | Notas |
| :---: | :---: | :---: | :--- |
| **GPIO 13** | **RX** (ej. Pin 2 / J3) | ESP32 TX -> ODrive RX | Envío de comandos ASCII de velocidad |
| **GPIO 12** | **TX** (ej. Pin 1 / J3) | ESP32 RX <- ODrive TX | Recepción de telemetría de voltaje/corriente |
| **GND** | **GND** | Masa común | **Mandatorio** |

*(Para la placa **LilyGO LoRa32 + HAT**, el pinout cambia internamente a **GPIO 4 (TX)** y **GPIO 39 (RX)** a 460800 baud según está definido en [src/board.h](src/board.h)).*

### 2. Conexión de Alimentación ODrive S1
- Alimentar el ODrive S1 con fuente DC o batería principal (típicamente 24V o 48V con corriente adecuada).
- Verificar que el ODrive esté configurado previamente con `odrivetool` en modo UART ASCII (`odrv0.config.enable_uart_a = True`).

---

## 🎯 Modos de Control

El controlador puede operar en tres modos, seleccionables remotamente desde la web o guardados en la memoria NVS:

### 1. Modo PID (`MODE_PID`)
- **Objetivo**: Mantener el Oxígeno Disuelto en el setpoint deseado (`target_do_mg_l`, defecto **7.5 mg/L**).
- **Cálculo del error**: `Error = Target_DO - DO_medido`.
  - Si el DO baja -> Error positivo -> Aumenta RPM del motor para airear más el agua.
  - Si el DO supera el setpoint -> Error negativo -> Disminuye RPM hasta apagarse.
- **Protección**: Integrador con sujeción anti-windup (`constrain`) acorde a los límites de velocidad del motor.

### 2. Modo Lógica Difusa (`MODE_FUZZY`)
- Utiliza conjuntos difusos basados en:
  - **Error de Oxígeno**: Negativo, Bajo, Medio, Alto.
  - **Tasa de Cambio ($dError/dt$)**: Mejorando, Estable, Empeorando.
- Proporciona un control suave, reduciendo sobrepasos bruscos (*overshoot*) y desgastes mecánicos en el motor y hélice.

### 3. Modo Manual (`MODE_MANUAL`)
- Control directo desde el slider del Dashboard Web.
- El usuario envía un porcentaje de acelerador (0% a 100%).
- La velocidad objetivo se calcula linealmente: `RPM = (Throttle% / 100) * MAX_MOTOR_RPM`.

### 🛡️ Mecanismos de Seguridad
- **Failsafe Watchdog**: Si transcurren más de **35 segundos** sin recibir paquetes de la boya vía ESP-NOW, el sistema asume pérdida de telemetría de proceso y conmuta a `failsafe_rpm` (por defecto 0 RPM para evitar sobre-aireación en vacío).
- **Parada de Emergencia (E-Stop)**: Cuando la nube emite `emergency_stop`, el Core 1 manda velocidad 0 RPM y solicita inmediatamente el estado `AXIS_STATE_IDLE` al ODrive.

---

## 🧪 Modo Simulación Virtual (bus de motor simulado)

Si no tienes a mano el ODrive S1, la fuente de poder o el motor, puedes compilar un entorno virtual:

1. Selecciona el entorno `esp32-s3-virtual` (o `esp32dev-virtual` / `lora32-virtual`).
2. Sube el código al ESP32.
3. El firmware enlaza [src/motor_bus_virtual.cpp](src/motor_bus_virtual.cpp), que conecta el
   modelo declarativo [src/motor_sim.cpp](src/motor_sim.cpp) al bus del motor. El lazo de
   control, el cloud worker y el failsafe son **exactamente** los mismos que en hardware real:
   - Subida de velocidad de primer orden hacia el envelope de 600 RPM.
   - Corriente de bus ($I_{bus}$) proporcional a la carga hidrodinámica (hasta 9.5 A a 600 RPM).
   - Caída de tensión de bus ($V_{bus} = V_{nom} - I_{bus}\,R$) y temperatura de FET.
   - Transiciones de estados `CLOSED_LOOP_CONTROL` e `IDLE` y desarme por fallo.
4. Podrás mover los mandos en el Dashboard Web y ver las respuestas dinámicas en el monitor serie sin riesgo de quemar hardware.

> El bus virtual se elige por entorno (`build_src_filter`): el binario virtual **no contiene**
> `ODriveUART` y el binario real **no contiene** el simulador. Ver `VIRTUAL_NODES.md` en la raíz.
>
> ⚠️ Nunca ejecutes un nodo real y uno virtual con la **misma `DEVICE_KEY`** a la vez.

---

## 🛠 Consola de Ingeniería y Calibración UART

Para diagnóstico en banco y puesta a punto del ODrive S1:

1. Selecciona el entorno `uart-test` en `platformio.ini`.
2. Sube el firmware y abre el monitor serial a 115200 baud.
3. Escribe `h` o `?` y presiona Enter para ver el menú:
   - **Telemetría y Estado**: `t` (leer telemetría completa), `e` (leer errores), `c` (borrar errores).
   - **Control de Estados**: `cl` (entrar a Closed Loop), `id` (pasar a Idle).
   - **Movimiento**: `v 500` (girar a 500 RPM), `v 0` (frenar), `stream` (streaming continuo).
   - **Configuración de Pines**: Cambia dinámicamente RX/TX y baud rate en caliente sin recompilar.

---

## ⚙️ Configuración de Parámetros (`config.h`)

Principales variables configurables en [src/config.h](src/config.h):

| Parámetro | Valor por Defecto | Descripción |
| :--- | :--- | :--- |
| `WIFI_SSID` / `WIFI_PASSWORD` | `"WifiDev"` / `"rpidev24"` | Credenciales de red local |
| `AQUACONTROL_HOST` | `"tesisutec.vercel.app"` | Servidor endpoint del backend |
| `AQUACONTROL_DEVICE_KEY` | `"ESP32_ODRIVE"` | Clave de autenticación en cabecera HTTP |
| `DEFAULT_TARGET_DO_MG_L` | `7.5f` | Setpoint objetivo de Oxígeno Disuelto (mg/L) |
| `DEFAULT_KP` / `KI` / `KD` | `100.0` / `1.5` / `10.0` | Ganancias iniciales del lazo PID |
| `MIN_MOTOR_RPM` | `0.0f` | Velocidad mínima de rotación |
| `MAX_MOTOR_RPM` | `600.0f` | Velocidad máxima de rotación permitida |
| `FAILSAFE_TIMEOUT_MS` | `35000UL` | Tiempo máximo de espera de paquete de boya (ms) |
| `CONTROL_LOOP_INTERVAL_MS`| `50UL` (20 Hz) | Período de cómputo del lazo en Core 1 |
| `TELEMETRY_PUSH_INTERVAL_MS`| `3000UL` (3 seg) | Frecuencia de envío de lote de telemetría a la nube |

---

## 🌐 Telemetría y Comandos en la Nube

El trabajador de fondo en Core 0 interactúa con las siguientes rutas REST:

1. **Envío de Telemetría**: `POST https://tesisutec.vercel.app/api/telemetry/motor`
   - Envía lotes (*batch*) de hasta 9 muestras de telemetría (RPM objetivo, RPM real, voltaje, corriente, potencia en watts, estado de error).
2. **Recepción de Comandos**: `GET https://tesisutec.vercel.app/api/commands/pending?device_key=ESP32_ODRIVE`
   - Consulta periódica (cada 2 s en Auto, cada 500 ms en Manual).
   - Comandos aceptados: `emergency_stop`, `stop`, `start`, `set_speed`, `set_mode`, `set_config`, `reboot`.
3. **Confirmación**: `POST https://tesisutec.vercel.app/api/commands/[id]/acknowledge`
   - Notifica a la nube que el comando fue ejecutado con éxito por el controlador.
4. **Persistencia en NVS (Flash)**:
   - Cualquier cambio de modo, setpoint o ganancias PID recibido por la nube se guarda automáticamente en la memoria Flash no volátil mediante `Preferences`, persistiendo incluso tras reinicios o cortes de energía.

---

## 🤝 Mantenimiento y Buenas Prácticas

- **Revisión de Errores ODrive**: Si el motor no gira al enviarle comando, revisa el monitor serial: si dice `[ERROR]` o `DISARMED`, es probable que el ODrive haya entrado en fallo por subtensión de la fuente de 24V o error de sensor de motor. Usa la consola `uart-test` para leer `active_errors` y `disarm_reason`.
- **Modificaciones de Código**: No ejecutes llamadas bloqueantes (`delay()`, escrituras pesadas a flash o peticiones HTTP) en el Core 1 (`main.cpp`), ya que degradarán la frecuencia del lazo de 20 Hz y la comunicación UART con el ODrive.
