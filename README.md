# Mini CNC para Dibujo

Sistema CNC completo para dibujo automatizado con 3 motores paso a paso 28-BYJ48, controlado por un STM32F030 y con conectividad WiFi vía ESP32-C3. Soporta archivos G-code (.nc), calibración manual por botones físicos, compensación de backlash y tres métodos de envío de trabajo.

---

## Tabla de Contenidos

- [Arquitectura General](#arquitectura-general)
- [Estructura del Repositorio](#estructura-del-repositorio)
- [Hardware](#hardware)
- [Subsistemas](#subsistemas)
  - [1. Dashboard Web](#1-dashboard-web)
  - [2. ESP32 CNC](#2-esp32-cnc)
  - [3. Mini CNC STM32](#3-mini-cnc-stm32)
- [Formato G-Code Soportado](#formato-g-code-soportado)
- [Flujo de Operación](#flujo-de-operación)
- [Parámetros Configurables](#parámetros-configurables)
- [Compilación y Despliegue](#compilación-y-despliegue)
- [Licencia](#licencia)

---

## Arquitectura General

```
┌──────────┐    HTTP     ┌──────────┐    UART      ┌───────────────┐
│ Navegador │ ────────▶  │  Flask    │ ──────────▶ │   ESP32-C3    │
│  (HTML)   │ ◀────────  │ Dashboard │ ◀────────── │  (WiFi↔UART)  │
└──────────┘    SSE      └──────────┘              └───────┬───────┘
                                                          │ UART 115200
                                              ┌───────────▼───────────┐
                                              │    STM32F030C8T6      │
                                              │  Controlador de CNC   │
                                              └───┬───────┬───────┬───┘
                                                  │       │       │
                                                  ▼       ▼       ▼
                                              Motor X Motor Y Motor Z
```

**Alternativa directa (sin WiFi):**
```
PC (Python) ── USB/Serial ──▶ STM32 ──▶ Motores
```

---

## Estructura del Repositorio

```
Mini-CNC-para-dibujo/
├── README.md                              ← Documentación general
├── .gitignore
│
├── dashboard/                             ← Aplicación web Flask
│   ├── app.py                                 Servidor HTTP + proxy al ESP32
│   └── templates/
│       └── index.html                         Interfaz de usuario
│
├── ESP32-CNC/                             ← Firmware ESP32-C3 (ESP-IDF v5.5)
│   ├── CMakeLists.txt
│   ├── sdkconfig                              Configuración de proyecto ESP-IDF
│   ├── .devcontainer/                         Entorno Docker para desarrollo
│   └── main/
│       ├── main.c                             Máquina de estados principal
│       ├── wifi_manager.c/.h                  Conexión WiFi (STA)
│       ├── http_manager.c/.h                  Servidor HTTP (puerto 80)
│       ├── gcode_parser.c/.h                  Almacenamiento SPIFFS + parser
│       └── uart_manager.c/.h                  Transmisión UART con handshake
│
└── Mini-CNC-para-dibujo-v2.1/            ← Firmware STM32F030 + script Python
    ├── README.md                              Documentación técnica detallada
    ├── CNC.py                                 Script Python emisor directo
    ├── CMakeLists.txt                         Sistema de compilación
    ├── CMakePresets.json
    ├── STM32F030XX_FLASH.ld                   Linker script
    ├── startup_stm32f030x8.s                  Vector de interrupciones
    ├── Circle.nc                              G-code de ejemplo (círculo)
    ├── Core/
    │   ├── Inc/                               Headers del firmware
    │   │   ├── main.h                             Pines GPIO (CubeMX)
    │   │   ├── CNC.h                              Orquestador + Bresenham
    │   │   ├── Stepping-sequence.h                Driver motores (8 pasos)
    │   │   ├── Gcode-parser.h                     Parser de coordenadas
    │   │   ├── Buttons.h                          Botones de calibración
    │   │   └── Backlash.h                         Compensación de backlash
    │   └── Src/                               Implementaciones
    │       ├── main.c                             Punto de entrada
    │       ├── CNC.c                              Lógica principal (340 líneas)
    │       ├── Stepping-sequence.c                Secuencia half-stepping
    │       ├── Gcode-parser.c                     Extracción mm → pasos
    │       ├── Buttons.c                          Interrupciones EXTI
    │       └── Backlash.c                         Cálculo de compensación
    ├── Drivers/                              HAL + CMSIS (ST)
    └── cmake/                                Toolchains GCC ARM + STM Clang
```

---

## Hardware

| Componente | Especificación |
|---|---|
| **MCU Principal** | STM32F030C8T6 (Cortex-M0, 64KB Flash, 8KB RAM, 48 MHz) |
| **MCU WiFi** | ESP32-C3 (RISC-V 160 MHz, WiFi 802.11 b/g/n, 2 MB Flash) |
| **Motores** | 3x 28-BYJ-48 (paso a paso unipolar, 4096 pasos/revolución) |
| **Botones** | 6 botones físicos (X+, X-, Y+, Y-, Z↑, Z↓) |
| **Comunicación** | UART 115200 bps 8N1 |
| **Alimentación** | 5V / 12V (según drivers de motor) |

### Mapa de Pines STM32F030

| Pin | Función | Pin | Función |
|---|---|---|---|
| PA6-PA7, PB0-PB1 | Motor Z (bobinas 1-4) | PA8-PA11 | Motor X (bobinas 1-4) |
| PB12-PB15 | Motor Y (bobinas 1-4) | PB6-PB7 | USART1 TX/RX |
| PA0 | Botón Z↓ (EXTI0) | PA1 | Botón Z↑ (EXTI1) |
| PB3 | Botón X- (EXTI3) | PB4 | Botón Y+ (EXTI4) |
| PB5 | Botón Y- (EXTI5) | PA15 | Botón X+ (EXTI15) |
| PB9 | LED indicador | PA2-PA3 | USART2 TX/RX |

---

## Subsistemas

### 1. Dashboard Web

**Tecnologías:** Python 3, Flask, HTML5/CSS3, JavaScript (Fetch API, SSE)

Servidor web local que funciona como puente HTTP entre el navegador y el ESP32.

**Endpoints:**
- `GET /` — Interfaz de usuario oscura con selector de archivos, barra de progreso y log
- `POST /upload` — Recibe archivo `.nc`, lo limpia y reenvía al ESP32 por HTTP
- `POST /start` — Inicia el trabajo en el ESP32
- `GET /status` — SSE: progreso en tiempo real hacia el navegador
- `GET /log` — Historial de eventos (últimos 200 mensajes)

**Ejecución:**
```bash
cd dashboard
pip install flask requests
python3 app.py
# Abrir http://localhost:5000 en el navegador
```

**Flujo:** `Navegador → Flask → HTTP POST → ESP32 → UART → STM32`

---

### 2. ESP32 CNC

**Tecnologías:** C, ESP-IDF v5.5.4, FreeRTOS, SPIFFS, LWIP

Firmware para el ESP32-C3 que actúa como puente WiFi↔UART.

**Módulos:**
- `wifi_manager` — Conexión WiFi en modo STA (SSID: `LA_OCULTA`). 10 reintentos máximo
- `http_manager` — Servidor HTTP en puerto 80. Endpoints `/upload` (recibe G-code en SPIFFS) y `/start` (activa envío UART)
- `gcode_parser` — SPIFFS para almacenamiento persistente + array RAM de 2000 líneas × 64 chars
- `uart_manager` — UART0 (GPIO20/21), 115200 bps, tarea FreeRTOS prioridad 12 para recepción asíncrona

**Máquina de Estados:**
```
WAIT_START → SEND_LINE → WAIT_OK → SEND_LINE → ... → DONE → WAIT_START
```

**Protocolo:** Cada línea requiere respuesta `ok` del STM32 antes de enviar la siguiente. Al finalizar, envía `f` para apagar motores.

**Compilación:**
```bash
cd ESP32-CNC
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

---

### 3. Mini CNC STM32

**Tecnologías:** C11 bare-metal, STM32CubeMX HAL, CMake + Ninja, GCC ARM

Firmware principal de control CNC con interpolación Bresenham y calibración manual.

**Módulos Custom:**

| Módulo | Descripción |
|---|---|
| `CNC` | Orquestador central: máquina de estados, Bresenham, protocolo UART |
| `Stepping-sequence` | Driver de motores con secuencia half-stepping de 8 pasos |
| `Gcode-parser` | Extrae coordenadas X, Y (mm→pasos) y estado Z (UP/DOWN) |
| `Buttons` | Detección de 6 botones vía EXTI con prioridad Z > Y > X |
| `Backlash` | Compensación por cambio de dirección (pasos extra configurables) |

**Máquina de Estados:**
```
                    ┌──────────────────────────────┐
                    │           IDLE                │
                    │  • Espera línea por UART      │
                    │  • Chequea botones            │
                    └──────┬───────────┬────────────┘
              rx_ready    │           │ Buttons_AnyPressed()
                          ▼           ▼
               ┌─────────────────┐   ┌──────────────────────┐
               │   MOVING_Z      │   │   CALIBRATING         │
               │ • Pasos fijos   │   │ • Mover mientras      │
               │   subir/bajar   │   │   esté presionado     │
               └────────┬────────┘   └──────────────────────┘
               Z terminado│               │ botón soltado
                          ▼               ▼
               ┌─────────────────┐   ┌──────────┐
               │   MOVING_XY     │   │   IDLE   │
               │ • Bresenham     │   └──────────┘
               │   paso a paso   │
               └────────┬────────┘
               XY terminado│ → envía "ok\r\n"
                          ▼
                     ┌──────────┐
                     │   IDLE   │
                     └──────────┘
```

**Algoritmo Bresenham:** Interpolación de línea con floats para movimiento suave en XY. Detecta cambios de dirección para disparar compensación de backlash.

**Compilación:**
```bash
cd Mini-CNC-para-dibujo-v2.1
cmake --preset Debug -B build/Debug
cmake --build build/Debug
# Flash con ST-Link, OpenOCD o STM32CubeProgrammer
```

---

## Formato G-Code Soportado

```
G0 X10.5 Y20.3              → mover lápiz a (X=10.5mm, Y=20.3mm)
G0 Z-2.0                    → bajar lápiz (Z < -1.0)
G0 Z2.0                     → subir lápiz (Z > 1.0)
G0 X10.5 Y20.3 Z-2.0        → bajar lápiz y mover a (X,Y)
```

- Se ignoran líneas que empiezan con `;` o `(` (comentarios)
- La letra `G` y el número `0` son opcionales; solo se buscan `X`, `Y`, `Z`
- Z entre -1.0 y 1.0 se ignora (sin cambio de estado)

---

## Flujo de Operación

### Modo WiFi (Dashboard → ESP32 → STM32)

1. Encender STM32 + ESP32
2. Conectarse a la red WiFi del ESP32 (`LA_OCULTA`)
3. Abrir dashboard en navegador (`http://localhost:5000`)
4. Calibrar posición inicial con botones físicos
5. Seleccionar archivo `.nc` y hacer clic en **Subir al ESP32**
6. Hacer clic en **Iniciar trabajo**
7. Monitorear progreso en tiempo real

### Modo Directo (Python → STM32)

```bash
cd Mini-CNC-para-dibujo-v2.1
pip install pyserial
python3 CNC.py Circle.nc /dev/ttyUSB0
```

---

## Parámetros Configurables

### STM32 (Macros)

| Macro | Archivo | Default | Descripción |
|---|---|---|---|
| `STEPS_PER_MM` | `Gcode-parser.h` | `204.8` | Pasos por milímetro (4096÷20) |
| `CNC_Z_STEPS` | `CNC.h` | `100` | Pasos fijos para subir/bajar lápiz |
| `CNC_STEP_DELAY_MS` | `CNC.h` | `4` | Milisegundos entre pasos de motor |
| `CNC_RX_BUF_SIZE` | `CNC.h` | `64` | Buffer de recepción UART |
| `BACKLASH_X_STEPS` | `Backlash.h` | `0` | Compensación de backlash en X |
| `BACKLASH_Y_STEPS` | `Backlash.h` | `0` | Compensación de backlash en Y |
| `STEPPER_DELAY_MS` | `Stepping-sequence.h` | `4` | Tiempo mínimo entre pasos |
| `GCODE_MM_PER_REV` | `Gcode-parser.h` | `20.0` | mm por revolución (mecánica) |

### ESP32

| Parámetro | Archivo | Default | Descripción |
|---|---|---|---|
| `WIFI_SSID` | `wifi_manager.h` | `LA_OCULTA` | Nombre de red WiFi |
| `WIFI_PASS` | `wifi_manager.h` | — | Contraseña WiFi |
| `MAX_RETRY` | `wifi_manager.c` | `10` | Reintentos de conexión |

### Dashboard

| Parámetro | Archivo | Default | Descripción |
|---|---|---|---|
| `ESP32_URL` | `app.py` | `http://192.168.4.1` | IP del ESP32 en modo AP |
| Puerto | `app.py` | `5000` | Puerto del servidor Flask |

---

## Compilación y Despliegue

### Requisitos

- **ESP32-C3:** [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/)
- **STM32:** `arm-none-eabi-gcc`, CMake ≥ 3.20, Ninja
- **Python:** Python 3.8+, Flask, requests, pyserial

### Flasheo

```bash
# STM32 (ST-Link v2)
st-flash write build/Debug/CNC_v2.0.bin 0x08000000

# ESP32-C3
cd ESP32-CNC && idf.py -p /dev/ttyUSB0 flash
```

---

## Archivos de Ejemplo

- **`Circle.nc`** — Círculo de ~48mm de diámetro con 66 puntos, velocidad de avance F762.0
- Ubicación: `Mini-CNC-para-dibujo-v2.1/Circle.nc`

---

*Proyecto académico — Semestre VII — Control Numérico Computarizado*
