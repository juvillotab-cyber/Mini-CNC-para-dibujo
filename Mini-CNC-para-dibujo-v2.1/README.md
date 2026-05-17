# Mini CNC para Dibujo — Firmware STM32F030

Proyecto de control CNC para una máquina de dibujo con tres motores paso a paso 28-BYJ48 y MCU STM32F030. Recibe instrucciones G-code por UART a 115200 bps, usa Bresenham para interpolación XY y botones físicos para calibración manual.

## Arquitectura del proyecto

```
Mini-CNC-para-dibujo-v2.1/
├── CNC.py                          # Script Python para enviar archivos .nc
├── CMakeLists.txt                  # Build system (CMake)
├── .clangd                         # Configuración de clangd
├── Core/
│   ├── Inc/                        # Headers
│   │   ├── main.h                  # Pines (auto-generado por CubeMX)
│   │   ├── CNC.h                   # Orquestador principal
│   │   ├── Stepping-sequence.h     # Driver de secuencia de 8 pasos
│   │   ├── Gcode-parser.h          # Parser de líneas G-code
│   │   ├── Buttons.h               # Manejo de botones de calibración
│   │   └── Backlash.h              # Compensación de backlash
│   └── Src/                        # Implementaciones
│       ├── main.c                  # Punto de entrada (modificado)
│       ├── CNC.c                   # Máquina de estados y Bresenham
│       ├── Stepping-sequence.c     # Secuencia de 8 pasos para motores
│       ├── Gcode-parser.c          # Extracción de coordenadas mm → pasos
│       ├── Buttons.c               # Polling e interrupciones de botones
│       └── Backlash.c              # Compensación por cambio de dirección
```

---

## Archivos creados — explicación detallada

### 1. `Stepping-sequence.h` / `Stepping-sequence.c`

**Propósito:** Driver de bajo nivel que controla la secuencia de pasos de los motores 28-BYJ48.

**Cómo funciona:**

- Define la secuencia de 8 pasos (`SEQ_8`) como matriz constante:

```
Paso 0: {1,0,0,0}    Paso 4: {0,0,1,0}
Paso 1: {1,1,0,0}    Paso 5: {0,0,1,1}
Paso 2: {0,1,0,0}    Paso 6: {0,0,0,1}
Paso 3: {0,1,1,0}    Paso 7: {1,0,0,1}
```

- Cada motor se representa con `StepperMotor_t`: 4 pines (IN1-IN4), dirección, índice de secuencia y posición absoluta en pasos.
- `StepperMotor_Init()` asigna los 4 pines GPIO del motor.
- `StepperMotor_SetDirection()` cambia la dirección (+1 avance, -1 retroceso). Guarda la dirección anterior en `last_dir` para que el módulo de backlash pueda detectar cambios.
- `StepperMotor_Step()` avanza o retrocede un paso en la secuencia usando la fórmula `(seq_idx + 8 + direction) % 8`. Aplica el nuevo estado a los 4 pines con `HAL_GPIO_WritePin`.
- `StepperMotor_Release()` apaga todas las bobinas (pines en LOW) para liberar el motor.
- Macro configurable: `STEPPER_DELAY_MS = 4` (tiempo mínimo entre pasos).

---

### 2. `Gcode-parser.h` / `Gcode-parser.c`

**Propósito:** Recibe una línea de texto (formato G-code) y extrae las coordenadas X, Y en milímetros y el estado deseado de Z (subir/bajar).

**Cómo funciona:**

- `GCode_Parse(line, cmd)`:
  1. Copia la línea a un buffer local (`strncpy`).
  2. Busca los caracteres `X`, `Y`, `Z` con `strchr`.
  3. Convierte el valor numérico con `atof`.
  4. **X/Y:** Si están presentes, `has_xy = true`. Los valores en mm se almacenan en `target_x_mm` / `target_y_mm`.
  5. **Z:** Interpretación especial (no usa mm para pasos):
     - `Z > 1` → `Z_STATE_UP` (subir el lápiz)
     - `Z < -1` → `Z_STATE_DOWN` (bajar el lápiz)
     - Entre -1 y 1 → se ignora (sin cambio)
     - Si el estado ya es el deseado, no se genera movimiento.
- `GCode_MmToSteps(mm)` convierte milímetros a pasos: `mm × STEPS_PER_MM`.
- Macros configurables:
  - `GCODE_STEPS_PER_REV = 4096` (pasos por revolución del motor)
  - `GCODE_MM_PER_REV = 20.0` (mm por revolución — depende de tu mecánica)
  - `STEPS_PER_MM` se calcula como `4096 / 20.0 = 204.8` pasos/mm

---

### 3. `Buttons.h` / `Buttons.c`

**Propósito:** Detectar qué botón de calibración está presionado. Soporta tanto polling como notificación por interrupción EXTI.

**Cómo funciona:**

- Define 6 botones: `BTN_XP`, `BTN_XN`, `BTN_YP`, `BTN_YN`, `BTN_Z1` (Z arriba), `BTN_Z2` (Z abajo).
- `Buttons_IsPressed(btn)` lee el pin GPIO directamente con `HAL_GPIO_ReadPin`. Con `PULLUP` y `IT_FALLING`, el pin está HIGH en reposo y LOW cuando se presiona. Retorna `true` si el pin está en `GPIO_PIN_RESET` (presionado).
- `Buttons_GetPressed()` escanea los 6 botones en orden de prioridad (Z > Y > X) y retorna el primero que esté presionado, o `BTN_NONE`.
- `Buttons_AnyPressed()` retorna `true` si cualquier botón está presionado. Es la función usada por el orquestador para detectar si debe entrar en modo calibración.
- `Buttons_EXTI_Callback(pin)` es llamada desde los handlers de interrupción EXTI (`stm32f0xx_it.c`). Actualiza flags internos (útil para futuros usos de detección por eventos).

**Flujo de calibración:**
1. El usuario presiona un botón físico.
2. El EXTI detecta el flanco de bajada y dispara la interrupción.
3. En el `while(1)` de `CNC_Run`, se llama a `Buttons_AnyPressed()`.
4. Si retorna `true`, el estado pasa a `CNC_STATE_CALIBRATING`.
5. Mientras el botón esté presionado, se mueve el motor correspondiente un paso cada `BUTTON_CAL_STEP_DELAY_MS = 4 ms`.
6. Al soltar, se liberan los motores y se vuelve a `IDLE`.
7. Si había un movimiento G-code en curso, se aborta inmediatamente.

---

### 4. `Backlash.h` / `Backlash.c`

**Propósito:** Compensar el juego mecánico (backlash) de los engranajes cuando se invierte la dirección de movimiento en X o Y.

**Cómo funciona:**

- `Backlash_Init()` inicializa los campos: `last_dir_x = 0`, `last_dir_y = 0`, `pending_x = 0`, `pending_y = 0`.
- `Backlash_Update()` detecta si la nueva dirección difiere de la anterior:
  - Si `last_dir_x != new_dir_x` → agrega `BACKLASH_X_STEPS` pasos extra en la nueva dirección.
  - Si `last_dir_y != new_dir_y` → agrega `BACKLASH_Y_STEPS` pasos extra en la nueva dirección.
- La compensación real ocurre en `CNC.c`, dentro de `bres_load()`: justo antes de cargar el algoritmo de Bresenham, se verifica si la dirección cambió respecto a la anterior. Si cambió, se añaden los pasos de backlash al destino (`tx += new_dir * BACKLASH_X_STEPS`), expandiendo la distancia total que el motor debe recorrer. Esos pasos extra absorben el juego mecánico.
- Macros configurables:
  - `BACKLASH_X_STEPS = 0` (pon el valor que necesites según tus engranajes)
  - `BACKLASH_Y_STEPS = 0`

---

### 5. `CNC.h` / `CNC.c`

**Propósito:** Orquestador central. Contiene la máquina de estados, el algoritmo de Bresenham, y la lógica de comunicación UART.

**Estructura `CNC_t`:**

```
CNC_t {
    motor_x, motor_y, motor_z   // StepperMotor_t — estado de cada motor
    state                        // CNC_State_t — estado actual de la máquina
    bres                         // CNC_Bresenham_t — parámetros de Bresenham
    cur_x_steps, cur_y_steps     // posición actual en pasos (float)
    target_x_steps, target_y_steps // destino en pasos
    current_z_state, target_z_state // estado actual/deseado del eje Z
    z_steps_left                 // pasos pendientes de Z
    backlash                     // Backlash_t — compensación
    last_tick                    // timestamp del último paso (para rate-limiting)
    rx_buf, rx_byte, rx_idx      // buffer de recepción UART
    rx_ready, rx_terminate       // flags de control
}
```

**Máquina de estados:**

```
                   ┌──────────────────────────────┐
                   │         IDLE                  │
                   │  - espera línea por UART      │
                   │  - chequea botones            │
                   └──────┬───────────┬────────────┘
                          │           │
              rx_ready    │           │ Buttons_AnyPressed()
                          ▼           ▼
              ┌─────────────────┐   ┌──────────────────────┐
              │   MOVING_Z      │   │   CALIBRATING        │
              │ - pasos fijos   │   │ - mover mientras     │
              │   subir/bajar   │   │   esté presionado    │
              └────────┬────────┘   └──────────┬───────────┘
                       │ Z terminado            │ botón soltado
                       ▼                        ▼
              ┌─────────────────┐          ┌──────────┐
              │   MOVING_XY     │          │   IDLE   │
              │ - Bresenham     │          └──────────┘
              │   paso a paso   │
              └────────┬────────┘
                       │ XY terminado → envía "ok\r\n"
                       ▼
                   ┌──────────┐
                   │   IDLE   │
                   └──────────┘
```

**Funciones clave:**

- **`CNC_Init()`** — Inicializa los 3 motores con sus pines (leídos de `main.h`), configura el buffer UART, libera todos los motores, y arranca la recepción UART por interrupción (`HAL_UART_Receive_IT`).

- **`CNC_UART_ISR()`** — Callback de recepción UART. Cada byte recibido se acumula en `rx_buf`. Al recibir `\n` o `\r`, se pone el terminador nulo, se marca `rx_ready = true` y se reinicia el índice. Luego se re-arma la interrupción para el siguiente byte.

- **`process_command()`** — Procesa la línea completa:
  1. Si la línea es `"f"`, marca `rx_terminate = true` (apagar motores).
  2. Si no, llama a `GCode_Parse()` para extraer X, Y, Z.
  3. Si Z requiere cambio y el estado es diferente al actual, configura `MOVING_Z`.
  4. Si hay XY sin Z, carga Bresenham y va a `MOVING_XY`.
  5. Si no hay ni Z ni XY, responde `"ok\r\n"` inmediatamente.

- **`bres_load()`** — Prepara el algoritmo de Bresenham:
  1. Guarda la dirección actual de X e Y (`old_dir`).
  2. Calcula `dx = target - cur`, `dy = target - cur`.
  3. Detecta cambio de dirección: si `old_dir != new_dir`, añade pasos de backlash al destino local.
  4. Configura signos (`sx`, `sy`), valores absolutos (`dx`, `dy`).
  5. Inicializa el error de Bresenham: si `dx >= dy`, `err = 2*dy - dx`; si no, `err = 2*dx - dy`.
  6. `steps_left` = max(dx, dy).

- **`bres_step()`** — Ejecuta un paso del algoritmo de Bresenham. Retorna `true` cuando termina.

- **`CNC_Run()`** — Bucle principal, llamado desde `main()`:
  1. Si `rx_terminate`, libera motores y retorna.
  2. Si hay botón presionado, fuerza `CALIBRATING`.
  3. Switch sobre `state`:
     - `IDLE`: si `rx_ready`, procesa comando.
     - `MOVING_Z`: un paso cada `CNC_STEP_DELAY_MS` ms. Al terminar Z, si hay XY pendiente salta a `MOVING_XY`, si no envía `ok`.
     - `MOVING_XY`: un paso de Bresenham cada `CNC_STEP_DELAY_MS` ms. Al terminar, envía `ok` y libera motores.
     - `CALIBRATING`: delega en `run_calibration()`.

**Macros configurables:**
- `CNC_RX_BUF_SIZE = 64` — tamaño del buffer de recepción UART.
- `CNC_STEP_DELAY_MS = 4` — milisegundos entre pasos.
- `CNC_Z_STEPS = 100` — pasos fijos para subir/bajar el lápiz.

---

### 6. `CNC.py`

**Propósito:** Script Python que lee un archivo `.nc` con instrucciones G-code y las envía línea por línea al STM32 por UART.

**Cómo funciona:**

1. Abre el puerto serie especificado (default `/dev/ttyUSB0`) a 115200 bps.
2. Lee el archivo `.nc`, ignorando líneas vacías y comentarios (`;` o `(`).
3. Envía cada línea seguida de `\n` y espera la respuesta `ok` del firmware.
4. Al terminar todas las líneas, envía `"f\n"` para ordenar al firmware que apague los motores.
5. Cierra el puerto serie.

**Uso:**
```bash
python3 CNC.py archivo.nc /dev/ttyUSB0
```

---

## Archivos modificados

### `main.c`
- **Inclusión:** `#include "CNC.h"`
- **Variable global:** `CNC_t cnc` (contexto de la CNC).
- **Callback UART:** `HAL_UART_RxCpltCallback()` redirige a `CNC_UART_ISR(&cnc)` cuando la interrupción es de USART1.
- **Inicialización:** `CNC_Init(&cnc)` después de `MX_USART1_UART_Init()`.
- **Bucle principal:** `while(1) { CNC_Run(&cnc); }`.

### `usart.c`
- Baud rate cambiado de 38400 a **115200** en USART1 y USART2.

### `gpio.c`
- Modo de interrupción de botones cambiado de `GPIO_MODE_IT_RISING` a `GPIO_MODE_IT_FALLING` para detectar la pulsación (flanco de bajada con pull-up).

### `stm32f0xx_it.c`
- Inclusión de `Buttons.h`.
- Llamadas a `Buttons_EXTI_Callback(pin)` **antes** de `HAL_GPIO_EXTI_IRQHandler(pin)` en cada handler EXTI (para detectar el pin antes de que se limpie el flag).

### `CMakeLists.txt`
- Agregados los 5 nuevos fuentes: `CNC.c`, `Stepping-sequence.c`, `Gcode-parser.c`, `Buttons.c`, `Backlash.c`.

### `.clangd`
- Agregado `UnusedIncludes: None` para suprimir warnings de includes no usados.

---

## Parámetros configurables

| Macro | Archivo | Default | Descripción |
|-------|---------|---------|-------------|
| `STEPPER_DELAY_MS` | Stepping-sequence.h | `4` | Tiempo entre pasos del motor |
| `GCODE_STEPS_PER_REV` | Gcode-parser.h | `4096` | Pasos por revolución (28BYJ-48) |
| `GCODE_MM_PER_REV` | Gcode-parser.h | `20.0` | mm por revolución (mecánica) |
| `GCODE_Z_UP_THRESHOLD` | Gcode-parser.h | `1.0` | Umbral para interpretar Z como "subir" |
| `GCODE_Z_DOWN_THRESHOLD` | Gcode-parser.h | `-1.0` | Umbral para interpretar Z como "bajar" |
| `CNC_Z_STEPS` | CNC.h | `100` | Pasos fijos para movimiento Z |
| `CNC_RX_BUF_SIZE` | CNC.h | `64` | Tamaño buffer recepción UART |
| `BACKLASH_X_STEPS` | Backlash.h | `0` | Pasos de compensación backlash X |
| `BACKLASH_Y_STEPS` | Backlash.h | `0` | Pasos de compensación backlash Y |
| `BUTTON_CAL_STEP_DELAY_MS` | Buttons.h | `4` | Tiempo entre pasos en calibración |

---

## Flujo completo de operación

1. **Encendido:** El STM32 inicializa periféricos, motores y UART. Entra en `IDLE`.
2. **Calibración:** El usuario usa los 6 botones para posicionar el lápiz manualmente.
3. **Envío de archivo:** Desde la PC se ejecuta `python3 CNC.py dibujo.nc`.
4. **Recepción UART:** Cada línea G-code es recibida carácter por carácter. Al recibir `\n`, se marca `rx_ready`.
5. **Parseo:** `GCode_Parse()` extrae X (mm→pasos), Y (mm→pasos) y Z (UP/DOWN).
6. **Movimiento Z:** Si Z necesita cambiar, se mueven `CNC_Z_STEPS` pasos fijos para subir o bajar el lápiz.
7. **Movimiento XY:** Se ejecuta Bresenham paso a paso (un paso cada 4 ms) interpolando X e Y.
8. **Respuesta:** Al terminar XY, se envía `"ok\r\n"` por UART. El script Python recibe el ok y envía la siguiente línea.
9. **Finalización:** Al terminar el archivo, Python envía `"f\n"`. El firmware libera los 3 motores.
10. **Interrupción por botón:** En cualquier momento, si se presiona un botón, el movimiento G-code se aborta y se entra en modo calibración.

---

## Formato G-code soportado

```
G0 X10.5 Y20.3          → mover a (X=10.5mm, Y=20.3mm)
G0 Z-2.0                → bajar lápiz
G0 Z2.0                 → subir lápiz
G0 X10.5 Y20.3 Z-2.0    → bajar lápiz y mover a (X,Y)
```

- Se ignoran líneas que empiezan con `;` o `(` (comentarios).
- La letra `G` y el número `0` son opcionales; el parser solo busca `X`, `Y`, `Z`.
