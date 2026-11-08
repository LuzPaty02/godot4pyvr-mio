# Reporte de workflow y estado del proyecto

**Corte:** 08-11-2026  
**Rama:** `luz-dev-2`  
**Objetivo:** flujo local de simulación: entrada XR/controladores → normalización → mapeo a `q` → cinemática directa STRAS → geometría → visualización en Godot. Velocidades, IK y control físico quedan fuera del alcance.

## Resumen ejecutivo

El proyecto funciona como una simulación local de STRAS. El visor XR y sus controladores son la única entrada física; la salida de la simulación nunca se envía a un robot real. La lógica de Python recibe snapshots de Godot, normaliza y valida los datos, convierten offsets respecto a una pose neutral en una configuración articulada `q`, calcula la forma del modelo y devuelve geometría para renderizarla en Godot.

La arquitectura actual queda dividida en dos capas:

- Godot: captura XR, serialización UDP y render de la escena/overlay.
- Python: validación de entrada, mapeo a `q`, cinemática directa y envío de geometría.

## Convención de entrada y normalización

El punto más importante del flujo es que la entrada no se trata como `q` directa, sino como señales normalizadas y luego convertidas a la configuración virtual del robot.

### 1. Normalización de controladores

- `trigger` y `grip` se normalizan a `[0, 1]`.
- Los sticks primarios/secundarios se preservan como valores firmados en `[-1, 1]` y solo se clampan, no se reescalan a `[0, 1]`.
- La entrada de prueba `joints` también usa una convención normalizada: cada una de las 10 etiquetas debe existir y cada valor debe estar en `[0, 1]`.
- Esta entrada normalizada se convierte a rango virtual real mediante `normalized_joints_to_q(...)` antes de entrar a cinemática directa.

Esto hace que el sistema separe claramente dos conceptos:

- entrada de control: valores normalizados o firmados para interacción,
- estado del robot: `q` real en su rango virtual de simulación.

### 2. HUD teleop y estado normalizado

La geometría emitida a Godot incluye un campo `hud` que reporta los brazos en forma normalizada para depuración y visualización.

- Los valores del HUD cubren `BL`, `RL`, `TL`, `BR`, `RR` y `TR`.
- Cada articulación se normaliza contra su rango virtual y se clampa a `[0, 1]`.
- El HUD es una vista auxiliar del estado del robot; no reemplaza la vista VR real del usuario.
- El overlay de cámara/endoscopio es un panel visual de simulación, no la vista principal del casco.

## Flujo operativo actual

```mermaid
flowchart LR
    A[Controladores + cámara XR] --> B[Godot: XR capture + UDP JSON]
    B -->|UDP 127.0.0.1:9000| C[InputServer / Mailbox / SeqFilter]
    C --> D[ControllerToConfiguration + RobotLimits]
    D --> E[VirtualFollower + q]
    E --> F[Kinematics.shape(q)]
    F --> G[Geometry packet + hud normalized]
    G -->|UDP 127.0.0.1:9100| H[Godot viewer / floating simulation display]
    H --> I[HUD teleop + endoscope preview overlay]
```

El vector `q` usa el orden:

`BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET`

Las entradas son offsets absolutos respecto a una pose neutral; no se integra velocidad ni se usa IK.

## Etapas del pipeline

### 1. Captura XR en Godot

- `main.gd` y la escena principal toman las poses del visor y los controladores.
- El flujo de scene/viewport de VR se usa como referencia del usuario.
- Los datos de control y pose se serializan para enviar al backend Python.

### 2. Transporte Godot → Python

- El paquete JSON se manda por UDP a `127.0.0.1:9000`.
- La validación incluye secuencia, versión de protocolo, integridad del paquete y detección de paquetes atrasados o reinicios de sesión.
- El backend usa `Mailbox` para quedarnos con el último frame vigente, no con una cola desfasada.

### 3. Mapeo a configuración virtual del robot

El mapeo se realiza con `ControllerToConfiguration` y `RobotLimits`.

- Offset lateral y vertical de la mano define curva y ángulo del plano de flexión.
- El giro relativo de muñeca alrededor del eje local -Z se suma al ángulo del plano, con sensibilidad angular 1:1 y simétrica.
- Desplazamiento longitudinal controla inserción del brazo alrededor de una posición inicial visible de 50 mm (±20 mm de recorrido simétrico).
- Movimiento de la cámara XR mapea `EBH`, `EBV`, `ET` y `ER`.
- La primera pose capturada se usa como referencia neutra; todo el resto trabaja en offsets desde allí.
- A nivel de detalles de percepción: el eje vertical de Godot usa +Y y se corrige al mapearlo para mantener la convención del modelo calibrado.

### 4. Cálculo de cinemática directa

- La configuración virtual `q` entra en `Kinematics.shape(...)`.
- El resultado es `ShapeOutput`, con geometría para miembros izquierdos, derechos y endoscopio.
- El simulador mantiene un `VirtualFollower` local que refleja el estado actualizado del modelo sin actuar sobre hardware físico.

### 5. Salida de geometría a Godot

- `GodotGeometrySender` arma un JSON con `frame`, `right`, `left`, `endoscope`, secciones y metadatos de cámara.
- El paquete se envía a `127.0.0.1:9100`.
- La geometría se usa para dibujar tubos y la vista de endoscopio en la escena.
- El campo `hud` usa dimensiones normalizadas para UI y diagnóstico.

### 6. Visualización y overlay

- La vista principal del usuario sigue siendo la del casco XR.
- La pantalla flotante con la imagen del endoscopio es un overlay visual secundario.
- Si está habilitado, el preview de endoscopio usa una cámara adicional; no reemplaza la vista real del usuario ni la experiencia del headset.

## Archivo y módulos clave

| Área | Archivos relevantes |
|---|---|
| Escena principal y transporte Godot | `main.gd`, `main.tscn` |
| Servidor UDP y validación | `robot_control/stras/input/transport_and_validation/input_server.py`, `latest_slot.py`, `input_protocol.py` |
| Simulador y runner FK | `robot_control/stras_vr_simulator.py` |
| Mapeo de entrada y límites | `robot_control/stras/input/mapping_and_limits/*.py` |
| Kinemática y modelo | `robot_control/stras/model/*.py`, `module_main/model_utilities/*.py` |
| Visualización Godot y HUD | `robot_control/stras/visualization/*.gd`, `stras_camera_display.gd` |
| Pruebas | `robot_control/tests/*.py` |

## Estado real del proyecto

### Lo que sí está funcionando

- Captura XR desde Godot.
- Serialización y validación UDP local.
- Reader/normalizador de inputs del controlador.
- Mapeo de offsets a `q` virtual.
- Emisión de geometría de endoscopio y brazos a Godot.
- HUD normalizado de estado articular.
- Simulación local sin hardware real.

### Lo que sigue siendo una restricción

- La visualización del endoscopio es un preview y no sustituye la vista principal del usuario.

## Estado de verificación

La revisión del proyecto en este entorno evidenció que la suite de Python no corre sin preparar el entorno correcto:

- `pytest` en el entorno base falló por dependencias externas del sistema (`launch_testing` / `lark`).
- con `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, la colección siguió hasta un error de dependencia del modelo: falta `liegroups`.

Esto indica que la verificación real del proyecto requiere activar el entorno `pyvr` y asegurar que sus dependencias científicas estén instaladas antes de ejecutar la batería de tests.

## Lectura recomendada

El recorrido principal para entender el sistema es:

`main.gd` → UDP 9000 → `InputServer` → `ControllerToConfiguration` → `VirtualFollower` → `Kinematics.shape` → geometría UDP 9100 → `stras_camera_display.gd` / HUD

## Conclusión

La implementación actual está enfocada en una simulación local visual y de prueba, con una clara separación entre:

- señal de entrada del usuario,
- valores normalizados de control,
- configuración virtual del robot `q`,
- render de geometría y HUD teleop.

Este enfoque es apropiado para un entorno de desarrollo VR sin hardware físico, y resulta especialmente útil para validar el mapeo del input, la cinemática directa y la visualización del endoscopio antes de pasar a una integración más real con robot físico o cirugía teleoperada.
