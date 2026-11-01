# Reporte de workflow y estado del proyecto

**Corte:** 30-10-2026  
**Rama:** `luz-dev-2` (alineada con `origin/luz-dev-2`)  
**Alcance objetivo:** entrada normalizada → configuración articular STRAS → cinemática directa → geometría de brazos/endoscopio → visualización Godot. Velocidades, IK y actuación física quedan fuera de alcance.

## Resumen ejecutivo

El objetivo funcional es una simulación local: recibir poses VR, convertir sus offsets respecto de una pose neutral en `q`, calcular las formas STRAS y enviar la geometría a Godot. No se integra velocidad, no se resuelve IK y no se conecta ni envía comandos a hardware.

El runner activo carga los archivos de configuración del modelo, recibe snapshots XR, aplica el mapeo explícito de `robot_control/stras/input_mapping.py`, limita `q` a los límites virtuales de simulación, calcula `Kinematics.shape(q)` y envía la geometría a Godot. También conserva el modo de entrada `joints` normalizado para pruebas sin headset.

## Flujo de datos de alto nivel

```mermaid
flowchart LR
    A[Controladores y cámara XR] --> B[Captura de poses y dt en Godot]
    B -->|UDP localhost:9000| C[Python: offsets respecto a neutral]
    C --> D[Mapeo y límites virtuales: q]
    D --> E[StrasModel.computeShape(q)]
    E --> F[Geometría: brazo derecho, brazo izquierdo, endoscopio]
    F -->|UDP geometría| G[Godot: stras_receiver.gd]
    G --> H[Visualización]
```

El vector `q` usa el orden `[BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET]`. Las entradas son offsets absolutos desde la pose neutral, no velocidades ni objetivos cartesianos.

## Etapas y transformaciones

1. **Captura XR en Godot.** `main.gd` habilita el viewport XR y el fondo transparente, y captura cada frame las poses de ambas manos, posición/yaw de la cámara XR y `dt`. Los marcadores de las manos también se crean en GDScript. `xr_init.gd` marca el viewport como XR desde la entrada al árbol.

2. **Transporte Godot → Python.** `main.gd` construye un paquete JSON v1 con número de secuencia y lo envía usando `PacketPeerUDP` a `127.0.0.1:9000`. No se usa un intérprete Python embebido en Godot.

3. **Recepción y selección del frame vigente.** `robot_control/input_server.py` valida JSON, versión, secuencia y presencia de ambas manos. Rechaza paquetes duplicados o atrasados, contabiliza huecos y detecta reinicios del cliente. `robot_control/latest_slot.py` sirve de mailbox de último valor: el consumidor no procesa una cola atrasada.

4. **Mapeo de poses a `q`.** `robot_control/stras/input_mapping.py` captura cada pose neutral al primer frame. Controlador derecho X/Y/Z → `BR/RR/TR`; izquierdo X/Y/Z → `BL/RL/TL`; cámara X/Y → `EBH/EBV`, avance/retroceso → `ET` y yaw → `ER`. 0.1 m alcanza el límite virtual firmado de cada entrada de posición y 90° alcanza el límite de `ER`. Se limita cada valor con límites virtuales definidos para simulación, no para hardware.

5. **Cinemática directa.** `robot_control/stras/kinematics.py` valida y adapta `q` al `StrasModel`. `shape(q)` produce las secciones geométricas de los brazos y del endoscopio; el runner activa `defineEndoscopeShapeOn()`. No se necesita registro VR→STRAS porque las entradas son valores articulares.

6. **Salida a Godot.** `GodotGeometrySender` serializa las polilíneas de `ShapeOutput` en el protocolo que espera `robot_control/stras_receiver.gd` (JSON v1, `frame`, `right`, `left`, `endoscope`; puntos en unidades del modelo) y las envía por UDP al puerto 9100. Godot crea/actualiza los tubos; no se envían comandos al robot.

7. **Ciclo y cierre.** El flujo se repite por frame. Al salir, el nodo Python cierra el socket UDP de entrada; el runner detiene el receptor y cierra el emisor de geometría.

## Archivos principales por responsabilidad

| Área | Archivos / rol |
|---|---|
| Escena XR, lectura y transporte UDP nativo | [`main.gd`](main.gd), [`xr_init.gd`](xr_init.gd), [`main.tscn`](main.tscn) |
| Backend Python → Godot | [`robot_control/input_server.py`](robot_control/input_server.py), [`robot_control/latest_slot.py`](robot_control/latest_slot.py), [`robot_control/fk_runner.py`](robot_control/fk_runner.py) |
| Runner FK y límites | [`robot_control/fk_runner.py`](robot_control/fk_runner.py), [`robot_control/stras/limits.py`](robot_control/stras/limits.py), [`robot_control/stras/kinematics.py`](robot_control/stras/kinematics.py) |
| Modelo y configuración | [`module_main/model_utilities/stras_model.py`](module_main/model_utilities/stras_model.py), [`module_main/model_utilities/robot_config_right.txt`](module_main/model_utilities/robot_config_right.txt), [`module_main/model_utilities/robot_config_left.txt`](module_main/model_utilities/robot_config_left.txt), [`module_main/model_utilities/robot_config_endoscope.txt`](module_main/model_utilities/robot_config_endoscope.txt) |
| Visualización de geometría STRAS en Godot | [`robot_control/fk_runner.py`](robot_control/fk_runner.py) emite frames UDP; [`robot_control/stras_receiver.gd`](robot_control/stras_receiver.gd) construye los tubos en el puerto 9100. |
| Pruebas y entrada simulada | [`mock_input.tscn`](mock_input.tscn), [`mock_input.gd`](mock_input.gd), [`robot_control/tests/test_client_server.py`](robot_control/tests/test_client_server.py), [`robot_control/tests/test_input_server.py`](robot_control/tests/test_input_server.py), [`robot_control/tests/test_kinematics.py`](robot_control/tests/test_kinematics.py), [`robot_control/tests/test_fk_runner.py`](robot_control/tests/test_fk_runner.py), más prueba de mailbox. |

## Estado y límites observados

### Piezas existentes en el árbol de trabajo

- Captura de controladores y poses desde Godot, empaquetado UDP local con secuencia y receptor con validación y manejo de paquetes atrasados.
- Normalización de controles, transporte de entrada y recepción de paquetes.
- Runner que mapea snapshots VR a `q`, calcula `Kinematics.shape(q)` y transmite las formas de brazos y endoscopio a Godot.
- Receptor `stras_receiver.gd` incluido en `main.tscn`, que acepta geometría por UDP/9100 y dibuja tubos en la escena XR.
- Escena de entrada simulada y pruebas unitarias para transporte, cinemática y runner FK presentes en `robot_control/tests/`.

### Pendientes / restricciones visibles

- **Sin actuación física:** el flujo objetivo es simulación y visualización; no envía comandos a un robot real.
- **Convención visual:** `robot_to_godot` sigue siendo ajustable en el receptor; validar la orientación del modelo al ejecutar en headset.
- **Verificación:** hay pruebas de unidad para el mapeo, el transporte y FK; su ejecución depende del intérprete Python seleccionado en el entorno.

## Estado de control de versiones

El árbol de trabajo puede contener cambios locales además de esta implementación. Revisar el diff antes de integrar o compartir.

## Lectura recomendada

Para comprender el recorrido en orden: `main.gd` → UDP/9000 → `robot_control/input_server.py` → `robot_control/fk_runner.py` → `robot_control/stras/input_mapping.py` → `robot_control/stras/kinematics.py` → UDP/9100 → `robot_control/stras_receiver.gd`.
