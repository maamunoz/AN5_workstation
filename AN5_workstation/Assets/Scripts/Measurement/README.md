# Arnés de mediciones — plan experimental SENIE

Instrumentación para ejecutar las pruebas P1–P10 sobre la plataforma AN5, tomando los
datos desde Unity. **No forma parte de la aplicación**: vive en esta rama
(`measurement-harness-senie`), pensada específicamente para bajar y evaluar sin fricción.

> **En esta rama el nodo sonda de ROS 2 arranca activo por defecto** — alcanza con
> `ros2 launch ... sim.launch.py` sin argumentos extra. En `main` el default es `false`
> (ahí no forma parte de la operación normal y hay que pedirlo a propósito). Del lado de
> Unity sí hay un paso manual inevitable: agregar los componentes a la escena (sección 2).

---

## Puesta en marcha

### 1. Lado ROS 2

```bash
cd ros2_ws
colcon build --packages-select an5_mock_sim
source install/setup.bash

# Emulador (C1/C3) — la sonda ya arranca sola en esta rama
ros2 launch an5_mock_sim sim.launch.py

# Robot físico (C2/C4) — ídem
ros2 launch an5_mock_sim real.launch.py

# Para desactivarla puntualmente (p. ej. para correr la plataforma sin instrumentar):
ros2 launch an5_mock_sim sim.launch.py measurement_probe_enabled:=false
```

Comprobación rápida de que la sonda está viva:

```bash
ros2 topic hz /probe/seq          # ~50 Hz
ros2 topic echo /probe/pong       # debe moverse mientras corre P4
```

Para P5 y P10 hace falta además `matlab_ik_node` levantado desde MATLAB
(`inverse_kinematics.m` en modo nativo, `inverse_kinematics_docker.m` en Docker).

### 2. Lado Unity

En `AN5_sim.unity` (la escena de build), crear un GameObject vacío `MeasurementHarness`
y agregarle:

- `MeasurementSession` — obligatorio, es el orquestador.
- Los componentes de las pruebas que se quieran correr (`P1LinkEstablishment`,
  `P2TopicIntegrity`, `P3RateAndLoss`, `P4TransportLatency`, `P5ApplicationLatency`,
  `P6JointAccuracy`, `P7GraphicsPerf`, `P9KinematicConsistency`, `P10TrajectoryPrep`).

En el Inspector de `MeasurementSession` hay que fijar, como mínimo:

| Campo | Qué poner |
|---|---|
| `configuration` | C1..C4 según dónde corra el middleware y cuál sea el destino |
| `platformLabel` | `Ubuntu-PC`, `Windows-PC`, `Quest3`... |
| `networkLinkType` | p. ej. `Ethernet 1 Gbps` — solo C1/C2 |
| `networkLoadCondition` | `red desocupada` o `uso normal del laboratorio` |
| `robotFirmware` | versión del controlador, si aplica |

Entrar en Play. El panel aparece arriba a la izquierda (`F9` lo oculta). Cada prueba
tiene su botón, y «Ejecutar todas las aplicables» corre la secuencia completa saltando
las que no correspondan a la configuración.

Para el visor, donde el panel no es usable, activar `autoRunOnStart`.

---

## Dónde quedan los resultados

```
measurements/<fecha>_<C#>_<plataforma>/
    environment.csv              condiciones del equipo, versiones, red, reloj
    P1_enlace.csv                + _resumen
    P2_topicos.csv               + _topicos_del_grafo, _resumen
    P3_frecuencia_bins.csv       + _resumen, _unidireccional_estado
    P4_transporte.csv            + _resumen
    P5_aplicacion_ik.csv         + _resumen
    P6_articular.csv             + _resumen
    P7_grafico_cuadros.csv       + _resumen
    P9_poses.csv                 + _errores, _consistencia_resumen
    P10_preparacion.csv          + _resumen, _meta, P10_archivos/
```

La carpeta va en la raíz del proyecto; si no se puede escribir ahí (visor, donde
`dataPath` apunta adentro del APK), cae automáticamente a `persistentDataPath` y lo
informa en consola.

Las series individuales se guardan **completas**, no solo el resumen: cualquier
estadístico se puede rehacer después sin volver a medir.

---

## Salvedades que deben aparecer en el artículo

Estas no son detalles de implementación. Son limitaciones metodológicas que el propio
plan de mediciones pide declarar, y que si no se explicitan vuelven las cifras
incomparables o directamente engañosas.

### 1. Ida y vuelta ≠ unidireccional

La medida principal de P4 y P5 es el **tiempo de ida y vuelta**, cronometrado contra un
solo reloj (el de Unity). Esa fue la decisión deliberada de la estrategia A: con el
middleware en otro equipo, una latencia unidireccional exigiría comparar marcas de dos
relojes no sincronizados, y el desfase entre ellos quedaría indistinguible del retardo
real.

**Los 77,67 ms de Singh son una magnitud unidireccional y NO son directamente
comparables con las cifras de ida y vuelta de C1/C2.** Hay que decirlo en la Discusión
en lugar de comparar números que no miden lo mismo. Solo las columnas unidireccionales
de C3/C4 admiten esa comparación.

### 2. En C3/C4 sí hay unidireccional, y viene de dos lados

Con middleware local, Unity y ROS comparten reloj. El arnés aprovecha eso de dos formas:

- `P4` descompone la ida y vuelta en tramo de subida, costo interno de la sonda y tramo
  de bajada, usando las marcas que `measurement_probe` agrega al mensaje de retorno.
- `P3` mide la latencia unidireccional del flujo de estado con la marca de tiempo que
  viaja en `probe/seq`.

En C1/C2 esas columnas se registran igual, pero con `tramos_interpretables = false` /
`unidireccional_interpretable = false`. **No hay que reportarlas.** Están ahí por si más
adelante se sincronizan los relojes y se quiere reprocesar.

El reloj de pared se ancla con precisión de microsegundos (`HighResolutionClock`),
porque `DateTime.UtcNow` tiene granularidad de ~15 ms en Windows y con eso un tramo de
2 ms sería inmedible. La calidad del anclaje queda registrada en `environment.csv`
(`reloj_ancla_dispersion_us`).

### 3. La brecha entre mensajes llegados y procesados NO es pérdida de red

`P3` reporta tres cifras que es tentador confundir:

| Columna | Qué es |
|---|---|
| `sonda_perdida_pct` | pérdida real, contada por huecos en el contador monótono |
| `estado_frecuencia_llegada_hz` | mensajes que llegaron al cliente |
| `estado_frecuencia_efectiva_hz` | mensajes que el cliente **procesó** |

La diferencia entre las dos últimas es un **descarte deliberado de la aplicación**:
`JointPositionSubscriber` guarda cada mensaje entrante en un buffer de un solo slot que
`Update()` drena una vez por cuadro, así que a 50 Hz de publicación y ~60 cuadros por
segundo necesariamente se pierden mensajes en el cliente. La red no tiene nada que ver.

Presentar esa brecha como pérdida de red sería atribuir a la infraestructura una
decisión de diseño del cliente. Van separadas, y así hay que reportarlas.

### 4. P5 incluye cómputo; P4 no

`P4` usa una sonda vacía que solo reenvía: aísla puente + red. `P5` mide la ida y vuelta
real de cinemática inversa, que incluye el tiempo de MATLAB. **No son sumables ni
comparables entre sí**, y solo `P4` es comparable contra latencias de comunicación de la
literatura.

La primera muestra de `P5` se registra aparte (`primera_muestra_excluida = true`): la
primera llamada a `fr5_ik()` en una sesión fresca de MATLAB tarda del orden de 13 s por
compilación al vuelo, no por el algoritmo. Incluirla desplazaría media y máximo de forma
que no representa la operación.

### 5. P10 nunca se suma a las latencias

El tiempo de preparación de trayectoria es un costo por lotes que se paga una vez al
cargar un archivo, con una ida y vuelta a MATLAB por pose resueltas en serie. Sumarlo a
las latencias de comando distorsionaría por completo la comparación con la literatura.
Se reporta en su propia figura, como tiempo en función de la cantidad de poses.

### 6. P9 compara tres implementaciones que no comparten código

La plataforma calcula la pose del efector de tres maneras independientes:

| Fuente | Dónde |
|---|---|
| `dh_unity` | tabla Denavit-Hartenberg en `LocalForwardKinematics.cs` |
| `jerarquia_urdf` | composición de transformadas padre-hijo de la escena importada |
| `middleware_*` | cadena de `<origin>` del URDF (emulador) o cinemática del fabricante (robot) |

**Hay motivo concreto para esperar divergencia**, y cuantificarla es el resultado:
la tabla DH cierra con `d6 = 0,267 m` mientras que la cadena URDF suma `0,102 + 0,100 =
0,202 m`; y la rama de la jerarquía pasa por `JointStateWriter`, que aplica desfases
fijos de ±90° e inversión de signo por articulación en vez de usar el eje del URDF.

El error de orientación se reporta como **ángulo geodésico** entre rotaciones, no como
diferencia de ángulos de Euler eje a eje: cerca del bloqueo de cardán dos orientaciones
idénticas pueden tener representaciones RPY muy distintas, y la resta daría un error
enorme e inexistente.

La comparación **emulador contra robot** exige dos corridas (una con destino emulado y
otra con destino físico) apareadas después por `config_id`: en una sola sesión hay un
único destino publicando.

### 7. Hardware dispar

`environment.csv` registra procesador, memoria y GPU de cada equipo. Si el equipo con
Windows tiene una tarjeta gráfica superior a la del equipo con Ubuntu, la diferencia de
cuadros por segundo de `P7` **no dice nada sobre el sistema operativo** y hay que
declararlo.

### 8. Medir en operación, no en reposo

`P3` mueve el robot durante toda la ventana por su cuenta. `P7` es **pasiva**: quien
ejecuta tiene que estar usando la interfaz (girando cámara, moviendo el brazo) durante
los dos minutos. Medir con la aplicación quieta y presentarlo como desempeño en
operación es una de las trampas que el plan enumera.

### 9. `environment.csv` puede llegar sin el ping base en Linux/macOS

La latencia base de red (sección "Red" de `environment.csv`, `red_ping_*`) usa
`System.Net.NetworkInformation.Ping`, que pide sockets ICMP crudos. En Windows
funciona sin nada especial. En **Linux** suele hacer falta root o asignarle
`CAP_NET_RAW` al binario (`sudo setcap cap_net_raw+ep ./TuApp`, o correr el Editor con
`sudo`). En **macOS** depende de si el build está firmado. Si falla, no se pierde nada
más: el resto de `environment.csv` se escribe igual y queda `red_ping_error` con el
motivo en vez de las cifras — pero si esa fila aparece vacía en una corrida de Linux o
macOS, es este permiso y no un problema de red real.

---

## Reparto sugerido de corridas

Del propio plan, para que el volumen sea manejable:

- **Las 13 combinaciones**: P1, P2 (y P8 se sostiene sobre P2 + P9).
- **4 a 6 combinaciones**: P3, P4, P5, P7 — eligiendo para aislar variables
  (Ubuntu C1 vs Ubuntu C3 aísla la red; Ubuntu C2 vs Windows C2 aísla la plataforma;
  Quest C2 por portabilidad; Windows C4 por configuración completa sin ROS nativo).
- **Una sola vez**: P9 y P10 (no dependen de la plataforma del cliente) — salvo que P9
  necesita las dos corridas de destino para la comparación emulador/robot.
- **Solo con robot físico**: P6.

---

## Detalle de las modificaciones al código existente

El arnés es aditivo salvo en un archivo:

- **`Assets/Scripts/SecTrajController.cs`**
  - `LoadTrajectoryFile(string)` extraído de `OpenFileDialog()`, que ahora solo elige la
    ruta y delega. Mismo comportamiento; existe como método público porque el diálogo
    nativo no se puede accionar desde código y sin esto P10 tendría que reimplementar la
    carga y estaría cronometrando una copia.
  - Evento estático `TrajectoryResolved(poses, segundos, éxito)` disparado por un
    envoltorio delgado sobre `ResolveJointTrajectory()`. Nada de la aplicación lo
    escucha.

- **`ros2_ws/src/an5_mock_sim/`** — nodo `measurement_probe.py` nuevo, entrada en
  `setup.py`, y el argumento `measurement_probe_enabled` en ambos launch (por defecto
  `true` **en esta rama**; `false` en `main`).

Ningún subscriptor ni el `RosConnector` fueron modificados.
