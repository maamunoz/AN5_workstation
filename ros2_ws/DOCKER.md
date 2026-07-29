# Docker

Imagen con ROS 2 Humble + el workspace (`fr_ros2`, `code`, `frhal_msgs`,
`an5_mock_sim`) ya compilado, para correr en cualquier equipo sin instalar
ROS ni dependencias a mano.

## Build

```bash
docker compose build
```

(o `docker build -t fr5_ros2 .` sin compose)

## Modo simulado (default, sin robot fisico)

```bash
docker compose up fr5-sim
```

Levanta `rosbridge_websocket` (puerto **9090**, para Unity/RosSharp) +
`mock_cmd_server` + `publisher_subscriber` + `matlab_ik_bridge` (puerto
**9091**, ver "Cinematica inversa con MATLAB" abajo), igual que
`ros2 launch an5_mock_sim sim.launch.py matlab_bridge_enabled:=true`
corrido nativo.

## Modo real (requiere el controlador FR5/AN5 en 192.168.58.2)

Bundleado, los tres procesos en un solo contenedor (`real.launch.py`):

```bash
docker compose --profile real up fr5-real
```

## Cinematica inversa con MATLAB (Windows/Mac)

Correr `AN5_Matlab/inverse_kinematics_docker.m` (en vez de
`inverse_kinematics.m`, que necesita DDS nativo y no funciona contra el
contenedor). Se conecta por TCP al puente `matlab_ik_bridge` expuesto en
el puerto **9091**:

```matlab
ikBridge = inverse_kinematics_docker("<IP del host Docker>");
```
