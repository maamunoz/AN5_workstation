# ros2_ws — Workspace ROS2 FR5/AN5

Workspace de ROS 2 (Humble) para controlar el brazo robot Fairino FR5/AN5 y
alimentar una interfaz Unity (RosSharp) por `rosbridge_websocket`. Ofrece
tres formas de correr el stack, todas exponiendo la **misma** interfaz
ROS2 (`/FR_ROS_API_service`, `nonrt_state_data`, `/joint_states`,
`current_joint_position`, `current_cartesian_position`), asi que Unity
nunca necesita saber cual de las tres esta corriendo por debajo.

## Requisitos (nativo)

Para clonar este repo en otro equipo y correr el modo sim o real de forma
nativa:

- Ubuntu 22.04 (u otra distro compatible) con **ROS 2 Humble** instalado
  (`ros-humble-ros-base` alcanza, no hace falta `desktop`).
- `build-essential`, `python3-colcon-common-extensions`, `python3-rosdep`.
- **`ros-humble-rosbridge-server`** — no viene con el ROS base, hay que
  instalarlo aparte (`sudo apt install ros-humble-rosbridge-server`).
  Necesario en ambos modos: tanto `sim.launch.py`/`real.launch.py` como
  las terminales sueltas del modo real levantan `rosbridge_websocket`.
- Python: nada fuera de la stdlib + `rclpy` (`publisher_subscriber.py` y
  `mock_cmd_server.py` solo usan `xmlrpc.client`, `socket`, `threading`,
  `time`, `math`, `random`, `re`) — no hace falta `pip install` nada.
- Solo para modo real: el equipo tiene que poder alcanzar
  `192.168.58.2` (IP del controlador, hardcodeada en `ROS_API.cpp` /
  `state_feedback.cpp` / `publisher_subscriber.py`) en los puertos
  `8080`/`8082`/`8083` (driver TCP) y `20003` (XML-RPC).
- Puerto `9090` alcanzable para que Unity se conecte por
  `rosbridge_websocket` (ambos modos).
- Si Unity corre en otra maquina que los nodos ROS2, todos necesitan el
  mismo `ROS_DOMAIN_ID`.

`build/`, `install/` y `log/` **no** estan en el repo (ver `.gitignore`):
son artefactos de `colcon build`, se regeneran localmente y no son
portables entre equipos (traen rutas absolutas hardcodeadas).

## Build

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Paquetes

| Paquete | Tipo | Rol |
|---|---|---|
| `frhal_msgs` | interfaces | `srv/ROSCmdInterface` + `msg/FRState`, compartidos por los demas paquetes |
| `fr_ros2` | C++ | nodo `ros2_cmd_server`: driver del robot real, habla por TCP con el controlador |
| `code` | Python | nodo `publisher_subscriber`: conecta `/api_command` con el servicio del driver y consulta al robot por XML-RPC |
| `an5_mock_sim` | Python | nodo `mock_cmd_server` + launch files: robot simulado, y los wrappers de launch para modo sim/real |

## 1. Nodos ROS2 para el robot FR5 real

`fr_ros2/ros2_cmd_server` es el driver real: se conecta por TCP crudo al
controlador del robot en `192.168.58.2` (puertos `8080`/`8082` para
comandos, `8083` para el feedback de estado) y expone
`/FR_ROS_API_service` (`frhal_msgs/srv/ROSCmdInterface`) mas el topic
`nonrt_state_data` (`frhal_msgs/msg/FRState`) cada 100 ms.

`code/publisher_subscriber` se ubica delante: se suscribe a
`/api_command` (`std_msgs/String`) y reenvia cada mensaje a
`/FR_ROS_API_service`, y por su cuenta consulta al controlador por
XML-RPC (SDK Fairino, puerto `20003`) para publicar
`current_joint_position` / `current_cartesian_position` — los topics que
Unity realmente lee.

A diferencia del mock (que agrupa todo en un unico `sim.launch.py`), el
modo real se corre como nodos separados en terminales separadas — un
proceso por responsabilidad, para poder reiniciar/inspeccionar cada uno
por separado. Requiere el controlador fisico accesible en
`192.168.58.2`.

```bash
# Terminal 1: ROS2 command server (el driver real)
cd ~/ros2_ws
source install/setup.bash
ros2 run fr_ros2 ros2_cmd_server
```

```bash
# Terminal 2: ROSBridge server (puerto 9090, para Unity/RosSharp)
cd ~/ros2_ws
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
# Terminal 3: puente publisher/subscriber (/api_command -> /FR_ROS_API_service)
cd ~/ros2_ws
source install/setup.bash
ros2 run code publisher_subscriber
```

```bash
# Terminal 4 (opcional): ver los comandos que manda Unity
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /api_command
```

`setup_an5_robot_windows.sh` automatiza la apertura de estas 4 terminales
— enrutadas por Docker en vez de una instalacion nativa, ver seccion 3.

## 2. Mock de ROS2 para simular el robot

`an5_mock_sim/mock_cmd_server` es un reemplazo directo de
`ros2_cmd_server`: sirve el mismo `/FR_ROS_API_service` y
`nonrt_state_data`, y ademas publica `/joint_states`
(`sensor_msgs/JointState`) interpolando hacia los setpoints comandados,
mas `current_joint_position`/`current_cartesian_position` calculados por
cinematica directa — asi Unity ve movimiento sin robot real ni conexion
XML-RPC.

```bash
source install/setup.bash
ros2 launch an5_mock_sim sim.launch.py
```

Argumentos utiles: `joint_states_rate_hz`, `easing`
(`ease_in_out`/`linear`), `initial_joint_positions_deg`,
`include_publisher_subscriber`. Ver
[`src/an5_mock_sim/README.md`](src/an5_mock_sim/README.md) para la
gramatica completa de comandos, las simplificaciones y advertencias
importantes (nunca correr `sim.launch.py` y `real.launch.py` al mismo
tiempo).

## 3. Docker: correr en cualquier lado

Una imagen Docker con ROS 2 Humble y todo el workspace ya compilado, para
correr en cualquier equipo sin instalar ROS ni dependencias a mano.

```bash
docker compose build

# Modo simulado (default, sin robot fisico)
docker compose up fr5-sim

# Modo real (requiere el controlador FR5/AN5 en 192.168.58.2)
docker compose --profile real up fr5-real
```

Funciona igual en Linux nativo y en Docker Desktop (Mac/Windows) -- el
puerto 9090 se expone via `ports` en `docker-compose.yml`, no con
`network_mode: host` (que en Docker Desktop deja rosbridge inalcanzable
para Unity). Unity se conecta a `ws://localhost:9090` (o la IP de esta
maquina si Unity corre en otra).

Ver [`DOCKER.md`](DOCKER.md) para correr el modo real como contenedores
separados por nodo (lo que automatiza `setup_an5_robot_windows.sh`),
pasar argumentos de launch con `docker compose run`, e instrucciones de
rebuild.
