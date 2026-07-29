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
`mock_cmd_server` + `publisher_subscriber`, igual que
`ros2 launch an5_mock_sim sim.launch.py` corrido nativo.

## Modo real (requiere el controlador FR5/AN5 en 192.168.58.2)

Bundleado, los tres procesos en un solo contenedor (`real.launch.py`):

```bash
docker compose --profile real up fr5-real
```

### Modo real: nodos separados (un contenedor por nodo)

Igual que corriendolo nativo en 4 terminales separadas, pero cada uno en su
propio contenedor. Preferible si queres poder reiniciar/inspeccionar un
nodo sin tocar los demas -- es lo que automatiza
`setup_an5_robot_windows.sh`:

```bash
# Terminal 1: driver real
docker compose --profile real run --rm --name an5_ros2_cmd_server fr5-real \
    ros2 run fr_ros2 ros2_cmd_server

# Terminal 2: rosbridge (puerto 9090, Unity/RosSharp)
docker compose --profile real run --rm --name an5_rosbridge fr5-real \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 3: puente /api_command -> /FR_ROS_API_service
docker compose --profile real run --rm --name an5_publisher_subscriber fr5-real \
    ros2 run code publisher_subscriber

# Terminal 4 (opcional): ver los comandos que manda Unity
docker compose --profile real run --rm --name an5_api_command_echo fr5-real \
    ros2 topic echo /api_command
```

## Notas

- El compose usa `ports: ["9090:9090"]` (no `network_mode: host`), asi que
  funciona igual en Linux nativo y en Docker Desktop (Mac/Windows) sin
  tocar nada -- ese fue justamente el bug que teniamos antes con
  `network_mode: host` en Docker Desktop: rosbridge arrancaba bien
  adentro del contenedor pero el puerto quedaba aislado en la red interna
  de la VM de Docker Desktop, y Unity nunca lograba conectar. El costo:
  nodos ROS2 nativos corriendo fuera del contenedor no van a poder
  descubrir los de aca por DDS (el mapeo de puertos no alcanza para eso,
  solo sirve para rosbridge). Para inspeccionar el grafo ROS2 desde
  afuera sin eso, usar `docker exec fr5_ros2_sim ros2 topic list` (o el
  nombre del contenedor que corresponda) en vez de un `ros2` nativo.

- Pasar argumentos de launch (ver tabla en `src/an5_mock_sim/README.md`),
  por ejemplo:

  ```bash
  docker compose run --rm fr5-sim \
    ros2 launch an5_mock_sim sim.launch.py easing:=linear
  ```

- Para reconstruir despues de tocar codigo: `docker compose build` de
  nuevo (no hay bind mount del `src/`, el codigo queda copiado dentro de
  la imagen en el build).
