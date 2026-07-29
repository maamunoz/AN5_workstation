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

# Terminal 5 (opcional): puente IK para MATLAB, ver seccion de abajo
docker compose --profile real run --rm --name an5_matlab_ik_bridge -p 9091:9091 fr5-real \
    ros2 run an5_mock_sim matlab_ik_bridge
```

## Cinematica inversa con MATLAB (Windows/Mac)

`AN5_Matlab/inverse_kinematics.m` (el `matlab_ik_node` que documenta el
README raiz) se conecta al grafo ROS 2 como nodo DDS nativo. Eso funciona
si MATLAB corre en la misma maquina Linux que el ROS 2 nativo, pero **no
funciona en modo Docker**: DDS necesita descubrimiento por multicast (o
Discovery Server + locators externos), y el compose usa
`ports: ["9090:9090", ...]` en vez de `network_mode: host` justamente para
que rosbridge funcione igual en Docker Desktop -- el costo es que ningun
nodo DDS de afuera del contenedor (MATLAB incluido) puede descubrir los de
adentro.

Para cubrir ese caso se agrego `matlab_ik_bridge` (`an5_mock_sim`): un nodo
que corre dentro del contenedor, se suscribe/publica en ROS 2 con
normalidad (`input_cartesian_position` / `output_joint_position`) y expone
un socket TCP simple (JSON por linea) en el puerto **9091**, publicado en
`docker-compose.yml` igual que el 9090 de rosbridge. Del lado de MATLAB,
`AN5_Matlab/inverse_kinematics_docker.m` reemplaza a `inverse_kinematics.m`
usando `tcpclient` en vez de `ros2node`:

```matlab
ikBridge = inverse_kinematics_docker("<IP del host Docker>");
```

(`"localhost"` si Docker corre en la misma maquina que MATLAB; si no, la
IP de esa maquina en la red local). Guardar el resultado en una variable
del workspace base es necesario -- `tcpclient` es un objeto handle y sin
ninguna referencia viva MATLAB lo cierra apenas termina la funcion.

Activado por defecto en `docker compose up fr5-sim` /
`--profile real up fr5-real` (`matlab_bridge_enabled:=true` en el
`command:` del compose). Solo cubre el intercambio de IK
(`input_cartesian_position -> output_joint_position`); el resto de
`inverse_kinematics.m` (ejecucion de trayectorias por archivo local,
control directo del robot via `/api_command`) asume una instalacion nativa
de ROS 2 en el mismo equipo que MATLAB y no esta cubierto por este puente.

No correr `inverse_kinematics.m` (DDS nativo) e
`inverse_kinematics_docker.m` (TCP) al mismo tiempo contra el mismo
grafo ROS 2: ambos publican en `output_joint_position` y competirian igual
que describe el comentario sobre ese topico en `mock_cmd_server.py`.

## Notas

- El compose usa `ports: ["9090:9090"]` (no `network_mode: host`), asi que
  funciona igual en Linux nativo y en Docker Desktop (Mac/Windows) sin
  tocar nada -- ese fue justamente el bug que teniamos antes con
  `network_mode: host` en Docker Desktop: rosbridge arrancaba bien
  adentro del contenedor pero el puerto quedaba aislado en la red interna
  de la VM de Docker Desktop, y Unity nunca lograba conectar. El costo:
  nodos ROS2 nativos corriendo fuera del contenedor no van a poder
  descubrir los de aca por DDS (el mapeo de puertos no alcanza para eso,
  solo sirve para rosbridge/matlab_ik_bridge). Para inspeccionar el grafo
  ROS2 desde afuera sin eso, usar `docker exec fr5_ros2_sim ros2 topic
  list` (o el nombre del contenedor que corresponda) en vez de un `ros2`
  nativo. Para MATLAB especificamente, ver "Cinematica inversa con
  MATLAB" arriba -- ese es el caso puntual ya cubierto por un puente TCP
  en vez de DDS.

- Pasar argumentos de launch (ver tabla en `src/an5_mock_sim/README.md`),
  por ejemplo:

  ```bash
  docker compose run --rm fr5-sim \
    ros2 launch an5_mock_sim sim.launch.py easing:=linear
  ```

- Para reconstruir despues de tocar codigo: `docker compose build` de
  nuevo (no hay bind mount del `src/`, el codigo queda copiado dentro de
  la imagen en el build).
