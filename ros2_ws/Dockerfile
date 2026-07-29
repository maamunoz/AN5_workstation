# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Dependencias de build/runtime del workspace:
# - python3-colcon-common-extensions / python3-rosdep / build-essential: para compilar
# - ros-humble-rosbridge-server: usado por sim.launch.py y real.launch.py (puerto 9090,
#   consumido por Unity/RosSharp)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        build-essential \
        ros-humble-rosbridge-server \
    # `|| true`: algunas builds de la imagen base ya traen rosdep
    # inicializado (20-default.list preexistente), y `rosdep init` falla
    # si el sources list ya existe -- no es un error real, solo ya esta.
    && (rosdep init || true) \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copiamos primero solo lo necesario para resolver dependencias con rosdep
# (aprovecha la cache de capas de Docker si despues solo cambia el codigo).
COPY src ./src

RUN rosdep update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Puerto de rosbridge_websocket (Unity/RosSharp se conecta aca).
EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
# Por defecto arranca en modo simulado (sin necesidad del controlador real).
# Para modo real: docker run ... fr5_ros2 ros2 launch an5_mock_sim real.launch.py
CMD ["ros2", "launch", "an5_mock_sim", "sim.launch.py"]
