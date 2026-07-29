#!/usr/bin/env python3
"""Puente TCP para matlab_ik_node cuando ros2_ws corre en Docker.

En Windows/Mac (Docker Desktop), MATLAB no puede conectarse al grafo ROS2
como nodo nativo (DDS no atraviesa el NAT de la VM de Docker Desktop, ver
ros2_ws/DOCKER.md), asi que AN5_Matlab/inverse_kinematics_docker.m no usa
ros2node/ros2subscriber: en su lugar abre un socket TCP contra este nodo,
que si vive dentro del contenedor y puede suscribirse/publicar por DDS con
total normalidad.

Solo cubre el intercambio de cinematica inversa que documenta el README
raiz (input_cartesian_position -> output_joint_position); el resto de
inverse_kinematics.m (ejecucion de trayectorias por archivo local,
control directo del robot via /api_command) asume una instalacion nativa
de ROS2 en el mismo equipo y no aplica en modo Docker.

Protocolo: lineas JSON terminadas en '\n', UTF-8, un solo cliente a la vez
(la conexion nueva reemplaza a la anterior).

  Contenedor -> MATLAB, una vez por cada pose pedida:
      {"topic": "input_cartesian_position", "data": "x,y,z,rx,ry,rz"}

  MATLAB -> Contenedor, resultado de la cinematica inversa:
      {"topic": "output_joint_position", "data": "j1,j2,j3,j4,j5,j6"}
"""
import json
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_PORT = 9091


class MatlabIkBridge(Node):

    def __init__(self):
        super().__init__('matlab_ik_bridge')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('bind_address', '0.0.0.0')
        port = int(self.get_parameter('port').value)
        bind_address = str(self.get_parameter('bind_address').value)

        self._client_lock = threading.Lock()
        self._client_sock = None

        self._output_pub = self.create_publisher(String, 'output_joint_position', 10)
        self.create_subscription(
            String, 'input_cartesian_position', self._on_cartesian_position, 10)

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((bind_address, port))
        self._server_sock.listen(1)

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        self.get_logger().info(
            f"matlab_ik_bridge escuchando en {bind_address}:{port} "
            "(esperando a inverse_kinematics_docker.m)")

    def _accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self._server_sock.accept()
            except OSError:
                break
            self.get_logger().info(f"MATLAB conectado desde {addr}")
            with self._client_lock:
                self._close_client_locked()
                self._client_sock = conn
            self._read_client_loop(conn)

    def _read_client_loop(self, conn):
        try:
            with conn.makefile('r', encoding='utf-8', newline='\n') as f:
                for line in f:
                    line = line.strip()
                    if line:
                        self._handle_line(line)
        except OSError:
            pass
        finally:
            with self._client_lock:
                if self._client_sock is conn:
                    self._close_client_locked()
            self.get_logger().info("MATLAB desconectado")

    def _handle_line(self, line):
        try:
            msg = json.loads(line)
            topic = msg['topic']
            data = msg['data']
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warning(f"Linea invalida de MATLAB, ignorada: {line!r}")
            return

        if topic != 'output_joint_position':
            self.get_logger().warning(f"Topico inesperado de MATLAB: {topic!r}")
            return

        out = String()
        out.data = data
        self._output_pub.publish(out)

    def _on_cartesian_position(self, msg: String):
        payload = (json.dumps({'topic': 'input_cartesian_position', 'data': msg.data}) + '\n')
        with self._client_lock:
            if self._client_sock is None:
                self.get_logger().warning(
                    "input_cartesian_position recibido pero MATLAB no esta "
                    "conectado a matlab_ik_bridge; se descarta.")
                return
            try:
                self._client_sock.sendall(payload.encode('utf-8'))
            except OSError:
                self.get_logger().warning("Error enviando a MATLAB, cerrando conexion.")
                self._close_client_locked()

    def _close_client_locked(self):
        if self._client_sock is not None:
            try:
                self._client_sock.close()
            except OSError:
                pass
        self._client_sock = None

    def destroy_node(self):
        with self._client_lock:
            self._close_client_locked()
        try:
            self._server_sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MatlabIkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
