#!/usr/bin/env python3
"""Nodo sonda para las mediciones de desempeno de la plataforma.

Existe SOLO para instrumentar el sistema durante las pruebas del articulo;
esta apagado por defecto en los dos launch (measurement_probe_enabled:=true
para activarlo) y no participa de la operacion normal del robot ni del mock.

Cubre dos huecos que Unity no puede resolver por su cuenta:

1. ECO PARA IDA Y VUELTA (/probe/ping -> /probe/pong)

   Unity publica en 'probe/ping' y este nodo republica de inmediato en
   'probe/pong'. Sirve para medir el tiempo de ida y vuelta contra un solo
   reloj (el de Unity), que es la unica magnitud defendible cuando el
   middleware corre en OTRO equipo y los relojes no estan sincronizados
   (configuraciones C1/C2 del plan de mediciones).

   Un eco explicito es preferible a que Unity se suscriba a su propio topico
   y espere que rosbridge le devuelva lo que acaba de publicar: eso depende
   de un detalle de implementacion de rosbridge, mientras que esto no.

   Ademas el eco agrega DOS marcas de tiempo propias, lo que permite
   descomponer el viaje en tramo de subida, procesamiento y tramo de bajada
   cuando Unity y este nodo comparten equipo (C3/C4, un solo reloj):

       Unity  -> probe/ping : "<id>,<unity_send_unix_ns>"
       Sonda  -> probe/pong : "<id>,<unity_send_unix_ns>,<recv_ns>,<pub_ns>"

   Las marcas son tiempo del sistema en nanosegundos desde la epoca Unix
   (get_clock().now(), que por defecto es el reloj de pared del equipo), asi
   que son directamente comparables con DateTime.UtcNow del lado de Unity.
   En C1/C2 los tramos individuales NO son interpretables (relojes
   distintos) y por eso el arnes los registra pero los marca como tales; el
   tiempo de ida y vuelta, medido con un cronometro en Unity, sigue siendo
   valido en las cuatro configuraciones.

2. CONTADOR PARA PERDIDA EXACTA (/probe/seq)

   Los topicos CSV que Unity consume de verdad (current_joint_position,
   current_cartesian_position, setpoint_cartesian_position) son seis floats
   y nada mas: sin numero de secuencia ni marca de tiempo, un receptor no
   puede distinguir "no me llego" de "nunca se publico". Este nodo publica
   un contador monotono a la misma cadencia nominal que joint_states, de
   modo que la perdida se CUENTA (huecos en la secuencia) en vez de
   inferirse.

       Sonda -> probe/seq : "<seq>,<stamp_ns>"

   La marca de tiempo del mismo mensaje da, ademas, la latencia
   unidireccional de estado en C3/C4, donde hay un solo reloj.

El eco y el contador corren en grupos de callback reentrantes bajo un
executor multihilo a proposito: con el executor de un solo hilo por defecto,
el eco quedaria encolado detras del timer del contador y esa espera se
sumaria a la latencia reportada, que es justo lo que se quiere medir.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

PING_TOPIC = 'probe/ping'
PONG_TOPIC = 'probe/pong'
SEQ_TOPIC = 'probe/seq'

# Misma cadencia por defecto que joint_states en mock_cmd_server, para que la
# perdida medida sobre el contador sea representativa de la del flujo real.
DEFAULT_SEQ_RATE_HZ = 50.0


class MeasurementProbe(Node):

    def __init__(self):
        super().__init__('measurement_probe')

        self.declare_parameter('seq_rate_hz', DEFAULT_SEQ_RATE_HZ)
        self.declare_parameter('seq_enabled', True)
        self.declare_parameter('echo_enabled', True)

        seq_rate_hz = float(self.get_parameter('seq_rate_hz').value)
        seq_enabled = bool(self.get_parameter('seq_enabled').value)
        echo_enabled = bool(self.get_parameter('echo_enabled').value)

        # Reentrante para que el eco no espere al timer del contador (ver
        # docstring): esa espera se sumaria a la latencia medida.
        self._cb_group = ReentrantCallbackGroup()

        self._seq = 0
        self._ping_count = 0

        if echo_enabled:
            self._pong_pub = self.create_publisher(String, PONG_TOPIC, 10)
            self.create_subscription(
                String, PING_TOPIC, self._on_ping, 10,
                callback_group=self._cb_group)
            self.get_logger().info(
                f'Eco activo: {PING_TOPIC} -> {PONG_TOPIC}')
        else:
            self._pong_pub = None

        if seq_enabled and seq_rate_hz > 0.0:
            self._seq_pub = self.create_publisher(String, SEQ_TOPIC, 10)
            self.create_timer(
                1.0 / seq_rate_hz, self._tick_seq,
                callback_group=self._cb_group)
            self.get_logger().info(
                f'Contador activo: {SEQ_TOPIC} a {seq_rate_hz:g} Hz')
        else:
            self._seq_pub = None

        self.get_logger().info(
            'measurement_probe listo. Solo para mediciones: no forma parte '
            'de la operacion normal.')

    def _on_ping(self, msg: String):
        """Republica el ping recibido agregandole dos marcas propias.

        Se toma la marca de recepcion como PRIMERA sentencia y la de
        publicacion como ULTIMA: lo que queda entre ambas es el costo real
        de este nodo, que el arnes descuenta del viaje total.
        """
        recv_ns = self.get_clock().now().nanoseconds

        if self._pong_pub is None:
            return

        out = String()
        # El payload de ida viaja verbatim para que Unity aparee la respuesta
        # con su solicitud por el <id> que puso al principio; este nodo no lo
        # interpreta ni valida a proposito, asi cualquier formato futuro del
        # lado de Unity sigue funcionando sin tocar el nodo.
        pub_ns = self.get_clock().now().nanoseconds
        out.data = f'{msg.data},{recv_ns},{pub_ns}'
        self._pong_pub.publish(out)

        self._ping_count += 1
        if self._ping_count % 100 == 0:
            self.get_logger().info(f'Pings respondidos: {self._ping_count}')

    def _tick_seq(self):
        if self._seq_pub is None:
            return
        stamp_ns = self.get_clock().now().nanoseconds
        msg = String()
        msg.data = f'{self._seq},{stamp_ns}'
        self._seq_pub.publish(msg)
        self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementProbe()
    # Multihilo por el mismo motivo que los grupos reentrantes: que el eco no
    # herede la latencia del timer del contador.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
