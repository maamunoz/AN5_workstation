/// Mensaje custom, NO parte del RosBridgeClient.dll de terceros (que solo trae los
/// tipos estándar de ROS). Espejo mínimo de frhal_msgs/msg/FRState.msg -- ver
/// ros2_ws/src/frhal_msgs/msg/FRState.msg para la definición completa. Se
/// declaran solo los campos que Unity necesita hoy; rosbridge serializa el
/// mensaje real a JSON con todos sus campos, pero el deserializador de RosSharp
/// ignora las claves que no tengan una propiedad correspondiente acá, así que
/// no hace falta espejar el mensaje entero.
///
/// Publicado en el tópico nonrt_state_data, tanto por el driver real
/// (fr_ros2/ros2_cmd_server, que lo llena leyendo el socket de estado del
/// controlador -- ver fr_ros2/src/state_feedback.cpp) como por el emulador
/// (an5_mock_sim/mock_cmd_server.py, que simula el mismo campo). robot_motion_done
/// es la señal explícita del controlador para "este movimiento terminó" -- ver
/// RobotMotionDoneSubscriber.cs.
namespace RosSharp.RosBridgeClient.MessageTypes.Frhal
{
    public class FRState : Message
    {
        public const string RosMessageName = "frhal_msgs/FRState";

        /// 1 cuando el controlador reporta el movimiento en curso como terminado,
        /// 0 mientras se sigue moviendo. int32 en el .msg original.
        public int robot_motion_done { get; set; }
    }
}
