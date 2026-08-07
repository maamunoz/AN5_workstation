"""Modo SIMULACION del AN5/FR5.

Levanta:
  - rosbridge_websocket (puerto 9090, igual que en modo real -> Unity/RosSharp
    no necesita ningun cambio de configuracion)
  - publisher_subscriber (code) tal cual esta en modo real: sigue siendo el
    puente /api_command -> /FR_ROS_API_service. Sus llamadas XML-RPC directas
    al robot (current_joint_position / current_cartesian_position) van a
    fallar en modo sim salvo que actives el mock XML-RPC embebido (ver
    README, es opcional y tiene una limitacion de puerto documentada).
  - mock_cmd_server (an5_mock_sim): reemplaza a ros2_cmd_server. Expone
    /FR_ROS_API_service y nonrt_state_data igual que el driver real, y
    agrega /joint_states (sensor_msgs/JointState) interpolado.

No levanta ros2_cmd_server real. NO correr junto con real.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    include_publisher_subscriber = DeclareLaunchArgument(
        'include_publisher_subscriber', default_value='true',
        description=(
            'Si es true, levanta code/publisher_subscriber (necesario para '
            'que /api_command llegue al mock). Si es false, solo levanta '
            'rosbridge + mock_cmd_server.'))

    joint_states_rate_hz = DeclareLaunchArgument(
        'joint_states_rate_hz', default_value='50.0',
        description='Frecuencia de publicacion de /joint_states (Hz).')

    easing = DeclareLaunchArgument(
        'easing', default_value='ease_in_out',
        description="Interpolacion de movimiento: 'linear' o 'ease_in_out'.")

    xmlrpc_mock_enabled = DeclareLaunchArgument(
        'xmlrpc_mock_enabled', default_value='false',
        description=(
            'Avanzado: levanta un servidor XML-RPC local que imita '
            'GetActualJointPosDegree/GetActualTCPPose del robot real, para '
            'que publisher_subscriber.py tambien publique '
            'current_joint_position/current_cartesian_position en modo sim. '
            'Requiere aliasing manual de IP (ver README) y puede chocar de '
            'puerto con el TCPServer propio de publisher_subscriber.py.'))

    initial_joint_positions_deg = DeclareLaunchArgument(
        'initial_joint_positions_deg', default_value='0,-90,90,-90,90,0',
        description=(
            "Pose articular inicial en grados, como 6 valores separados "
            "por coma 'j1,j2,j3,j4,j5,j6'. Dejar vacio ('') para arrancar "
            "en todos los joints en 0."))

    matlab_bridge_enabled = DeclareLaunchArgument(
        'matlab_bridge_enabled', default_value='false',
        description=(
            'Si es true, levanta matlab_ik_bridge (puente TCP en '
            'matlab_bridge_port para inverse_kinematics_docker.m). Default '
            'false porque colisiona con un matlab_ik_node nativo '
            '(inverse_kinematics.m via DDS) publicando en el mismo topico '
            'output_joint_position -- solo activar en modo Docker, donde '
            'MATLAB no puede conectarse por DDS de todas formas (ver '
            'ros2_ws/DOCKER.md).'))

    matlab_bridge_port = DeclareLaunchArgument(
        'matlab_bridge_port', default_value='9091',
        description='Puerto TCP de matlab_ik_bridge.')

    measurement_probe_enabled = DeclareLaunchArgument(
        'measurement_probe_enabled', default_value='false',
        description=(
            'Si es true, levanta measurement_probe: eco probe/ping -> '
            'probe/pong y contador probe/seq, usados por el arnes de '
            'mediciones de Unity para medir ida y vuelta y perdida de '
            'mensajes. Default false: solo sirve para instrumentar las '
            'pruebas, no forma parte de la operacion normal.'))

    measurement_probe_seq_rate_hz = DeclareLaunchArgument(
        'measurement_probe_seq_rate_hz', default_value='50.0',
        description=(
            'Frecuencia del contador probe/seq (Hz). Por defecto igual a '
            'joint_states_rate_hz para que la perdida medida sea '
            'representativa del flujo real.'))

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml',
            ])
        ),
    )

    publisher_subscriber_node = Node(
        package='code',
        executable='publisher_subscriber',
        name='robot_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('include_publisher_subscriber')),
    )

    mock_cmd_server_node = Node(
        package='an5_mock_sim',
        executable='mock_cmd_server',
        name='mock_cmd_server',
        output='screen',
        parameters=[{
            'joint_states_rate_hz': LaunchConfiguration('joint_states_rate_hz'),
            'easing': LaunchConfiguration('easing'),
            'xmlrpc_mock.enabled': LaunchConfiguration('xmlrpc_mock_enabled'),
            'initial_joint_positions_deg': LaunchConfiguration('initial_joint_positions_deg'),
        }],
    )

    matlab_ik_bridge_node = Node(
        package='an5_mock_sim',
        executable='matlab_ik_bridge',
        name='matlab_ik_bridge',
        output='screen',
        parameters=[{'port': LaunchConfiguration('matlab_bridge_port')}],
        condition=IfCondition(LaunchConfiguration('matlab_bridge_enabled')),
    )

    measurement_probe_node = Node(
        package='an5_mock_sim',
        executable='measurement_probe',
        name='measurement_probe',
        output='screen',
        parameters=[{
            'seq_rate_hz': LaunchConfiguration('measurement_probe_seq_rate_hz'),
        }],
        condition=IfCondition(LaunchConfiguration('measurement_probe_enabled')),
    )

    return LaunchDescription([
        include_publisher_subscriber,
        joint_states_rate_hz,
        easing,
        xmlrpc_mock_enabled,
        initial_joint_positions_deg,
        matlab_bridge_enabled,
        matlab_bridge_port,
        measurement_probe_enabled,
        measurement_probe_seq_rate_hz,
        rosbridge_launch,
        publisher_subscriber_node,
        mock_cmd_server_node,
        matlab_ik_bridge_node,
        measurement_probe_node,
    ])
