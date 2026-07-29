function client = inverse_kinematics_docker(host, port)
%INVERSE_KINEMATICS_DOCKER Puente IK MATLAB <-> ros2_ws corriendo en Docker.
%
% Reemplazo de inverse_kinematics.m para cuando ros2_ws corre en Docker
% (Windows/Mac): en ese caso MATLAB no puede conectarse al grafo ROS2 como
% nodo nativo (DDS no atraviesa el NAT de Docker Desktop -- ver
% ros2_ws/DOCKER.md), asi que en vez de ros2node/ros2subscriber/
% ros2publisher esta version usa un socket TCP simple contra
% matlab_ik_bridge (an5_mock_sim), que corre dentro del contenedor cuando
% se lanza con matlab_bridge_enabled:=true (docker-compose.yml lo activa
% por defecto, puerto 9091).
%
% Solo cubre el intercambio de cinematica inversa que documenta el README
% raiz (input_cartesian_position -> output_joint_position); NO reemplaza
% el resto de inverse_kinematics.m (ejecucion de trayectorias por archivo
% local, control directo del robot via /api_command), que asume una
% instalacion nativa de ROS2 en el mismo equipo que MATLAB.
%
% Uso:
%   ikBridge = inverse_kinematics_docker();                  % localhost:9091
%   ikBridge = inverse_kinematics_docker("192.168.1.50");     % IP del host Docker
%   ikBridge = inverse_kinematics_docker("192.168.1.50", 9091);
%
% Guardar el valor de retorno en una variable del workspace base es
% necesario: tcpclient es un objeto handle y, si no queda ninguna
% referencia viva, MATLAB lo cierra apenas termina la funcion.

if nargin < 1 || isempty(host)
    host = 'localhost';
end
if nargin < 2 || isempty(port)
    port = 9091;
end

fprintf('Conectando a matlab_ik_bridge en %s:%d ...\n', host, port);
client = tcpclient(host, port, 'ConnectTimeout', 10);
configureTerminator(client, 'LF');
configureCallback(client, 'terminator', @onLine);

fprintf('Conectado. Esperando peticiones de cinematica inversa...\n');
end

function onLine(src, ~)
line = strtrim(readline(src));
if line == ""
    return
end

try
    req = jsondecode(line);
catch
    warning('inverse_kinematics_docker:badLine', ...
        'Linea invalida recibida de matlab_ik_bridge: %s', line);
    return
end

if ~isfield(req, 'topic') || ~isfield(req, 'data') || ...
        ~strcmp(req.topic, 'input_cartesian_position')
    return
end

values = str2double(strsplit(req.data, ','));
if numel(values) ~= 6
    warning('inverse_kinematics_docker:badPose', ...
        'Datos de pose invalidos recibidos: %s', req.data);
    return
end

x  = values(1); y  = values(2); z  = values(3);
rx = values(4); ry = values(5); rz = values(6);

hi = fr5_ik(x, y, z, rx, ry, rz);
if isempty(hi) || numel(hi) ~= 6
    warning('inverse_kinematics_docker:ikFailed', ...
        'Error en la cinematica inversa para la pose recibida: %s', req.data);
    return
end

hi_str = sprintf('%.17f,%.17f,%.17f,%.17f,%.17f,%.17f', hi);
resp = struct('topic', 'output_joint_position', 'data', hi_str);
writeline(src, jsonencode(resp));

fprintf('IK: [%s] -> [%s]\n', req.data, hi_str);
end
