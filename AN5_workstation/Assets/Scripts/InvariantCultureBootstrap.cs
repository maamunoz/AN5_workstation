using System.Globalization;
using System.Threading;
using UnityEngine;

// Fuerza la cultura invariante (separador decimal '.') antes de que cargue
// cualquier escena. Sin esto, float.ToString()/string.Join sobre floats en
// los comandos JNTPoint/CARTPoint/MoveJ/SplinePTP (ControlArticular.cs,
// CartesianStateWriterNew.cs, Record_Panel.cs, SecTrajController.cs,
// SecCoordQueueController.cs) usan CultureInfo.CurrentCulture, que en
// Windows viene de la configuracion regional del SO. En una maquina con
// decimal-coma (comun en instalaciones de Windows en espanol), un valor
// como 45.5 se serializa "45,5" -- la misma coma que separa los campos del
// comando -- corrompiendo el conteo de parametros que espera mock_cmd_server
// (JNTPoint requiere exactamente 7). El mock rechaza el comando y lo loguea
// solo en la consola de ROS2, por lo que del lado de Unity no se ve ningun
// error: la posicion simplemente no se ejecuta.
public static class InvariantCultureBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void SetInvariantCulture()
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.CurrentUICulture = CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentUICulture = CultureInfo.InvariantCulture;
        Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
    }
}
