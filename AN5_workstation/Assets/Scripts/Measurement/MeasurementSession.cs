using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using RosSharp.RosBridgeClient;

namespace AN5.Measurement
{
    /// Las cuatro configuraciones del plan de mediciones.
    ///
    /// El eje que importa para el reloj es dónde corre el middleware: con middleware
    /// local (C3/C4) Unity y rosbridge comparten equipo y por lo tanto reloj, y la
    /// latencia unidireccional es medible directamente. Con middleware remoto (C1/C2)
    /// no lo es, y solo el tiempo de ida y vuelta resulta defendible.
    public enum MeasurementConfiguration
    {
        C1_MiddlewareRemoto_Emulador = 1,
        C2_MiddlewareRemoto_RobotFisico = 2,
        C3_MiddlewareLocal_Emulador = 3,
        C4_MiddlewareLocal_RobotFisico = 4,
    }

    /// Orquestador del arnés de mediciones.
    ///
    /// Se agrega a un GameObject vacío de la escena junto con las pruebas que se
    /// quieran correr (cada una es un MonoBehaviour que hereda de MeasurementTest).
    /// No se activa solo: hay que poner el componente a mano en la escena, así una
    /// build normal de la aplicación no arrastra nada de esto.
    ///
    /// Todo lo que produce va a measurements/<fecha>_<config>_<plataforma>/, con un
    /// CSV por prueba más environment.csv, que documenta las condiciones exigidas por
    /// la sección 1 del plan (equipo, sistema operativo, versiones, red).
    public class MeasurementSession : MonoBehaviour
    {
        [Header("Identificación de la corrida")]
        [Tooltip("Configuración evaluada. Determina qué pruebas aplican y si las " +
                 "latencias unidireccionales son interpretables.")]
        public MeasurementConfiguration configuration = MeasurementConfiguration.C3_MiddlewareLocal_Emulador;

        [Tooltip("Etiqueta corta de la plataforma cliente: Ubuntu-PC, Windows-PC, Quest3...")]
        public string platformLabel = "Windows-PC";

        [Tooltip("Notas libres que quedan registradas en environment.csv.")]
        [TextArea(2, 4)]
        public string runNotes = "";

        [Header("Condiciones de red (solo C1/C2; se registran tal cual)")]
        [Tooltip("Cableado o inalámbrico, y velocidad nominal. Ej: 'Ethernet 1 Gbps'.")]
        public string networkLinkType = "";
        [Tooltip("Si el tráfico atraviesa conmutadores o enrutadores intermedios.")]
        public string networkIntermediateHops = "";
        [Tooltip("Red desocupada o en uso normal del laboratorio.")]
        public string networkLoadCondition = "";
        [Tooltip("Si está activo y el host no es local, mide la latencia base de red " +
                 "con ICMP antes de empezar. Es la cifra que permite separar el costo " +
                 "de la red del costo del puente.")]
        public bool measureNetworkBaseline = true;
        [Tooltip("Cantidad de paquetes ICMP. El plan pide al menos 100.")]
        public int networkBaselinePackets = 100;

        [Header("Robot")]
        public string robotModel = "AN5 / FR5v6 (Fairino)";
        public string robotFirmware = "";
        [Tooltip("Configuración articular de partida, en grados.")]
        public string robotInitialPose = "0,-90,90,-90,90,0";

        [Header("Ejecución")]
        [Tooltip("Arranca la secuencia completa sola al entrar en Play. Pensado para " +
                 "el visor, donde no hay panel en pantalla usable.")]
        public bool autoRunOnStart = false;
        [Tooltip("Segundos de espera antes del arranque automático, para dar tiempo a " +
                 "que la conexión se establezca.")]
        public float autoRunDelaySeconds = 10f;
        [Tooltip("Tecla que muestra u oculta el panel en pantalla.")]
        public KeyCode togglePanelKey = KeyCode.F9;

        [Header("Posición del panel en pantalla")]
        [Tooltip("Borde izquierdo del panel, como fracción del ancho de pantalla " +
                 "(0 = pegado a la izquierda, 0.5 = arranca en el centro y se extiende " +
                 "hacia la derecha, 1 = pegado a la derecha). Default 0.5 para no " +
                 "superponerse con la UI de la escena, que ocupa el costado izquierdo.")]
        [Range(0f, 1f)]
        public float panelAnchorX = 0.5f;

        [Tooltip("Borde superior del panel, como fracción del alto de pantalla.")]
        [Range(0f, 1f)]
        public float panelAnchorY = 0f;

        [Tooltip("Ancho del panel en píxeles.")]
        public float panelWidth = 430f;

        // --- Estado de la sesión ---
        public string RunDirectory { get; private set; }
        public bool IsRunning { get; private set; }
        public MeasurementTest CurrentTest { get; private set; }

        /// Verdadero cuando Unity y el middleware comparten equipo y por lo tanto
        /// reloj: solo ahí las latencias unidireccionales son interpretables.
        public bool SingleClock
        {
            get
            {
                return configuration == MeasurementConfiguration.C3_MiddlewareLocal_Emulador
                    || configuration == MeasurementConfiguration.C4_MiddlewareLocal_RobotFisico;
            }
        }

        /// Verdadero cuando el destino es el robot físico y no el emulador.
        public bool HasPhysicalRobot
        {
            get
            {
                return configuration == MeasurementConfiguration.C2_MiddlewareRemoto_RobotFisico
                    || configuration == MeasurementConfiguration.C4_MiddlewareLocal_RobotFisico;
            }
        }

        // --- Componentes de la aplicación que las pruebas reutilizan ---
        public RosConnector Connector { get; private set; }
        public Ros2CommandSender CommandSender { get; private set; }
        public JointPositionSubscriber JointSubscriber { get; private set; }
        public CartesianPositionSubscriber CartesianSubscriber { get; private set; }

        /// Opcional: null si la escena no tiene el componente, o si el driver/emulador
        /// no está publicando nonrt_state_data. Quien lo use debe chequear ambos casos
        /// (ver RobotMotionDoneSubscriber.HasReceivedMessage) -- no es un requisito
        /// para que las pruebas corran, solo un refinamiento de timing (ver P6).
        public RobotMotionDoneSubscriber MotionDoneSubscriber { get; private set; }

        public RosSocket Socket
        {
            get { return Connector != null ? Connector.RosSocket : null; }
        }

        public bool IsConnected
        {
            get { return Connector != null && Connector.IsOnline; }
        }

        private readonly List<MeasurementTest> _tests = new List<MeasurementTest>();
        private readonly List<CsvWriter> _openWriters = new List<CsvWriter>();
        private bool _panelVisible = true;
        private Vector2 _scroll;

        void Awake()
        {
            ResolveAppComponents();
            RunDirectory = CreateRunDirectory();
            GetComponents(_tests);
            Debug.Log($"[MeasurementSession] Corrida en: {RunDirectory}");
        }

        IEnumerator Start()
        {
            // El anclaje del reloj se rehace acá, ya arrancada la aplicación: hacerlo
            // en Awake lo dejaría contaminado por la carga inicial de la escena.
            HighResolutionClock.Reanchor();

            yield return StartCoroutine(WriteEnvironmentFile());

            if (autoRunOnStart)
            {
                yield return new WaitForSeconds(autoRunDelaySeconds);
                yield return StartCoroutine(RunAllApplicable());
            }
        }

        void Update()
        {
            if (Input.GetKeyDown(togglePanelKey))
                _panelVisible = !_panelVisible;
        }

        private void ResolveAppComponents()
        {
            Connector = FindObjectOfType<RosConnector>();
            CommandSender = FindObjectOfType<Ros2CommandSender>();
            JointSubscriber = FindObjectOfType<JointPositionSubscriber>();
            CartesianSubscriber = FindObjectOfType<CartesianPositionSubscriber>();
            MotionDoneSubscriber = FindObjectOfType<RobotMotionDoneSubscriber>();

            if (Connector == null)
                Debug.LogError("[MeasurementSession] No se encontró RosConnector en la escena.");
        }

        private string CreateRunDirectory()
        {
            string stamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string safePlatform = SanitizeForPath(platformLabel);
            string folder = $"{stamp}_{ShortConfigLabel()}_{safePlatform}";

            // Se prefiere la raíz del proyecto (mismo convenio que
            // SecCoordQueueController y ScreenRecorder), con repliegue a
            // persistentDataPath: en el visor, dataPath apunta adentro del APK y es
            // de solo lectura, así que sin este repliegue no se guardaría nada.
            string preferred = Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", "measurements", folder));
            if (TryPrepareDirectory(preferred)) return preferred;

            string fallback = Path.Combine(Application.persistentDataPath, "measurements", folder);
            if (TryPrepareDirectory(fallback)) return fallback;

            Debug.LogError("[MeasurementSession] No se pudo crear ninguna carpeta de salida.");
            return fallback;
        }

        private static bool TryPrepareDirectory(string path)
        {
            try
            {
                Directory.CreateDirectory(path);
                string probe = Path.Combine(path, ".escritura");
                File.WriteAllText(probe, "ok");
                File.Delete(probe);
                return true;
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[MeasurementSession] Carpeta no utilizable ({path}): {e.Message}");
                return false;
            }
        }

        private static string SanitizeForPath(string s)
        {
            if (string.IsNullOrEmpty(s)) return "sin-etiqueta";
            foreach (char c in Path.GetInvalidFileNameChars())
                s = s.Replace(c, '-');
            return s.Replace(' ', '-');
        }

        public string ShortConfigLabel()
        {
            return "C" + ((int)configuration);
        }

        /// Abre un CSV dentro de la carpeta de la corrida. La sesión conserva la
        /// referencia para cerrarlos todos al salir.
        public CsvWriter OpenCsv(string fileSuffix, params string[] header)
        {
            string path = Path.Combine(RunDirectory, $"{fileSuffix}.csv");
            var writer = new CsvWriter(path, header);
            _openWriters.Add(writer);
            return writer;
        }

        // -----------------------------------------------------------------
        // environment.csv — las condiciones que exige la sección 1 del plan.
        // Se escribe al arrancar porque después no se reconstruye.
        // -----------------------------------------------------------------
        private IEnumerator WriteEnvironmentFile()
        {
            var csv = OpenCsv("environment", "clave", "valor");

            csv.WriteRow("fecha_hora_inicio", System.DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            csv.WriteRow("configuracion", configuration.ToString());
            csv.WriteRow("configuracion_corta", ShortConfigLabel());
            csv.WriteRow("reloj_unico", SingleClock);
            csv.WriteRow("destino_robot_fisico", HasPhysicalRobot);
            csv.WriteRow("plataforma_etiqueta", platformLabel);
            csv.WriteRow("notas", runNotes);

            csv.WriteRow("procesador", SystemInfo.processorType);
            csv.WriteRow("procesador_nucleos", SystemInfo.processorCount);
            csv.WriteRow("procesador_frecuencia_mhz", SystemInfo.processorFrequency);
            csv.WriteRow("memoria_mb", SystemInfo.systemMemorySize);
            csv.WriteRow("gpu", SystemInfo.graphicsDeviceName);
            csv.WriteRow("gpu_api", SystemInfo.graphicsDeviceType.ToString());
            csv.WriteRow("gpu_version", SystemInfo.graphicsDeviceVersion);
            csv.WriteRow("gpu_memoria_mb", SystemInfo.graphicsMemorySize);
            csv.WriteRow("sistema_operativo", SystemInfo.operatingSystem);
            csv.WriteRow("modelo_dispositivo", SystemInfo.deviceModel);
            csv.WriteRow("tipo_dispositivo", SystemInfo.deviceType.ToString());

            csv.WriteRow("unity_version", Application.unityVersion);
            csv.WriteRow("plataforma_runtime", Application.platform.ToString());
            csv.WriteRow("resolucion", $"{Screen.currentResolution.width}x{Screen.currentResolution.height}");
            csv.WriteRow("refresco_pantalla_hz", Screen.currentResolution.refreshRateRatio.value);
            csv.WriteRow("target_frame_rate", Application.targetFrameRate);
            csv.WriteRow("vsync_count", QualitySettings.vSyncCount);

            csv.WriteRow("rosbridge_url", Connector != null ? Connector.RosBridgeServerUrl : "(sin RosConnector)");
            csv.WriteRow("rosbridge_conectado_al_inicio", IsConnected);

            // Calidad del reloj: se declara junto a los resultados en vez de quedar
            // como supuesto tácito (ver HighResolutionClock).
            csv.WriteRow("reloj_alta_resolucion", HighResolutionClock.IsHighResolution);
            csv.WriteRow("reloj_resolucion_ns", HighResolutionClock.TimestampResolutionNanos);
            csv.WriteRow("reloj_ancla_dispersion_us", HighResolutionClock.AnchorSpreadMicroseconds);

            csv.WriteRow("red_tipo_enlace", networkLinkType);
            csv.WriteRow("red_saltos_intermedios", networkIntermediateHops);
            csv.WriteRow("red_condicion_carga", networkLoadCondition);

            csv.WriteRow("robot_modelo", robotModel);
            csv.WriteRow("robot_firmware", robotFirmware);
            csv.WriteRow("robot_pose_inicial", robotInitialPose);

            // Latencia base de red: es lo que permite separar el costo de la red del
            // costo del puente, que es justamente la comparación entre C1/C2 y C3/C4.
            string host = NetworkBaseline.ExtractHost(
                Connector != null ? Connector.RosBridgeServerUrl : null);
            if (measureNetworkBaseline && !NetworkBaseline.IsLoopback(host))
            {
                SetTransientStatus($"Midiendo latencia base de red contra {host}...");
                NetworkBaseline.Result result = null;
                yield return StartCoroutine(
                    NetworkBaseline.Measure(host, networkBaselinePackets, r => result = r));

                if (result != null && result.Succeeded)
                {
                    csv.WriteRow("red_ping_host", host);
                    csv.WriteRow("red_ping_paquetes_enviados", result.Sent);
                    csv.WriteRow("red_ping_paquetes_recibidos", result.Received);
                    csv.WriteRow("red_ping_media_ms", result.MeanMs);
                    csv.WriteRow("red_ping_desviacion_ms", result.StdDevMs);
                    csv.WriteRow("red_ping_minimo_ms", result.MinMs);
                    csv.WriteRow("red_ping_maximo_ms", result.MaxMs);
                }
                else
                {
                    csv.WriteRow("red_ping_host", host);
                    csv.WriteRow("red_ping_error",
                        result != null ? result.Error : "sin resultado");
                }
                SetTransientStatus("");
            }
            else
            {
                csv.WriteRow("red_ping_host", host);
                csv.WriteRow("red_ping_omitido",
                    NetworkBaseline.IsLoopback(host)
                        ? "host local: no hay red que medir"
                        : "desactivado por configuración");
            }

            csv.Dispose();
            _openWriters.Remove(csv);
        }

        // -----------------------------------------------------------------
        // Ejecución de pruebas
        // -----------------------------------------------------------------

        public IReadOnlyList<MeasurementTest> Tests { get { return _tests; } }

        public Coroutine RunTest(MeasurementTest test)
        {
            if (IsRunning)
            {
                Debug.LogWarning("[MeasurementSession] Ya hay una prueba en curso.");
                return null;
            }
            return StartCoroutine(RunTestRoutine(test));
        }

        private IEnumerator RunTestRoutine(MeasurementTest test)
        {
            IsRunning = true;
            CurrentTest = test;
            // Reanclar antes de cada prueba: la deriva entre el contador de alta
            // resolución y el reloj de pared no se acumula así entre pruebas de una
            // sesión larga.
            HighResolutionClock.Reanchor();

            Debug.Log($"[MeasurementSession] === {test.TestId} {test.DisplayName} ===");
            yield return StartCoroutine(test.Run(this));
            Debug.Log($"[MeasurementSession] === {test.TestId} terminada: {test.Status} ===");

            CurrentTest = null;
            IsRunning = false;
        }

        public Coroutine RunAll()
        {
            return IsRunning ? null : StartCoroutine(RunAllApplicable());
        }

        private IEnumerator RunAllApplicable()
        {
            foreach (var test in _tests)
            {
                if (!test.AppliesTo(configuration))
                {
                    Debug.Log($"[MeasurementSession] {test.TestId} no aplica a " +
                              $"{ShortConfigLabel()}: {test.NotApplicableReason}");
                    continue;
                }
                yield return StartCoroutine(RunTestRoutine(test));
                // Respiro entre pruebas para que el estado transitorio de una no
                // contamine el arranque de la siguiente.
                yield return new WaitForSeconds(2f);
            }
            Debug.Log($"[MeasurementSession] Secuencia completa. Resultados en {RunDirectory}");
        }

        private string _transientStatus = "";
        public void SetTransientStatus(string s) { _transientStatus = s; }

        void OnDestroy()
        {
            foreach (var w in _openWriters) w.Dispose();
            _openWriters.Clear();
        }

        // -----------------------------------------------------------------
        // Panel en pantalla (IMGUI, para no depender del Canvas de la app)
        // -----------------------------------------------------------------
        void OnGUI()
        {
            if (!_panelVisible) return;

            float w = panelWidth;
            float h = Mathf.Min(Screen.height - 20f, 180f + _tests.Count * 46f);
            // Clamp para que un anchor cercano a 1 (o una pantalla angosta) no empuje
            // el panel fuera del área visible en vez de simplemente pegarlo al borde.
            float x = Mathf.Clamp(Screen.width * panelAnchorX, 10f, Screen.width - w - 10f);
            float y = Mathf.Clamp(Screen.height * panelAnchorY + 10f, 10f, Screen.height - h - 10f);
            GUILayout.BeginArea(new Rect(x, y, w, h), GUI.skin.box);

            GUILayout.Label($"<b>Arnés de mediciones — {ShortConfigLabel()} / {platformLabel}</b>",
                new GUIStyle(GUI.skin.label) { richText = true });
            GUILayout.Label(IsConnected ? "rosbridge: CONECTADO" : "rosbridge: SIN CONEXIÓN");
            GUILayout.Label($"Reloj único (unidireccional válida): {(SingleClock ? "sí" : "no")}");

            if (!string.IsNullOrEmpty(_transientStatus))
                GUILayout.Label(_transientStatus);

            GUILayout.Space(4);

            GUI.enabled = !IsRunning;
            if (GUILayout.Button("Ejecutar todas las aplicables"))
                RunAll();
            GUI.enabled = true;

            GUILayout.Space(4);
            _scroll = GUILayout.BeginScrollView(_scroll);

            foreach (var test in _tests)
            {
                bool applies = test.AppliesTo(configuration);
                GUILayout.BeginHorizontal();

                GUI.enabled = !IsRunning && applies;
                if (GUILayout.Button($"{test.TestId}", GUILayout.Width(46)))
                    RunTest(test);
                GUI.enabled = true;

                string line = applies
                    ? $"{test.DisplayName} — {test.Status}"
                    : $"{test.DisplayName} — no aplica ({test.NotApplicableReason})";
                GUILayout.Label(line);

                GUILayout.EndHorizontal();
            }

            GUILayout.EndScrollView();
            GUILayout.Label($"<size=10>{RunDirectory}</size>",
                new GUIStyle(GUI.skin.label) { richText = true, wordWrap = true });
            GUILayout.Label($"<size=10>{togglePanelKey} oculta este panel</size>",
                new GUIStyle(GUI.skin.label) { richText = true });

            GUILayout.EndArea();
        }
    }
}
