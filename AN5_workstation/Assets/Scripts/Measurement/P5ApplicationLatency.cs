using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;
using Debug = UnityEngine.Debug;

namespace AN5.Measurement
{
    /// P5 — Latencia de ida y vuelta de la aplicación (resolución de cinemática inversa).
    ///
    /// Mide el intercambio real que la interfaz usa para todo lo cartesiano:
    /// se publica una pose en input_cartesian_position y se espera la solución
    /// articular en output_joint_position, resuelta por MATLAB.
    ///
    /// SE REPORTA SEPARADA DE P4 Y NUNCA SUMADA A ELLA. P4 aísla transporte; esta
    /// cifra incluye además el cómputo del solver, que es de otra naturaleza y de otro
    /// orden de magnitud. Presentarlas juntas, o compararlas contra latencias de
    /// comunicación de la literatura, mezclaría dos magnitudes distintas — el mismo
    /// error que el plan advierte para P10.
    ///
    /// LA PRIMERA MUESTRA SE REGISTRA APARTE. La primera llamada a fr5_ik() en una
    /// sesión fresca de MATLAB tarda del orden de 13 s frente a milisegundos para
    /// todas las siguientes: es un costo único de compilación al vuelo, no del
    /// algoritmo, y ya está documentado en SecTrajController.cs:33-39. Incluirlo en el
    /// resumen desplazaría la media y el máximo de forma que no representa la
    /// operación. Queda guardado en su propia columna para que la exclusión sea
    /// declarable en vez de silenciosa.
    ///
    /// LAS POSES SE DERIVAN DE LA POSICIÓN ACTUAL DEL ROBOT. inverse_kinematics.m
    /// arrastra reglas de seguridad de otra celda (caja de posición segura, banda
    /// prohibida en Rx, restricción J4/J5) que pueden rechazar poses legítimas de este
    /// proyecto; ver la nota en ros2_ws/src/an5_mock_sim/README.md. Partir de donde el
    /// robot ya está, con perturbaciones de pocos milímetros, garantiza que lo que se
    /// mide sea el tiempo de resolución y no una racha de rechazos.
    public class P5ApplicationLatency : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("El plan pide un mínimo de 30 y preferentemente 50.")]
        public int samples = 50;

        [Tooltip("Espera entre solicitudes. El protocolo no tiene identificador de " +
                 "correlación, así que las solicitudes NO pueden superponerse: dos en " +
                 "vuelo se cruzarían las respuestas.")]
        public float intervalSeconds = 0.3f;

        [Tooltip("Plazo por solicitud, ya en régimen.")]
        public float timeoutSeconds = 5f;

        [Tooltip("Plazo de la primera solicitud, que absorbe el arranque en frío de " +
                 "MATLAB (~13 s medidos en la práctica).")]
        public float coldStartTimeoutSeconds = 25f;

        [Tooltip("Perturbación aplicada a la pose actual, en milímetros.")]
        public float perturbationMm = 5f;

        private const string InputTopic = "input_cartesian_position";
        private const string OutputTopic = "output_joint_position";

        public override string TestId { get { return "P5"; } }
        public override string DisplayName { get { return "Latencia de aplicación (IK)"; } }

        private readonly object _lock = new object();
        private string _pendingPayload;
        private long _pendingRecvTimestamp;
        private bool _hasPending;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }
            if (session.CartesianSubscriber == null)
            {
                Finish(false, "no se encontró CartesianPositionSubscriber en la escena");
                yield break;
            }

            // Pose base: donde el robot está ahora mismo.
            float[] basePose = session.CartesianSubscriber.GetLastKnownCartesianPositions();
            if (basePose == null || basePose.Length != 6 || IsAllZero(basePose))
            {
                Finish(false, "sin lectura válida de current_cartesian_position " +
                              "(¿está publicando el emulador o el robot?)");
                yield break;
            }

            RosSocket socket = session.Socket;
            string advId = socket.Advertise<RosString>(InputTopic);
            string subId = socket.Subscribe<RosString>(
                OutputTopic, OnIkResult, throttle_rate: 0, queue_length: 0);

            yield return new WaitForSeconds(0.3f);

            var csv = session.OpenCsv($"{TestId}_aplicacion_ik",
                "muestra", "exito", "ida_vuelta_ms", "arranque_en_frio",
                "pose_x", "pose_y", "pose_z", "pose_rx", "pose_ry", "pose_rz",
                "respuesta", "motivo_fallo");

            var latencies = new List<double>();
            double coldStartMs = double.NaN;
            int failures = 0;

            for (int i = 0; i < samples; i++)
            {
                bool isFirst = (i == 0);
                SetStatus(isFirst
                    ? "muestra 1 (posible arranque en frío de MATLAB, hasta " +
                      $"{coldStartTimeoutSeconds:F0} s)"
                    : $"muestra {i + 1}/{samples}");

                float[] pose = (float[])basePose.Clone();
                // Perturbación alternada en X para que cada solicitud sea distinta y
                // MATLAB no pueda estar devolviendo un resultado memorizado.
                pose[0] += (i % 2 == 0 ? perturbationMm : -perturbationMm);

                double ms = double.NaN;
                string reply = null, failReason = null;

                yield return StartCoroutine(SingleRequest(
                    socket, advId, pose,
                    isFirst ? coldStartTimeoutSeconds : timeoutSeconds,
                    (okResult, msResult, replyResult, reasonResult) =>
                    {
                        ms = msResult; reply = replyResult; failReason = reasonResult;
                    }));

                bool ok = failReason == null;

                csv.WriteRow(i, ok, ok ? (object)ms : null, isFirst,
                    pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
                    reply, failReason);

                if (ok)
                {
                    if (isFirst) coldStartMs = ms;
                    else latencies.Add(ms);
                }
                else
                {
                    failures++;
                }

                yield return new WaitForSeconds(intervalSeconds);
            }

            csv.Dispose();

            var st = Stats.From(latencies);
            var summary = session.OpenCsv($"{TestId}_aplicacion_ik_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("plataforma", session.platformLabel);
            summary.WriteRow("muestras_solicitadas", samples);
            summary.WriteRow("muestras_validas_en_regimen", st.Count);
            summary.WriteRow("muestras_fallidas", failures);
            summary.WriteRow("ida_vuelta_media_ms", st.Mean);
            summary.WriteRow("ida_vuelta_mediana_ms", st.Median);
            summary.WriteRow("ida_vuelta_p95_ms", st.P95);
            summary.WriteRow("ida_vuelta_maximo_ms", st.Max);
            summary.WriteRow("ida_vuelta_minimo_ms", st.Min);
            summary.WriteRow("ida_vuelta_desviacion_ms", st.StdDev);
            summary.WriteRow("primera_muestra_ms", coldStartMs);
            summary.WriteRow("primera_muestra_excluida", true);
            summary.WriteRow("primera_muestra_motivo",
                "arranque en frío de MATLAB: costo único de compilación al vuelo, " +
                "no representa la operación en régimen");
            summary.WriteRow("incluye_computo_del_solver", true);
            summary.WriteRow("comparable_con_P4", false);
            summary.Dispose();

            socket.Unsubscribe(subId);
            socket.Unadvertise(advId);

            bool succeeded = st.Count > 0;
            Finish(succeeded, succeeded
                ? $"n={st.Count}, mediana={st.Median:F1} ms, p95={st.P95:F1} ms" +
                  (double.IsNaN(coldStartMs) ? "" : $" (1ª muestra {coldStartMs:F0} ms, excluida)")
                : "sin muestras válidas");
        }

        private static bool IsAllZero(float[] v)
        {
            foreach (float f in v) if (Mathf.Abs(f) > 1e-6f) return false;
            return true;
        }

        private IEnumerator SingleRequest(RosSocket socket, string advId, float[] pose,
                                          float timeout,
                                          System.Action<bool, double, string, string> onDone)
        {
            lock (_lock) { _hasPending = false; }

            string payload = string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},{3},{4},{5}",
                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

            long sendTimestamp = Stopwatch.GetTimestamp();
            socket.Publish(advId, new RosString { data = payload });

            float waited = 0f;
            while (waited < timeout)
            {
                string data = null;
                long recvTimestamp = 0;
                lock (_lock)
                {
                    if (_hasPending)
                    {
                        data = _pendingPayload;
                        recvTimestamp = _pendingRecvTimestamp;
                        _hasPending = false;
                    }
                }

                if (data != null)
                {
                    double ms = (recvTimestamp - sendTimestamp) * 1000.0 / Stopwatch.Frequency;

                    // Las dos convenciones de fallo que este tópico usa en la práctica:
                    // el prefijo explícito del emulador, y el NaN literal de MATLAB
                    // para una pose sin solución (su comprobación por isempty() no
                    // atrapa un arreglo de NaN). Ver SecTrajController.cs:344-351.
                    if (data.StartsWith("ERROR:"))
                    {
                        onDone?.Invoke(false, ms, data, data.Substring("ERROR:".Length));
                        yield break;
                    }
                    if (data.IndexOf("NaN", System.StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        onDone?.Invoke(false, ms, data, "pose sin solución (IK devolvió NaN)");
                        yield break;
                    }

                    onDone?.Invoke(true, ms, data, null);
                    yield break;
                }

                yield return null;
                waited += Time.unscaledDeltaTime;
            }

            onDone?.Invoke(false, double.NaN, null,
                "plazo vencido esperando respuesta de IK (¿está corriendo matlab_ik_node?)");
        }

        /// Hilo de red. El instante de llegada se sella acá y no en la corrutina, por
        /// el mismo motivo detallado en P4TransportLatency.SingleProbe: detectarlo un
        /// cuadro después sumaría hasta ~16 ms a cada muestra.
        private void OnIkResult(RosString message)
        {
            long ts = Stopwatch.GetTimestamp();
            lock (_lock)
            {
                _pendingPayload = message.data;
                _pendingRecvTimestamp = ts;
                _hasPending = true;
            }
        }
    }
}
