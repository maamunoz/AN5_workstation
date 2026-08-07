using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Debug = UnityEngine.Debug;

namespace AN5.Measurement
{
    /// P1 — Establecimiento de enlace.
    ///
    /// Mide cuánto tarda el cliente en quedar conectado a rosbridge, desde el intento
    /// hasta la confirmación, y con qué tasa de éxito.
    ///
    /// ABRE SU PROPIO SOCKET en vez de reconectar el de la aplicación. Hay dos motivos:
    ///
    ///   - RosConnector.ReconnectNow() aborta el hilo de reconexión con Thread.Abort()
    ///     (RosConnector.cs:139), que en los tiempos de ejecución actuales de .NET es
    ///     poco fiable y puede dejar el hilo en estado indefinido. Llamarlo diez veces
    ///     seguidas para medir es justamente el peor uso posible.
    ///   - Cortar la conexión viva rompería el resto de la sesión de mediciones: los
    ///     subscriptores de la aplicación quedarían apuntando a un socket muerto (es el
    ///     problema que JointPositionSubscriber.cs:34-40 documenta y remedia a mano).
    ///
    /// Un socket propio y desechable mide exactamente lo mismo — el costo de establecer
    /// una conexión nueva contra el mismo endpoint — sin tocar nada de lo demás.
    public class P1LinkEstablishment : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Repeticiones. El plan pide 10 por combinación.")]
        public int repetitions = 10;

        [Tooltip("Plazo por intento.")]
        public float timeoutSeconds = 20f;

        [Tooltip("Pausa entre intentos, para no dejar sockets a medio cerrar.")]
        public float pauseSeconds = 1.5f;

        public override string TestId { get { return "P1"; } }
        public override string DisplayName { get { return "Establecimiento de enlace"; } }

        private volatile bool _connected;
        private long _connectedTimestamp;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (session.Connector == null)
            {
                Finish(false, "no se encontró RosConnector del que tomar la configuración");
                yield break;
            }

            string url = session.Connector.RosBridgeServerUrl;
            var protocol = session.Connector.protocol;
            var serializer = session.Connector.Serializer;

            var csv = session.OpenCsv($"{TestId}_enlace",
                "repeticion", "exito", "segundos", "url");

            var times = new List<double>();
            int failures = 0;

            for (int i = 0; i < repetitions; i++)
            {
                SetStatus($"intento {i + 1}/{repetitions}");

                _connected = false;
                _connectedTimestamp = 0;

                long start = Stopwatch.GetTimestamp();
                RosSocket socket = null;

                try
                {
                    socket = RosConnector.ConnectToRos(
                        protocol, url,
                        (s, e) =>
                        {
                            // Sello del instante en el propio callback: detectarlo un
                            // cuadro después le sumaría hasta ~16 ms a cada medición.
                            _connectedTimestamp = Stopwatch.GetTimestamp();
                            _connected = true;
                        },
                        (s, e) => { },
                        serializer);
                }
                catch (System.Exception e)
                {
                    Debug.LogWarning($"[P1] Intento {i + 1} falló al crear el socket: {e.Message}");
                }

                float waited = 0f;
                while (!_connected && waited < timeoutSeconds)
                {
                    yield return null;
                    waited += Time.unscaledDeltaTime;
                }

                if (_connected)
                {
                    double seconds = (_connectedTimestamp - start) / (double)Stopwatch.Frequency;
                    times.Add(seconds);
                    csv.WriteRow(i + 1, true, seconds, url);
                }
                else
                {
                    failures++;
                    csv.WriteRow(i + 1, false, null, url);
                }

                if (socket != null)
                {
                    try { socket.Close(); }
                    catch (System.Exception e)
                    {
                        Debug.LogWarning($"[P1] Error cerrando el socket de prueba: {e.Message}");
                    }
                }

                yield return new WaitForSeconds(pauseSeconds);
            }

            csv.Dispose();

            var st = Stats.From(times);
            var summary = session.OpenCsv($"{TestId}_enlace_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("plataforma", session.platformLabel);
            summary.WriteRow("url", url);
            summary.WriteRow("intentos", repetitions);
            summary.WriteRow("exitosos", st.Count);
            summary.WriteRow("fallidos", failures);
            summary.WriteRow("media_s", st.Mean);
            summary.WriteRow("mediana_s", st.Median);
            summary.WriteRow("maximo_s", st.Max);
            summary.WriteRow("minimo_s", st.Min);
            // El plan lo dice explícitamente: si siempre es inferior a un segundo,
            // alcanza con una frase en el texto y no merece una tabla propia.
            summary.WriteRow("siempre_menor_a_1s", st.Count > 0 && st.Max < 1.0);
            summary.Dispose();

            bool ok = st.Count > 0;
            Finish(ok, ok
                ? $"{st.Count}/{repetitions} conectados, media {st.Mean:F3} s, máx {st.Max:F3} s"
                : "ningún intento se conectó");
        }
    }
}
