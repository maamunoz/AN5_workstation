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
    /// P4 — Latencia de transporte (tiempo de ida y vuelta).
    ///
    /// Mide el viaje completo Unity -> rosbridge -> grafo ROS 2 -> measurement_probe
    /// -> grafo -> rosbridge -> Unity, cronometrado íntegramente contra el reloj de
    /// Unity. Al ocurrir toda la aritmética en un solo reloj, el desfase entre equipos
    /// desaparece del problema: por eso es la medida principal en las cuatro
    /// configuraciones, incluidas C1/C2 donde los relojes no están sincronizados.
    ///
    /// Es una sonda DELIBERADAMENTE VACÍA: el nodo sonda solo reenvía lo que recibe,
    /// sin resolver cinemática ni mover nada. Así la cifra aísla el costo de
    /// puente + red, que es exactamente lo que distingue a C1/C3. La ida y vuelta de
    /// la aplicación real, que sí incluye el cómputo del solver, se mide aparte en P5
    /// y se reporta por separado — sumarlas mezclaría transporte con cómputo.
    ///
    /// EN C3/C4 además se descompone el viaje en tramos. El nodo sonda agrega su
    /// instante de recepción y de publicación al mensaje de vuelta; como ahí Unity y
    /// el middleware comparten equipo y reloj, restar esas marcas contra las de Unity
    /// da tramo de subida, costo interno de la sonda y tramo de bajada. En C1/C2 esas
    /// mismas columnas se registran igual pero quedan marcadas como no interpretables,
    /// porque compararían marcas de dos relojes distintos.
    public class P4TransportLatency : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("El plan pide un mínimo de 30 y preferentemente 50.")]
        public int samples = 50;

        [Tooltip("Espera entre sondas. Suficiente para que no se encolen unas sobre " +
                 "otras: sondas superpuestas medirían tiempo de cola, no de transporte.")]
        public float intervalSeconds = 0.2f;

        [Tooltip("Plazo por sonda. Vencido, la muestra se registra como fallida y se " +
                 "sigue con la siguiente.")]
        public float timeoutSeconds = 5f;

        [Tooltip("Sondas de calentamiento previas, descartadas. La primera publicación " +
                 "sobre un tópico recién anunciado paga el establecimiento de la " +
                 "suscripción del lado de rosbridge y no representa el estado estable.")]
        public int warmupSamples = 5;

        private const string PingTopic = "probe/ping";
        private const string PongTopic = "probe/pong";
        private const string LoopbackTopic = "probe/unity_loopback";

        public override string TestId { get { return "P4"; } }
        public override string DisplayName { get { return "Latencia de transporte (ida y vuelta)"; } }

        // Traspaso desde el hilo de red de RosSharp al hilo principal.
        private readonly object _lock = new object();
        private string _pendingPayload;
        private long _pendingRecvTimestamp;
        private long _pendingRecvUnixNanos;
        private bool _hasPending;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }

            RosSocket socket = session.Socket;
            bool usingProbeNode = true;

            // Se anuncia y se suscribe con queue_length 0 y sin regulación: cualquier
            // encolado del lado de rosbridge se sumaría a lo que se quiere medir.
            string pingAdvId = socket.Advertise<RosString>(PingTopic);
            string pingId = pingAdvId;
            string pongSubId = socket.Subscribe<RosString>(
                PongTopic, OnProbeMessage, throttle_rate: 0, queue_length: 0);

            string loopbackAdvId = null;
            string loopbackSubId = null;

            SetStatus("verificando nodo sonda...");
            yield return new WaitForSeconds(0.5f);

            // ¿Está corriendo measurement_probe? Se comprueba con una sonda suelta
            // antes de comprometerse, para poder replegarse en vez de registrar 50
            // fallos seguidos.
            bool probeAlive = false;
            yield return StartCoroutine(SingleProbe(socket, pingId, -1, 1.5f,
                (ok, _, __) => probeAlive = ok));

            if (!probeAlive)
            {
                usingProbeNode = false;
                // En esta rama measurement_probe_enabled ya viene en true por defecto
                // (ver sim.launch.py/real.launch.py); si no responde igual, lo más
                // probable es que no se haya levantado el launch, que corra un
                // paquete an5_mock_sim viejo sin recompilar, o que alguien lo haya
                // desactivado a mano con measurement_probe_enabled:=false.
                Debug.LogWarning("[P4] measurement_probe no responde. Repliegue a lazo " +
                                 "propio por rosbridge (sin descomposición en tramos). " +
                                 "Verificá que el launch esté corriendo con " +
                                 "measurement_probe_enabled:=true (default en esta rama) " +
                                 "y que an5_mock_sim esté recompilado.");
                socket.Unsubscribe(pongSubId);
                pongSubId = null;

                loopbackAdvId = socket.Advertise<RosString>(LoopbackTopic);
                loopbackSubId = socket.Subscribe<RosString>(
                    LoopbackTopic, OnProbeMessage, throttle_rate: 0, queue_length: 0);
                pingId = loopbackAdvId;
                yield return new WaitForSeconds(0.5f);
            }

            var csv = session.OpenCsv($"{TestId}_transporte",
                "muestra", "id", "exito", "ida_vuelta_ms",
                "unity_envio_unix_ns", "sonda_recepcion_unix_ns",
                "sonda_publicacion_unix_ns", "unity_recepcion_unix_ns",
                "sonda_interno_ms", "tramo_subida_ms", "tramo_bajada_ms",
                "tramos_interpretables", "modo");

            string mode = usingProbeNode ? "sonda_ros" : "lazo_unity";
            var rtts = new List<double>();
            var uplinks = new List<double>();
            var downlinks = new List<double>();
            int failures = 0;

            for (int i = 0; i < warmupSamples; i++)
            {
                SetStatus($"calentamiento {i + 1}/{warmupSamples}");
                yield return StartCoroutine(SingleProbe(socket, pingId, -100 - i, timeoutSeconds, null));
                yield return new WaitForSeconds(intervalSeconds);
            }

            for (int i = 0; i < samples; i++)
            {
                SetStatus($"muestra {i + 1}/{samples}");

                bool ok = false;
                double rttMs = double.NaN;
                ProbeStamps stamps = default;

                yield return StartCoroutine(SingleProbe(socket, pingId, i, timeoutSeconds,
                    (success, ms, s) => { ok = success; rttMs = ms; stamps = s; }));

                double internalMs = double.NaN, uplinkMs = double.NaN, downlinkMs = double.NaN;
                if (ok && stamps.HasProbeStamps)
                {
                    internalMs = (stamps.ProbePublishNanos - stamps.ProbeReceiveNanos) / 1e6;
                    uplinkMs = (stamps.ProbeReceiveNanos - stamps.UnitySendNanos) / 1e6;
                    downlinkMs = (stamps.UnityReceiveNanos - stamps.ProbePublishNanos) / 1e6;
                }

                csv.WriteRow(i, ok ? i : -1, ok, ok ? (object)rttMs : null,
                    stamps.UnitySendNanos,
                    stamps.HasProbeStamps ? (object)stamps.ProbeReceiveNanos : null,
                    stamps.HasProbeStamps ? (object)stamps.ProbePublishNanos : null,
                    stamps.UnityReceiveNanos,
                    double.IsNaN(internalMs) ? null : (object)internalMs,
                    double.IsNaN(uplinkMs) ? null : (object)uplinkMs,
                    double.IsNaN(downlinkMs) ? null : (object)downlinkMs,
                    session.SingleClock, mode);

                if (ok)
                {
                    rtts.Add(rttMs);
                    // Los tramos solo se agregan al resumen cuando hay un reloj único:
                    // en C1/C2 restarían marcas de relojes distintos y el resultado no
                    // significaría nada.
                    if (session.SingleClock && stamps.HasProbeStamps)
                    {
                        uplinks.Add(uplinkMs);
                        downlinks.Add(downlinkMs);
                    }
                }
                else
                {
                    failures++;
                }

                yield return new WaitForSeconds(intervalSeconds);
            }

            csv.Dispose();

            // --- Resumen ---
            var rttStats = Stats.From(rtts);
            var summary = session.OpenCsv($"{TestId}_transporte_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("plataforma", session.platformLabel);
            summary.WriteRow("modo_sonda", mode);
            summary.WriteRow("muestras_solicitadas", samples);
            summary.WriteRow("muestras_validas", rttStats.Count);
            summary.WriteRow("muestras_fallidas", failures);
            summary.WriteRow("ida_vuelta_media_ms", rttStats.Mean);
            summary.WriteRow("ida_vuelta_mediana_ms", rttStats.Median);
            summary.WriteRow("ida_vuelta_p95_ms", rttStats.P95);
            summary.WriteRow("ida_vuelta_maximo_ms", rttStats.Max);
            summary.WriteRow("ida_vuelta_minimo_ms", rttStats.Min);
            summary.WriteRow("ida_vuelta_desviacion_ms", rttStats.StdDev);

            if (session.SingleClock && uplinks.Count > 0)
            {
                var up = Stats.From(uplinks);
                var down = Stats.From(downlinks);
                summary.WriteRow("tramos_interpretables", true);
                summary.WriteRow("subida_mediana_ms", up.Median);
                summary.WriteRow("subida_p95_ms", up.P95);
                summary.WriteRow("bajada_mediana_ms", down.Median);
                summary.WriteRow("bajada_p95_ms", down.P95);
            }
            else
            {
                summary.WriteRow("tramos_interpretables", false);
                summary.WriteRow("tramos_motivo", session.SingleClock
                    ? "sin marcas de la sonda (modo lazo propio)"
                    : "middleware remoto: relojes distintos, tramos no comparables");
            }
            summary.Dispose();

            // Limpieza de todo lo anunciado y suscrito, incluido el anuncio original
            // de probe/ping cuando hubo repliegue: dejarlo colgado mantendría un
            // publicador vivo en el grafo para el resto de la sesión.
            if (pongSubId != null) socket.Unsubscribe(pongSubId);
            if (loopbackSubId != null) socket.Unsubscribe(loopbackSubId);
            socket.Unadvertise(pingAdvId);
            if (loopbackAdvId != null) socket.Unadvertise(loopbackAdvId);

            bool succeeded = rttStats.Count > 0;
            Finish(succeeded, succeeded
                ? $"n={rttStats.Count}, mediana={rttStats.Median:F2} ms, p95={rttStats.P95:F2} ms" +
                  (failures > 0 ? $" ({failures} fallidas)" : "")
                : "sin muestras válidas");
        }

        private struct ProbeStamps
        {
            public long UnitySendNanos;
            public long UnityReceiveNanos;
            public long ProbeReceiveNanos;
            public long ProbePublishNanos;
            public bool HasProbeStamps;
        }

        /// Emite una sonda y espera su retorno.
        ///
        /// EL DETALLE QUE HACE O DESHACE ESTA MEDICIÓN: el instante de llegada se
        /// toma DENTRO del callback de red (OnProbeMessage), no acá. RosSharp entrega
        /// los mensajes en el hilo de WebSocketSharp, y una corrutina recién puede
        /// enterarse en el siguiente frame. Cronometrar en el frame en que se
        /// "detecta" la respuesta le sumaría a cada muestra hasta un cuadro entero
        /// (~16 ms a 60 FPS), que en localhost es varias veces la latencia real: la
        /// medición reportaría, en la práctica, el período de refresco de Unity.
        private IEnumerator SingleProbe(RosSocket socket, string advertiseId, int id,
                                        float timeout, System.Action<bool, double, ProbeStamps> onDone)
        {
            lock (_lock) { _hasPending = false; }

            var stamps = new ProbeStamps();
            stamps.UnitySendNanos = HighResolutionClock.UnixNanos();

            long sendTimestamp = Stopwatch.GetTimestamp();
            var msg = new RosString
            {
                data = string.Format(CultureInfo.InvariantCulture, "{0},{1}",
                                     id, stamps.UnitySendNanos)
            };
            socket.Publish(advertiseId, msg);

            float waited = 0f;
            while (waited < timeout)
            {
                string payload = null;
                long recvTimestamp = 0, recvUnix = 0;

                lock (_lock)
                {
                    if (_hasPending)
                    {
                        payload = _pendingPayload;
                        recvTimestamp = _pendingRecvTimestamp;
                        recvUnix = _pendingRecvUnixNanos;
                        _hasPending = false;
                    }
                }

                if (payload != null)
                {
                    string[] parts = payload.Split(',');
                    if (parts.Length >= 2 &&
                        int.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture,
                                     out int replyId) &&
                        replyId == id)
                    {
                        stamps.UnityReceiveNanos = recvUnix;

                        // Cuatro campos = el nodo sonda agregó sus dos marcas.
                        // Dos campos = eco propio de Unity, sin descomposición.
                        if (parts.Length >= 4 &&
                            long.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture,
                                          out long probeRecv) &&
                            long.TryParse(parts[3], NumberStyles.Integer, CultureInfo.InvariantCulture,
                                          out long probePub))
                        {
                            stamps.ProbeReceiveNanos = probeRecv;
                            stamps.ProbePublishNanos = probePub;
                            stamps.HasProbeStamps = true;
                        }

                        double rttMs = (recvTimestamp - sendTimestamp) * 1000.0 / Stopwatch.Frequency;
                        onDone?.Invoke(true, rttMs, stamps);
                        yield break;
                    }
                    // Identificador que no coincide: respuesta rezagada de una sonda
                    // anterior. Se descarta y se sigue esperando la propia.
                }

                yield return null;
                waited += Time.unscaledDeltaTime;
            }

            stamps.UnityReceiveNanos = 0;
            onDone?.Invoke(false, double.NaN, stamps);
        }

        /// Corre en el hilo de red de RosSharp. Lo único que hace es sellar el
        /// instante de llegada y encolar el payload crudo (ver comentario en
        /// SingleProbe sobre por qué el sellado va acá y no en la corrutina).
        private void OnProbeMessage(RosString message)
        {
            long ts = Stopwatch.GetTimestamp();
            long unix = HighResolutionClock.UnixNanos();
            lock (_lock)
            {
                _pendingPayload = message.data;
                _pendingRecvTimestamp = ts;
                _pendingRecvUnixNanos = unix;
                _hasPending = true;
            }
        }
    }
}
