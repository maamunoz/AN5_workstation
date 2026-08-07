using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosString = RosSharp.RosBridgeClient.MessageTypes.Std.String;
using JointStateMsg = RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState;
using Debug = UnityEngine.Debug;

namespace AN5.Measurement
{
    /// P3 — Frecuencia efectiva de actualización y pérdida de mensajes.
    ///
    /// Mide CUATRO cosas distintas que es tentador confundir en una sola cifra:
    ///
    ///   (a) Mensajes que el middleware realmente emitió, contados sin ambigüedad
    ///       gracias al contador monótono de measurement_probe (probe/seq). Los
    ///       tópicos CSV que la aplicación consume de verdad son seis floats pelados,
    ///       sin secuencia ni marca de tiempo: sobre ellos es imposible distinguir
    ///       "no me llegó" de "nunca se publicó". Con el contador, la pérdida se
    ///       CUENTA por huecos en vez de inferirse.
    ///
    ///   (b) Mensajes de estado que efectivamente llegaron al cliente, contados en el
    ///       hilo de red con una suscripción propia a current_joint_position.
    ///
    ///   (c) Mensajes de estado que el cliente efectivamente PROCESÓ, contados sobre
    ///       el evento OnJointPositionsUpdated que la aplicación ya expone.
    ///
    ///   (d) Opcionalmente, pérdida sobre /joint_states por huecos en header.stamp,
    ///       como control que no depende del nodo sonda.
    ///
    /// LA DIFERENCIA ENTRE (b) Y (c) ES UN RESULTADO, NO UN ERROR. El subscriptor de
    /// la aplicación (JointPositionSubscriber.cs:115-122) guarda cada mensaje entrante
    /// en un buffer de UN SOLO SLOT que Update() drena una vez por cuadro: si llegan
    /// dos mensajes entre cuadro y cuadro, el primero se sobrescribe y se pierde. A
    /// 50 Hz de publicación y ~60 cuadros por segundo eso ya descarta mensajes en el
    /// cliente, sin que la red tenga absolutamente nada que ver. Reportar esa brecha
    /// como "pérdida" sería atribuir a la red un descarte deliberado de la aplicación;
    /// por eso las dos cifras van separadas y así deben presentarse en el artículo.
    ///
    /// La ventana corre con el robot en movimiento continuo, como pide el plan: medir
    /// con el sistema en reposo y presentarlo como desempeño en operación sería una de
    /// las trampas que el propio plan enumera.
    public class P3RateAndLoss : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Duración de la ventana de observación. El plan pide 60 s.")]
        public float windowSeconds = 60f;

        [Tooltip("Mantiene el robot en movimiento continuo durante la ventana. " +
                 "Desactivalo solo si vas a mover el brazo por otro medio.")]
        public bool driveRobotDuringWindow = true;

        [Tooltip("Segundos entre cambios de consigna del movimiento de fondo.")]
        public float motionSwitchSeconds = 5f;

        [Tooltip("Velocidad del movimiento de fondo, en porcentaje.")]
        public float motionSpeedPct = 30f;

        [Tooltip("Control adicional sobre /joint_states usando header.stamp. Es " +
                 "independiente del nodo sonda, pero agrega tráfico que puede alterar " +
                 "lo que se está midiendo: por eso está apagado por defecto.")]
        public bool alsoMeasureJointStates = false;

        private const string SeqTopic = "probe/seq";
        private const string StateTopic = "current_joint_position";
        private const string JointStatesTopic = "joint_states";

        public override string TestId { get { return "P3"; } }
        public override string DisplayName { get { return "Frecuencia efectiva y pérdida"; } }

        // Contadores tocados desde el hilo de red: Interlocked para los simples,
        // lock para lo que necesita coherencia entre varios campos a la vez.
        private long _stateArrivals;
        private long _processedByClient;

        private readonly object _seqLock = new object();
        private long _seqReceived;
        private long _seqGaps;
        private long _seqFirst = -1;
        private long _seqLast = -1;
        private readonly List<double> _oneWayMs = new List<double>();
        private readonly List<long> _oneWaySeq = new List<long>();

        private readonly object _jsLock = new object();
        private long _jsReceived;
        private long _jsGapMessages;
        private double _jsLastStampSeconds = -1.0;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }
            if (session.JointSubscriber == null)
            {
                Finish(false, "no se encontró JointPositionSubscriber en la escena");
                yield break;
            }

            ResetCounters();

            RosSocket socket = session.Socket;

            // queue_length 0 y sin regulación: cualquier encolado o descarte del lado
            // de rosbridge se confundiría con pérdida de red.
            string seqSub = socket.Subscribe<RosString>(
                SeqTopic, OnSeqMessage, throttle_rate: 0, queue_length: 0);
            string stateSub = socket.Subscribe<RosString>(
                StateTopic, OnStateMessage, throttle_rate: 0, queue_length: 0);
            string jsSub = null;
            if (alsoMeasureJointStates)
            {
                jsSub = socket.Subscribe<JointStateMsg>(
                    JointStatesTopic, OnJointStateMessage, throttle_rate: 0, queue_length: 0);
            }

            session.JointSubscriber.OnJointPositionsUpdated += OnClientProcessed;

            Coroutine motion = null;
            if (driveRobotDuringWindow)
            {
                SetStatus("preparando el robot para el movimiento de fondo...");
                yield return StartCoroutine(PrepareRobotForMotion(session));
                motion = StartCoroutine(KeepRobotMoving(session));
            }

            // Un instante de asentamiento para que las suscripciones estén activas
            // antes de empezar a contar; si no, el primer segundo aparecería vacío.
            yield return new WaitForSeconds(1.0f);
            ResetCounters();

            var bins = session.OpenCsv($"{TestId}_frecuencia_bins",
                "segundo", "estado_llegados", "estado_procesados",
                "sonda_recibidos", "sonda_huecos",
                "joint_states_recibidos", "joint_states_huecos");

            long prevArrivals = 0, prevProcessed = 0, prevSeqRecv = 0, prevSeqGaps = 0;
            long prevJs = 0, prevJsGaps = 0;

            int totalSeconds = Mathf.CeilToInt(windowSeconds);
            for (int s = 0; s < totalSeconds; s++)
            {
                yield return new WaitForSeconds(1f);

                long arrivals = Interlocked.Read(ref _stateArrivals);
                long processed = Interlocked.Read(ref _processedByClient);
                long seqRecv, seqGaps, jsRecv, jsGaps;
                lock (_seqLock) { seqRecv = _seqReceived; seqGaps = _seqGaps; }
                lock (_jsLock) { jsRecv = _jsReceived; jsGaps = _jsGapMessages; }

                bins.WriteRow(s + 1,
                    arrivals - prevArrivals,
                    processed - prevProcessed,
                    seqRecv - prevSeqRecv,
                    seqGaps - prevSeqGaps,
                    jsRecv - prevJs,
                    jsGaps - prevJsGaps);

                prevArrivals = arrivals; prevProcessed = processed;
                prevSeqRecv = seqRecv; prevSeqGaps = seqGaps;
                prevJs = jsRecv; prevJsGaps = jsGaps;

                SetStatus($"ventana {s + 1}/{totalSeconds} s — " +
                          $"llegados {arrivals}, procesados {processed}");
            }
            bins.Dispose();

            // --- Cierre y limpieza ---
            if (motion != null) StopCoroutine(motion);
            if (session.CommandSender != null) session.CommandSender.SendCommand("StopMotion()");

            session.JointSubscriber.OnJointPositionsUpdated -= OnClientProcessed;
            socket.Unsubscribe(seqSub);
            socket.Unsubscribe(stateSub);
            if (jsSub != null) socket.Unsubscribe(jsSub);

            yield return WriteSummary(session);
        }

        private IEnumerator WriteSummary(MeasurementSession session)
        {
            long arrivals = Interlocked.Read(ref _stateArrivals);
            long processed = Interlocked.Read(ref _processedByClient);

            long seqRecv, seqGaps, seqFirst, seqLast;
            List<double> oneWay;
            List<long> oneWaySeq;
            lock (_seqLock)
            {
                seqRecv = _seqReceived; seqGaps = _seqGaps;
                seqFirst = _seqFirst; seqLast = _seqLast;
                oneWay = new List<double>(_oneWayMs);
                oneWaySeq = new List<long>(_oneWaySeq);
            }

            // Emitidos por el middleware = recorrido del contador. Es una cuenta
            // exacta, no una estimación a partir de la frecuencia nominal.
            long seqEmitted = (seqFirst >= 0 && seqLast >= seqFirst) ? (seqLast - seqFirst + 1) : 0;
            double seqLossPct = seqEmitted > 0 ? 100.0 * (seqEmitted - seqRecv) / seqEmitted : double.NaN;

            long jsRecv, jsGaps;
            lock (_jsLock) { jsRecv = _jsReceived; jsGaps = _jsGapMessages; }

            var csv = session.OpenCsv($"{TestId}_frecuencia_resumen", "metrica", "valor");
            csv.WriteRow("configuracion", session.ShortConfigLabel());
            csv.WriteRow("plataforma", session.platformLabel);
            csv.WriteRow("ventana_s", windowSeconds);
            csv.WriteRow("robot_en_movimiento", driveRobotDuringWindow);

            csv.WriteRow("sonda_emitidos", seqEmitted);
            csv.WriteRow("sonda_recibidos", seqRecv);
            csv.WriteRow("sonda_perdidos", seqEmitted - seqRecv);
            csv.WriteRow("sonda_perdida_pct", seqLossPct);
            csv.WriteRow("sonda_huecos_detectados", seqGaps);
            csv.WriteRow("sonda_frecuencia_recibida_hz", seqRecv / windowSeconds);

            csv.WriteRow("estado_llegados", arrivals);
            csv.WriteRow("estado_frecuencia_llegada_hz", arrivals / windowSeconds);
            csv.WriteRow("estado_procesados_cliente", processed);
            csv.WriteRow("estado_frecuencia_efectiva_hz", processed / windowSeconds);
            csv.WriteRow("estado_descartados_por_cliente", arrivals - processed);
            csv.WriteRow("estado_descarte_cliente_pct",
                arrivals > 0 ? 100.0 * (arrivals - processed) / arrivals : double.NaN);
            // Se nombra explícitamente para que nadie lo lea como pérdida de red.
            csv.WriteRow("estado_descarte_cliente_causa",
                "buffer de un slot en JointPositionSubscriber, drenado una vez por cuadro; " +
                "no es pérdida de red");

            if (alsoMeasureJointStates)
            {
                csv.WriteRow("joint_states_recibidos", jsRecv);
                csv.WriteRow("joint_states_mensajes_faltantes_por_stamp", jsGaps);
            }

            if (oneWay.Count > 0)
            {
                var st = Stats.From(oneWay);
                csv.WriteRow("unidireccional_interpretable", session.SingleClock);
                if (session.SingleClock)
                {
                    csv.WriteRow("unidireccional_estado_mediana_ms", st.Median);
                    csv.WriteRow("unidireccional_estado_p95_ms", st.P95);
                    csv.WriteRow("unidireccional_estado_maximo_ms", st.Max);
                    csv.WriteRow("unidireccional_estado_n", st.Count);
                }
                else
                {
                    csv.WriteRow("unidireccional_motivo",
                        "middleware remoto: relojes distintos, magnitud no interpretable");
                }
            }
            csv.Dispose();

            // Serie completa de la latencia unidireccional, para no reportar solo el
            // resumen. Se guarda también en C1/C2, marcada como no interpretable, por
            // si más adelante se sincronizan los relojes y se quiere reprocesar.
            if (oneWay.Count > 0)
            {
                var series = session.OpenCsv($"{TestId}_unidireccional_estado",
                    "secuencia", "latencia_ms", "interpretable");
                for (int i = 0; i < oneWay.Count; i++)
                    series.WriteRow(oneWaySeq[i], oneWay[i], session.SingleClock);
                series.Dispose();
            }

            double effectiveHz = processed / windowSeconds;
            Finish(true, $"efectiva {effectiveHz:F1} Hz, llegada {arrivals / windowSeconds:F1} Hz, " +
                         $"pérdida sonda {(double.IsNaN(seqLossPct) ? 0 : seqLossPct):F2} %");
            yield break;
        }

        private void ResetCounters()
        {
            Interlocked.Exchange(ref _stateArrivals, 0);
            Interlocked.Exchange(ref _processedByClient, 0);
            lock (_seqLock)
            {
                _seqReceived = 0; _seqGaps = 0; _seqFirst = -1; _seqLast = -1;
                _oneWayMs.Clear(); _oneWaySeq.Clear();
            }
            lock (_jsLock) { _jsReceived = 0; _jsGapMessages = 0; _jsLastStampSeconds = -1.0; }
        }

        // --- Callbacks del hilo de red ---

        private void OnStateMessage(RosString message)
        {
            Interlocked.Increment(ref _stateArrivals);
        }

        private void OnSeqMessage(RosString message)
        {
            long recvUnix = HighResolutionClock.UnixNanos();

            string[] parts = message.data.Split(',');
            if (parts.Length < 2) return;
            if (!long.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out long seq))
                return;
            long.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out long stampNs);

            lock (_seqLock)
            {
                _seqReceived++;
                if (_seqFirst < 0) _seqFirst = seq;
                else if (seq > _seqLast + 1) _seqGaps += seq - _seqLast - 1;
                if (seq > _seqLast) _seqLast = seq;

                if (stampNs > 0)
                {
                    _oneWayMs.Add((recvUnix - stampNs) / 1e6);
                    _oneWaySeq.Add(seq);
                }
            }
        }

        private void OnJointStateMessage(JointStateMsg message)
        {
            if (message == null || message.header == null) return;
            double stamp = message.header.stamp.secs + message.header.stamp.nsecs / 1e9;

            lock (_jsLock)
            {
                _jsReceived++;
                if (_jsLastStampSeconds > 0)
                {
                    // A cadencia nominal fija, un salto de más de 1,5 períodos delata
                    // mensajes que no llegaron. Es una inferencia, a diferencia del
                    // conteo exacto de probe/seq: por eso este control es secundario.
                    double nominal = 1.0 / 50.0;
                    double gap = stamp - _jsLastStampSeconds;
                    if (gap > nominal * 1.5)
                        _jsGapMessages += (long)System.Math.Round(gap / nominal) - 1;
                }
                _jsLastStampSeconds = stamp;
            }
        }

        // --- Callback del hilo principal ---
        private void OnClientProcessed(float[] positions)
        {
            Interlocked.Increment(ref _processedByClient);
        }

        /// Movimiento de fondo: alterna entre dos configuraciones articulares para que
        /// el flujo de estado tenga contenido cambiante durante toda la ventana. Sin
        /// esto se estaría midiendo el sistema en reposo y presentándolo como
        /// desempeño en operación, que es justamente uno de los errores que el plan
        /// de mediciones pide evitar.
        private IEnumerator KeepRobotMoving(MeasurementSession session)
        {
            var sender = session.CommandSender;
            if (sender == null) yield break;

            float[] poseA = { 0f, -90f, 90f, -90f, 90f, 0f };
            float[] poseB = { 30f, -70f, 70f, -100f, 70f, 40f };

            bool useA = false;
            while (true)
            {
                float[] target = useA ? poseA : poseB;
                useA = !useA;

                sender.SendCommand(string.Format(CultureInfo.InvariantCulture,
                    "JNTPoint(1,{0},{1},{2},{3},{4},{5})",
                    target[0], target[1], target[2], target[3], target[4], target[5]));
                yield return new WaitForSeconds(0.1f);
                sender.SendCommand(string.Format(CultureInfo.InvariantCulture,
                    "MoveJ(JNT1,{0:F0})", motionSpeedPct));

                yield return new WaitForSeconds(motionSwitchSeconds);
            }
        }
    }
}
