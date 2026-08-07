using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AN5.Measurement
{
    /// P6 — Teleoperación articular.
    ///
    /// Para un conjunto de consignas conocidas, compara el valor articular alcanzado
    /// —según el estado que el propio robot reporta— contra el comandado, y reporta
    /// error medio y máximo en grados.
    ///
    /// SOLO APLICA A C2 Y C4. Contra el emulador la comparación no significaría nada:
    /// el mock interpola hacia exactamente los ángulos comandados y después publica esa
    /// misma interpolación como estado, así que el error medido sería cero por
    /// construcción. Eso no diría nada sobre teleoperación, solo confirmaría que un
    /// número copiado de una variable a otra no cambia.
    ///
    /// Cada consigna mueve UNA articulación por vez desde una pose base común. Mover
    /// varias a la vez mezclaría los errores y no permitiría atribuirlos.
    ///
    /// SEGURIDAD FÍSICA. Los sectores se recorren dentro de una fracción del rango
    /// declarado en el URDF, no hasta los límites -- pero eso NO alcanza por sí solo:
    /// con `rangeFraction` en su default (0,6), j1 barría directo por dentro de la
    /// zona prohibida de pared (WorkspaceSafety.J1ForbiddenRangeDeg) en dos de sus
    /// cinco consignas, y j2 barría fuera de su rango operativo permitido
    /// (WorkspaceSafety.J2AllowedRangeDeg) en tres, dos de ellas con el efector
    /// atravesando la mesa (Z hasta -464mm). Esto se encontró y corrigió, verificado
    /// con RealRobotForwardKinematics antes de tocar el robot físico:
    ///
    ///   - j1: la zona prohibida de pared, sumada a que basePoseDeg (radio de
    ///     alcance ~507mm) barre la zona trasera cartesiana en un tramo bastante más
    ///     ancho que la zona de pared misma, deja el rango mecánico completo
    ///     ([-175,175]) partido en TRES tramos seguros separados por huecos
    ///     completamente prohibidos: aprox. [-175,-121], [-29,34] y [123,175]. No se
    ///     puede despejar a mano cuáles son (dependen de basePoseDeg completa, no
    ///     solo del rango de j1) -- se escanean en tiempo de ejecución, ver
    ///     ScanSafeJ1Intervals(). Y NO se reparten consignas entre los tres: con
    ///     MoveJ (interpolación articular en línea recta) no hay forma de ir de un
    ///     tramo a otro sin cruzar el hueco entre medio -- ni siquiera pasando por
    ///     `basePoseDeg`, porque basePoseDeg cae en el tramo del MEDIO, así que
    ///     "rutear por la base" para llegar a un tramo lateral simplemente cambia
    ///     cuál mitad del cruce prohibido se hace, no lo elimina (confirmado
    ///     probándolo). Por eso BuildJ1Setpoints() usa SOLO el tramo que contiene
    ///     basePoseDeg[0] -- con la pose base default, un barrido de apenas -24° a
    ///     29°, mucho menos que el resto de las articulaciones. Es una limitación
    ///     real de cobertura para j1, no un descuido.
    ///   - j2: usa directamente WorkspaceSafety.J2AllowedRangeDeg en vez del rango
    ///     mecánico real, igual que ya hace P9.
    ///
    /// TIMING DE LA LECTURA FINAL. Contra el emulador el error medio daba ~0,1-0,2°
    /// en vez de 0, con los errores más grandes agrupados justo debajo de
    /// arrivalToleranceDeg (0,5°) -- señal de un desfase de asentamiento, no de ruido
    /// aleatorio. La causa real NO estaba acá: estaba en
    /// JointPositionSubscriber.ProcessMessage(), en la rama de "freeze" (diferencia
    /// chica + freezeMode=true, activo en la escena de medición), que actualizaba el
    /// modelo visual pero se saltaba la actualización de `lastPositions` -- así que
    /// GetLastKnownPositions() podía devolver un valor de mitad de movimiento en vez
    /// del final, desfasado hasta freezeThresholdDeg. Ya corregido ahí; se comprobó
    /// contra el emulador (hablando directo con el mock por rosbridge, sin pasar por
    /// Unity) que la secuencia completa da 0,0000° exacto. De todos modos, además de
    /// settleSeconds, Run() espera -acotado por motionDoneExtraTimeoutSeconds- a que
    /// el controlador confirme el movimiento como terminado vía
    /// nonrt_state_data/robot_motion_done (ver RobotMotionDoneSubscriber y
    /// WaitForMotionDone en MeasurementTest.cs): una señal explícita del propio
    /// driver/emulador, útil como margen adicional contra el robot físico, cuya
    /// dinámica de asentamiento no es la del mock. Si ese componente no está cableado
    /// en la escena, o el driver real no llega a publicarlo a tiempo, el
    /// comportamiento cae de vuelta a solo settleSeconds -- no bloquea la prueba.
    ///
    /// Además, ANTES de comandar cada consigna, Run() valida el destino
    /// (WorkspaceSafety.IsCartesianGeometrySafe() + IsJointConfigurationSafe() -- NO
    /// IsSafe() completo: basePoseDeg no tiene la pinza hacia abajo, así que exigirlo
    /// acá rechazaría cada consigna por un motivo ajeno a P6) y el CAMINO desde la
    /// consigna anterior (WorkspaceSafety.IsPathSafe()) -- para CADA movimiento, no
    /// solo entre articulaciones distintas: pasar de una consigna a la siguiente
    /// DENTRO del barrido de la misma junta también es un movimiento en línea recta
    /// que podría cruzar algo, si esa junta tuviera más de un tramo seguro (por eso
    /// j1 usa un solo tramo -- ver arriba). Si el camino directo no es seguro, Run()
    /// prueba pasar por `basePoseDeg` primero (por construcción, punto de tránsito
    /// seguro: es donde arranca y termina la prueba) antes de rechazar la consigna --
    /// misma red de seguridad general que ya usa P9KinematicConsistency. Con las
    /// consignas de arriba, verificado que las 29 transiciones de la secuencia
    /// completa (30 movimientos: 6 juntas × 5 consignas menos 1) son seguras
    /// DIRECTAS, sin necesitar el ruteo por base ninguna vez -- pero el chequeo
    /// queda para cuando alguien cambie `basePoseDeg`, `rangeFraction` o el orden a
    /// mano y esto deje de ser cierto en silencio.
    public class P6JointAccuracy : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Consignas por articulación. El plan pide al menos 5, cubriendo " +
                 "distintos sectores del rango.")]
        public int setpointsPerJoint = 5;

        [Tooltip("Fracción del rango articular que se recorre (j3-j6, y j2 sobre su " +
                 "rango operativo permitido). j1 no usa esto -- ver ScanSafeJ1Intervals.")]
        [Range(0.1f, 1f)]
        public float rangeFraction = 0.6f;

        [Tooltip("Pose base desde la que se mueve cada articulación, en grados. " +
                 "También sirve como punto de tránsito seguro si una transición entre " +
                 "articulaciones no es segura directa -- ver comentario de clase.")]
        public float[] basePoseDeg = { 0f, -90f, 90f, -90f, 90f, 0f };

        public float speedPct = 20f;
        [Tooltip("Tolerancia con la que se considera alcanzada la consigna. Solo " +
                 "gobierna la espera; el error se mide sobre el valor real reportado.")]
        public float arrivalToleranceDeg = 0.5f;
        public float arrivalTimeoutSeconds = 30f;
        [Tooltip("Espera tras confirmar llegada, para medir con el brazo asentado.")]
        public float settleSeconds = 1.5f;

        [Tooltip("Tras settleSeconds, tiempo extra máximo a esperar a que el driver " +
                 "confirme el movimiento como terminado (nonrt_state_data/" +
                 "robot_motion_done, vía RobotMotionDoneSubscriber) antes de leer el " +
                 "valor final. No bloquea si ese componente no está en la escena, o si " +
                 "nunca reportó nada. Ver comentario de clase.")]
        public float motionDoneExtraTimeoutSeconds = 3f;

        [Tooltip("Margen hacia adentro de cualquier tramo seguro calculado para j1/j2 " +
                 "(nunca lo agranda). Mismo orden de magnitud que usa P9.")]
        public float safetyMarginDeg = 5f;

        /// Límites articulares del URDF fr5v6, en grados, en el orden j1..j6.
        /// Son los mismos valores que JOINT_LIMITS en mock_cmd_server.py:74, ahí en
        /// radianes. j1 y j2 solo se usan acá como límite MECÁNICO de referencia para
        /// el escaneo/cálculo -- el barrido real de esas dos juntas queda acotado por
        /// las restricciones operativas de la celda (ver comentario de clase), no por
        /// esto directamente.
        private static readonly float[,] JointLimitsDeg =
        {
            { -175.0f,  175.0f }, // j1
            { -265.0f,   85.0f }, // j2
            { -162.0f,  162.0f }, // j3
            { -265.0f,   85.0f }, // j4
            { -175.0f,  175.0f }, // j5
            { -175.0f,  175.0f }, // j6
        };

        public override string TestId { get { return "P6"; } }
        public override string DisplayName { get { return "Teleoperación articular"; } }

        public override bool AppliesTo(MeasurementConfiguration config)
        {
            return config == MeasurementConfiguration.C2_MiddlewareRemoto_RobotFisico
                || config == MeasurementConfiguration.C4_MiddlewareLocal_RobotFisico;
        }

        public override string NotApplicableReason
        {
            get { return "requiere el robot físico: contra el emulador el error sería cero por construcción"; }
        }

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }
            if (session.JointSubscriber == null || session.CommandSender == null)
            {
                Finish(false, "faltan JointPositionSubscriber o Ros2CommandSender");
                yield break;
            }
            if (basePoseDeg == null || basePoseDeg.Length != 6)
            {
                Finish(false, "la pose base debe tener 6 valores");
                yield break;
            }

            session.JointSubscriber.StartUpdating();

            var csv = session.OpenCsv($"{TestId}_articular",
                "articulacion", "consigna_idx", "consigna_deg", "alcanzado_deg",
                "error_deg", "error_absoluto_deg", "llegada_confirmada",
                "otras_articulaciones_desvio_max_deg");

            SetStatus("preparando el robot...");
            yield return StartCoroutine(PrepareRobotForMotion(session));

            // Se parte llevando el brazo a la pose base -- también el punto de
            // referencia para el primer chequeo de camino de abajo.
            bool baseOk = false;
            yield return StartCoroutine(MoveToJointConfiguration(
                session, basePoseDeg, speedPct, arrivalToleranceDeg,
                arrivalTimeoutSeconds, ok => baseOk = ok));
            if (!baseOk)
                Debug.LogWarning("[P6] No se confirmó la llegada a la pose base; se continúa igual.");
            float[] previousCommanded = basePoseDeg;

            var allErrors = new List<double>();
            var perJointErrors = new Dictionary<int, List<double>>();
            int notReached = 0;
            int rejectedForSafety = 0;
            int routedThroughBase = 0;

            for (int j = 0; j < 6; j++)
            {
                perJointErrors[j] = new List<double>();
                float[] setpoints = BuildSetpoints(j);

                for (int k = 0; k < setpoints.Length; k++)
                {
                    SetStatus($"j{j + 1}, consigna {k + 1}/{setpoints.Length} " +
                              $"({setpoints[k]:F1}°)");

                    float[] target = (float[])basePoseDeg.Clone();
                    target[j] = setpoints[k];

                    // Chequeo de seguridad ANTES de mover el robot físico -- destino y
                    // camino, mismo esquema que P9KinematicConsistency (ver su
                    // comentario de clase para el razonamiento completo). Acá el
                    // destino se valida con IsCartesianGeometrySafe(), NO IsSafe():
                    // basePoseDeg no tiene la pinza hacia abajo, y P6 no la necesita
                    // -- exigirla rechazaría cada consigna por un motivo ajeno a lo
                    // que esta prueba mide.
                    bool jointsSafe = WorkspaceSafety.IsJointConfigurationSafe(target, out string jointReason);
                    float[] predictedPose = RealRobotForwardKinematics.CartesianFromJointsDeg(target);
                    bool poseSafe = WorkspaceSafety.IsCartesianGeometrySafe(predictedPose, out string poseReason);

                    if (!jointsSafe || !poseSafe)
                    {
                        string unsafeReason = string.Join("; ", new[] { jointReason, poseReason }
                            .Where(s => !string.IsNullOrEmpty(s)));
                        Debug.LogError($"[P6] j{j + 1} consigna {k + 1} ({setpoints[k]:F1}°) RECHAZADA " +
                                       $"por seguridad, no se comanda al robot: {unsafeReason}");
                        rejectedForSafety++;
                        csv.WriteRow($"j{j + 1}", k + 1, setpoints[k], null, null, null,
                                     false, null);
                        continue;
                    }

                    bool viaBase = false;
                    bool directPathSafe = WorkspaceSafety.IsPathSafe(previousCommanded, target, out string directPathReason);
                    if (!directPathSafe)
                    {
                        bool leg1Safe = WorkspaceSafety.IsPathSafe(previousCommanded, basePoseDeg, out string leg1Reason);
                        bool leg2Safe = WorkspaceSafety.IsPathSafe(basePoseDeg, target, out string leg2Reason);
                        if (leg1Safe && leg2Safe)
                        {
                            Debug.LogWarning($"[P6] j{j + 1} consigna {k + 1}: camino directo inseguro " +
                                             $"({directPathReason}); pasando por la pose base.");
                            viaBase = true;
                        }
                        else
                        {
                            string combined = string.Join("; ", new[] { directPathReason, leg1Reason, leg2Reason }
                                .Where(s => !string.IsNullOrEmpty(s)));
                            Debug.LogError($"[P6] j{j + 1} consigna {k + 1} RECHAZADA por seguridad " +
                                           $"(camino, ni directo ni vía pose base son seguros): {combined}");
                            rejectedForSafety++;
                            csv.WriteRow($"j{j + 1}", k + 1, setpoints[k], null, null, null,
                                         false, null);
                            continue;
                        }
                    }

                    if (viaBase)
                    {
                        routedThroughBase++;
                        bool baseArrived = false;
                        yield return StartCoroutine(MoveToJointConfiguration(
                            session, basePoseDeg, speedPct, arrivalToleranceDeg,
                            arrivalTimeoutSeconds, ok => baseArrived = ok));
                        if (!baseArrived)
                            Debug.LogWarning($"[P6] No se confirmó la llegada a la pose base de tránsito " +
                                             $"antes de j{j + 1} consigna {k + 1}; se continúa igual.");
                    }

                    bool arrived = false;
                    yield return StartCoroutine(MoveToJointConfiguration(
                        session, target, speedPct, arrivalToleranceDeg,
                        arrivalTimeoutSeconds, ok => arrived = ok));

                    // Se haya confirmado la llegada o no, el robot ya recibió el MoveJ:
                    // es el punto de partida real para el chequeo de camino de la
                    // PRÓXIMA consigna.
                    previousCommanded = target;

                    if (!arrived)
                    {
                        Debug.LogWarning($"[P6] j{j + 1} consigna {k + 1} no alcanzada dentro " +
                                         $"del plazo; se descarta.");
                        csv.WriteRow($"j{j + 1}", k + 1, setpoints[k], null, null, null,
                                     false, null);
                        notReached++;
                        continue;
                    }

                    yield return new WaitForSeconds(settleSeconds);
                    yield return StartCoroutine(WaitForMotionDone(session, motionDoneExtraTimeoutSeconds));

                    float[] reached = session.JointSubscriber.GetLastKnownPositions();
                    if (reached == null || reached.Length != 6)
                    {
                        notReached++;
                        csv.WriteRow($"j{j + 1}", k + 1, setpoints[k], null, null, null,
                                     false, null);
                        continue;
                    }

                    double error = reached[j] - setpoints[k];

                    // Desvío de las demás articulaciones respecto de la pose base: si
                    // alguna se movió, el error de esta no es atribuible solo a ella.
                    double otherMax = 0.0;
                    for (int o = 0; o < 6; o++)
                    {
                        if (o == j) continue;
                        double d = Mathf.Abs(reached[o] - basePoseDeg[o]);
                        if (d > otherMax) otherMax = d;
                    }

                    csv.WriteRow($"j{j + 1}", k + 1, setpoints[k], reached[j],
                                 error, Mathf.Abs((float)error), arrived, otherMax);

                    allErrors.Add(Mathf.Abs((float)error));
                    perJointErrors[j].Add(Mathf.Abs((float)error));
                }
            }

            csv.Dispose();

            // Devolver el brazo a la pose base al terminar.
            yield return StartCoroutine(MoveToJointConfiguration(
                session, basePoseDeg, speedPct, arrivalToleranceDeg,
                arrivalTimeoutSeconds, _ => { }));

            var st = Stats.From(allErrors);
            var summary = session.OpenCsv($"{TestId}_articular_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("plataforma", session.platformLabel);
            summary.WriteRow("consignas_por_articulacion", setpointsPerJoint);
            summary.WriteRow("fraccion_de_rango", rangeFraction);
            summary.WriteRow("consignas_totales", 6 * setpointsPerJoint);
            summary.WriteRow("consignas_no_confirmadas", notReached);
            summary.WriteRow("consignas_rechazadas_por_seguridad", rejectedForSafety);
            summary.WriteRow("transiciones_ruteadas_por_pose_base", routedThroughBase);
            summary.WriteRow("error_medio_deg", st.Mean);
            summary.WriteRow("error_mediana_deg", st.Median);
            summary.WriteRow("error_maximo_deg", st.Max);
            summary.WriteRow("error_desviacion_deg", st.StdDev);

            for (int j = 0; j < 6; j++)
            {
                var js = Stats.From(perJointErrors[j]);
                summary.WriteRow($"j{j + 1}_error_medio_deg", js.Mean);
                summary.WriteRow($"j{j + 1}_error_maximo_deg", js.Max);
                summary.WriteRow($"j{j + 1}_n", js.Count);
            }
            summary.Dispose();

            bool ok2 = st.Count > 0;
            string safetyNote = (rejectedForSafety > 0 ? $" ({rejectedForSafety} rechazada(s) por seguridad)" : "") +
                (routedThroughBase > 0 ? $" ({routedThroughBase} ruteada(s) por pose base)" : "");
            Finish(ok2, ok2
                ? $"error medio {st.Mean:F3}°, máximo {st.Max:F3}° (n={st.Count}){safetyNote}"
                : $"ninguna consigna confirmada{safetyNote}");
        }

        /// Reparte las consignas por sectores del rango útil. j1 y j2 tienen
        /// restricciones operativas propias de esta celda (ver comentario de
        /// clase); j3-j6 usan el rango mecánico del URDF tal cual, centrado y
        /// acotado por `rangeFraction` como antes.
        private float[] BuildSetpoints(int jointIndex)
        {
            int n = Mathf.Max(2, setpointsPerJoint);

            if (jointIndex == 0)
                return BuildJ1Setpoints(n);

            float lo, hi;
            if (jointIndex == 1)
            {
                // j2: rango operativo permitido en esta celda, no el mecánico real.
                lo = WorkspaceSafety.J2AllowedRangeDeg.x + safetyMarginDeg;
                hi = WorkspaceSafety.J2AllowedRangeDeg.y - safetyMarginDeg;
            }
            else
            {
                lo = JointLimitsDeg[jointIndex, 0];
                hi = JointLimitsDeg[jointIndex, 1];
            }

            float mid = (lo + hi) * 0.5f;
            float half = (hi - lo) * 0.5f * rangeFraction;
            float from = mid - half;
            float to = mid + half;

            var result = new float[n];
            for (int i = 0; i < n; i++)
                result[i] = Mathf.Lerp(from, to, i / (float)(n - 1));
            return result;
        }

        /// A diferencia de j2 (un solo rango permitido), j1 puede quedar partido en
        /// VARIOS tramos seguros separados por huecos completamente prohibidos (zona
        /// de pared + tramos donde el radio de alcance de basePoseDeg cruza la zona
        /// trasera). No hay forma de ir de un tramo a otro con MoveJ (interpolación
        /// articular en línea recta) sin cruzar el hueco -- y rutear por
        /// `basePoseDeg` NO LO RESUELVE si basePoseDeg cae en un tercer tramo
        /// distinto: solo mueve el problema, no lo elimina (confirmado probándolo:
        /// con los 3 tramos que da la pose base default, ir del tramo izquierdo al
        /// derecho pasando por el tramo del medio -- donde cae la base -- sigue
        /// cruzando un hueco prohibido en cada mitad). Por eso se usa SOLO el tramo
        /// que contiene el valor de j1 en basePoseDeg: es el único que se puede
        /// recorrer de punta a punta sin salir nunca de zona segura. Sacrifica
        /// cobertura del rango mecánico completo de j1 por goles de que la
        /// secuencia entera sea segura con los movimientos que MoveJ puede hacer.
        private float[] BuildJ1Setpoints(int n)
        {
            var intervals = ScanSafeJ1Intervals();
            if (intervals.Count == 0)
            {
                Debug.LogError("[P6] No se encontró ningún tramo seguro para j1 con la pose " +
                               "base actual; se usa un único punto (la propia base) para no " +
                               "comandar nada fuera de límites.");
                return new float[] { basePoseDeg[0] };
            }

            float baseJ1 = basePoseDeg[0];
            (float lo, float hi)? containing = null;
            foreach (var iv in intervals)
            {
                if (baseJ1 >= iv.lo && baseJ1 <= iv.hi) { containing = iv; break; }
            }
            if (containing == null)
            {
                // No debería pasar (basePoseDeg ya se valida como seguro antes de
                // Run() llegar acá), pero si pasara, el tramo más ancho es la
                // mejor alternativa disponible.
                containing = intervals.OrderByDescending(iv => iv.hi - iv.lo).First();
                Debug.LogWarning($"[P6] basePoseDeg[0]={baseJ1:F1}° no cae en ningún tramo " +
                                 "seguro encontrado; se usa el tramo más ancho en su lugar.");
            }

            return DistributeAcrossIntervals(new List<(float, float)> { containing.Value }, n);
        }

        /// Escanea j1 en pasos de 1° (con las demás juntas fijas en basePoseDeg) y
        /// devuelve los tramos que quedan simultáneamente fuera de la zona
        /// prohibida de j1 (WorkspaceSafety.J1ForbiddenRangeDeg) Y de la zona
        /// trasera cartesiana, con `safetyMarginDeg` aplicado hacia adentro de
        /// cada tramo resultante.
        ///
        /// POR QUÉ SE ESCANEA EN VEZ DE CALCULARSE: la zona trasera depende de la
        /// pose COMPLETA del brazo (radio y altura de alcance con las demás juntas
        /// en su valor de basePoseDeg), no es algo que se pueda despejar
        /// analíticamente solo a partir del rango de j1 -- y si se cambia
        /// basePoseDeg a mano, los tramos seguros cambian con ella. Escanear acá
        /// asegura que siempre estén actualizados a la pose base que realmente se
        /// esté usando, en vez de quedar pegados a un cálculo hecho a mano para
        /// OTRA pose base que ya no es la vigente.
        private List<(float lo, float hi)> ScanSafeJ1Intervals(float stepDeg = 1f)
        {
            float lo = JointLimitsDeg[0, 0], hi = JointLimitsDeg[0, 1];
            int steps = Mathf.CeilToInt((hi - lo) / stepDeg);

            var raw = new List<(float lo, float hi)>();
            float? segStart = null;

            for (int i = 0; i <= steps; i++)
            {
                float v = Mathf.Min(lo + i * stepDeg, hi);
                float[] cfg = (float[])basePoseDeg.Clone();
                cfg[0] = v;

                bool jointOk = WorkspaceSafety.IsJointConfigurationSafe(cfg, out _);
                float[] pose = RealRobotForwardKinematics.CartesianFromJointsDeg(cfg);
                bool geometryOk = WorkspaceSafety.IsCartesianGeometrySafe(pose, out _);
                bool ok = jointOk && geometryOk;

                if (ok && segStart == null)
                {
                    segStart = v;
                }
                else if (!ok && segStart != null)
                {
                    raw.Add((segStart.Value, v - stepDeg));
                    segStart = null;
                }
            }
            if (segStart != null) raw.Add((segStart.Value, hi));

            var margined = new List<(float, float)>();
            foreach (var (a, b) in raw)
            {
                float ma = a + safetyMarginDeg, mb = b - safetyMarginDeg;
                if (mb > ma) margined.Add((ma, mb));
            }
            return margined;
        }

        /// Reparte `n` puntos proporcionalmente al ancho de cada tramo de la
        /// lista, recorriéndolos en orden como si fueran uno solo concatenado.
        /// Con un único tramo, es exactamente un Lerp normal.
        private static float[] DistributeAcrossIntervals(List<(float lo, float hi)> intervals, int n)
        {
            float totalWidth = 0f;
            foreach (var (a, b) in intervals) totalWidth += (b - a);

            var result = new float[n];
            for (int i = 0; i < n; i++)
            {
                float target = (n == 1) ? 0f : totalWidth * i / (n - 1);
                float acc = 0f;
                float value = intervals[intervals.Count - 1].hi;

                for (int k = 0; k < intervals.Count; k++)
                {
                    var (a, b) = intervals[k];
                    float w = b - a;
                    if (target <= acc + w || k == intervals.Count - 1)
                    {
                        float frac = w > 0f ? (target - acc) / w : 0f;
                        value = a + Mathf.Clamp01(frac) * w;
                        break;
                    }
                    acc += w;
                }
                result[i] = value;
            }
            return result;
        }
    }
}
