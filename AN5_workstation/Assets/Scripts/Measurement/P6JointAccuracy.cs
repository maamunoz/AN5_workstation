using System.Collections;
using System.Collections.Generic;
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
    /// Los sectores se recorren dentro de una fracción del rango declarado en el URDF,
    /// no hasta los límites: acercarse al tope con el brazo físico arriesga colisión
    /// contra la propia estructura o el entorno de la celda, y ese riesgo no aporta
    /// nada a lo que la prueba quiere medir.
    public class P6JointAccuracy : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Consignas por articulación. El plan pide al menos 5, cubriendo " +
                 "distintos sectores del rango.")]
        public int setpointsPerJoint = 5;

        [Tooltip("Fracción del rango articular que se recorre. 0,6 mantiene un margen " +
                 "cómodo respecto de los topes del URDF.")]
        [Range(0.1f, 1f)]
        public float rangeFraction = 0.6f;

        [Tooltip("Pose base desde la que se mueve cada articulación, en grados.")]
        public float[] basePoseDeg = { 0f, -90f, 90f, -90f, 90f, 0f };

        public float speedPct = 20f;
        [Tooltip("Tolerancia con la que se considera alcanzada la consigna. Solo " +
                 "gobierna la espera; el error se mide sobre el valor real reportado.")]
        public float arrivalToleranceDeg = 0.5f;
        public float arrivalTimeoutSeconds = 30f;
        [Tooltip("Espera tras confirmar llegada, para medir con el brazo asentado.")]
        public float settleSeconds = 1.5f;

        /// Límites articulares del URDF fr5v6, en grados, en el orden j1..j6.
        /// Son los mismos valores que JOINT_LIMITS en mock_cmd_server.py:74, ahí en
        /// radianes.
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

            // Se parte llevando el brazo a la pose base.
            bool baseOk = false;
            yield return StartCoroutine(MoveToJointConfiguration(
                session, basePoseDeg, speedPct, arrivalToleranceDeg,
                arrivalTimeoutSeconds, ok => baseOk = ok));
            if (!baseOk)
                Debug.LogWarning("[P6] No se confirmó la llegada a la pose base; se continúa igual.");

            var allErrors = new List<double>();
            var perJointErrors = new Dictionary<int, List<double>>();
            int notReached = 0;

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

                    bool arrived = false;
                    yield return StartCoroutine(MoveToJointConfiguration(
                        session, target, speedPct, arrivalToleranceDeg,
                        arrivalTimeoutSeconds, ok => arrived = ok));

                    yield return new WaitForSeconds(settleSeconds);

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

                    if (arrived)
                    {
                        allErrors.Add(Mathf.Abs((float)error));
                        perJointErrors[j].Add(Mathf.Abs((float)error));
                    }
                    else
                    {
                        notReached++;
                    }
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
            Finish(ok2, ok2
                ? $"error medio {st.Mean:F3}°, máximo {st.Max:F3}° (n={st.Count})"
                : "ninguna consigna confirmada");
        }

        /// Reparte las consignas por sectores del rango útil, centrado en el valor de
        /// la pose base y acotado por la fracción configurada.
        private float[] BuildSetpoints(int jointIndex)
        {
            float lo = JointLimitsDeg[jointIndex, 0];
            float hi = JointLimitsDeg[jointIndex, 1];
            float mid = (lo + hi) * 0.5f;
            float half = (hi - lo) * 0.5f * rangeFraction;

            float from = mid - half;
            float to = mid + half;

            int n = Mathf.Max(2, setpointsPerJoint);
            var result = new float[n];
            for (int i = 0; i < n; i++)
                result[i] = Mathf.Lerp(from, to, i / (float)(n - 1));

            return result;
        }
    }
}
