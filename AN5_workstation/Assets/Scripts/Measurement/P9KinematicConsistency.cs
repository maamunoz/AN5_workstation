using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;

namespace AN5.Measurement
{
    /// P9 — Consistencia cinemática entre las distintas implementaciones de
    /// cinemática directa que conviven en la plataforma.
    ///
    /// Este proyecto calcula la pose del efector de TRES maneras independientes, que
    /// deberían coincidir porque describen la misma geometría, pero que no comparten
    /// una sola línea de código:
    ///
    ///   (a) DH en Unity — LocalForwardKinematics.cs:15-22, tabla Denavit-Hartenberg.
    ///   (b) Jerarquía de escena — el importador de URDF encadena cada eslabón como
    ///       hijo del anterior, así que la composición de transformadas padre-hijo que
    ///       hace Unity ES la cinemática directa. JointStateWriter solo fija la
    ///       rotación local de cada articulación (:254); nadie calcula la pose del
    ///       efector, emerge del árbol.
    ///   (c) Middleware — el emulador la calcula encadenando los <origin> del URDF
    ///       (_forward_kinematics_mm_deg, mock_cmd_server.py:202); el robot físico la
    ///       obtiene de la cinemática propia del controlador del fabricante
    ///       (GetActualTCPPose, publisher_subscriber.py:37).
    ///
    /// HAY MOTIVO CONCRETO PARA ESPERAR DIVERGENCIA, y cuantificarla es el resultado:
    ///   - La tabla DH cierra con d6 = 0,267 m, mientras que la cadena URDF suma
    ///     0,102 m del origen de j6 más 0,100 m de la herramienta = 0,202 m. Son
    ///     definiciones distintas del extremo.
    ///   - La rama (b) pasa por JointStateWriter, que aplica desfases fijos de ±90° e
    ///     inversión de signo por articulación (:107-141) en vez de usar el eje del
    ///     URDF.
    ///
    /// INTERPRETACIÓN. Si la divergencia entre emulador y robot resulta despreciable,
    /// se sostiene que una trayectoria registrada en simulación es válida sobre el
    /// equipo físico, que es lo que respalda el uso de C3 como estación de formación.
    /// Si no lo es, se reporta la magnitud y se acota el alcance en la Discusión.
    ///
    /// La comparación emulador contra robot exige DOS sesiones (una en C1/C3 y otra en
    /// C2/C4) que después se aparean por identificador de configuración: en una misma
    /// sesión solo hay un destino publicando.
    public class P9KinematicConsistency : MeasurementTest
    {
        [Header("Configuraciones de referencia (grados)")]
        [Tooltip("El plan pide al menos 8, con varias articulaciones simultáneamente " +
                 "fuera de cero y alguna cerca de los límites. La pose de origen se " +
                 "evita a propósito: ahí las diferencias de convención tienden a " +
                 "cancelarse y la prueba no detectaría nada.")]
        public List<Vector6> referenceConfigurations = new List<Vector6>
        {
            new Vector6(  30f,  -60f,   60f,  -90f,   60f,   45f),
            new Vector6( -45f,  -80f,  100f, -110f,  -70f,  -60f),
            new Vector6(  90f,  -45f,   45f,  -60f,   90f,  120f),
            new Vector6(-120f, -100f,  120f,  -80f,   45f,  -90f),
            new Vector6(  15f, -120f,   90f,  -45f,  120f,   30f),
            new Vector6( 160f,  -70f,   70f, -100f,  160f,  150f), // cerca de límites
            new Vector6(-160f,  -50f,  140f, -140f, -160f, -150f), // cerca de límites
            new Vector6(  60f,  -30f,   30f, -150f,   30f,   75f),
            new Vector6( -75f, -140f,  110f,  -70f, -100f,  100f),
            new Vector6(  45f,  -95f,   85f,  -95f,   85f,  -45f),
        };

        [Header("Movimiento")]
        public float speedPct = 30f;
        public float arrivalToleranceDeg = 0.5f;
        public float arrivalTimeoutSeconds = 30f;
        [Tooltip("Espera adicional tras confirmar llegada, para que el estado " +
                 "publicado y el modelo terminen de asentarse antes de muestrear.")]
        public float settleSeconds = 1.5f;

        [Header("Jerarquía de escena")]
        [Tooltip("Eslabón base del robot. Se resuelve por nombre si queda vacío.")]
        public Transform baseLink;
        [Tooltip("Extremo de la cadena. Se resuelve por nombre si queda vacío.")]
        public Transform toolLink;
        public string baseLinkName = "base_link";
        public string toolLinkName = "tool_Link";
        [Tooltip("Si tool_Link no existe o está desactivado, se usa j6_Link. La " +
                 "diferencia entre ambos queda registrada en el CSV.")]
        public string fallbackToolLinkName = "j6_Link";

        public override string TestId { get { return "P9"; } }
        public override string DisplayName { get { return "Consistencia cinemática"; } }

        [System.Serializable]
        public struct Vector6
        {
            public float j1, j2, j3, j4, j5, j6;
            public Vector6(float a, float b, float c, float d, float e, float f)
            { j1 = a; j2 = b; j3 = c; j4 = d; j5 = e; j6 = f; }
            public float[] ToArray() { return new[] { j1, j2, j3, j4, j5, j6 }; }
        }

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }
            if (session.JointSubscriber == null || session.CartesianSubscriber == null)
            {
                Finish(false, "faltan JointPositionSubscriber o CartesianPositionSubscriber");
                yield break;
            }

            string toolResolved = ResolveHierarchy();
            bool hierarchyAvailable = baseLink != null && toolLink != null;
            if (!hierarchyAvailable)
            {
                Debug.LogWarning("[P9] No se resolvió la jerarquía del robot en la " +
                                 "escena; se medirán solo las otras dos fuentes.");
            }

            // El modelo de la escena solo se mueve si el subscriptor está actualizando
            // y algo llama a ApplyToModel(). Según qué panel esté activo,
            // driveRobotModel puede estar en false y la jerarquía quedaría congelada:
            // se estaría muestreando una pose vieja sin ninguna señal de que lo es.
            session.JointSubscriber.StartUpdating();

            string middlewareSource = session.HasPhysicalRobot
                ? "middleware_robot" : "middleware_emulador";

            var poses = session.OpenCsv($"{TestId}_poses",
                "config_id", "fuente", "destino",
                "j1_cmd", "j2_cmd", "j3_cmd", "j4_cmd", "j5_cmd", "j6_cmd",
                "j1_real", "j2_real", "j3_real", "j4_real", "j5_real", "j6_real",
                "x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg", "llegada_confirmada");

            var errors = session.OpenCsv($"{TestId}_errores",
                "config_id", "comparacion", "destino",
                "error_posicion_mm", "error_orientacion_deg",
                "dx_mm", "dy_mm", "dz_mm");

            SetStatus("preparando el robot...");
            yield return StartCoroutine(PrepareRobotForMotion(session));

            var posErrDhVsMw = new List<double>();
            var oriErrDhVsMw = new List<double>();
            var posErrHierVsMw = new List<double>();
            var oriErrHierVsMw = new List<double>();
            var posErrDhVsHier = new List<double>();
            var oriErrDhVsHier = new List<double>();

            int measured = 0;

            for (int c = 0; c < referenceConfigurations.Count; c++)
            {
                float[] commanded = referenceConfigurations[c].ToArray();
                SetStatus($"configuración {c + 1}/{referenceConfigurations.Count}");

                bool arrived = false;
                yield return StartCoroutine(MoveToJointConfiguration(
                    session, commanded, speedPct, arrivalToleranceDeg,
                    arrivalTimeoutSeconds, ok => arrived = ok));

                if (!arrived)
                {
                    // Una muestra sin llegada confirmada no es un dato de consistencia
                    // cinemática: sería la diferencia entre dos configuraciones
                    // distintas, no entre dos implementaciones. Se registra el intento
                    // y se descarta del resumen.
                    Debug.LogWarning($"[P9] Configuración {c + 1} no alcanzada dentro " +
                                     $"del plazo; se descarta.");
                    poses.WriteRow(c, "no_alcanzada", middlewareSource,
                        commanded[0], commanded[1], commanded[2],
                        commanded[3], commanded[4], commanded[5],
                        null, null, null, null, null, null,
                        null, null, null, null, null, null, false);
                    continue;
                }

                yield return new WaitForSeconds(settleSeconds);

                // Todas las fuentes deben evaluarse en la MISMA configuración real
                // alcanzada, no en la comandada: si el robot quedó a 0,3° del objetivo,
                // comparar (a) calculada sobre la consigna contra (c) calculada sobre
                // lo real mediría el error de seguimiento, no el de convención.
                float[] reached = session.JointSubscriber.GetLastKnownPositions();

                // Forzar la escritura al modelo y esperar dos cuadros: JointStateWriter
                // no aplica la rotación dentro de Write(), solo marca el nuevo estado;
                // el Transform recién cambia en su propio Update() (:55-74). Leer las
                // transformadas antes de eso devolvería la pose anterior.
                session.JointSubscriber.ApplyToModel();
                yield return null;
                yield return null;

                double[] dh = ToDouble(LocalForwardKinematics.CartesianFromJointsDeg(reached));
                double[] mw = ToDouble(session.CartesianSubscriber.GetLastKnownCartesianPositions());
                double[] hier = hierarchyAvailable ? SampleHierarchyPose() : null;

                WritePose(poses, c, "dh_unity", middlewareSource, commanded, reached, dh);
                WritePose(poses, c, middlewareSource, middlewareSource, commanded, reached, mw);
                if (hier != null)
                    WritePose(poses, c, "jerarquia_urdf", middlewareSource, commanded, reached, hier);

                Compare(errors, c, "dh_unity_vs_middleware", middlewareSource, dh, mw,
                        posErrDhVsMw, oriErrDhVsMw);
                if (hier != null)
                {
                    Compare(errors, c, "jerarquia_vs_middleware", middlewareSource, hier, mw,
                            posErrHierVsMw, oriErrHierVsMw);
                    Compare(errors, c, "dh_unity_vs_jerarquia", middlewareSource, dh, hier,
                            posErrDhVsHier, oriErrDhVsHier);
                }

                measured++;
            }

            poses.Dispose();
            errors.Dispose();

            var summary = session.OpenCsv($"{TestId}_consistencia_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("destino", middlewareSource);
            summary.WriteRow("configuraciones_solicitadas", referenceConfigurations.Count);
            summary.WriteRow("configuraciones_medidas", measured);
            summary.WriteRow("jerarquia_disponible", hierarchyAvailable);
            summary.WriteRow("jerarquia_extremo_usado", toolResolved);

            EmitPairSummary(summary, "dh_unity_vs_middleware", posErrDhVsMw, oriErrDhVsMw);
            EmitPairSummary(summary, "jerarquia_vs_middleware", posErrHierVsMw, oriErrHierVsMw);
            EmitPairSummary(summary, "dh_unity_vs_jerarquia", posErrDhVsHier, oriErrDhVsHier);

            summary.WriteRow("nota_emulador_vs_robot",
                "requiere aparear esta corrida con otra de destino opuesto por config_id; " +
                "en una sola sesión hay un único destino publicando");
            summary.Dispose();

            bool ok2 = measured > 0;
            var st = Stats.From(posErrDhVsMw);
            Finish(ok2, ok2
                ? $"{measured} configuraciones; DH vs middleware: mediana " +
                  $"{st.Median:F2} mm, máx {st.Max:F2} mm"
                : "ninguna configuración alcanzada");
        }

        private static void EmitPairSummary(CsvWriter csv, string pair,
                                            List<double> pos, List<double> ori)
        {
            if (pos.Count == 0) return;
            var p = Stats.From(pos);
            var o = Stats.From(ori);
            csv.WriteRow(pair + "_n", p.Count);
            csv.WriteRow(pair + "_posicion_media_mm", p.Mean);
            csv.WriteRow(pair + "_posicion_mediana_mm", p.Median);
            csv.WriteRow(pair + "_posicion_maxima_mm", p.Max);
            csv.WriteRow(pair + "_orientacion_media_deg", o.Mean);
            csv.WriteRow(pair + "_orientacion_mediana_deg", o.Median);
            csv.WriteRow(pair + "_orientacion_maxima_deg", o.Max);
        }

        private static void Compare(CsvWriter csv, int configId, string label, string destino,
                                    double[] a, double[] b,
                                    List<double> posAcc, List<double> oriAcc)
        {
            double posErr = PoseMath.PositionErrorNorm(a, b);
            double oriErr = PoseMath.OrientationErrorDeg(
                new[] { a[3], a[4], a[5] }, new[] { b[3], b[4], b[5] });

            csv.WriteRow(configId, label, destino, posErr, oriErr,
                         a[0] - b[0], a[1] - b[1], a[2] - b[2]);

            posAcc.Add(posErr);
            oriAcc.Add(oriErr);
        }

        private static void WritePose(CsvWriter csv, int configId, string source, string destino,
                                      float[] commanded, float[] reached, double[] pose)
        {
            csv.WriteRow(configId, source, destino,
                commanded[0], commanded[1], commanded[2],
                commanded[3], commanded[4], commanded[5],
                reached[0], reached[1], reached[2],
                reached[3], reached[4], reached[5],
                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], true);
        }

        private static double[] ToDouble(float[] v)
        {
            var d = new double[v.Length];
            for (int i = 0; i < v.Length; i++) d[i] = v[i];
            return d;
        }

        /// Pose del extremo relativa a la base, leída de la jerarquía de la escena y
        /// convertida al marco de ROS.
        ///
        /// Unity es zurdo y el importador de URDF aplica su propio mapeo de ejes al
        /// construir el árbol; para comparar contra las otras dos fuentes hay que
        /// deshacerlo con las mismas funciones que usa el importador
        /// (TransformExtensions.Unity2Ros, :91 y :111) en lugar de con una conversión
        /// inventada acá.
        private double[] SampleHierarchyPose()
        {
            Vector3 localPos = baseLink.InverseTransformPoint(toolLink.position);
            Quaternion localRot = Quaternion.Inverse(baseLink.rotation) * toolLink.rotation;

            Vector3 rosPos = localPos.Unity2Ros();
            Quaternion rosRot = localRot.Unity2Ros();

            double[] rpy = PoseMath.QuatToRpyDeg(rosRot.x, rosRot.y, rosRot.z, rosRot.w);

            // A milímetros, para quedar en las mismas unidades que las otras fuentes.
            return new double[]
            {
                rosPos.x * 1000.0, rosPos.y * 1000.0, rosPos.z * 1000.0,
                rpy[0], rpy[1], rpy[2],
            };
        }

        /// Resuelve base y extremo por nombre. Devuelve qué eslabón terminó usándose
        /// como extremo, que se registra en el CSV: tool_Link está DESACTIVADO en
        /// panels.unity aunque sí esté activo en la escena de build (AN5_sim.unity), y
        /// medir contra j6_Link en su lugar cambia la pose en los 100 mm de la
        /// herramienta. Registrarlo evita atribuir esa diferencia a un error de
        /// convención que no existe.
        private string ResolveHierarchy()
        {
            if (baseLink == null) baseLink = FindByName(baseLinkName);
            if (toolLink == null) toolLink = FindByName(toolLinkName);

            if (toolLink == null)
            {
                toolLink = FindByName(fallbackToolLinkName);
                if (toolLink != null)
                {
                    Debug.LogWarning($"[P9] '{toolLinkName}' no encontrado; se usa " +
                                     $"'{fallbackToolLinkName}' como extremo.");
                    return fallbackToolLinkName;
                }
                return "(ninguno)";
            }
            return toolLink.name;
        }

        private static Transform FindByName(string name)
        {
            if (string.IsNullOrEmpty(name)) return null;
            // Se recorren también los objetos desactivados: tool_Link puede estar
            // inactivo según la escena, y GameObject.Find() no lo encontraría.
            foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
            {
                if (t.name != name) continue;
                if (t.hideFlags != HideFlags.None) continue;
                // Descartar prefabs del proyecto que no están en la escena.
                if (!t.gameObject.scene.IsValid()) continue;
                return t;
            }
            return null;
        }
    }
}
