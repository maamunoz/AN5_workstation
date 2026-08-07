using System.Collections;
using System.Collections.Generic;
using System.Linq;
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
    ///   - Medido empíricamente al generar las poses de referencia de abajo: para las
    ///     mismas 10 configuraciones articulares, la tabla DH da una Z mundial
    ///     sistemáticamente ~167 mm distinta de la real (RealRobotForwardKinematics.cs) —
    ///     más que los 65 mm de la sola herramienta, así que hay otra diferencia de
    ///     convención de ejes sumándose. Es un desplazamiento constante, no depende de
    ///     la configuración articular probada.
    ///
    /// INTERPRETACIÓN. Si la divergencia entre emulador y robot resulta despreciable,
    /// se sostiene que una trayectoria registrada en simulación es válida sobre el
    /// equipo físico, que es lo que respalda el uso de C3 como estación de formación.
    /// Si no lo es, se reporta la magnitud y se acota el alcance en la Discusión.
    ///
    /// La comparación emulador contra robot exige DOS sesiones (una en C1/C3 y otra en
    /// C2/C4) que después se aparean por identificador de configuración: en una misma
    /// sesión solo hay un destino publicando.
    ///
    /// SEGURIDAD FÍSICA. Esta prueba mueve el robot real (C2/C4). Las 10 configuraciones
    /// de referencia de abajo NO son arbitrarias: se resolvieron por cinemática inversa
    /// numérica contra la geometría real del URDF (mismo modelo que
    /// RealRobotForwardKinematics.cs) para que la pose cartesiana resultante cumpla
    /// SIEMPRE, con margen: Z ≥ 140 mm (piso pedido: 110 mm), fuera de la zona trasera
    /// X∈[-350,350]∧Y<-300 mm (piso pedido: Y≥-190 mm), pinza apuntando derecho hacia
    /// abajo (rx≈180°, ry≈0°, la convención que ya usan las trayectorias validadas de
    /// routines/), j2 dentro de [-145°,-45°] (restricción operativa de esta celda, más
    /// estricta que el límite mecánico real del motor), Y j1 FUERA de [-120°,-30°]
    /// (zona prohibida por la pared detrás del robot -- ver
    /// WorkspaceSafety.J2AllowedRangeDeg/J1ForbiddenRangeDeg).
    ///
    /// EL ORDEN DE LA LISTA TAMBIÉN ES DELIBERADO, no solo cada valor por separado.
    /// La primera versión de estas 10 configuraciones validaba cada una como endpoint
    /// individual y daba por seguro el conjunto -- pero P9 no salta de una a otra, las
    /// alcanza con MoveJ (interpolación ARTICULAR punto a punto, línea recta en espacio
    /// de juntas, NO en espacio cartesiano). Dos endpoints perfectamente seguros pueden
    /// tener, en el medio de ese camino, una junta cruzando la zona prohibida sin que
    /// ninguno de los dos extremos lo delate -- exactamente lo que pasaba entre estas
    /// configuraciones cuando estaban en su orden "natural": dos transiciones cruzaban
    /// de lleno J1ForbiddenRangeDeg a mitad de movimiento. El orden de abajo visita los
    /// puntos por cercanía en espacio articular (recorrido tipo vecino-más-cercano)
    /// para que cada paso sea chico, y CADA UNA de las 9 transiciones se verificó por
    /// separado (muestreo denso de la interpolación, no solo los extremos) contra los
    /// mismos límites. Reordenar esta lista a mano invalida esa verificación.
    ///
    /// EXCEPCIONES ACEPTADAS Y DOCUMENTADAS: dos transiciones apartan la pinza de "hacia
    /// abajo" a mitad de camino, ninguna de las dos por elección sino porque las ramas de
    /// IK disponibles no dejaron otra opción sin violar j1/j2:
    ///   - 8ª→9ª (hacia 150,560,140mm): queda de costado (rx≈90°) en, aprox.,
    ///     x∈[-350,150], y∈[560,630], z∈[130,250]mm. Consecuencia de que ese punto solo
    ///     tiene solución con la muñeca en la rama j5=-90°, distinta de los otros 9
    ///     (todos j5=+90°).
    ///   - 9ª→10ª (desde ese mismo punto hacia -560,-120,260mm): más severa -- a mitad de
    ///     camino DIRECTO la pinza queda completamente invertida (180°, apuntando hacia
    ///     arriba) y el brazo sube hasta Z≈1077mm. No se encontró una rama de IK para el
    ///     punto 10 que evite esto (se probó, entre otras, sembrando el solver con los
    ///     propios ángulos del punto 9: solo reconverge a la misma rama lejana).
    /// Ninguna de las dos baja de Z=110mm ni entra a la zona trasera en ningún momento
    /// (verificado con muestreo denso). La de 9ª→10ª sube mucho más que la otra, pero se
    /// confirmó que ni siquiera al límite de extensión el brazo llega al techo/estructura
    /// de esta celda, así que Z≈1077mm no es un riesgo físico distinto del resto
    /// -- se acepta el giro de la muñeca en tránsito en ambas transiciones porque los dos
    /// riesgos concretos que motivaron todo esto (mesa, pared) siguen cubiertos. Si en tu
    /// celda cambia el entorno (algo nuevo por encima o a los costados del brazo) y una
    /// pinza de costado o invertida podría chocar contra eso, revisá esto antes de correr
    /// la prueba con el robot físico.
    ///
    /// Además, ANTES de comandar cada configuración, Run() valida en tiempo de ejecución
    /// tanto el DESTINO (WorkspaceSafety.IsSafe() + IsJointConfigurationSafe()) como el
    /// CAMINO desde la configuración anterior (WorkspaceSafety.IsPathSafe(), con el mismo
    /// muestreo denso que se usó para verificar esta lista) contra los límites de Z,
    /// zona trasera, j1 y j2 -- NO contra la orientación de la pinza en tránsito (ver
    /// más arriba) ni contra ningún límite de altura máxima (esta celda no tiene uno: el
    /// brazo no alcanza el techo ni a extensión completa). Si el camino directo violara
    /// alguno de esos cuatro límites, Run() prueba automáticamente pasar por
    /// `safetyWaypoint` (partir el movimiento grande en dos chicos, cada uno verificado
    /// por separado) antes de rechazar la configuración -- una red de seguridad general
    /// para cualquier reordenamiento futuro que sí introduzca ese tipo de salto, aunque
    /// hoy ninguna de las 9 transiciones lo necesite. El paso por `safetyWaypoint`, si
    /// llegara a ocurrir, NO se registra en el CSV: no es una configuración de
    /// referencia, solo un punto de tránsito seguro.
    public class P9KinematicConsistency : MeasurementTest
    {
        [Header("Configuraciones de referencia (grados)")]
        [Tooltip("El plan pide al menos 8, con varias articulaciones simultáneamente " +
                 "fuera de cero y alguna cerca de los límites. La pose de origen se " +
                 "evita a propósito: ahí las diferencias de convención tienden a " +
                 "cancelarse y la prueba no detectaría nada. Cada valor Y EL ORDEN DE " +
                 "LA LISTA están resueltos por IK numérica para que tanto los destinos " +
                 "como las transiciones entre ellos sean seguros -- ver el comentario " +
                 "de clase antes de reordenar o tocar esta lista a mano, y confirmar " +
                 "cualquier valor nuevo contra WorkspaceSafety (destino Y camino) antes " +
                 "de correr con el robot físico.")]
        public List<Vector6> referenceConfigurations = new List<Vector6>
        {
            // Orden por cercanía articular (vecino más cercano), no el orden en que se
            // resolvieron -- ver el comentario de clase.
            // j1,      j2,       j3,       j4,       j5,     j6       // x,y,z_mm ~ yaw°
            new Vector6(  35.773f, -111.534f, -117.293f,  -41.173f,  90.000f,  -69.227f), // 1: 480,220,150 ~15
            new Vector6(  -5.217f, -105.019f, -111.035f,  -53.946f,  90.000f,  -35.217f), // 2: 520,-150,230 ~-60
            new Vector6( -11.545f,  -84.562f, -106.396f,  -79.042f,  90.000f,  -81.545f), // 3: 420,-190,400 ~-20
            new Vector6(  13.590f, -103.032f,  -74.944f,  -92.023f,  90.000f, -121.410f), // 4: 600,40,480 ~45
            new Vector6(  72.337f,  -98.882f, -102.172f,  -68.946f,  90.000f, -107.663f), // 5: 260,480,330 ~90
            new Vector6( 132.397f, -112.380f, -109.753f,  -47.868f,  90.000f, -107.603f), // 6: -300,480,180 ~150
            new Vector6( 162.337f,  -95.817f,  -92.159f,  -82.024f,  90.000f, -147.663f), // 7: -480,260,420 ~-140
            new Vector6( 146.536f, -108.082f, -103.354f,  -58.565f,  90.000f,  -43.464f), // 8: -420,400,250 ~100
            new Vector6(  85.148f, -139.769f,  -68.403f, -241.828f, -90.000f,    5.148f), // 9: 150,560,140 ~170 (rama j5=-90, ver EXCEPCIÓN)
            new Vector6(   1.825f,  -73.512f,  103.850f, -120.339f, -90.000f,  -98.175f), // 10: -560,-120,260 ~-170
        };

        [Header("Movimiento")]
        public float speedPct = 30f;
        public float arrivalToleranceDeg = 0.5f;
        public float arrivalTimeoutSeconds = 30f;
        [Tooltip("Espera adicional tras confirmar llegada, para que el estado " +
                 "publicado y el modelo terminen de asentarse antes de muestrear.")]
        public float settleSeconds = 1.5f;

        [Tooltip("Punto de paso de seguridad. Cuando el camino DIRECTO entre dos " +
                 "configuraciones consecutivas no pasa WorkspaceSafety.IsPathSafe(), " +
                 "Run() prueba ir primero acá (verificando también ambos tramos por " +
                 "separado) antes de rechazar la configuración de destino. Default: la " +
                 "misma pose base que ya usa el resto del proyecto (SecTrajController, " +
                 "P6) -- NO tiene la pinza hacia abajo, así que solo sirve como tránsito, " +
                 "nunca se registra como configuración de referencia.")]
        public Vector6 safetyWaypoint = new Vector6(0f, -90f, 90f, -90f, 90f, 0f);

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
            int rejectedForSafety = 0;
            int routedThroughWaypoint = 0;

            // Punto de partida real del robot, para poder validar también el
            // camino hacia la PRIMERA configuración (no solo entre consecutivas).
            // Si todavía no hay una lectura válida, se arranca sin ese primer
            // chequeo de camino en vez de bloquear la prueba entera por algo que
            // no es culpa de ninguna configuración de referencia.
            float[] previousCommanded = session.JointSubscriber.GetLastKnownPositions();

            for (int c = 0; c < referenceConfigurations.Count; c++)
            {
                float[] commanded = referenceConfigurations[c].ToArray();
                SetStatus($"configuración {c + 1}/{referenceConfigurations.Count}");

                // Chequeo de seguridad ANTES de mover el robot físico -- tres
                // validaciones independientes, ninguna reemplaza a las otras:
                //   1) espacio articular del DESTINO: IsJointConfigurationSafe()
                //      (j1/j2, restricciones operativas de esta celda, más
                //      estrictas que los límites mecánicos reales del motor).
                //   2) pose cartesiana del DESTINO: IsSafe(), vía
                //      RealRobotForwardKinematics (geometría real del URDF, NUNCA
                //      LocalForwardKinematics -- esa tabla DH difiere ~167 mm en Z
                //      para las mismas juntas, ver el comentario de clase).
                //   3) el CAMINO hasta ahí (IsPathSafe): dos destinos válidos
                //      pueden tener, en el medio del movimiento articular
                //      interpolado, algo que ninguno de los dos extremos delata
                //      -- es justo lo que motivó todo esto (ver comentario de
                //      clase). Si el camino directo falla, se prueba pasar por
                //      safetyWaypoint antes de rechazar.
                // Esto es la red de seguridad si alguien edita referenceConfigurations
                // en el Inspector sin volver a correr el solver de IK.
                bool jointsSafe = WorkspaceSafety.IsJointConfigurationSafe(commanded, out string jointUnsafeReason);
                float[] predictedPose = RealRobotForwardKinematics.CartesianFromJointsDeg(commanded);
                bool poseSafe = WorkspaceSafety.IsSafe(predictedPose, out string poseUnsafeReason);

                if (!jointsSafe || !poseSafe)
                {
                    string unsafeReason = string.Join("; ", new[] { jointUnsafeReason, poseUnsafeReason }
                        .Where(s => !string.IsNullOrEmpty(s)));
                    Debug.LogError($"[P9] Configuración {c + 1} RECHAZADA por seguridad, " +
                                   $"no se comanda al robot: {unsafeReason}");
                    rejectedForSafety++;
                    poses.WriteRow(c, "rechazada_por_seguridad", middlewareSource,
                        commanded[0], commanded[1], commanded[2],
                        commanded[3], commanded[4], commanded[5],
                        null, null, null, null, null, null,
                        predictedPose[0], predictedPose[1], predictedPose[2],
                        predictedPose[3], predictedPose[4], predictedPose[5], false);
                    continue;
                }

                // Chequeo de CAMINO, con repliegue a un tránsito por safetyWaypoint.
                bool viaWaypoint = false;
                if (previousCommanded != null)
                {
                    bool directPathSafe = WorkspaceSafety.IsPathSafe(previousCommanded, commanded, out string directPathReason);
                    if (!directPathSafe)
                    {
                        float[] waypoint = safetyWaypoint.ToArray();
                        bool waypointItselfSafe = WorkspaceSafety.IsJointConfigurationSafe(waypoint, out _) &&
                            WorkspaceSafety.IsSafe(RealRobotForwardKinematics.CartesianFromJointsDeg(waypoint), out _);
                        // IsPathSafe se llama SIEMPRE (no encadenado con && a
                        // waypointItselfSafe): un out declarado dentro de un && con
                        // cortocircuito queda sin asignar si el lado izquierdo ya dio
                        // false, y leg1Reason/leg2Reason hacen falta más abajo pase lo
                        // que pase.
                        bool leg1PathSafe = WorkspaceSafety.IsPathSafe(previousCommanded, waypoint, out string leg1Reason);
                        bool leg2PathSafe = WorkspaceSafety.IsPathSafe(waypoint, commanded, out string leg2Reason);
                        bool leg1Safe = waypointItselfSafe && leg1PathSafe;
                        bool leg2Safe = waypointItselfSafe && leg2PathSafe;

                        if (leg1Safe && leg2Safe)
                        {
                            Debug.LogWarning($"[P9] Configuración {c + 1}: camino directo inseguro " +
                                             $"({directPathReason}); pasando por safetyWaypoint.");
                            viaWaypoint = true;
                        }
                        else
                        {
                            string combined = string.Join("; ", new[] { directPathReason, leg1Reason, leg2Reason }
                                .Where(s => !string.IsNullOrEmpty(s)));
                            Debug.LogError($"[P9] Configuración {c + 1} RECHAZADA por seguridad " +
                                           $"(camino, ni directo ni vía safetyWaypoint son seguros), " +
                                           $"no se comanda al robot: {combined}");
                            rejectedForSafety++;
                            poses.WriteRow(c, "rechazada_por_seguridad_camino", middlewareSource,
                                commanded[0], commanded[1], commanded[2],
                                commanded[3], commanded[4], commanded[5],
                                null, null, null, null, null, null,
                                predictedPose[0], predictedPose[1], predictedPose[2],
                                predictedPose[3], predictedPose[4], predictedPose[5], false);
                            continue;
                        }
                    }
                }

                if (viaWaypoint)
                {
                    routedThroughWaypoint++;
                    bool waypointArrived = false;
                    yield return StartCoroutine(MoveToJointConfiguration(
                        session, safetyWaypoint.ToArray(), speedPct, arrivalToleranceDeg,
                        arrivalTimeoutSeconds, ok => waypointArrived = ok));
                    if (!waypointArrived)
                        Debug.LogWarning($"[P9] No se confirmó la llegada al safetyWaypoint antes " +
                                         $"de la configuración {c + 1}; se continúa igual.");
                }

                bool arrived = false;
                yield return StartCoroutine(MoveToJointConfiguration(
                    session, commanded, speedPct, arrivalToleranceDeg,
                    arrivalTimeoutSeconds, ok => arrived = ok));

                // Se haya confirmado la llegada o no, el robot ya recibió el MoveJ y
                // terminó su intento en (aprox.) `commanded` -- es el punto de partida
                // real para el chequeo de camino de la PRÓXIMA configuración, así que
                // se actualiza acá, antes de cualquiera de los dos `continue` de abajo.
                previousCommanded = commanded;

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
            summary.WriteRow("configuraciones_rechazadas_por_seguridad", rejectedForSafety);
            summary.WriteRow("transiciones_ruteadas_por_safetyWaypoint", routedThroughWaypoint);
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
            string safetyNote = (rejectedForSafety > 0 ? $" ({rejectedForSafety} rechazada(s) por seguridad)" : "") +
                (routedThroughWaypoint > 0 ? $" ({routedThroughWaypoint} ruteada(s) por safetyWaypoint)" : "");
            Finish(ok2, ok2
                ? $"{measured} configuraciones; DH vs middleware: mediana " +
                  $"{st.Median:F2} mm, máx {st.Max:F2} mm{safetyNote}"
                : $"ninguna configuración alcanzada{safetyNote}");
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
