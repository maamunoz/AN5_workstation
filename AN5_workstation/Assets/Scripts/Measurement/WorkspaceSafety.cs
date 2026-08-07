using UnityEngine;

namespace AN5.Measurement
{
    /// Límites de espacio de trabajo seguro para pruebas que mueven el
    /// ROBOT FÍSICO (C2/C4). Específicos de esta celda, no derivados del
    /// URDF: la mesa y el entorno de montaje no están modelados en ningún
    /// lado del proyecto, así que estos valores vienen de la observación
    /// directa de qué configuraciones son físicamente alcanzables sin
    /// colisión, no de un cálculo geométrico.
    ///
    /// USAR SIEMPRE `RealRobotForwardKinematics`, NUNCA `LocalForwardKinematics`,
    /// para evaluar estos límites: la tabla DH de LocalForwardKinematics
    /// difiere del URDF real en ~167 mm de Z para las mismas juntas (medido
    /// empíricamente al generar las poses de P9; ver el comentario de
    /// RealRobotForwardKinematics.cs) — un margen de seguridad de 110 mm
    /// evaluado con ese modelo no significa nada.
    public static class WorkspaceSafety
    {
        /// Piso de altura: por debajo de esto el efector atraviesa la mesa.
        public const float MinZMm = 110f;

        /// Zona restringida "hacia atrás" del robot: cualquier punto con
        /// X dentro de [-350, 350] mm Y ADEMÁS con Y por debajo de -300 mm.
        /// Fuera de esa combinación (aunque Y sea muy negativo, si X está
        /// bien afuera de la banda central) no aplica la restricción.
        public const float BackZoneXMin = -350f;
        public const float BackZoneXMax = 350f;
        public const float BackZoneYMax = -300f;

        /// Tolerancia angular para considerar que la pinza "mira hacia
        /// abajo" (rx ≈ ±180°, ry ≈ 0°, convención RPY de este proyecto —
        /// ver routines/*.txt, todas las poses validadas usan rx=180,ry=0).
        public const float GripperDownToleranceDeg = 5f;

        /// Restricción OPERATIVA de j2 para esta celda: más estricta que el
        /// límite mecánico real del motor (-265,03° a 85°, ver JOINT_LIMITS
        /// en mock_cmd_server.py:74-81). No viene del URDF ni de ningún
        /// cálculo geométrico -- la fijó el usuario tras observar que, para
        /// esta celda concreta, valores de j2 fuera de este rango son
        /// peligrosos independientemente de dónde termine el efector (a
        /// diferencia de Z/zona trasera/pinza-abajo, que son restricciones
        /// sobre la POSE cartesiana resultante, esta es sobre el ÁNGULO DE
        /// JUNTA en sí — por eso tiene su propio método, IsJointConfigurationSafe,
        /// en vez de colarse en IsSafe()).
        public static readonly Vector2 J2AllowedRangeDeg = new Vector2(-145f, -45f);

        /// Restricción OPERATIVA de j1: zona PROHIBIDA (a diferencia de
        /// J2AllowedRangeDeg, que es un rango PERMITIDO) por riesgo de
        /// choque con la pared detrás del robot. Tampoco viene del URDF.
        public static readonly Vector2 J1ForbiddenRangeDeg = new Vector2(-120f, -30f);

        /// Evalúa SOLO Z y zona trasera de una pose {x_mm,y_mm,z_mm,...} -- las dos
        /// restricciones que dependen nada más de dónde queda el punto en el
        /// espacio, no de hacia dónde mira la herramienta. Separado de IsSafe()
        /// (que además exige pinza-hacia-abajo) porque hay usos legítimos que
        /// necesitan el chequeo puramente geométrico sin esa exigencia: IsPathSafe()
        /// de abajo (a mitad de camino la muñeca puede no estar "abajo" sin que
        /// eso implique riesgo), y P6JointAccuracy al escanear qué tramos de j1
        /// son seguros partiendo de basePoseDeg -- una pose que, en este proyecto,
        /// nunca tiene la pinza hacia abajo (rx=0, ver el comentario de
        /// P6JointAccuracy), así que exigírselo ahí rechazaría todo el escaneo por
        /// un motivo que no tiene nada que ver con colisión.
        public static bool IsCartesianGeometrySafe(float[] poseMm6, out string reason)
        {
            float x = poseMm6[0], y = poseMm6[1], z = poseMm6[2];
            var problems = new System.Collections.Generic.List<string>();

            if (z < MinZMm)
                problems.Add($"Z={z:F1}mm < {MinZMm}mm (colisión con la mesa)");

            bool inBackZone = x >= BackZoneXMin && x <= BackZoneXMax && y < BackZoneYMax;
            if (inBackZone)
                problems.Add($"X={x:F1}mm,Y={y:F1}mm dentro de la zona trasera restringida " +
                             $"(X en [{BackZoneXMin},{BackZoneXMax}], Y<{BackZoneYMax})");

            reason = string.Join("; ", problems);
            return problems.Count == 0;
        }

        /// Evalúa una pose {x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg} contra los
        /// tres límites. Devuelve true si es segura; si no, `reason` explica
        /// cuál se violó (puede haber más de uno; se listan todos).
        public static bool IsSafe(float[] poseMm6, out string reason)
        {
            float rx = poseMm6[3], ry = poseMm6[4];

            bool geometryOk = IsCartesianGeometrySafe(poseMm6, out string geometryReason);

            // "Pinza hacia abajo" = eje Z local de la herramienta alineado
            // con -Z mundial. Con rx≈180°,ry≈0° (la convención que usa este
            // proyecto) alcanza con comparar los ángulos directamente en
            // vez de reconstruir la matriz de rotación completa.
            float rxNorm = Mathf.DeltaAngle(0f, rx); // normaliza a [-180,180]
            bool gripperDown = Mathf.Abs(Mathf.Abs(rxNorm) - 180f) <= GripperDownToleranceDeg
                                && Mathf.Abs(ry) <= GripperDownToleranceDeg;

            var problems = new System.Collections.Generic.List<string>();
            if (!string.IsNullOrEmpty(geometryReason)) problems.Add(geometryReason);
            if (!gripperDown)
                problems.Add($"pinza no apunta hacia abajo (rx={rx:F1}°, ry={ry:F1}°, " +
                             $"se esperaba rx≈±180°, ry≈0°)");

            reason = string.Join("; ", problems);
            return geometryOk && gripperDown;
        }

        /// Evalúa la configuración articular en sí (grados, j1..j6) contra
        /// restricciones que no se pueden derivar de dónde termina el
        /// efector -- j2 (rango permitido) y j1 (zona prohibida). Se llama
        /// SEPARADO de IsSafe(): una tiene que pasar la pose cartesiana Y la
        /// otra la configuración articular, ninguna reemplaza a la otra.
        public static bool IsJointConfigurationSafe(float[] jointsDeg, out string reason)
        {
            var problems = new System.Collections.Generic.List<string>();

            float j1 = jointsDeg[0];
            if (j1 >= J1ForbiddenRangeDeg.x && j1 <= J1ForbiddenRangeDeg.y)
                problems.Add($"j1={j1:F1}° dentro de la zona prohibida " +
                             $"[{J1ForbiddenRangeDeg.x},{J1ForbiddenRangeDeg.y}] (riesgo de choque con la pared)");

            float j2 = jointsDeg[1];
            if (j2 < J2AllowedRangeDeg.x || j2 > J2AllowedRangeDeg.y)
                problems.Add($"j2={j2:F1}° fuera del rango operativo permitido " +
                             $"[{J2AllowedRangeDeg.x},{J2AllowedRangeDeg.y}]");

            reason = string.Join("; ", problems);
            return problems.Count == 0;
        }

        /// Evalúa el camino ARTICULAR INTERPOLADO entre dos configuraciones —
        /// el mismo tipo de movimiento que hace MoveJ punto a punto: cada junta
        /// interpola linealmente de su valor en `fromDeg` al de `toDeg`, NO es
        /// una línea recta cartesiana. Chequea, en `samples` pasos a lo largo
        /// de todo el camino: Z, zona trasera, y los rangos operativos de
        /// j1/j2. Deliberadamente NO exige pinza-hacia-abajo a mitad de
        /// camino (sí en los extremos, vía IsSafe() sobre cada configuración
        /// de destino, ya llamado aparte) -- a mitad de un movimiento largo la
        /// muñeca puede pasar por orientaciones que no son "abajo" sin que eso
        /// implique riesgo de colisión, a diferencia de los otros cuatro.
        ///
        /// POR QUÉ EXISTE: el problema real que motivó este método no era
        /// ninguna configuración individual insegura -- las 10 de referencia
        /// de P9 son, cada una, un extremo válido. Era el camino ARTICULAR
        /// ENTRE dos de ellas el que cruzaba de lleno la zona prohibida de j1
        /// sin que ninguno de los dos extremos lo delatara (confirmado: dos
        /// transiciones de las 9 originales pasaban derecho por
        /// J1ForbiddenRangeDeg a mitad de camino). Validar solo endpoints
        /// nunca iba a detectar eso.
        public static bool IsPathSafe(float[] fromDeg, float[] toDeg, out string reason, int samples = 100)
        {
            bool j1Bad = false, j2Bad = false, zBad = false, backZoneBad = false;
            float worstZ = float.MaxValue;

            for (int s = 0; s <= samples; s++)
            {
                float t = (float)s / samples;
                var q = new float[6];
                for (int i = 0; i < 6; i++) q[i] = fromDeg[i] + (toDeg[i] - fromDeg[i]) * t;

                if (!IsJointConfigurationSafe(q, out string jointReason))
                {
                    if (jointReason.Contains("j1")) j1Bad = true;
                    if (jointReason.Contains("j2")) j2Bad = true;
                }

                float[] pose = RealRobotForwardKinematics.CartesianFromJointsDeg(q);
                if (pose[2] < worstZ) worstZ = pose[2];
                if (!IsCartesianGeometrySafe(pose, out string geometryReason))
                {
                    if (geometryReason.Contains("Z=")) zBad = true;
                    if (geometryReason.Contains("zona trasera")) backZoneBad = true;
                }
            }

            var problems = new System.Collections.Generic.List<string>();
            if (j1Bad) problems.Add("el camino cruza la zona prohibida de j1 a mitad de movimiento");
            if (j2Bad) problems.Add("el camino saca a j2 de su rango operativo a mitad de movimiento");
            if (zBad) problems.Add($"el camino baja a Z={worstZ:F1}mm (< {MinZMm}mm) a mitad de movimiento");
            if (backZoneBad) problems.Add("el camino entra a la zona trasera restringida a mitad de movimiento");

            reason = string.Join("; ", problems);
            return problems.Count == 0;
        }
    }
}
