using System;

namespace AN5.Measurement
{
    /// Cinemática directa fiel a la geometría REAL del URDF (mismos
    /// `<origin>` xyz/rpy por junta que usa `mock_cmd_server.py`
    /// `_JOINT_ORIGINS`/`_TOOL_ORIGIN`/`_fk_pose`), no la tabla
    /// Denavit-Hartenberg de `LocalForwardKinematics.cs`.
    ///
    /// POR QUÉ EXISTE UNA SEGUNDA IMPLEMENTACIÓN DE FK EN C#
    ///
    /// `LocalForwardKinematics.cs` ya es conocido por diferir del URDF real
    /// en la longitud de herramienta (d6=0,267 m de la tabla DH vs 0,202 m
    /// real, ver el comentario de P9KinematicConsistency — de hecho medir
    /// esa diferencia es parte de lo que P9 reporta). Al generar las poses
    /// de referencia seguras para P9 se midió el desvío real, no solo el
    /// teórico: para las mismas 10 configuraciones articulares, la tabla DH
    /// da una Z mundial sistemáticamente ~167 mm distinta de la real —bien
    /// por encima de los 65 mm de la sola herramienta, así que hay otra
    /// diferencia de convención de ejes acumulándose además de la
    /// herramienta. Es un desplazamiento constante (no depende de la
    /// configuración articular probada), consistente con que las dos tablas
    /// difieren en dónde ponen el origen/eje Z, no en la cadena cinemática
    /// en sí — pero de cualquier forma es demasiado grande para usarlo como
    /// chequeo de seguridad antes de mover el ROBOT FÍSICO: un margen de
    /// seguridad de 110 mm sobre la mesa es inútil si el modelo que lo
    /// evalúa ya arrastra un error de 167 mm.
    ///
    /// Por eso el chequeo de seguridad de P9 (y de cualquier prueba que
    /// mueva el robot físico) usa ESTA clase, no `LocalForwardKinematics`.
    /// Esta sí replica la cadena real del URDF, así que su error frente al
    /// robot físico es el mismo que ya tiene `mock_cmd_server.py` en
    /// simulación: chico y conocido, no una discrepancia de eje sin acotar.
    public static class RealRobotForwardKinematics
    {
        // (xyz metros, rpy radianes) del <origin> de cada joint j1..j6,
        // aplicado ANTES de la rotación propia del joint (Rz(q), ya que el
        // <axis> de los 6 es "0 0 1" en su propio frame). Copia exacta de
        // mock_cmd_server.py:_JOINT_ORIGINS -- mismo URDF
        // (frcobot_description/urdf/fr5v6.urdf) como fuente de verdad.
        private static readonly (double[] xyz, double[] rpy)[] JointOrigins =
        {
            (new double[] { 0.0, 0.0, 0.0 },      new double[] { 0.0, 0.0, 0.0 }),
            (new double[] { 0.0, 0.0, 0.152 },    new double[] { Math.PI / 2, 0.0, 0.0 }),
            (new double[] { -0.425, 0.0, 0.0 },   new double[] { 0.0, 0.0, 0.0 }),
            (new double[] { -0.39501, 0.0, 0.0 }, new double[] { 0.0, 0.0, 0.0 }),
            (new double[] { 0.0, 0.0, 0.1021 },   new double[] { Math.PI / 2, 0.0, 0.0 }),
            (new double[] { 0.0, 0.0, 0.102 },    new double[] { -Math.PI / 2, 0.0, 0.0 }),
        };

        // <origin> del joint fijo "tool" (j6_Link -> tool_Link), mismo URDF.
        private static readonly double[] ToolXyz = { 0.0, 0.0, 0.1 };
        private static readonly double[] ToolRpy = { 0.0, 0.0, 0.0 };

        /// jointsDeg: 6 ángulos de junta en grados.
        /// Devuelve {x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg}, mismas
        /// unidades y convención RPY (R = Rz(yaw)·Ry(pitch)·Rx(roll)) que
        /// current_cartesian_position.
        public static float[] CartesianFromJointsDeg(float[] jointsDeg)
        {
            double[,] r = Identity3();
            double[] t = { 0.0, 0.0, 0.0 };

            for (int i = 0; i < 6; i++)
            {
                var origin = JointOrigins[i];
                Compose(ref r, ref t, RpyMatrix(origin.rpy[0], origin.rpy[1], origin.rpy[2]), origin.xyz);
                Compose(ref r, ref t, RotZ(jointsDeg[i] * Math.PI / 180.0), new double[] { 0.0, 0.0, 0.0 });
            }
            Compose(ref r, ref t, RpyMatrix(ToolRpy[0], ToolRpy[1], ToolRpy[2]), ToolXyz);

            double roll, pitch, yaw;
            RotToRpy(r, out roll, out pitch, out yaw);

            return new float[]
            {
                (float)(t[0] * 1000.0), (float)(t[1] * 1000.0), (float)(t[2] * 1000.0),
                (float)(roll * 180.0 / Math.PI), (float)(pitch * 180.0 / Math.PI), (float)(yaw * 180.0 / Math.PI),
            };
        }

        private static double[,] RotZ(double theta)
        {
            double c = Math.Cos(theta), s = Math.Sin(theta);
            return new double[,] { { c, -s, 0.0 }, { s, c, 0.0 }, { 0.0, 0.0, 1.0 } };
        }

        // Convención URDF/ROS: R = Rz(yaw) * Ry(pitch) * Rx(roll), ejes fijos
        // del frame padre. Idéntica a mock_cmd_server.py:_rpy_matrix.
        private static double[,] RpyMatrix(double roll, double pitch, double yaw)
        {
            double cr = Math.Cos(roll), sr = Math.Sin(roll);
            double cp = Math.Cos(pitch), sp = Math.Sin(pitch);
            double cy = Math.Cos(yaw), sy = Math.Sin(yaw);
            return new double[,]
            {
                { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
                { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
                { -sp,     cp * sr,                cp * cr },
            };
        }

        // Compone (r, t) := (r, t) * (rStep, tStep), igual que
        // mock_cmd_server.py:_compose ("resultado = T1 * T2").
        private static void Compose(ref double[,] r, ref double[] t, double[,] rStep, double[] tStep)
        {
            var rNew = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < 3; k++) sum += r[i, k] * rStep[k, j];
                    rNew[i, j] = sum;
                }

            var tNew = new double[3];
            for (int i = 0; i < 3; i++)
            {
                double sum = t[i];
                for (int k = 0; k < 3; k++) sum += r[i, k] * tStep[k];
                tNew[i] = sum;
            }

            r = rNew;
            t = tNew;
        }

        private static double[,] Identity3()
        {
            return new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        }

        // Extrae roll,pitch,yaw de R con la misma convención de RpyMatrix.
        // Copia de mock_cmd_server.py:_rot_to_rpy, incluida la rama de
        // bloqueo de cardán (pitch = ±90°).
        private static void RotToRpy(double[,] r, out double roll, out double pitch, out double yaw)
        {
            double sinp = Math.Max(-1.0, Math.Min(1.0, -r[2, 0]));
            pitch = Math.Asin(sinp);
            double cp = Math.Cos(pitch);
            if (Math.Abs(cp) > 1e-6)
            {
                roll = Math.Atan2(r[2, 1], r[2, 2]);
                yaw = Math.Atan2(r[1, 0], r[0, 0]);
            }
            else
            {
                roll = 0.0;
                yaw = Math.Atan2(-r[0, 1], r[1, 1]);
            }
        }
    }
}
