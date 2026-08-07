using System;

namespace AN5.Measurement
{
    /// Aritmética de poses compartida por P9.
    ///
    /// Todo en double y sin usar UnityEngine.Quaternion a propósito: el tipo de Unity
    /// vive en un sistema de coordenadas zurdo y arrastra sus propias convenciones,
    /// mientras que acá se necesita operar en el marco de ROS. Hacer la cuenta a mano
    /// evita una conversión implícita silenciosa justo en la prueba cuyo objetivo es
    /// detectar diferencias de convención.
    ///
    /// La convención de ángulos es la misma en las tres fuentes que P9 compara:
    /// R = Rz(rz) * Ry(ry) * Rx(rx), ejes fijos — la que usa _rot_to_rpy del emulador
    /// (mock_cmd_server.py:185-199) y la que usa LocalForwardKinematics.cs:37-39.
    public static class PoseMath
    {
        /// Cuaternión (x,y,z,w) a partir de ángulos RPY en grados, convención ZYX.
        public static double[] RpyDegToQuat(double rxDeg, double ryDeg, double rzDeg)
        {
            double roll = rxDeg * Math.PI / 180.0;
            double pitch = ryDeg * Math.PI / 180.0;
            double yaw = rzDeg * Math.PI / 180.0;

            double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
            double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
            double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);

            return new[]
            {
                sr * cp * cy - cr * sp * sy, // x
                cr * sp * cy + sr * cp * sy, // y
                cr * cp * sy - sr * sp * cy, // z
                cr * cp * cy + sr * sp * sy, // w
            };
        }

        /// Ángulos RPY en grados a partir de un cuaternión (x,y,z,w), convención ZYX.
        public static double[] QuatToRpyDeg(double x, double y, double z, double w)
        {
            double sinp = 2.0 * (w * y - z * x);
            sinp = Math.Max(-1.0, Math.Min(1.0, sinp));
            double pitch = Math.Asin(sinp);

            double roll = Math.Atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
            double yaw = Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            return new[]
            {
                roll * 180.0 / Math.PI,
                pitch * 180.0 / Math.PI,
                yaw * 180.0 / Math.PI,
            };
        }

        /// Ángulo geodésico entre dos orientaciones, en grados.
        ///
        /// Es el ángulo de la rotación mínima que lleva una a la otra, y es la única
        /// forma correcta de cuantificar un error de orientación. Restar ángulos de
        /// Euler eje a eje NO sirve: cerca del bloqueo de cardán dos orientaciones
        /// idénticas pueden tener representaciones RPY muy distintas y la resta daría
        /// un error enorme e inexistente. El emulador tiene una rama explícita para
        /// ese caso (mock_cmd_server.py:194-198), así que el riesgo es concreto, no
        /// teórico.
        public static double GeodesicAngleDeg(double[] qa, double[] qb)
        {
            double dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
            // El valor absoluto identifica q con -q: representan la misma orientación.
            dot = Math.Abs(dot);
            dot = Math.Max(-1.0, Math.Min(1.0, dot));
            return 2.0 * Math.Acos(dot) * 180.0 / Math.PI;
        }

        public static double OrientationErrorDeg(double[] rpyA, double[] rpyB)
        {
            return GeodesicAngleDeg(
                RpyDegToQuat(rpyA[0], rpyA[1], rpyA[2]),
                RpyDegToQuat(rpyB[0], rpyB[1], rpyB[2]));
        }

        /// Distancia euclídea entre dos posiciones (mismas unidades que la entrada).
        public static double PositionErrorNorm(double[] a, double[] b)
        {
            double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
}
