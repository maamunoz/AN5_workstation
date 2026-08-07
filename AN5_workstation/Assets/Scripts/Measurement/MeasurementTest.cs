using System;
using System.Collections;
using UnityEngine;

namespace AN5.Measurement
{
    /// Clase base de cada prueba del plan de mediciones (P1..P10).
    ///
    /// Cada prueba es un MonoBehaviour que se agrega al mismo GameObject que
    /// MeasurementSession; la sesión las descubre con GetComponents y las ejecuta
    /// SIEMPRE de a una por vez. Correr dos en paralelo invalidaría a ambas: P7 mide
    /// cuadros por segundo mientras P4 satura el enlace, y cada una alteraría lo que
    /// la otra intenta medir.
    public abstract class MeasurementTest : MonoBehaviour
    {
        /// Identificador corto usado en el nombre del CSV: "P1", "P4", etc.
        public abstract string TestId { get; }

        /// Nombre legible para el panel en pantalla.
        public abstract string DisplayName { get; }

        /// Si la prueba tiene sentido en esta configuración. P6, por ejemplo, solo
        /// aplica a C2/C4 porque mide exactitud contra el robot físico.
        public virtual bool AppliesTo(MeasurementConfiguration config) { return true; }

        /// Motivo por el que no aplica, mostrado en el panel.
        public virtual string NotApplicableReason { get { return ""; } }

        public abstract IEnumerator Run(MeasurementSession session);

        public string Status { get; protected set; } = "sin ejecutar";
        public bool HasRun { get; protected set; }
        public bool LastRunSucceeded { get; protected set; }

        protected void SetStatus(string status)
        {
            Status = status;
            Debug.Log($"[{TestId}] {status}");
        }

        protected void Finish(bool succeeded, string status)
        {
            HasRun = true;
            LastRunSucceeded = succeeded;
            SetStatus(status);
        }

        // ---------------------------------------------------------------------
        // Utilidades compartidas por las pruebas que mueven el robot (P6, P9).
        // Replican la secuencia que ya usa la aplicación real en
        // SecTrajController.ExecutePoints() para que lo medido corresponda al
        // camino de código de producción y no a una variante inventada acá.
        // ---------------------------------------------------------------------

        /// Secuencia de preparación previa a cualquier movimiento: saca el brazo del
        /// modo manual, corta un movimiento en curso y limpia errores. Copiada de
        /// SecTrajController.cs:675-684, incluidos los tiempos de espera.
        protected IEnumerator PrepareRobotForMotion(MeasurementSession session)
        {
            var sender = session.CommandSender;
            if (sender == null) yield break;

            sender.SendCommand("DragTeachSwitch(0)");
            yield return new WaitForSeconds(0.05f);
            sender.SendCommand("StopMotion()");
            yield return new WaitForSeconds(0.05f);
            sender.SendCommand("ResetAllError()");
            yield return new WaitForSeconds(0.05f);
            sender.SendCommand("StartJOG(0,6,0,100)");
            yield return new WaitForSeconds(0.5f);
            sender.SendCommand("StartJOG(0,6,1,100)");
            yield return new WaitForSeconds(1.5f);
        }

        /// Comanda una configuración articular y espera a que el robot la alcance.
        /// Usa JNTPoint+MoveJ (espacio articular) porque los comandos cartesianos
        /// dependen de la cinemática inversa embebida del controlador real, que viene
        /// fallando en este hardware — ver el comentario extenso en
        /// SecTrajController.cs:41-50.
        protected IEnumerator MoveToJointConfiguration(
            MeasurementSession session, float[] jointsDeg, float speedPct,
            float toleranceDeg, float timeoutSeconds, Action<bool> onArrived)
        {
            var sender = session.CommandSender;
            if (sender == null)
            {
                onArrived?.Invoke(false);
                yield break;
            }

            string jnt = string.Format(
                System.Globalization.CultureInfo.InvariantCulture,
                "JNTPoint(1,{0},{1},{2},{3},{4},{5})",
                jointsDeg[0], jointsDeg[1], jointsDeg[2],
                jointsDeg[3], jointsDeg[4], jointsDeg[5]);
            sender.SendCommand(jnt);
            yield return new WaitForSeconds(0.1f);

            sender.SendCommand(string.Format(
                System.Globalization.CultureInfo.InvariantCulture,
                "MoveJ(JNT1,{0:F0})", speedPct));

            bool arrived = false;
            yield return StartCoroutine(WaitForJointArrival(
                session, jointsDeg, toleranceDeg, timeoutSeconds, r => arrived = r));
            onArrived?.Invoke(arrived);
        }

        /// Espera a que current_joint_position confirme la llegada dentro de la
        /// tolerancia. Mismo bucle que SecTrajController.cs:838, con una diferencia
        /// importante para medición: acá el vencimiento del plazo se PROPAGA al
        /// llamador en vez de solo registrarse en consola. Una muestra tomada sin
        /// llegada confirmada no es un dato válido y no debe entrar al CSV como si
        /// lo fuera.
        protected IEnumerator WaitForJointArrival(
            MeasurementSession session, float[] targetDeg, float toleranceDeg,
            float timeoutSeconds, Action<bool> onResult)
        {
            var subscriber = session.JointSubscriber;
            if (subscriber == null)
            {
                onResult?.Invoke(false);
                yield break;
            }

            float elapsed = 0f;
            var wait = new WaitForSeconds(0.05f);

            while (elapsed < timeoutSeconds)
            {
                float[] current = subscriber.GetLastKnownPositions();
                if (current != null && current.Length == targetDeg.Length)
                {
                    bool reached = true;
                    for (int i = 0; i < targetDeg.Length; i++)
                    {
                        if (Mathf.Abs(current[i] - targetDeg[i]) > toleranceDeg)
                        {
                            reached = false;
                            break;
                        }
                    }
                    if (reached)
                    {
                        onResult?.Invoke(true);
                        yield break;
                    }
                }
                yield return wait;
                elapsed += 0.05f;
            }

            onResult?.Invoke(false);
        }

        /// Espera, acotado por `extraTimeoutSeconds`, a que el driver reporte el
        /// movimiento como terminado vía nonrt_state_data/robot_motion_done (ver
        /// RobotMotionDoneSubscriber). Complementa a WaitForJointArrival: esa espera
        /// solo mira si la posición YA ENTRÓ en tolerancia, que puede pasar antes de
        /// que el controlador termine de asentar del todo -- ver el comentario de
        /// clase de P6JointAccuracy sobre el error medio de 0,128° encontrado contra
        /// el emulador. Si la escena no tiene RobotMotionDoneSubscriber, o todavía no
        /// recibió ningún mensaje (driver que no publica esto, o nodo caído), vuelve
        /// de inmediato sin bloquear: es un refinamiento de timing, no un requisito.
        protected IEnumerator WaitForMotionDone(MeasurementSession session, float extraTimeoutSeconds)
        {
            var sub = session.MotionDoneSubscriber;
            if (sub == null || !sub.HasReceivedMessage)
                yield break;

            float elapsed = 0f;
            var wait = new WaitForSeconds(0.05f);
            while (elapsed < extraTimeoutSeconds && !sub.IsMotionDone)
            {
                yield return wait;
                elapsed += 0.05f;
            }
        }
    }
}
