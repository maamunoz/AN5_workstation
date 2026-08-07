using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Rosapi;

namespace AN5.Measurement
{
    /// P2 — Integridad de la interfaz de tópicos.
    ///
    /// Comprueba que el grafo ROS 2 expone los tópicos y tipos que el cliente espera.
    /// Es la verificación que respalda P8: si la lista y los tipos son idénticos contra
    /// el emulador y contra el robot físico, la afirmación de que la transición entre
    /// destinos no exigió modificar la aplicación queda sostenida por evidencia y no
    /// por una impresión.
    ///
    /// Se consulta por /rosapi/topics, que rosbridge levanta junto con el servidor
    /// websocket. La respuesta ya trae tópicos y tipos apareados, así que no hace falta
    /// una segunda llamada por cada uno.
    ///
    /// DISTINGUE REQUERIDOS DE OPCIONALES a propósito. Varios de estos tópicos solo
    /// existen mientras algo los está publicando o suscribiendo: los de cinemática
    /// inversa aparecen únicamente con matlab_ik_node levantado, y los de cinemática
    /// directa solo si el lazo interno de Unity está activo. Marcar como falla la
    /// ausencia de un tópico cuyo productor no se levantó sería confundir "la interfaz
    /// no coincide" con "no arranqué ese proceso".
    public class P2TopicIntegrity : MeasurementTest
    {
        [Tooltip("Plazo de la llamada a /rosapi/topics.")]
        public float timeoutSeconds = 10f;

        public override string TestId { get { return "P2"; } }
        public override string DisplayName { get { return "Integridad de tópicos"; } }

        private struct Expected
        {
            public string Topic;
            public string Type;
            public bool Required;
            public string Note;

            public Expected(string topic, string type, bool required, string note = "")
            {
                Topic = topic; Type = type; Required = required; Note = note;
            }
        }

        /// Lo que el cliente realmente consume, tomado del código de la aplicación y
        /// no de la documentación.
        private static readonly Expected[] ExpectedInterface =
        {
            new Expected("current_joint_position",     "std_msgs/String", true,
                "estado articular que anima el modelo (JointPositionSubscriber)"),
            new Expected("current_cartesian_position", "std_msgs/String", true,
                "pose cartesiana mostrada y grabada (CartesianPositionSubscriber)"),
            new Expected("api_command",                "std_msgs/String", true,
                "comandos hacia el robot (Ros2CommandSender)"),
            new Expected("setpoint_cartesian_position","std_msgs/String", false,
                "solo lo publica el emulador; el driver real nunca lo emite"),
            new Expected("joint_states",               "sensor_msgs/JointState", false,
                "lo agrega el emulador para consumidores ROS estándar"),
            new Expected("nonrt_state_data",           "frhal_msgs/FRState", false,
                "estado del driver, no lo consume Unity"),
            new Expected("input_cartesian_position",   "std_msgs/String", false,
                "requiere matlab_ik_node levantado"),
            new Expected("output_joint_position",      "std_msgs/String", false,
                "requiere matlab_ik_node levantado"),
            new Expected("input_joint_position",       "std_msgs/String", false,
                "lazo de cinemática directa interno de Unity (MGD_Node)"),
            new Expected("output_cartesian_position",  "std_msgs/String", false,
                "lazo de cinemática directa interno de Unity (MGD_Node)"),
        };

        private readonly object _lock = new object();
        private TopicsResponse _response;
        private bool _hasResponse;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }

            lock (_lock) { _response = null; _hasResponse = false; }

            session.Socket.CallService<TopicsRequest, TopicsResponse>(
                "/rosapi/topics", OnTopics, new TopicsRequest());

            float waited = 0f;
            while (waited < timeoutSeconds)
            {
                lock (_lock) { if (_hasResponse) break; }
                yield return null;
                waited += Time.unscaledDeltaTime;
            }

            TopicsResponse response;
            lock (_lock) { response = _response; }

            if (response == null || response.topics == null)
            {
                Finish(false, "sin respuesta de /rosapi/topics (¿está corriendo rosapi?)");
                yield break;
            }

            // rosapi devuelve los nombres con barra inicial; la aplicación los usa
            // mayormente sin ella. Se normaliza para que la comparación no falle por
            // eso y se reporte una discrepancia que no existe.
            var actual = new Dictionary<string, string>();
            for (int i = 0; i < response.topics.Length; i++)
            {
                string name = Normalize(response.topics[i]);
                string type = (response.types != null && i < response.types.Length)
                    ? response.types[i] : "";
                actual[name] = type;
            }

            var csv = session.OpenCsv($"{TestId}_topicos",
                "topico", "requerido", "presente", "tipo_esperado", "tipo_hallado",
                "tipo_coincide", "nota");

            int requiredMissing = 0, typeMismatches = 0, presentCount = 0;

            foreach (var exp in ExpectedInterface)
            {
                string key = Normalize(exp.Topic);
                bool present = actual.TryGetValue(key, out string foundType);
                bool typeOk = present && string.Equals(foundType, exp.Type,
                                                       System.StringComparison.Ordinal);

                if (present) presentCount++;
                if (!present && exp.Required) requiredMissing++;
                if (present && !typeOk) typeMismatches++;

                csv.WriteRow(exp.Topic, exp.Required, present, exp.Type,
                             present ? foundType : "", present ? (object)typeOk : null,
                             exp.Note);
            }
            csv.Dispose();

            // Lista completa del grafo, para poder comparar dos corridas (emulador
            // contra robot) sin depender de la lista esperada codificada acá.
            var all = session.OpenCsv($"{TestId}_topicos_del_grafo", "topico", "tipo");
            for (int i = 0; i < response.topics.Length; i++)
            {
                all.WriteRow(response.topics[i],
                    (response.types != null && i < response.types.Length) ? response.types[i] : "");
            }
            all.Dispose();

            var summary = session.OpenCsv($"{TestId}_topicos_resumen", "metrica", "valor");
            summary.WriteRow("configuracion", session.ShortConfigLabel());
            summary.WriteRow("plataforma", session.platformLabel);
            summary.WriteRow("destino", session.HasPhysicalRobot ? "robot_fisico" : "emulador");
            summary.WriteRow("topicos_en_el_grafo", response.topics.Length);
            summary.WriteRow("esperados_totales", ExpectedInterface.Length);
            summary.WriteRow("esperados_presentes", presentCount);
            summary.WriteRow("requeridos_faltantes", requiredMissing);
            summary.WriteRow("tipos_discrepantes", typeMismatches);
            // El veredicto binario que alimenta la tabla de cobertura.
            summary.WriteRow("interfaz_coincide", requiredMissing == 0 && typeMismatches == 0);
            summary.Dispose();

            bool ok = requiredMissing == 0 && typeMismatches == 0;
            Finish(ok, ok
                ? $"coincide ({presentCount}/{ExpectedInterface.Length} presentes, " +
                  $"{response.topics.Length} en el grafo)"
                : $"{requiredMissing} requeridos faltantes, {typeMismatches} tipos discrepantes");
        }

        private static string Normalize(string topic)
        {
            if (string.IsNullOrEmpty(topic)) return "";
            return topic.StartsWith("/") ? topic.Substring(1) : topic;
        }

        private void OnTopics(TopicsResponse response)
        {
            lock (_lock)
            {
                _response = response;
                _hasResponse = true;
            }
        }
    }
}
