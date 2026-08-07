using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace AN5.Measurement
{
    /// P10 — Tiempo de preparación de trayectoria.
    ///
    /// Mide cuánto tarda la aplicación desde que se carga un archivo de poses hasta
    /// que hay solución articular disponible para todas ellas. Es un proceso POR LOTES:
    /// cada pose es una ida y vuelta independiente contra MATLAB, resueltas en serie
    /// porque el protocolo no tiene identificador de correlación y dos solicitudes
    /// superpuestas se cruzarían las respuestas (SecTrajController.cs:271-280). Por eso
    /// un valor único no informa: lo que interesa es si el crecimiento con la cantidad
    /// de poses es lineal y cuál es el costo por pose.
    ///
    /// SE REPORTA SEPARADA DE P4 Y P5, Y JAMÁS SUMADA A ELLAS. Es un costo que se paga
    /// una vez al cargar un archivo, no una latencia de operación; sumarlo a las
    /// latencias de comando distorsionaría por completo la comparación con la
    /// literatura.
    ///
    /// LOS ARCHIVOS DE PRUEBA SE GENERAN REMUESTREANDO UNO YA VALIDADO de routines/,
    /// no con poses inventadas. inverse_kinematics.m arrastra reglas de seguridad de
    /// otra celda de robot (caja de posición segura, banda prohibida en Rx,
    /// restricción J4/J5) que pueden rechazar poses perfectamente legítimas, y la
    /// carga es todo-o-nada: un solo punto rechazado cancela el archivo entero y no
    /// habría nada que cronometrar. Partir de una trayectoria que ya se sabe que pasa
    /// el validador evita medir tasa de rechazo en lugar de tiempo de preparación.
    public class P10TrajectoryPrep : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Tamaños de archivo a evaluar, en cantidad de poses.")]
        public int[] poseCounts = { 10, 50, 200 };

        [Tooltip("Repeticiones por tamaño. El plan pide 5.")]
        public int repetitions = 5;

        [Tooltip("Archivo de routines/ del que se remuestrea. Vacío = el que tenga " +
                 "más poses válidas.")]
        public string sourceFileName = "";

        [Tooltip("Plazo máximo por carga. Un archivo de 200 poses contra un MATLAB " +
                 "lento puede tardar minutos.")]
        public float loadTimeoutSeconds = 300f;

        [Tooltip("Segundos de pausa entre cargas.")]
        public float pauseBetweenLoads = 3f;

        [Tooltip("Carga previa de un archivo mínimo, descartada, para absorber el " +
                 "arranque en frío de MATLAB (~13 s) fuera de las mediciones.")]
        public bool warmupLoad = true;

        public override string TestId { get { return "P10"; } }
        public override string DisplayName { get { return "Preparación de trayectoria"; } }

        // Recibe lo que reporta SecTrajController al terminar de resolver.
        private bool _resolveReported;
        private int _reportedPoints;
        private double _reportedSeconds;
        private bool _reportedSuccess;

        public override IEnumerator Run(MeasurementSession session)
        {
            if (!session.IsConnected)
            {
                Finish(false, "sin conexión a rosbridge");
                yield break;
            }

            SecTrajController controller = ResolveController();
            if (controller == null)
            {
                Finish(false, "no se encontró un SecTrajController con IK cableada " +
                              "(¿está la escena completa y conectada?)");
                yield break;
            }

            List<float[]> source = LoadSourceTrajectory(out string sourcePath);
            if (source == null || source.Count < 2)
            {
                Finish(false, $"no se pudo leer una trayectoria de referencia " +
                              $"utilizable en routines/ ({sourcePath})");
                yield break;
            }

            string filesDir = Path.Combine(session.RunDirectory, $"{TestId}_archivos");
            Directory.CreateDirectory(filesDir);

            SecTrajController.TrajectoryResolved += OnTrajectoryResolved;

            var csv = session.OpenCsv($"{TestId}_preparacion",
                "poses", "poses_leidas_por_la_app", "repeticion", "exito",
                "segundos_resolucion_ik", "segundos_total_carga",
                "segundos_por_pose", "archivo");

            if (warmupLoad)
            {
                SetStatus("carga de calentamiento (arranque en frío de MATLAB)...");
                string warmPath = Path.Combine(filesDir, "calentamiento.txt");
                WriteTrajectoryFile(warmPath, Resample(source, 3));
                yield return StartCoroutine(TimedLoad(controller, warmPath, null));
                yield return new WaitForSeconds(pauseBetweenLoads);
            }

            var perSize = new Dictionary<int, List<double>>();
            int total = poseCounts.Length * repetitions;
            int done = 0;

            foreach (int count in poseCounts)
            {
                perSize[count] = new List<double>();

                for (int rep = 0; rep < repetitions; rep++)
                {
                    done++;
                    SetStatus($"{count} poses, repetición {rep + 1}/{repetitions} " +
                              $"({done}/{total})");

                    string path = Path.Combine(filesDir, $"poses_{count}_rep{rep + 1}.txt");
                    WriteTrajectoryFile(path, Resample(source, count));

                    double totalSeconds = 0.0;
                    yield return StartCoroutine(TimedLoad(controller, path,
                        s => totalSeconds = s));

                    bool ok = _resolveReported && _reportedSuccess;
                    double ikSeconds = _resolveReported ? _reportedSeconds : double.NaN;

                    // Se registra también cuántas poses leyó realmente la aplicación:
                    // si no coincide con las escritas, alguna línea no pasó su
                    // validación y el tiempo correspondería a otro tamaño de archivo.
                    csv.WriteRow(count,
                        _resolveReported ? (object)_reportedPoints : null,
                        rep + 1, ok,
                        _resolveReported ? (object)ikSeconds : null,
                        totalSeconds,
                        ok && count > 0 ? (object)(ikSeconds / count) : null,
                        Path.GetFileName(path));

                    if (ok) perSize[count].Add(ikSeconds);

                    yield return new WaitForSeconds(pauseBetweenLoads);
                }
            }

            csv.Dispose();
            SecTrajController.TrajectoryResolved -= OnTrajectoryResolved;

            // --- Resumen: es lo que alimenta la figura de tiempo contra cantidad ---
            var summary = session.OpenCsv($"{TestId}_preparacion_resumen",
                "poses", "n", "media_s", "mediana_s", "minimo_s", "maximo_s",
                "desviacion_s", "media_por_pose_s");

            foreach (int count in poseCounts)
            {
                var st = Stats.From(perSize[count]);
                summary.WriteRow(count, st.Count, st.Mean, st.Median, st.Min, st.Max,
                                 st.StdDev, st.Count > 0 ? (object)(st.Mean / count) : null);
            }
            summary.Dispose();

            var meta = session.OpenCsv($"{TestId}_preparacion_meta", "clave", "valor");
            meta.WriteRow("configuracion", session.ShortConfigLabel());
            meta.WriteRow("plataforma", session.platformLabel);
            meta.WriteRow("archivo_referencia", sourcePath);
            meta.WriteRow("poses_referencia", source.Count);
            meta.WriteRow("calentamiento_descartado", warmupLoad);
            meta.WriteRow("sumable_a_latencias", false);
            meta.WriteRow("nota",
                "proceso por lotes, una ida y vuelta a MATLAB por pose resuelta en serie; " +
                "se reporta aparte de P4/P5 y nunca sumada a ellas");
            meta.Dispose();

            bool anySucceeded = false;
            foreach (var kv in perSize) if (kv.Value.Count > 0) anySucceeded = true;

            Finish(anySucceeded, anySucceeded
                ? DescribeResults(perSize)
                : "ninguna carga se resolvió con éxito");
        }

        private static string DescribeResults(Dictionary<int, List<double>> perSize)
        {
            var parts = new List<string>();
            foreach (var kv in perSize)
            {
                if (kv.Value.Count == 0) continue;
                var st = Stats.From(kv.Value);
                parts.Add($"{kv.Key} poses: {st.Median:F1} s");
            }
            return string.Join(" · ", parts);
        }

        private IEnumerator TimedLoad(SecTrajController controller, string path,
                                      System.Action<double> onTotalSeconds)
        {
            _resolveReported = false;
            var sw = System.Diagnostics.Stopwatch.StartNew();

            Coroutine load = controller.StartCoroutine(controller.LoadTrajectoryFile(path));

            float waited = 0f;
            while (!_resolveReported && waited < loadTimeoutSeconds)
            {
                yield return null;
                waited += Time.unscaledDeltaTime;
            }

            sw.Stop();
            if (!_resolveReported)
            {
                Debug.LogWarning($"[P10] Plazo vencido cargando {Path.GetFileName(path)}");
                controller.StopCoroutine(load);
            }

            onTotalSeconds?.Invoke(sw.Elapsed.TotalSeconds);
        }

        private void OnTrajectoryResolved(int points, double seconds, bool succeeded)
        {
            _reportedPoints = points;
            _reportedSeconds = seconds;
            _reportedSuccess = succeeded;
            _resolveReported = true;
        }

        /// Busca una instancia con la cinemática inversa efectivamente cableada. La
        /// escena arrastra varios GameObjects "SecTraj" duplicados y solo algunos
        /// terminan resueltos (ver SecTrajController.cs:85-88): tomar el primero que
        /// aparezca podría devolver uno inerte, que cargaría el archivo y no
        /// resolvería nada.
        private static SecTrajController ResolveController()
        {
            foreach (var c in FindObjectsOfType<SecTrajController>())
            {
                if (c.ros2CommandSender != null && c.ikSubscriber != null)
                    return c;
            }
            return null;
        }

        /// Lee una trayectoria de referencia de routines/, con el mismo criterio de
        /// línea válida que usa la aplicación (ocho números separados por coma).
        private List<float[]> LoadSourceTrajectory(out string chosenPath)
        {
            chosenPath = "";
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "routines"));
            if (!Directory.Exists(dir)) return null;

            string[] candidates = string.IsNullOrEmpty(sourceFileName)
                ? Directory.GetFiles(dir, "*.txt")
                : new[] { Path.Combine(dir, sourceFileName) };

            List<float[]> best = null;
            foreach (string file in candidates)
            {
                if (!File.Exists(file)) continue;
                var parsed = ParseFile(file);
                if (parsed.Count >= 2 && (best == null || parsed.Count > best.Count))
                {
                    best = parsed;
                    chosenPath = file;
                }
            }
            return best;
        }

        private static List<float[]> ParseFile(string path)
        {
            var result = new List<float[]>();
            foreach (string raw in File.ReadAllLines(path))
            {
                string line = raw.Trim();
                if (string.IsNullOrEmpty(line)) continue;

                string[] parts = line.Split(',');
                if (parts.Length != 8) continue;

                var v = new float[8];
                bool ok = true;
                for (int i = 0; i < 8; i++)
                {
                    if (!float.TryParse(parts[i].Trim(), NumberStyles.Float,
                                        CultureInfo.InvariantCulture, out v[i]))
                    {
                        ok = false;
                        break;
                    }
                }
                if (ok) result.Add(v);
            }
            return result;
        }

        /// Remuestrea linealmente la trayectoria de referencia a la cantidad de poses
        /// pedida, interpolando entre poses consecutivas. Así los archivos grandes
        /// recorren el mismo camino validado con mayor densidad, en vez de repetir
        /// poses idénticas.
        private static List<float[]> Resample(List<float[]> source, int count)
        {
            var result = new List<float[]>(count);
            if (count <= 0 || source.Count == 0) return result;
            if (count == 1) { result.Add((float[])source[0].Clone()); return result; }

            for (int k = 0; k < count; k++)
            {
                double t = (double)k * (source.Count - 1) / (count - 1);
                int i = (int)System.Math.Floor(t);
                if (i >= source.Count - 1) i = source.Count - 2;
                float frac = (float)(t - i);

                float[] a = source[i], b = source[i + 1];
                var p = new float[8];

                // Posición y parámetros: interpolación lineal directa.
                for (int j = 0; j < 3; j++) p[j] = Mathf.Lerp(a[j], b[j], frac);
                // Orientación: por el camino angular más corto. Interpolar los ángulos
                // en crudo cruzaría el salto de ±180° y generaría orientaciones
                // intermedias absurdas que MATLAB rechazaría.
                for (int j = 3; j < 6; j++)
                    p[j] = a[j] + Mathf.DeltaAngle(a[j], b[j]) * frac;
                // Velocidad y espera se toman del punto de partida del tramo, sin
                // interpolar: son parámetros de ejecución, no geometría.
                p[6] = a[6];
                p[7] = a[7];

                result.Add(p);
            }
            return result;
        }

        private static void WriteTrajectoryFile(string path, List<float[]> poses)
        {
            using (var w = new StreamWriter(path, false))
            {
                foreach (float[] p in poses)
                {
                    w.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "{0:F3},{1:F3},{2:F3},{3:F2},{4:F2},{5:F2},{6:F0},{7:F3}",
                        p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]));
                }
            }
        }
    }
}
