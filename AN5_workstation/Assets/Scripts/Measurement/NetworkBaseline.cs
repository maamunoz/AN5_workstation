using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace AN5.Measurement
{
    /// Latencia base de red por ICMP contra el equipo que hospeda el middleware.
    ///
    /// Es la cifra que permite separar el costo de la RED del costo del PUENTE, que
    /// es justamente la comparación interesante entre C1/C2 y C3/C4: si el tiempo de
    /// ida y vuelta por rosbridge en C1 es de 30 ms y el ping base es de 25 ms, el
    /// puente aporta 5 ms; sin esta referencia, esos 30 ms no se pueden atribuir a
    /// nada en particular.
    ///
    /// Se mide desde la propia aplicación en vez de a mano con la herramienta del
    /// sistema para que quede registrada en el mismo environment.csv que el resto de
    /// las condiciones, tomada en el mismo momento y contra el mismo host al que la
    /// aplicación está realmente conectada.
    public static class NetworkBaseline
    {
        public class Result
        {
            public bool Succeeded;
            public string Error = "";
            public int Sent;
            public int Received;
            public double MeanMs;
            public double StdDevMs;
            public double MinMs;
            public double MaxMs;
            public List<double> Samples = new List<double>();
        }

        /// Intervalo entre paquetes. La herramienta del sistema usa 1 s por defecto,
        /// lo que para 100 paquetes serían 100 s de espera al arrancar cada sesión.
        /// 50 ms cubre la misma cantidad de muestras en 5 s y sigue siendo suficiente
        /// para caracterizar el enlace en reposo. Queda declarado en el CSV para que
        /// el método sea reproducible.
        public const int IntervalMs = 50;

        private const int TimeoutMs = 1000;

        /// Extrae el host de una URL de rosbridge del estilo "ws://192.168.1.5:9090".
        public static string ExtractHost(string rosBridgeUrl)
        {
            if (string.IsNullOrEmpty(rosBridgeUrl)) return "";
            string s = rosBridgeUrl.Trim();

            int scheme = s.IndexOf("://", StringComparison.Ordinal);
            if (scheme >= 0) s = s.Substring(scheme + 3);

            int slash = s.IndexOf('/');
            if (slash >= 0) s = s.Substring(0, slash);

            // IPv6 entre corchetes: [::1]:9090
            if (s.StartsWith("["))
            {
                int close = s.IndexOf(']');
                return close > 0 ? s.Substring(1, close - 1) : s;
            }

            int colon = s.LastIndexOf(':');
            if (colon >= 0) s = s.Substring(0, colon);

            return s;
        }

        public static bool IsLoopback(string host)
        {
            if (string.IsNullOrEmpty(host)) return true;
            string h = host.Trim().ToLowerInvariant();
            return h == "localhost" || h == "127.0.0.1" || h == "::1" || h == "0.0.0.0";
        }

        /// Mide en un hilo aparte y entrega el resultado por callback. El ICMP
        /// síncrono bloquea, así que hacerlo en el hilo principal congelaría la
        /// aplicación varios segundos.
        public static IEnumerator Measure(string host, int packets, Action<Result> onDone)
        {
            var result = new Result();
            bool finished = false;

            var thread = new Thread(() =>
            {
                try
                {
                    RunPings(host, packets, result);
                }
                catch (Exception e)
                {
                    result.Succeeded = false;
                    // El ICMP crudo que usa Ping pide privilegios que la aplicación
                    // puede no tener: en Android/Quest directamente no los tiene: en
                    // Linux suele hacer falta root o CAP_NET_RAW en el binario
                    // (sudo setcap cap_net_raw+ep ./TuApp); en macOS depende de si el
                    // build está firmado. Se reporta el motivo en vez de dejar el
                    // campo vacío como si no se hubiera intentado.
                    result.Error = $"{e.GetType().Name}: {e.Message}";
                }
                finally
                {
                    finished = true;
                }
            });
            thread.IsBackground = true;
            thread.Start();

            while (!finished) yield return null;

            onDone?.Invoke(result);
        }

        private static void RunPings(string host, int packets, Result result)
        {
            using (var ping = new System.Net.NetworkInformation.Ping())
            {
                for (int i = 0; i < packets; i++)
                {
                    result.Sent++;
                    var reply = ping.Send(host, TimeoutMs);
                    if (reply != null &&
                        reply.Status == System.Net.NetworkInformation.IPStatus.Success)
                    {
                        result.Received++;
                        result.Samples.Add(reply.RoundtripTime);
                    }
                    Thread.Sleep(IntervalMs);
                }
            }

            if (result.Samples.Count == 0)
            {
                result.Succeeded = false;
                if (string.IsNullOrEmpty(result.Error))
                    result.Error = "ninguna respuesta ICMP (¿bloqueado por cortafuegos?)";
                return;
            }

            double sum = 0.0;
            result.MinMs = double.MaxValue;
            result.MaxMs = double.MinValue;
            foreach (double v in result.Samples)
            {
                sum += v;
                if (v < result.MinMs) result.MinMs = v;
                if (v > result.MaxMs) result.MaxMs = v;
            }
            result.MeanMs = sum / result.Samples.Count;

            double sqSum = 0.0;
            foreach (double v in result.Samples)
            {
                double d = v - result.MeanMs;
                sqSum += d * d;
            }
            // Desviación muestral (n-1): las muestras son un subconjunto del
            // comportamiento del enlace, no la población completa.
            result.StdDevMs = result.Samples.Count > 1
                ? Math.Sqrt(sqSum / (result.Samples.Count - 1))
                : 0.0;

            result.Succeeded = true;
        }
    }
}
