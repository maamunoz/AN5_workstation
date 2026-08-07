using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AN5.Measurement
{
    /// P7 — Rendimiento gráfico.
    ///
    /// Muestrea el tiempo de cada cuadro durante una sesión de operación y reporta
    /// media y percentil 1 inferior. La cifra que importa no es la absoluta sino si
    /// el sistema SOSTIENE la frecuencia objetivo del dispositivo: 72 cuadros por
    /// segundo de media en un visor que pide 72 es un resultado distinto que los
    /// mismos 72 en un monitor de 144, y por eso se registra también esa referencia.
    ///
    /// El percentil 1 inferior está por encima de la media en importancia: una caída
    /// breve pero repetida se percibe como tirones aunque la media no la delate,
    /// exactamente el mismo argumento por el que P3 separa pérdida de frecuencia
    /// media.
    ///
    /// La prueba es PASIVA: no comanda nada ni toca la aplicación. El plan pide medir
    /// durante operación normal, así que quien ejecuta debe estar usando la interfaz
    /// (girando cámara, moviendo el robot) mientras corre la ventana. Medir con la
    /// aplicación quieta y presentarlo como desempeño en operación es una de las
    /// trampas que el propio plan enumera.
    public class P7GraphicsPerf : MeasurementTest
    {
        [Header("Parámetros")]
        [Tooltip("Duración de la sesión de muestreo. El plan pide al menos 2 minutos.")]
        public float windowSeconds = 120f;

        [Tooltip("Guarda el tiempo de cada cuadro individual además del resumen. " +
                 "Son unas 7000 filas en 2 minutos, y permiten rehacer cualquier " +
                 "estadístico después sin volver a medir.")]
        public bool writeRawSamples = true;

        [Tooltip("Frecuencia objetivo del dispositivo, en Hz. Dejalo en 0 para que la " +
                 "tome de la pantalla; en el visor conviene fijarla a mano (72, 90, 120).")]
        public float targetRefreshHzOverride = 0f;

        public override string TestId { get { return "P7"; } }
        public override string DisplayName { get { return "Rendimiento gráfico"; } }

        public override IEnumerator Run(MeasurementSession session)
        {
            var frameMs = new List<double>(8192);

            float elapsed = 0f;
            int lastReported = -1;

            // Se descarta el primer cuadro: incluye el trabajo de arranque de la
            // propia corrutina y aparecería como un valor atípico que no corresponde
            // a nada del renderizado.
            yield return null;

            while (elapsed < windowSeconds)
            {
                yield return null;

                // unscaledDeltaTime y no deltaTime: si algo alterara Time.timeScale,
                // deltaTime dejaría de representar tiempo real de cuadro.
                float dt = Time.unscaledDeltaTime;
                if (dt > 0f) frameMs.Add(dt * 1000.0);
                elapsed += dt;

                int sec = Mathf.FloorToInt(elapsed);
                if (sec != lastReported)
                {
                    lastReported = sec;
                    SetStatus($"muestreando {sec}/{Mathf.CeilToInt(windowSeconds)} s " +
                              $"({frameMs.Count} cuadros)");
                }
            }

            if (frameMs.Count == 0)
            {
                Finish(false, "no se registró ningún cuadro");
                yield break;
            }

            // Frecuencia instantánea por cuadro: es sobre esta distribución que se
            // calcula el percentil 1 inferior.
            var fps = new List<double>(frameMs.Count);
            foreach (double ms in frameMs) fps.Add(1000.0 / ms);

            var fpsStats = Stats.From(fps);
            var msStats = Stats.From(frameMs);

            float targetHz = targetRefreshHzOverride > 0f
                ? targetRefreshHzOverride
                : (float)Screen.currentResolution.refreshRateRatio.value;

            var csv = session.OpenCsv($"{TestId}_grafico_resumen", "metrica", "valor");
            csv.WriteRow("configuracion", session.ShortConfigLabel());
            csv.WriteRow("plataforma", session.platformLabel);
            csv.WriteRow("ventana_s", windowSeconds);
            csv.WriteRow("cuadros_totales", frameMs.Count);

            csv.WriteRow("fps_medio", fpsStats.Mean);
            csv.WriteRow("fps_mediana", fpsStats.Median);
            // "Percentil 1 inferior": el 1 % de cuadros más lentos, que es donde se
            // manifiestan los tirones.
            csv.WriteRow("fps_percentil_1_inferior", fpsStats.P1);
            csv.WriteRow("fps_minimo", fpsStats.Min);
            csv.WriteRow("fps_maximo", fpsStats.Max);

            csv.WriteRow("tiempo_cuadro_medio_ms", msStats.Mean);
            csv.WriteRow("tiempo_cuadro_mediana_ms", msStats.Median);
            csv.WriteRow("tiempo_cuadro_p95_ms", msStats.P95);
            csv.WriteRow("tiempo_cuadro_p99_ms", msStats.P99);
            csv.WriteRow("tiempo_cuadro_maximo_ms", msStats.Max);

            csv.WriteRow("frecuencia_objetivo_hz", targetHz);
            csv.WriteRow("sostiene_objetivo_media", targetHz > 0 && fpsStats.Mean >= targetHz * 0.95);
            csv.WriteRow("sostiene_objetivo_p1", targetHz > 0 && fpsStats.P1 >= targetHz * 0.95);
            csv.WriteRow("target_frame_rate_configurado", Application.targetFrameRate);
            csv.WriteRow("vsync_count", QualitySettings.vSyncCount);
            csv.WriteRow("gpu", SystemInfo.graphicsDeviceName);
            csv.Dispose();

            if (writeRawSamples)
            {
                var raw = session.OpenCsv($"{TestId}_grafico_cuadros", "indice", "tiempo_cuadro_ms", "fps");
                for (int i = 0; i < frameMs.Count; i++)
                    raw.WriteRow(i, frameMs[i], fps[i]);
                raw.Dispose();
            }

            Finish(true, $"media {fpsStats.Mean:F1} FPS, p1 inferior {fpsStats.P1:F1} FPS, " +
                         $"objetivo {targetHz:F0} Hz ({frameMs.Count} cuadros)");
        }
    }
}
