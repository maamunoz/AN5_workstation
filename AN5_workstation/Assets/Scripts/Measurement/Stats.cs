using System;
using System.Collections.Generic;

namespace AN5.Measurement
{
    /// Resumen estadístico de una serie de muestras.
    ///
    /// El plan de mediciones es explícito en que reportar solo la media no sirve:
    /// las distribuciones de latencia de red son casi siempre asimétricas, y una
    /// media sin dispersión ni percentil alto no es interpretable. Por eso todas las
    /// pruebas que producen series usan esta misma estructura y escriben SIEMPRE
    /// mediana, percentil 95, máximo y cantidad de muestras junto a la media.
    ///
    /// Las series individuales igual se guardan completas en su propio CSV: el
    /// resumen es para las tablas del artículo, no un reemplazo de los datos crudos.
    public struct Stats
    {
        public int Count;
        public double Mean;
        public double Median;
        public double P95;
        public double P99;
        public double P1;
        public double Min;
        public double Max;
        public double StdDev;

        /// Columnas del resumen, en el mismo orden que EmitRow().
        public static string[] SummaryColumns(string prefix)
        {
            return new[]
            {
                prefix + "_n",
                prefix + "_media",
                prefix + "_mediana",
                prefix + "_p95",
                prefix + "_maximo",
                prefix + "_minimo",
                prefix + "_desviacion",
            };
        }

        public object[] SummaryValues()
        {
            return new object[] { Count, Mean, Median, P95, Max, Min, StdDev };
        }

        public static Stats From(IEnumerable<double> values)
        {
            var list = new List<double>(values);
            var s = new Stats { Count = list.Count };
            if (list.Count == 0)
            {
                s.Mean = s.Median = s.P95 = s.P99 = s.P1 = s.Min = s.Max = s.StdDev = double.NaN;
                return s;
            }

            list.Sort();

            double sum = 0.0;
            foreach (double v in list) sum += v;
            s.Mean = sum / list.Count;

            s.Min = list[0];
            s.Max = list[list.Count - 1];
            s.Median = Percentile(list, 50.0);
            s.P95 = Percentile(list, 95.0);
            s.P99 = Percentile(list, 99.0);
            s.P1 = Percentile(list, 1.0);

            if (list.Count > 1)
            {
                double sq = 0.0;
                foreach (double v in list)
                {
                    double d = v - s.Mean;
                    sq += d * d;
                }
                s.StdDev = Math.Sqrt(sq / (list.Count - 1));
            }
            else
            {
                s.StdDev = 0.0;
            }

            return s;
        }

        /// Percentil por interpolación lineal entre rangos contiguos, sobre una lista
        /// YA ORDENADA. Es la definición que usan numpy y MATLAB por defecto, así que
        /// las cifras del artículo coinciden con las que dé cualquier reproceso
        /// posterior de los CSV crudos.
        public static double Percentile(List<double> sorted, double p)
        {
            if (sorted.Count == 0) return double.NaN;
            if (sorted.Count == 1) return sorted[0];

            double rank = (p / 100.0) * (sorted.Count - 1);
            int lo = (int)Math.Floor(rank);
            int hi = (int)Math.Ceiling(rank);
            if (lo == hi) return sorted[lo];

            double frac = rank - lo;
            return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
        }
    }
}
