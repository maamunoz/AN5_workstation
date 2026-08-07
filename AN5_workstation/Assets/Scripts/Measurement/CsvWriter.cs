using System;
using System.Globalization;
using System.IO;
using System.Text;

namespace AN5.Measurement
{
    /// Escritor CSV para los archivos de medición.
    ///
    /// Deliberadamente distinto de los GuardarEnTxt() que ya existen en el proyecto
    /// (Record_Panel.cs:520, ControlArticular.cs:508, CartesianStateWriterNew.cs:563):
    /// aquellos abren el StreamWriter en modo truncado y escriben todo de una vez al
    /// final, que para datos experimentales es la peor opción posible — si la corrida
    /// se corta a los 55 s de una ventana de 60 s, se pierde todo en vez de los
    /// últimos 5 s. Acá se escribe en modo append y se vuelca a disco fila por fila.
    ///
    /// Todo el formateo numérico usa InvariantCulture: en un equipo con configuración
    /// regional que usa coma decimal, un CSV separado por comas quedaría corrupto de
    /// forma silenciosa y no se notaría hasta intentar procesarlo.
    public sealed class CsvWriter : IDisposable
    {
        private StreamWriter _writer;

        public string Path { get; }
        public int RowsWritten { get; private set; }

        public CsvWriter(string path, params string[] headerColumns)
        {
            Path = path;
            string dir = System.IO.Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            _writer = new StreamWriter(path, append: true, encoding: new UTF8Encoding(false));

            if (headerColumns != null && headerColumns.Length > 0)
                _writer.WriteLine(string.Join(",", headerColumns));
            _writer.Flush();
        }

        /// Escribe una fila. Los valores se formatean por tipo:
        /// double/float con "R" (ida y vuelta exacta, sin perder precisión en los
        /// tiempos), enteros tal cual, bool como true/false, null como cadena vacía.
        public void WriteRow(params object[] values)
        {
            if (_writer == null) return;

            var sb = new StringBuilder();
            for (int i = 0; i < values.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append(Format(values[i]));
            }
            _writer.WriteLine(sb.ToString());
            // Volcado por fila: una corrida interrumpida conserva todo lo medido
            // hasta ese instante.
            _writer.Flush();
            RowsWritten++;
        }

        private static string Format(object v)
        {
            switch (v)
            {
                case null:    return "";
                case double d: return d.ToString("R", CultureInfo.InvariantCulture);
                case float f:  return f.ToString("R", CultureInfo.InvariantCulture);
                case bool b:   return b ? "true" : "false";
                case string s: return Escape(s);
                default:       return Escape(Convert.ToString(v, CultureInfo.InvariantCulture));
            }
        }

        private static string Escape(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            if (s.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0) return s;
            return "\"" + s.Replace("\"", "\"\"") + "\"";
        }

        public void Dispose()
        {
            if (_writer == null) return;
            _writer.Flush();
            _writer.Dispose();
            _writer = null;
        }
    }
}
