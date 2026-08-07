using System;
using System.Diagnostics;

namespace AN5.Measurement
{
    /// Reloj de pared de alta resolución, anclado al reloj del sistema.
    ///
    /// POR QUÉ EXISTE
    ///
    /// El plan de mediciones adoptó la estrategia A+C: el tiempo de ida y vuelta es
    /// la medida principal en las cuatro configuraciones, y además se reporta latencia
    /// unidireccional en C3/C4, donde Unity y el middleware comparten equipo y por lo
    /// tanto comparten reloj. Esa segunda parte exige comparar una marca tomada en
    /// Unity con una tomada por measurement_probe (get_clock().now() de ROS 2, que es
    /// el reloj de pared del sistema).
    ///
    /// El problema: DateTime.UtcNow NO sirve para eso. En Windows su granularidad es
    /// del orden de 15 ms, así que un tramo real de 2 ms es sencillamente inmedible —
    /// se leería como 0 o como 15, sin señal de que la cifra no significa nada.
    /// Stopwatch sí tiene resolución de microsegundos, pero su origen es arbitrario:
    /// no se puede comparar contra una marca de otro proceso.
    ///
    /// CÓMO SE RESUELVE
    ///
    /// Se combinan los dos: se ancla el contador de alta resolución de Stopwatch a un
    /// instante conocido del reloj de pared, y de ahí en más se lee el tiempo de pared
    /// como "ancla + lo que avanzó el Stopwatch".
    ///
    /// El ancla se toma esperando activamente al SALTO de tick de UtcNow en vez de
    /// leerlo una vez. Leerlo una vez daría un valor con hasta 15 ms de error —
    /// justamente el error que se quiere evitar, y encima constante, así que sesgaría
    /// todas las mediciones de la sesión en la misma dirección sin cancelarse nunca.
    /// Esperando el salto, el ancla queda fijada al instante del cambio con la
    /// precisión de una iteración del bucle (microsegundos).
    ///
    /// LÍMITES QUE HAY QUE DECLARAR EN EL ARTÍCULO
    ///
    /// - Esto NO sincroniza relojes entre equipos. En C1/C2 (middleware remoto) los
    ///   tramos unidireccionales siguen sin ser interpretables, y el arnés los marca
    ///   como tales. Solo el tiempo de ida y vuelta es válido ahí.
    /// - Stopwatch y el reloj de pared derivan entre sí (típicamente unas pocas partes
    ///   por millón). En ventanas de 60 s eso son decenas de microsegundos: irrelevante
    ///   frente a las latencias medidas, pero por eso conviene reanclar entre pruebas
    ///   largas, que es lo que hace Reanchor().
    public static class HighResolutionClock
    {
        private static readonly DateTime UnixEpoch =
            new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        private static long _anchorUnixNanos;
        private static long _anchorTimestamp;
        private static long _anchorSpreadNanos;
        private static bool _initialized;

        /// Error estimado del anclaje, en nanosegundos: cuánto tardó la iteración del
        /// bucle que detectó el salto de tick. Se registra en environment.csv para que
        /// la calidad del reloj quede declarada junto a los resultados, en vez de ser
        /// un supuesto tácito.
        public static double AnchorSpreadMicroseconds
        {
            get { EnsureInitialized(); return _anchorSpreadNanos / 1000.0; }
        }

        /// Resolución nominal del contador subyacente, en nanosegundos por tick.
        public static double TimestampResolutionNanos
        {
            get { return 1e9 / Stopwatch.Frequency; }
        }

        public static bool IsHighResolution { get { return Stopwatch.IsHighResolution; } }

        private static void EnsureInitialized()
        {
            if (!_initialized) Reanchor();
        }

        /// Vuelve a anclar el reloj al instante actual. Conviene llamarlo al comenzar
        /// cada prueba larga para que la deriva entre Stopwatch y el reloj de pared no
        /// se acumule a lo largo de una sesión de varias horas.
        public static void Reanchor()
        {
            // Esperar al salto de tick de UtcNow. Sin esto, el ancla arrastraría el
            // error de granularidad completo (~15 ms en Windows) como sesgo constante.
            DateTime before = DateTime.UtcNow;
            DateTime after;
            long tsBefore = Stopwatch.GetTimestamp();
            long tsAfter;

            do
            {
                tsBefore = Stopwatch.GetTimestamp();
                after = DateTime.UtcNow;
                tsAfter = Stopwatch.GetTimestamp();
            }
            while (after == before);

            _anchorUnixNanos = (after - UnixEpoch).Ticks * 100L;
            // Se toma el punto medio de la iteración que detectó el salto: el instante
            // real del cambio de tick está en algún lugar de ese intervalo.
            _anchorTimestamp = tsBefore + (tsAfter - tsBefore) / 2;
            _anchorSpreadNanos = (long)((tsAfter - tsBefore) * TimestampResolutionNanos);
            _initialized = true;
        }

        /// Tiempo de pared actual en nanosegundos desde la época Unix, con resolución
        /// de microsegundos. Directamente comparable con las marcas que publica
        /// measurement_probe (get_clock().now().nanoseconds) SIEMPRE Y CUANDO ambos
        /// procesos corran en el mismo equipo.
        public static long UnixNanos()
        {
            EnsureInitialized();
            long elapsedTicks = Stopwatch.GetTimestamp() - _anchorTimestamp;
            return _anchorUnixNanos + (long)(elapsedTicks * TimestampResolutionNanos);
        }
    }
}
