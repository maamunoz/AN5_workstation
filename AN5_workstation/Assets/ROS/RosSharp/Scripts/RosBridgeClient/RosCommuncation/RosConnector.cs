/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing,
software distributed under the License is distributed on an "AS IS",
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Threading;
using RosSharp.RosBridgeClient.Protocols;
using UnityEngine;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        [Header("Connection Settings")]
        public string RosBridgeServerUrl = "ws://localhost:9090";
        public Protocol protocol;
        public RosSocket.SerializerEnum Serializer;

        [Header("Timeouts")]
        public int SecondsTimeout = 20;
        public int ReconnectDelaySeconds = 5;

        [Header("UI Components (Opcional)")]
        public Text statusText;

        [Header("UI Button (Opcional)")]
       
        public Button reconnectButton;

        // FIX: publicación segura entre hilos. Esta referencia se ESCRIBE desde hilos
        // de fondo (ConnectAndWait/ConnectOnce hacen "RosSocket = ConnectToRos(...)")
        // y se LEE desde el hilo principal (los WatchForReconnect de los subscribers,
        // Ros2CommandSender) y desde el hilo de UnitySubscriber<T>.Subscribe(). Como
        // auto-property normal no había ninguna barrera de memoria en medio, y C# --a
        // diferencia de los campos final de Java-- NO garantiza que un objeto se
        // publique completamente construido: el procesador puede hacer visible la
        // escritura de la REFERENCIA antes que las escrituras de los campos internos
        // del RosSocket (protocol, Serializer, los diccionarios de Subscribers). En
        // x86 eso casi no se nota porque su modelo de memoria es fuerte y no reordena
        // stores, pero el iPad es ARM, con modelo débil, y ahí sí puede pasar que el
        // lector vea RosSocket != null y un campo interno todavía en null. volatile da
        // semántica release en la escritura y acquire en la lectura: quien lea la
        // referencia nueva ve sí o sí todo lo que se escribió antes de publicarla.
        //
        // OJO, para que no se lea de más: esto es un endurecimiento preventivo, NO la
        // causa del "Fallo al re-suscribirse: Object reference not set to an instance
        // of an object" que se vio en el log del iPad. Ese resultó ser otra cosa: el
        // managed stripping de IL2CPP borraba el const RosMessageName, que RosSharp
        // busca por reflexión en Communicator.GetRosName<T>() -- ver Assets/link.xml.
        // Se deja igual porque la publicación insegura era un problema real y latente
        // aparte, aunque no fuera el que se estaba persiguiendo.
        private volatile RosSocket _rosSocket;
        public RosSocket RosSocket
        {
            get { return _rosSocket; }
            private set { _rosSocket = value; }
        }

        public ManualResetEvent IsConnected { get; private set; }

        public bool IsOnline
        {
            get { return IsConnected != null && IsConnected.WaitOne(0); }
        }

        private bool isReconnecting = false;
        private Thread connectionThread;

        // FIX: ReconnectNow() cierra el RosSocket viejo con RosSocket.Close(), y ese
        // Close() dispara OnClosed() -- que, sin este flag, no tiene forma de saber que
        // el cierre fue intencional (pedido por el propio ReconnectNow(), que ya está
        // arrancando su propio intento de conexión) en vez de una caída real de red.
        // Sin distinguirlo, OnClosed() arrancaba UN SEGUNDO hilo de auto-reconexión
        // (ConnectAndWait) por cada click en "Reconnect", que corría en paralelo con el
        // ConnectOnce() que ReconnectNow() ya había lanzado -- los dos hilos abrían
        // sockets distintos y competían por escribir la misma propiedad RosSocket. Con 4
        // clicks seguidos esto medía 8 conexiones reales al servidor en vez de 4, y cuál
        // de los dos hilos "ganaba" (y por lo tanto qué socket quedaba con la suscripción
        // viva a current_joint_position) dependía del timing de red -- en loopback local
        // casi siempre se resolvía solo, pero por WiFi hacia el robot real es la clase de
        // carrera que deja la posición congelada de forma intermitente sin ningún error
        // visible. Se cuenta (no un simple bool) porque varios Reconnect intencionales
        // pueden solaparse antes de que llegue el primer OnClosed().
        private int _intentionalCloseCount = 0;

        // Cancelación cooperativa del hilo de auto-reconexión: Thread.Abort() (más
        // abajo) no existe en IL2CPP -- el backend de scripting obligatorio para
        // Android/Quest -- así que ahí tirar Abort() lanzaba
        // PlatformNotSupportedException apenas se llamaba. La excepción abortaba
        // ReconnectNow() a mitad de camino: el socket viejo a veces ni se cerraba, y
        // el hilo de auto-reconexión seguía vivo de fondo y podía pisar el RosSocket
        // nuevo con uno propio al despertar de su Sleep/WaitOne -- el robot dejaba de
        // actualizar posiciones tras cambiar la IP, sin ninguna forma de recuperarse
        // sin reiniciar la app. En vez de abortar, cada hilo que arranca revisa esta
        // marca de generación antes de cada vuelta del bucle y de tocar RosSocket; si
        // ya no es la generación vigente, se retira solo.
        private volatile int _connectionGeneration = 0;

        public virtual void Awake()
        {
            IsConnected = new ManualResetEvent(false);

            // Iniciamos la conexión en un hilo separado (auto reconexión con bucle y retraso)
            connectionThread = new Thread(ConnectAndWait);
            connectionThread.Start();
        }

        private void Start()
        {
        
            if (reconnectButton != null)
                reconnectButton.onClick.AddListener(ReconnectNow);
        }

        private void Update()
        {
          
            if (statusText != null)
            {
                if (IsOnline)
                {
                    statusText.text = "ONLINE";
                    statusText.color = Color.green;
                }
                else
                {
                    statusText.text = "OFFLINE";
                    statusText.color = Color.red;
                }
            }
        }

        // Bucle para auto-reconexión con un retardo (se inicia en Awake y en OnClosed)
        private void ConnectAndWait()
        {
            int myGeneration = _connectionGeneration;

            while (myGeneration == _connectionGeneration)
            {
                RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

                if (!IsConnected.WaitOne(SecondsTimeout * 1000))
                {
                    Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);
                }
                else
                {
                    // Conexión exitosa, salir del bucle
                    break;
                }

                // Alguien pidió una reconexión mientras este hilo esperaba el timeout
                // de conexión: se retira sin dormir ni volver a tocar RosSocket.
                if (myGeneration != _connectionGeneration) return;

                Debug.Log("Retrying connection in " + ReconnectDelaySeconds + " seconds...");
                Thread.Sleep(ReconnectDelaySeconds * 1000);
            }
        }

        // Método estático para crear el RosSocket y enlazar OnConnected y OnClosed
        public static RosSocket ConnectToRos(
            Protocol protocolType,
            string serverUrl,
            EventHandler onConnected = null,
            EventHandler onClosed = null,
            RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.Microsoft)
        {
            IProtocol protocol = ProtocolInitializer.GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;
            return new RosSocket(protocol, serializer);
        }

        // Método para reconexión inmediata, sin esperar ReconnectDelaySeconds.
        // No hace early-return si IsOnline: los llamadores (ModeToggleController,
        // SecConfigController) ya lo invocan solo cuando RosBridgeServerUrl
        // cambió, así que "ya conectado" acá significaba "conectado al endpoint
        // VIEJO" -- saltear el reconnect dejaba la app pegada al host anterior
        // aunque el campo de IP mostrara el nuevo.
        public void ReconnectNow()
        {
            Debug.Log("Manual reconnect now...");

            // Cancela cooperativamente el hilo que pudiera estar en medio del bucle de
            // auto reconexión (ver el comentario de _connectionGeneration): sin
            // Thread.Abort(), que no existe en IL2CPP/Quest.
            _connectionGeneration++;

            // Marca este cierre como intencional ANTES de pedirlo, para que OnClosed()
            // (que puede disparar en este mismo hilo o en uno de red, según el
            // protocolo) no arranque un segundo hilo de auto-reconexión a competir con
            // ConnectOnce() más abajo. Ver el comentario de _intentionalCloseCount.
            Interlocked.Increment(ref _intentionalCloseCount);

            // Cerramos el socket anterior si seguía abierto
            if (RosSocket != null)
                RosSocket.Close();

            // Reset para poder volver a esperar la señal
            IsConnected.Reset();
            isReconnecting = false;

            // Iniciamos un hilo nuevo que haga un único intento de conexión
            connectionThread = new Thread(ConnectOnce);
            connectionThread.Start();
        }

        // Únicamente intenta conectar una vez, sin bucles de reintento
        private void ConnectOnce()
        {
            RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

            // Esperamos a que se establezca la conexión (o agotar el timeout)
            if (!IsConnected.WaitOne(SecondsTimeout * 1000))
            {
                Debug.LogWarning("Failed to connect (manual attempt) to RosBridge at: " + RosBridgeServerUrl);
            }
            else
            {
                Debug.Log("Connected to RosBridge (manual attempt): " + RosBridgeServerUrl);
            }
        }

        // Evento que se llama cuando se cierra la conexión
        private void OnClosed(object sender, EventArgs e)
        {
            IsConnected.Reset();
            Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);

            // FIX: si este cierre lo pidió ReconnectNow() (o OnApplicationQuit()), quien
            // lo pidió ya se está encargando de reconectar (o la app se está cerrando) --
            // arrancar OTRO hilo de auto-reconexión aquí solo compite por RosSocket con
            // el que ya está en marcha. Ver el comentario de _intentionalCloseCount.
            if (_intentionalCloseCount > 0)
            {
                Interlocked.Decrement(ref _intentionalCloseCount);
                return;
            }

            // Lógica de auto reconexión (con retardo) -- solo ante una caída real,
            // no pedida por el usuario ni por el cierre de la app.
            if (!isReconnecting)
            {
                isReconnecting = true;
                _connectionGeneration++;
                connectionThread = new Thread(ConnectAndWait);
                connectionThread.Start();
            }
        }

        // Evento que se llama cuando la conexión se realiza con éxito
        private void OnConnected(object sender, EventArgs e)
        {
            IsConnected.Set();
            isReconnecting = false;
            Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
        }

        private void OnApplicationQuit()
        {
            // La app se está cerrando igual, así que no hace falta esperar a que el
            // hilo de fondo note la nueva generación y se retire solo -- alcanza con
            // que no siga reconectando ni tocando el socket cerrado.
            _connectionGeneration++;
            Interlocked.Increment(ref _intentionalCloseCount);

            if (RosSocket != null)
                RosSocket.Close();
        }
    }
}
