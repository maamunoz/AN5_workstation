/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public abstract class UnitySubscriber<T> : MonoBehaviour where T: Message
    {
        public string Topic;
        public float TimeStep;

        private RosConnector rosConnector;

        protected virtual void Start()
        {
            rosConnector = GetComponent<RosConnector>();
            new Thread(Subscribe).Start();
        }

        private void Subscribe()
        {
            // FIX: esto esperaba la conexión con un timeout fijo de 1s y, si se
            // vencía, igual llamaba a rosConnector.RosSocket.Subscribe() -- en el
            // flujo normal (la escena carga, recién después el usuario escribe la
            // IP y conecta) ese segundo casi siempre se agota con RosSocket
            // todavía en null, así que esto tiraba una NullReferenceException en
            // este hilo de fondo -- silenciosa, no aparece en la consola de la
            // app -- y la suscripción real nunca llegaba a ocurrir. Los watchers
            // de reconexión de las subclases (JointPositionSubscriber,
            // CartesianPositionSubscriber, InverseKinematicsSubscriber,
            // RobotMotionDoneSubscriber, SetpointCartesianPositionSubscriber)
            // asumen que esta primera suscripción sí ocurrió y por diseño solo
            // actúan ante cambios posteriores de RosSocket, así que el robot se
            // quedaba sin actualizar posiciones hasta una SEGUNDA reconexión.
            // Se espera indefinidamente a la conexión real, igual que ya hace
            // Ros2CommandSender.WaitForConnectionAndAdvertise() del lado de
            // publicación, y se valida RosSocket antes de usarlo por si acaso.
            rosConnector.IsConnected.WaitOne();

            if (rosConnector.RosSocket == null)
            {
                Debug.LogWarning("Failed to subscribe: RosSocket is null after connecting");
                return;
            }

            rosConnector.RosSocket.Subscribe<T>(Topic, ReceiveMessage, (int)(TimeStep * 1000)); // the rate(in ms in between messages) at which to throttle the topics
        }

        protected abstract void ReceiveMessage(T message);

    }
}