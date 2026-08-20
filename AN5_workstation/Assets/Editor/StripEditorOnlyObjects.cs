using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AN5.EditorTools
{
    /// Saca de la escena, ANTES de que se empaquete en el build del jugador, cualquier
    /// GameObject marcado con EditorOnlyGameObject.
    ///
    /// No alcanza con que esos objetos se autodestruyan en su propio Awake() fuera del
    /// Editor (ver EditorOnlyGameObject) -- confirmado en la práctica con el "XR
    /// Interaction Simulator" que agrega QuestSceneBuilder: Unity llama Awake(),
    /// OnEnable() y el primer Update() de TODOS los componentes que ya traía el prefab
    /// (XRInteractionSimulator entre ellos) en el mismo cuadro en que se instancia, y
    /// recién después el Awake() de EditorOnlyGameObject, que llama Destroy() -- que no
    /// es inmediato, se aplica a fin de cuadro. Ese primer cuadro le alcanza a
    /// XRInteractionSimulator.Update() para empujar un HMD y unos mandos simulados al
    /// Input System (ApplyHMDState/ApplyControllerState, sin ningún #if de por medio en
    /// ENABLE_VR), lo bastante para que el TrackedPoseDriver de la cámara del XR Origin
    /// quede enganchado al dispositivo simulado en vez del real -- consistente con el
    /// síntoma persistente ("pegado al suelo") y no un parpadeo de un cuadro.
    ///
    /// Sacarlo de la escena antes de compilar el player elimina esa carrera del todo:
    /// en el build no existe ni un cuadro.
    public class StripEditorOnlyObjects : IProcessSceneWithReport
    {
        public int callbackOrder => 0;

        public void OnProcessScene(Scene scene, BuildReport report)
        {
            int removed = 0;
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var marker in root.GetComponentsInChildren<EditorOnlyGameObject>(true))
                {
                    Object.DestroyImmediate(marker.gameObject);
                    removed++;
                }
            }

            if (removed > 0)
                Debug.Log($"[StripEditorOnlyObjects] {removed} objeto(s) solo-Editor sacado(s) de " +
                          $"'{scene.name}' antes de empaquetar el build.");
        }
    }
}
