using UnityEngine;

/// Borra el GameObject al arrancar fuera del Editor.
///
/// Se usa para dejar en la escena piezas que solo tienen sentido en modo Play dentro del
/// Editor —como el XR Interaction Simulator, que suplanta el visor y los mandos con
/// teclado y ratón— sin que viajen al build. `UNITY_EDITOR` se decide en tiempo de
/// compilación por plataforma: en un build de Android el Destroy() ni siquiera se
/// compila, así que el objeto se queda quieto ahí; en el Editor sí se compila pero nunca
/// se ejecuta, porque UNITY_EDITOR está definido.
///
/// Que el simulador viajara al build es justo lo que dejaba las gafas con la cabeza
/// "pegada" a la escena y los mandos sin representar: sus dispositivos virtuales de XR
/// competían con los reales del visor por los mismos bindings (<XRHMD>, <XRController>),
/// y encima no reaccionaban a la cabeza real ni a los Touch de verdad.
public class EditorOnlyGameObject : MonoBehaviour
{
    void Awake()
    {
#if !UNITY_EDITOR
        Destroy(gameObject);
#endif
    }
}
