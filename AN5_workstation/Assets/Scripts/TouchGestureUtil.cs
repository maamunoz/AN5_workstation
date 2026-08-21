using UnityEngine;
using UnityEngine.EventSystems;

/// Pequeño helper compartido por DPadCameraController (un dedo para orbitar,
/// pellizco para zoom) y DPadCameraTranslator (arrastre de dos dedos para
/// desplazar), para que ambos lean los toques de la misma forma y con la
/// misma regla de exclusión sobre UI que ya usa el arrastre con mouse
/// (!EventSystem.current.IsPointerOverGameObject()).
///
/// Usa la API vieja UnityEngine.Input (touchCount/GetTouch) a propósito, no
/// UnityEngine.InputSystem.Touchscreen: el EventSystem de esta escena corre
/// StandaloneInputModule (no InputSystemUIInputModule), que por dentro sigue
/// el pipeline de Input viejo para touch -- e IsPointerOverGameObject(fingerId)
/// solo empareja con Input.GetTouch(i).fingerId, no con ningún id del Input
/// System nuevo. DPadCameraController/Translator usan Keyboard.current/
/// Mouse.current en vez de Input.* específicamente porque este proyecto tuvo
/// en algún momento el Input System puesto como ÚNICO manejador (ahí Input.*
/// tira InvalidOperationException); hoy activeInputHandler está en "Both"
/// (ProjectSettings), que sí deja usar Input.* con seguridad -- y es la API
/// que la UI ya está usando de hecho para los toques.
public static class TouchGestureUtil
{
    /// True mientras haya exactamente un dedo apoyado y no esté sobre un
    /// elemento de UI (un botón del DPad, un panel, etc.). Devuelve su
    /// posición de pantalla actual. Se exige touchCount == 1 (no <= 1) para
    /// que este gesto y TryGetTwoFingerGesture nunca se disparen a la vez
    /// dentro del mismo frame.
    public static bool TryGetOneFingerGesture(out Vector2 point)
    {
        point = default;

        if (Input.touchCount != 1)
            return false;

        Touch t0 = Input.GetTouch(0);

        if (t0.phase == TouchPhase.Ended || t0.phase == TouchPhase.Canceled) return false;

        if (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject(t0.fingerId))
            return false;

        point = t0.position;
        return true;
    }

    /// True mientras haya exactamente dos dedos apoyados y ninguno esté sobre
    /// un elemento de UI (un botón del DPad, un panel, etc.). Devuelve sus
    /// posiciones de pantalla actuales.
    public static bool TryGetTwoFingerGesture(out Vector2 point0, out Vector2 point1)
    {
        point0 = point1 = default;

        if (Input.touchCount != 2)
            return false;

        Touch t0 = Input.GetTouch(0);
        Touch t1 = Input.GetTouch(1);

        if (t0.phase == TouchPhase.Ended || t0.phase == TouchPhase.Canceled) return false;
        if (t1.phase == TouchPhase.Ended || t1.phase == TouchPhase.Canceled) return false;

        if (EventSystem.current != null &&
            (EventSystem.current.IsPointerOverGameObject(t0.fingerId) ||
             EventSystem.current.IsPointerOverGameObject(t1.fingerId)))
            return false;

        point0 = t0.position;
        point1 = t1.position;
        return true;
    }
}
