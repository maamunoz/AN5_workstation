using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public class DPadCameraTranslator : MonoBehaviour
{
    [Header("Settings")]
    public float panSpeed = 2f;
    public float mouseSensitivity = 0.01f;

    [Header("Touch (iPad) - arrastrar con dos dedos")]
    public float touchPanSensitivity = 0.004f;

    DPadButton _btnUp, _btnDown, _btnLeft, _btnRight;
    bool _wasAnyPressed;

    // Estado del arrastre de dos dedos entre cuadros -- ver TouchGestureUtil.
    Vector2 _prevPanMid;
    bool _hadTouchPan;

    void Start()
    {
        // Search by name so the path survives reparenting
        GameObject dpad = null;
        foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
            if (t.name == "DPad_Translate" && t.gameObject.activeInHierarchy) { dpad = t.gameObject; break; }
        if (dpad == null)
        {
            // No es un fallo: QuestSceneBuilder.DisableDesktopOnly apaga DPad_Translate
            // entero en la Quest (este control es solo para trasladar la cámara de
            // escritorio), así que faltar ahí es el caso normal y no un error real.
            Debug.Log("[DPadTranslate] DPad_Translate not found (normal en la Quest, ver DisableDesktopOnly).");
            return;
        }

        foreach (Transform child in dpad.transform)
        {
            var txt = child.GetComponentInChildren<Text>();
            if (txt == null) continue;
            var db = child.GetComponent<DPadButton>() ?? child.gameObject.AddComponent<DPadButton>();
            switch (txt.text.Trim())
            {
                case "↑": _btnUp    = db; break;
                case "↓": _btnDown  = db; break;
                case "←": _btnLeft  = db; break;
                case "→": _btnRight = db; break;
            }
        }
    }

    void Update()
    {
        // Igual que DPadCameraController: solo escritorio, y Keyboard.current/
        // Mouse.current vienen null en la Quest sin teclado/ratón -- todo esto queda
        // en no-op ahí. La UnityEngine.Input vieja lanzaba InvalidOperationException
        // en cuanto se la tocaba, cuadro a cuadro, con el Input System como único
        // manejador activo.
        var keyboard = Keyboard.current;
        var mouseDevice = Mouse.current;

        bool isTyping = EventSystem.current != null
                     && EventSystem.current.currentSelectedGameObject != null
                     && EventSystem.current.currentSelectedGameObject.GetComponent<InputField>() != null;

        bool kUp    = !isTyping && keyboard != null && keyboard.upArrowKey.isPressed;
        bool kDown  = !isTyping && keyboard != null && keyboard.downArrowKey.isPressed;
        bool kLeft  = !isTyping && keyboard != null && keyboard.leftArrowKey.isPressed;
        bool kRight = !isTyping && keyboard != null && keyboard.rightArrowKey.isPressed;
        bool mouse  = mouseDevice != null && mouseDevice.rightButton.isPressed && !EventSystem.current.IsPointerOverGameObject();

        // Dos dedos arrastrando (iPad): mismo desplazamiento que ya hace el arrastre
        // con el botón derecho del mouse, pero a partir del punto medio entre los dos
        // dedos en vez del delta del mouse.
        bool touchPan = TouchGestureUtil.TryGetTwoFingerGesture(out var pan0, out var pan1);
        Vector2 touchDelta = Vector2.zero;
        if (touchPan)
        {
            Vector2 mid = (pan0 + pan1) * 0.5f;
            if (_hadTouchPan)
                touchDelta = mid - _prevPanMid;
            _prevPanMid = mid;
            _hadTouchPan = true;
        }
        else
        {
            _hadTouchPan = false;
        }

        bool anyPressed = kUp || kDown || kLeft || kRight || mouse || touchPan
                       || (_btnUp    != null && _btnUp.isPressed)
                       || (_btnDown  != null && _btnDown.isPressed)
                       || (_btnLeft  != null && _btnLeft.isPressed)
                       || (_btnRight != null && _btnRight.isPressed);

        _wasAnyPressed = anyPressed;
        if (!anyPressed) return;

        float dt = Time.deltaTime * panSpeed;
        Vector3 move = Vector3.zero;

        if (kUp    || (_btnUp    != null && _btnUp.isPressed))    move += transform.up;
        if (kDown  || (_btnDown  != null && _btnDown.isPressed))  move -= transform.up;
        if (kLeft  || (_btnLeft  != null && _btnLeft.isPressed))  move -= transform.right;
        if (kRight || (_btnRight != null && _btnRight.isPressed)) move += transform.right;

        if (move != Vector3.zero)
            transform.position += move.normalized * dt;

        if (mouse)
        {
            // Mouse.current.delta es px crudos del cuadro, no la escala ~0.1 de la
            // vieja Input.GetAxis -- mouseSensitivity puede necesitar retocarse.
            var delta = mouseDevice.delta.ReadValue();
            transform.position -= transform.right * (delta.x * mouseSensitivity);
            transform.position -= transform.up    * (delta.y * mouseSensitivity);
        }

        if (touchPan && touchDelta != Vector2.zero)
        {
            transform.position -= transform.right * (touchDelta.x * touchPanSensitivity);
            transform.position -= transform.up    * (touchDelta.y * touchPanSensitivity);
        }
    }
}
