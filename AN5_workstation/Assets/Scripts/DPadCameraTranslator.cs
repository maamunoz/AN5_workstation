using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public class DPadCameraTranslator : MonoBehaviour
{
    [Header("Settings")]
    public float panSpeed = 2f;
    public float mouseSensitivity = 0.01f;

    DPadButton _btnUp, _btnDown, _btnLeft, _btnRight;
    bool _wasAnyPressed;

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

        bool anyPressed = kUp || kDown || kLeft || kRight || mouse
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
    }
}
