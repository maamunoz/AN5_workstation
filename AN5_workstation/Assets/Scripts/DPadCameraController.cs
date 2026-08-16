using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public class DPadCameraController : MonoBehaviour
{
    [Header("Orbit Target")]
    public Transform target;
    public float orbitSpeed = 90f;
    public float mouseSensitivity = 3f;
    public float scrollSensitivity = 0.5f;
    public float minRadius = 0.2f;
    public float maxRadius = 5f;
    public float minPitch = -80f;
    public float maxPitch = 80f;

    DPadButton _btnUp, _btnDown, _btnLeft, _btnRight;
    Slider _zoomSlider;
    float _yaw, _pitch, _radius;
    bool _wasAnyPressed;

    void Start()
    {
        if (target == null) target = GameObject.Find("fr5v6")?.transform;

        // Search by name so the path survives reparenting
        GameObject dpad = null;
        foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
            if (t.name == "DPad_Orbit" && t.gameObject.activeInHierarchy) { dpad = t.gameObject; break; }
        if (dpad == null)
        {
            // No es un fallo: QuestSceneBuilder.DisableDesktopOnly apaga DPad_Orbit
            // entero en la Quest (este control es solo para orbitar la cámara de
            // escritorio), así que faltar ahí es el caso normal y no un error real.
            Debug.Log("[DPad] DPad_Orbit not found (normal en la Quest, ver DisableDesktopOnly).");
            return;
        }

        foreach (Transform child in dpad.transform)
        {
            var txt = child.GetComponentInChildren<Text>();
            if (txt == null) continue;
            var db = child.GetComponent<DPadButton>() ?? child.gameObject.AddComponent<DPadButton>();
            switch (txt.text.Trim())
            {
                case "W": _btnUp    = db; break;
                case "S": _btnDown  = db; break;
                case "A": _btnLeft  = db; break;
                case "D": _btnRight = db; break;
            }
        }

        // Find zoom slider
        foreach (var t in Resources.FindObjectsOfTypeAll<Transform>())
            if (t.name == "ZoomSlider" && t.gameObject.activeInHierarchy)
                { _zoomSlider = t.GetComponent<Slider>(); break; }

        if (_zoomSlider != null)
        {
            _zoomSlider.minValue = minRadius;
            _zoomSlider.maxValue = maxRadius;
            _zoomSlider.onValueChanged.AddListener(OnSliderChanged);
        }

        SyncOrbit();

        if (_zoomSlider != null)
            _zoomSlider.SetValueWithoutNotify(_radius);
    }

    void OnSliderChanged(float value)
    {
        SyncOrbit();
        _radius = Mathf.Clamp(value, minRadius, maxRadius);
        ApplyOrbit();
    }

    // Re-reads current camera position into orbit angles.
    void SyncOrbit()
    {
        Vector3 pivot  = target != null ? target.position : Vector3.zero;
        Vector3 offset = transform.position - pivot;
        _radius = Mathf.Max(offset.magnitude, 0.01f);
        _pitch  = Mathf.Asin(Mathf.Clamp(offset.normalized.y, -1f, 1f)) * Mathf.Rad2Deg;
        _yaw    = Mathf.Atan2(offset.normalized.x, offset.normalized.z)  * Mathf.Rad2Deg;
    }

    void ApplyOrbit()
    {
        _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);

        float pr = _pitch * Mathf.Deg2Rad;
        float yr = _yaw   * Mathf.Deg2Rad;
        Vector3 dir = new Vector3(
            Mathf.Sin(yr) * Mathf.Cos(pr),
            Mathf.Sin(pr),
            Mathf.Cos(yr) * Mathf.Cos(pr));

        Vector3 pivot = target != null ? target.position : Vector3.zero;
        transform.position = pivot + dir * _radius;
        transform.LookAt(pivot, Vector3.up);
    }

    void Update()
    {
        // Este controlador es solo para orbitar la cámara de escritorio con
        // teclado/ratón; en la Quest no hay ninguno de los dos conectados y
        // Keyboard.current/Mouse.current vienen null -- así que todo esto queda en
        // no-op sin más, que es justo lo que hace falta ahí. La UnityEngine.Input
        // vieja no vale para eso: con el Input System puesto como único manejador
        // activo lanza InvalidOperationException en cuanto se la toca, en escritorio
        // y en la Quest por igual, y como esto corre en Update() inundaba SecLog con
        // la misma excepción cuadro a cuadro.
        var keyboard = Keyboard.current;
        var mouseDevice = Mouse.current;

        bool isTyping = EventSystem.current != null
                     && EventSystem.current.currentSelectedGameObject != null
                     && EventSystem.current.currentSelectedGameObject.GetComponent<InputField>() != null;

        bool kUp    = !isTyping && keyboard != null && keyboard.wKey.isPressed;
        bool kDown  = !isTyping && keyboard != null && keyboard.sKey.isPressed;
        bool kLeft  = !isTyping && keyboard != null && keyboard.aKey.isPressed;
        bool kRight = !isTyping && keyboard != null && keyboard.dKey.isPressed;
        bool mouse  = mouseDevice != null && mouseDevice.leftButton.isPressed && !EventSystem.current.IsPointerOverGameObject();

        bool anyPressed = kUp || kDown || kLeft || kRight || mouse
                       || (_btnUp    != null && _btnUp.isPressed)
                       || (_btnDown  != null && _btnDown.isPressed)
                       || (_btnLeft  != null && _btnLeft.isPressed)
                       || (_btnRight != null && _btnRight.isPressed);

        if (anyPressed && !_wasAnyPressed)
            SyncOrbit();

        // Mouse.current.scroll viene en "notches * 120" (delta crudo de rueda en
        // Windows), no en la escala ~0.1/notch de la vieja Input.GetAxis; se divide
        // para que scrollSensitivity siga sintiéndose igual que antes.
        float scroll = mouseDevice != null ? mouseDevice.scroll.ReadValue().y / 1200f : 0f;
        if (scroll != 0f)
        {
            if (!anyPressed) SyncOrbit();
            _radius = Mathf.Max(_radius - scroll * scrollSensitivity * _radius, minRadius);
            _radius = Mathf.Min(_radius, maxRadius);
            anyPressed = true;
        }

        _wasAnyPressed = anyPressed;
        if (!anyPressed) return;

        float dt = Time.deltaTime * orbitSpeed;
        if (kUp    || (_btnUp    != null && _btnUp.isPressed))    _pitch += dt;
        if (kDown  || (_btnDown  != null && _btnDown.isPressed))  _pitch -= dt;
        if (kLeft  || (_btnLeft  != null && _btnLeft.isPressed))  _yaw   += dt;
        if (kRight || (_btnRight != null && _btnRight.isPressed)) _yaw   -= dt;

        if (mouse)
        {
            // Mouse.current.delta es el desplazamiento en px crudos del cuadro, sin la
            // escala ~0.1 que traía la vieja Input.GetAxis -- mouseSensitivity puede
            // necesitar retocarse, pero mouse ya viene en null-checked por keyboard/
            // mouseDevice arriba, así que esto no revienta sin ratón.
            var delta = mouseDevice.delta.ReadValue();
            _pitch -= delta.y * mouseSensitivity;
            _yaw   += delta.x * mouseSensitivity;
        }

        ApplyOrbit();

        if (_zoomSlider != null)
            _zoomSlider.SetValueWithoutNotify(_radius);
    }
}
