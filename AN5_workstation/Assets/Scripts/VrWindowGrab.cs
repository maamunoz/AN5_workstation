using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// Deja mover una ventana World Space con los mandos del visor.
///
/// El sitio de cada ventana de Quest_test_arnes lo fija el constructor de la escena
/// (QuestSceneBuilder) y es el mismo para todo el mundo. Para lo que solo se mira eso
/// vale, pero SecTraj —cargar el .csv, ejecutar, pausar, parar— se maneja con las manos,
/// y ahí la altura y la distancia buenas dependen de quién lleve el visor y de si está
/// de pie o sentado. Este componente le pone a la ventana un asa: se agarra con el
/// gatillo (de lejos con el rayo, de cerca con la mano) y la ventana acompaña al mando
/// hasta que se suelta.
///
/// Se monta a sí mismo en runtime, igual que VrKeyboard y por el mismo motivo: la escena
/// Quest se reconstruye entera desde AN5_sim en cada pasada, así que cualquier jerarquía
/// dejada a mano se perdería. Al constructor le basta con añadir este componente a la
/// ventana (ver k_MovableWindows en QuestSceneBuilder).
///
/// Lo que se mueve es la ventana entera, o sea el GameObject donde está este componente;
/// el asa es solo el trozo por el que se la coge.
[RequireComponent(typeof(RectTransform))]
public class VrWindowGrab : MonoBehaviour
{
    [Header("Asa")]
    [Tooltip("Rótulo del asa. Va girado 90°, que es como cae en una tira vertical.")]
    public string label = "MOVER";

    // El asa se mide en METROS y no en px del canvas, al revés que todo lo demás de una
    // ventana. Es lo que se coge con la mano, así que lo que tiene que ser igual en todas
    // es su tamaño real, y cada ventana va a una escala muy distinta: SecTraj a 0.00184
    // m/px y Panel_trayectorias a 0.0009, o sea que un asa de 96 px saldría de 17.7 cm en
    // una y de 8.6 en la otra. Build() los pasa a px con la escala de la propia ventana.
    [Tooltip("Ancho del asa, en metros.")]
    public float barWidth = 0.18f;

    [Tooltip("Separación entre el contenido de la ventana y el asa, en metros.")]
    public float barGap = 0.02f;

    [Tooltip("Grosor de la caja de agarre, en metros. Es lo que sobresale hacia el " +
             "operador para que la mano tenga volumen que tocar; el asa se dibuja plana.")]
    public float grabDepth = 0.09f;

    [Header("Límites de colocación")]
    [Tooltip("Altura mínima del centro de la ventana, en metros. Impide dejarla bajo el suelo.")]
    public float minHeight = 0.35f;

    [Tooltip("Altura máxima del centro de la ventana, en metros.")]
    public float maxHeight = 2.2f;

    // El asa va en el costado derecho y no arriba a propósito, y en las dos ventanas que
    // la llevan hoy sale del mismo sitio por motivos distintos (huellas medidas sobre la
    // escena, no calculadas, como pide docs/quest_test_arnes.md):
    //
    // - SecTraj está encajada entre el robot, que empieza en -22° de elevación, y el
    //   footer, cuyas esquinas suben a -34.4°: por arriba o por abajo, la tira se comería
    //   uno de esos dos márgenes. A lo ancho sobra: pasa de ±12.8° de azimut a 23.6°.
    // - Panel_trayectorias tiene sitio arriba y abajo, pero no a la izquierda: ahí está la
    //   pantalla de lecturas de la pared, que acaba en 28.5°, y el panel ya empieza en
    //   29.0°. Con el asa a la izquierda bajaría a 23.9° y le taparía una esquina (las
    //   elevaciones se solapan entre -1.3° y 1.4°). A la derecha pasa de 71.0° a 76.1°,
    //   con LeftPanel empezando en 80.3°.
    static readonly Color k_Idle = new Color(0.11f, 0.13f, 0.16f, 0.96f);
    static readonly Color k_Hover = new Color(0.16f, 0.22f, 0.28f, 0.98f);
    static readonly Color k_Held = new Color(0.06f, 0.35f, 0.29f, 0.98f);

    Image _bar;
    XRSimpleInteractable _interactable;

    // Pose del mando que la sostiene, null mientras no la sostenga nadie.
    Transform _attach;
    // Sitio y orientación de la ventana vistos desde ese mando, congelados al agarrar.
    Vector3 _offset;
    Quaternion _rotation;

    void Start()
    {
        Build();
    }

    void OnDestroy()
    {
        if (_interactable == null) return;

        _interactable.selectEntered.RemoveListener(OnSelectEntered);
        _interactable.selectExited.RemoveListener(OnSelectExited);
        _interactable.hoverEntered.RemoveListener(OnHoverEntered);
        _interactable.hoverExited.RemoveListener(OnHoverExited);
    }

    /// Sigue al mando en LateUpdate, después de que XRI haya procesado los interactores
    /// en su Update: hacerlo antes dejaría la ventana un fotograma por detrás de la mano,
    /// que en el visor se nota como que va "blanda".
    void LateUpdate()
    {
        if (_attach == null) return;

        var frame = YawOf(_attach.rotation);
        var position = _attach.position + frame * _offset;
        position.y = Mathf.Clamp(position.y, minHeight, maxHeight);

        transform.SetPositionAndRotation(position, frame * _rotation);
    }

    void OnSelectEntered(SelectEnterEventArgs args)
    {
        var attach = args.interactorObject.GetAttachTransform(_interactable);
        _attach = attach != null ? attach : args.interactorObject.transform;

        // Se congela la ventana en el marco del mando, y solo en yaw: así el cabeceo y el
        // alabeo de la muñeca no la vuelcan, y su plano sigue conteniendo el eje vertical
        // del mundo —perpendicular a la tapa de la mesa, como todas las demás ventanas de
        // la escena— la dejes donde la dejes.
        var frame = Quaternion.Inverse(YawOf(_attach.rotation));
        _offset = frame * (transform.position - _attach.position);
        _rotation = frame * YawOf(transform.rotation);

        SetBarColor(k_Held);
    }

    void OnSelectExited(SelectExitEventArgs args)
    {
        _attach = null;
        SetBarColor(_interactable != null && _interactable.isHovered ? k_Hover : k_Idle);
    }

    void OnHoverEntered(HoverEnterEventArgs args)
    {
        if (_attach == null) SetBarColor(k_Hover);
    }

    void OnHoverExited(HoverExitEventArgs args)
    {
        if (_attach == null && _interactable != null && !_interactable.isHovered) SetBarColor(k_Idle);
    }

    void SetBarColor(Color color)
    {
        if (_bar != null) _bar.color = color;
    }

    /// Yaw puro de una orientación: la vuelta que le da alrededor de la vertical del
    /// mundo, sin cabeceo ni alabeo.
    ///
    /// No se saca de eulerAngles.y: apuntando a plomo (que es como se coge una ventana
    /// que está por debajo de la vista, justo el caso de SecTraj) el yaw y el alabeo se
    /// confunden y ese valor pega saltos. Se proyecta el eje del mando sobre el plano
    /// horizontal, y si ese eje es justo el vertical se recurre a su "arriba", que
    /// entonces sí es horizontal.
    static Quaternion YawOf(Quaternion rotation)
    {
        var direction = rotation * Vector3.forward;
        direction.y = 0f;

        if (direction.sqrMagnitude < 1e-6f)
        {
            direction = rotation * Vector3.up;
            direction.y = 0f;
            if (direction.sqrMagnitude < 1e-6f) return Quaternion.identity;
        }

        return Quaternion.LookRotation(direction.normalized, Vector3.up);
    }

    // -----------------------------------------------------------------
    // Construcción
    // -----------------------------------------------------------------
    void Build()
    {
        var window = (RectTransform)transform;
        var content = ContentBounds(window);

        // Escala px -> m del canvas de esta ventana (se la pone QuestSceneBuilder con el
        // pixelToMeter de su fila). Si la ventana no estuviera escalada, sus px ya serían
        // metros y no hay nada que convertir.
        var scale = window.lossyScale.x;
        if (scale < 1e-6f) scale = 1f;
        var widthPx = barWidth / scale;
        var gapPx = barGap / scale;
        var depthPx = grabDepth / scale;

        var go = new GameObject("Grab Handle", typeof(RectTransform));
        var bar = (RectTransform)go.transform;
        bar.SetParent(window, worldPositionStays: false);
        bar.anchorMin = new Vector2(0.5f, 0.5f);
        bar.anchorMax = new Vector2(0.5f, 0.5f);
        bar.pivot = new Vector2(0.5f, 0.5f);
        bar.sizeDelta = new Vector2(widthPx, content.size.y);

        // El ancla está en el centro del rect de la ventana, que no tiene por qué caer en
        // el centro de lo que la ventana dibuja: las secciones heredadas de AN5_sim traen
        // el rect de la maquetación de escritorio con zonas vacías dentro.
        var center = new Vector2(content.max.x + gapPx + widthPx * 0.5f, content.center.y);
        bar.anchoredPosition = center - window.rect.center;

        _bar = go.AddComponent<Image>();
        _bar.color = k_Idle;
        // El agarre entra por el collider, no por el raycaster de UI. Si el asa fuese
        // además blanco de UI, el rayo tendría dos cosas que golpear en el mismo sitio y
        // la de UI, que queda por delante, le ganaría al collider.
        _bar.raycastTarget = false;

        MakeLabel(bar, widthPx);

        // Los interactores del rig disparan sus rayos contra las capas Default y UI. El
        // canvas de la ventana viene de AN5_sim y puede estar en cualquiera de ellas, así
        // que el asa se pone en Default explícitamente y no hereda nada.
        go.layer = 0;

        var box = go.AddComponent<BoxCollider>();
        box.size = new Vector3(widthPx, content.size.y, depthPx);
        // Un canvas se ve desde su cara -Z, así que el volumen sobresale hacia ahí: hacia
        // el lado por el que llega la mano.
        box.center = new Vector3(0f, 0f, -depthPx * 0.5f);

        // Después del collider a propósito: XRBaseInteractable.Awake —que corre dentro de
        // este AddComponent— recoge los colliders que encuentre en su GameObject y sus
        // hijos y los registra en el XRInteractionManager. Uno añadido más tarde no
        // entraría en ese registro y el asa no se podría coger.
        _interactable = go.AddComponent<XRSimpleInteractable>();
        _interactable.selectEntered.AddListener(OnSelectEntered);
        _interactable.selectExited.AddListener(OnSelectExited);
        _interactable.hoverEntered.AddListener(OnHoverEntered);
        _interactable.hoverExited.AddListener(OnHoverExited);
    }

    void MakeLabel(RectTransform bar, float widthPx)
    {
        var go = new GameObject("Text", typeof(RectTransform));
        var rect = (RectTransform)go.transform;
        rect.SetParent(bar, worldPositionStays: false);
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.pivot = new Vector2(0.5f, 0.5f);
        // Tumbado: se mide a lo largo de la tira y se gira un cuarto de vuelta, así que el
        // rótulo se lee de abajo arriba.
        rect.sizeDelta = new Vector2(bar.sizeDelta.y, bar.sizeDelta.x);
        rect.anchoredPosition = Vector2.zero;
        rect.localRotation = Quaternion.Euler(0f, 0f, 90f);

        var text = go.AddComponent<Text>();
        text.font = ResolveFont();
        text.fontSize = Mathf.RoundToInt(widthPx * 0.45f);
        text.alignment = TextAnchor.MiddleCenter;
        text.color = Color.white;
        text.text = label;
        text.horizontalOverflow = HorizontalWrapMode.Overflow;
        text.verticalOverflow = VerticalWrapMode.Overflow;
        text.raycastTarget = false;
    }

    /// Caja de lo que la ventana dibuja de verdad, en su espacio local: la unión de sus
    /// hijos activos, igual que hace QuestSceneBuilder para colocarla. Si no hay hijos
    /// que medir se cae al rect, que es lo único que queda.
    static Bounds ContentBounds(RectTransform window)
    {
        var bounds = new Bounds();
        var any = false;

        for (var i = 0; i < window.childCount; i++)
        {
            var child = window.GetChild(i);
            if (!child.gameObject.activeSelf) continue;

            var childBounds = RectTransformUtility.CalculateRelativeRectTransformBounds(window, child);
            if (!any) { bounds = childBounds; any = true; }
            else bounds.Encapsulate(childBounds);
        }

        if (!any)
            bounds = new Bounds(window.rect.center, window.rect.size);

        return bounds;
    }

    /// Reutiliza la fuente que ya use la UI de la aplicación, igual que VrKeyboard: así el
    /// asa no desentona y no se depende del nombre de los recursos internos de Unity, que
    /// ha cambiado entre versiones.
    static Font ResolveFont()
    {
        foreach (var text in FindObjectsByType<Text>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (text.font != null) return text.font;

        var builtin = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        return builtin != null ? builtin : Resources.GetBuiltinResource<Font>("Arial.ttf");
    }
}
