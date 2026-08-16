using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

namespace AN5.Measurement
{
    /// Ventana en VR del arnés de mediciones, para Android/Quest.
    ///
    /// El panel de MeasurementSession es OnGUI (ver el comentario de su propio OnGUI():
    /// "para no depender del Canvas de la app"), y OnGUI dibuja directo sobre el
    /// backbuffer 2D -- eso nunca llega al compositor de la sesión estéreo de OpenXR, así
    /// que en el visor ese panel simplemente no se ve. El README de esta rama documentaba
    /// como única alternativa `autoRunOnStart` (correr todo sin intervención); esto agrega
    /// la otra mitad -- poder revisar el estado de cada prueba y lanzarlas una por una a
    /// mano, con el rayo de los mandos, igual que en escritorio con el mouse.
    ///
    /// Se monta a sí mismo en runtime, igual que VrKeyboard/VrWindowGrab/
    /// TrajectoryFileList: QuestSceneBuilder solo tiene que dejar puesto el componente
    /// (ver ConfigureMeasurementHarness) en el mismo GameObject que MeasurementSession. En
    /// escritorio no hace nada -- el panel OnGUI de siempre sigue siendo el camino ahí,
    /// esto es aditivo, igual que el resto del arnés (ver Measurement/README.md).
    ///
    /// Ventana fija (no HUD, no colgada de la cabeza): se ancla al arrancar a un punto del
    /// mundo relativo a dónde empieza el operador, en el mismo hueco angular que
    /// QuestSceneBuilder reserva para Panel_ppal (ver su comentario ahí: "aparcada...
    /// por si algún día se reactiva" -- ese ángulo no lo usa ninguna otra ventana). Lleva
    /// además VrWindowGrab, así que si en la práctica cae superpuesta con algo se puede
    /// correr con los mandos sin tocar código.
    [RequireComponent(typeof(MeasurementSession))]
    public class MeasurementHarnessVrPanel : MonoBehaviour
    {
        const float k_YawDeg       = -95f; // mismo hueco que Panel_ppal en QuestSceneBuilder
        const float k_Radius       = 1.8f;
        const float k_Height       = 1.5f;
        const float k_PixelToMeter = 0.0016f;
        const float k_PanelWidthPx = 620f;
        const float k_PollSeconds  = 0.5f;

        MeasurementSession _session;
        Text _headerText;
        Button _runAllButton;
        readonly List<(MeasurementTest test, Button button, Text label)> _rows =
            new List<(MeasurementTest, Button, Text)>();

        void Start()
        {
            if (Application.platform != RuntimePlatform.Android) return;

            _session = GetComponent<MeasurementSession>();
            if (_session == null)
            {
                Debug.LogWarning("[MeasurementHarnessVrPanel] No hay MeasurementSession en este GameObject.");
                return;
            }

            var cam = Camera.main;
            if (cam == null)
            {
                Debug.LogWarning("[MeasurementHarnessVrPanel] No se encontró Camera.main (cámara del XR " +
                                  "Origin); no se arma la ventana del arnés.");
                return;
            }

            Build(cam);
            StartCoroutine(PollLoop());
        }

        IEnumerator PollLoop()
        {
            var wait = new WaitForSeconds(k_PollSeconds);
            while (true)
            {
                Refresh();
                yield return wait;
            }
        }

        void Refresh()
        {
            _headerText.text =
                $"Arnés — {_session.ShortConfigLabel()} / {_session.platformLabel}\n" +
                $"rosbridge: {(_session.IsConnected ? "CONECTADO" : "SIN CONEXIÓN")}   " +
                $"reloj único: {(_session.SingleClock ? "sí" : "no")}" +
                (_session.IsRunning && _session.CurrentTest != null
                    ? $"\nCorriendo: {_session.CurrentTest.TestId}"
                    : "");

            _runAllButton.interactable = !_session.IsRunning;

            foreach (var (test, button, label) in _rows)
            {
                bool applies = test.AppliesTo(_session.configuration);
                button.interactable = !_session.IsRunning && applies;
                label.text = applies
                    ? $"{test.DisplayName} — {test.Status}"
                    : $"{test.DisplayName} — no aplica ({test.NotApplicableReason})";
            }
        }

        void Build(Camera cam)
        {
            var font = VrUiKit.DefaultFont();

            var go = new GameObject("MeasurementHarnessWindow (VR)", typeof(RectTransform));
            var root = (RectTransform)go.transform;

            // Mismo cálculo que QuestSceneBuilder.PlaceWindow, pero a runtime: no hay
            // forma de llegar desde acá al k_UserStart de tiempo de edición, y la
            // posición de la cámara al arrancar (antes de que el operador camine) es
            // equivalente.
            var pivot = cam.transform.position;
            var direction = Quaternion.Euler(0f, k_YawDeg, 0f) * Vector3.forward;
            var rotation = Quaternion.LookRotation(direction, Vector3.up);
            var center = pivot + direction * k_Radius;
            center.y = k_Height;

            root.pivot = new Vector2(0.5f, 0.5f);
            root.anchorMin = root.anchorMax = new Vector2(0.5f, 0.5f);
            root.sizeDelta = new Vector2(k_PanelWidthPx, 0f);
            root.localScale = Vector3.one * k_PixelToMeter;
            root.SetPositionAndRotation(center, rotation);

            var bg = go.AddComponent<Image>();
            bg.color = new Color(0.05f, 0.05f, 0.07f, 0.96f);

            var rootLayout = go.AddComponent<VerticalLayoutGroup>();
            rootLayout.padding = new RectOffset(18, 18, 18, 18);
            rootLayout.spacing = 10f;
            rootLayout.childControlWidth = true;
            rootLayout.childForceExpandWidth = true;
            rootLayout.childControlHeight = false;
            rootLayout.childForceExpandHeight = false;

            var fitter = go.AddComponent<ContentSizeFitter>();
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            var canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.worldCamera = cam;

            var scaler = go.AddComponent<CanvasScaler>();
            scaler.dynamicPixelsPerUnit = 2f;

            go.AddComponent<GraphicRaycaster>();
            go.AddComponent<TrackedDeviceGraphicRaycaster>();

            // Asa de arrastre por si en la práctica esta posición cae superpuesta con
            // otra ventana -- se monta sola en runtime, ver VrWindowGrab.
            go.AddComponent<VrWindowGrab>();

            var header = VrUiKit.MakeRow(root, "Header");
            header.GetComponent<LayoutElement>().preferredHeight = 90f;
            _headerText = VrUiKit.MakeText(header, "", font, 22, TextAnchor.UpperLeft);

            _runAllButton = VrUiKit.MakeButton(root, "Ejecutar todas las aplicables", font, 24);
            _runAllButton.GetComponent<LayoutElement>().preferredHeight = 56f;
            _runAllButton.onClick.AddListener(() => _session.RunAll());

            foreach (var test in _session.Tests)
            {
                var row = VrUiKit.MakeRow(root, $"Row_{test.TestId}");
                row.GetComponent<LayoutElement>().preferredHeight = 46f;
                var rowLayout = row.gameObject.AddComponent<HorizontalLayoutGroup>();
                rowLayout.spacing = 10f;
                rowLayout.childControlWidth = true;
                rowLayout.childForceExpandWidth = false;

                var runButton = VrUiKit.MakeButton(row, test.TestId, font, 20);
                runButton.GetComponent<LayoutElement>().preferredWidth = 70f;
                var capturedTest = test; // closure -- una copia por iteración
                runButton.onClick.AddListener(() => _session.RunTest(capturedTest));

                var label = VrUiKit.MakeText(row, "", font, 20, TextAnchor.MiddleLeft);
                label.GetComponent<LayoutElement>().flexibleWidth = 1f;

                _rows.Add((test, runButton, label));
            }

            var footer = VrUiKit.MakeText(root, _session.RunDirectory, font, 14, TextAnchor.UpperLeft);
            footer.color = new Color(0.75f, 0.75f, 0.75f, 1f);

            // Sin esto, el primer Text de cada fila mide su ajuste de línea contra un
            // rect que el LayoutGroup todavía no terminó de resolver -- con
            // horizontalOverflow en Wrap eso parte cada etiqueta letra por letra, en
            // columna, en vez de leerse de corrido. Ver el mismo comentario en
            // TrajectoryFileList.Refresh(), que tenía el mismo problema.
            LayoutRebuilder.ForceRebuildLayoutImmediate(root);

            Refresh();
        }
    }
}
