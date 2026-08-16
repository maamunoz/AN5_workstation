using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

/// Lista desplegable en VR de los archivos de trayectorias disponibles en una carpeta.
///
/// Existe para Android/Quest, donde no hay diálogo nativo de archivos que abrir (ver el
/// comentario de SecTrajController.ShowNativeFileDialog) y encima no habría dónde
/// pintarlo dentro de la sesión inmersiva de OpenXR aunque lo hubiera. En vez de eso,
/// esto arma a mano una columna de botones -- uno por archivo -- colgando al lado de la
/// ventana de SecTraj, tocable con el mismo rayo de los mandos que el resto del panel.
///
/// Se monta a sí mismo en runtime, igual que VrKeyboard y VrWindowGrab: no depende de
/// nada puesto a mano en la escena, así que sobrevive a que QuestSceneBuilder la
/// reconstruya entera desde AN5_sim en cada pasada.
///
/// Vuelve a leer el disco (Directory.GetFiles) cada vez que se abre y cada vez que se
/// toca "Refrescar" -- un archivo copiado por adb push mientras la app ya está corriendo
/// aparece sin reinstalar nada, basta con cerrar y reabrir la lista.
public class TrajectoryFileList : MonoBehaviour
{
    const float k_PanelWidth  = 480f;
    const float k_RowHeight   = 52f;
    const int   k_MaxRows     = 12; // sin ScrollRect: cap razonable para no salirse de la sala.
    static readonly string[] k_Extensions = { "*.txt", "*.csv" };

    RectTransform _rows;
    Text          _titleText;
    string        _directory;
    Action<string> _onChosen;

    /// Muestra (creándola si hace falta) la lista colgando a la derecha de `anchor` y la
    /// refresca contra `directory`. `anchor` tiene que ser un RectTransform que ya cuelgue
    /// de un Canvas World Space con TrackedDeviceGraphicRaycaster -- la ventana de SecTraj
    /// ya lo es, ver QuestSceneBuilder.DetachAsWindow.
    public static TrajectoryFileList Show(RectTransform anchor, string directory, Action<string> onChosen)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        if (picker == null) picker = Create(anchor);

        picker._directory = directory;
        picker._onChosen  = onChosen;
        picker.gameObject.SetActive(true);
        picker.Refresh();
        return picker;
    }

    public static void Hide(RectTransform anchor)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        if (picker != null) picker.gameObject.SetActive(false);
    }

    public static bool IsVisible(RectTransform anchor)
    {
        var picker = anchor.GetComponentInChildren<TrajectoryFileList>(true);
        return picker != null && picker.gameObject.activeSelf;
    }

    void Refresh()
    {
        foreach (Transform child in _rows)
            Destroy(child.gameObject);

        var font = VrUiKit.DefaultFont();

        List<string> files;
        try
        {
            files = k_Extensions
                .SelectMany(pattern => Directory.GetFiles(_directory, pattern))
                .Distinct()
                .OrderByDescending(File.GetLastWriteTimeUtc) // lo recién copiado por adb push arriba de todo
                .ToList();
        }
        catch (Exception e)
        {
            files = new List<string>();
            Debug.LogWarning($"[TrajectoryFileList] No se pudo leer '{_directory}': {e.Message}");
        }

        _titleText.text = $"Archivos ({files.Count})";

        if (files.Count == 0)
        {
            var empty = VrUiKit.MakeRow(_rows, "Empty");
            VrUiKit.MakeText(empty, $"No hay archivos en\n{_directory}", font, 22, TextAnchor.MiddleLeft);
            var empties = empty.GetComponent<LayoutElement>();
            empties.preferredHeight = k_RowHeight * 1.5f;
        }
        else
        {
            int shown = Mathf.Min(files.Count, k_MaxRows);
            for (int i = 0; i < shown; i++)
            {
                string path = files[i];
                var button = VrUiKit.MakeButton(_rows, Path.GetFileName(path), font, 22);
                button.GetComponent<LayoutElement>().preferredHeight = k_RowHeight;
                button.onClick.AddListener(() =>
                {
                    _onChosen?.Invoke(path);
                    gameObject.SetActive(false);
                });
            }

            if (files.Count > shown)
            {
                var more = VrUiKit.MakeRow(_rows, "More");
                VrUiKit.MakeText(more, $"(+{files.Count - shown} más -- no se muestran)", font, 18, TextAnchor.MiddleLeft);
            }
        }

        // Sin esto, el primer Text de cada fila recién creada mide su ajuste de línea
        // contra un rect que el LayoutGroup todavía no terminó de resolver (a veces
        // efectivamente 0 de ancho) -- con horizontalOverflow en Wrap eso parte cada
        // nombre de archivo letra por letra, en una columna vertical, en vez de leerse
        // de corrido. ForceRebuildLayoutImmediate por sí solo no basta: solo repasa lo
        // que ya está marcado sucio, y el propio Destroy/creación de filas de arriba deja
        // sin vaciar la cola de CanvasUpdateRegistry de este mismo fotograma. Vaciarla
        // primero con ForceUpdateCanvases dejas todo resuelto antes del rebuild explícito,
        // así cada Text mide contra su ancho real antes de que se vea nada.
        Canvas.ForceUpdateCanvases();
        LayoutRebuilder.ForceRebuildLayoutImmediate((RectTransform)transform);
    }

    static TrajectoryFileList Create(RectTransform anchor)
    {
        var font = VrUiKit.DefaultFont();

        var rootGo = new GameObject("TrajectoryFileList", typeof(RectTransform));
        var root = (RectTransform)rootGo.transform;
        root.SetParent(anchor, worldPositionStays: false);
        root.pivot     = new Vector2(0f, 1f);
        root.anchorMin = new Vector2(1f, 1f);
        root.anchorMax = new Vector2(1f, 1f);
        root.anchoredPosition = new Vector2(24f, 0f);
        root.sizeDelta = new Vector2(k_PanelWidth, 0f);

        var bg = rootGo.AddComponent<Image>();
        bg.color = new Color(0.05f, 0.05f, 0.07f, 0.96f);

        var rootLayout = rootGo.AddComponent<VerticalLayoutGroup>();
        rootLayout.padding = new RectOffset(14, 14, 14, 14);
        rootLayout.spacing = 8f;
        rootLayout.childControlWidth = true;
        rootLayout.childForceExpandWidth = true;
        rootLayout.childControlHeight = false;
        rootLayout.childForceExpandHeight = false;

        var fitter = rootGo.AddComponent<ContentSizeFitter>();
        fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        var picker = rootGo.AddComponent<TrajectoryFileList>();

        // Título + Refrescar + Cerrar, en una fila.
        var header = VrUiKit.MakeRow(root, "Header");
        var headerLayout = header.gameObject.AddComponent<HorizontalLayoutGroup>();
        headerLayout.spacing = 8f;
        headerLayout.childControlWidth = true;
        headerLayout.childForceExpandWidth = false;
        header.GetComponent<LayoutElement>().preferredHeight = k_RowHeight;

        picker._titleText = VrUiKit.MakeText(header, "Archivos", font, 24, TextAnchor.MiddleLeft);
        picker._titleText.GetComponent<LayoutElement>().flexibleWidth = 1f;

        var refreshBtn = VrUiKit.MakeButton(header, "⟳", font, 26); // ⟳
        refreshBtn.GetComponent<LayoutElement>().preferredWidth = k_RowHeight;
        refreshBtn.onClick.AddListener(picker.Refresh);

        var closeBtn = VrUiKit.MakeButton(header, "✕", font, 26); // ✕
        closeBtn.GetComponent<LayoutElement>().preferredWidth = k_RowHeight;
        closeBtn.onClick.AddListener(() => picker.gameObject.SetActive(false));

        var rowsGo = VrUiKit.MakeRow(root, "Rows");
        var rowsLayout = rowsGo.gameObject.AddComponent<VerticalLayoutGroup>();
        rowsLayout.spacing = 6f;
        rowsLayout.childControlWidth = true;
        rowsLayout.childForceExpandWidth = true;
        rowsLayout.childControlHeight = false;
        rowsLayout.childForceExpandHeight = false;
        Destroy(rowsGo.GetComponent<LayoutElement>()); // el contenedor de filas no necesita tamaño propio
        picker._rows = rowsGo;

        return picker;
    }
}
