using UnityEngine;
using UnityEngine.UI;
#if UNITY_STANDALONE_WIN
using System;
using System.Runtime.InteropServices;
#endif

/// Attached to PersistentLayer/Header/WindowButtons.
/// Provides Close / Minimize / Maximize-Restore behavior for the app's
/// own header bar, since the build runs borderless fullscreen
/// (ProjectSettings fullscreenMode: FullScreenWindow) and has no native
/// OS title bar to supply these controls.
public class WindowControls : MonoBehaviour
{
    Button _btnClose, _btnMinimize, _btnMaximizeRestore;
    bool _isMaximized;

#if UNITY_STANDALONE_WIN
    [DllImport("user32.dll")] static extern IntPtr GetActiveWindow();
    [DllImport("user32.dll")] static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    const int SW_MINIMIZE = 6;
#endif

    void Start()
    {
        _btnClose           = transform.Find("Close")?.GetComponent<Button>();
        _btnMinimize        = transform.Find("Minimize")?.GetComponent<Button>();
        _btnMaximizeRestore = transform.Find("MaximizeRestore")?.GetComponent<Button>();

        if (_btnClose)           _btnClose.onClick.AddListener(CloseApplication);
        if (_btnMinimize)        _btnMinimize.onClick.AddListener(MinimizeWindow);
        if (_btnMaximizeRestore) _btnMaximizeRestore.onClick.AddListener(ToggleMaximizeRestore);

        _isMaximized = Screen.fullScreen;
    }

    public void MinimizeWindow()
    {
#if UNITY_STANDALONE_WIN
        ShowWindow(GetActiveWindow(), SW_MINIMIZE);
#else
        Debug.LogWarning("[WindowControls] Minimize is only supported on Windows standalone builds.");
#endif
    }

    public void ToggleMaximizeRestore()
    {
        if (_isMaximized)
        {
            var res = Screen.currentResolution;
            Screen.SetResolution(res.width * 3 / 4, res.height * 3 / 4, FullScreenMode.Windowed);
        }
        else
        {
            var res = Screen.currentResolution;
            Screen.SetResolution(res.width, res.height, FullScreenMode.FullScreenWindow);
        }
        _isMaximized = !_isMaximized;
    }

    public void CloseApplication()
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }
}
