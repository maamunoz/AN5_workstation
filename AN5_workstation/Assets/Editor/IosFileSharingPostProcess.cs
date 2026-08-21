#if UNITY_IOS
using UnityEditor;
using UnityEditor.Callbacks;
using UnityEditor.iOS.Xcode;
using UnityEngine;

/// Habilita en el Info.plist del proyecto Xcode exportado las dos claves que dejan ver y
/// escribir la carpeta Documents de la app desde fuera.
///
/// POR QUÉ: en iOS, SecTrajController.GetRoutinesDir() resuelve a
/// Application.persistentDataPath/routines, y persistentDataPath es .../Documents. Sin
/// estas claves esa carpeta es invisible: no aparece ni en Finder (iPad conectado por
/// cable, pestaña Archivos) ni en la app Archivos del propio iPad, así que no hay ninguna
/// forma de meterle un .txt de trayectoria y la lista del botón CARGAR sale siempre vacía.
/// Es el equivalente iOS del `adb push` que se usa para la Quest.
///
///   UIFileSharingEnabled              -> expone Documents en Finder / iTunes File Sharing.
///   LSSupportsOpeningDocumentsInPlace -> además la muestra en la app Archivos, para poder
///                                        copiar ahí sin cable.
///
/// Se hace por post-proceso porque Unity no expone estas claves en Player Settings; el
/// Info.plist lo regenera cada exportación, así que editarlo a mano se perdería en el
/// siguiente build.
public static class IosFileSharingPostProcess
{
    [PostProcessBuild(100)]
    public static void OnPostProcessBuild(BuildTarget target, string pathToBuiltProject)
    {
        if (target != BuildTarget.iOS) return;

        string plistPath = System.IO.Path.Combine(pathToBuiltProject, "Info.plist");
        if (!System.IO.File.Exists(plistPath))
        {
            Debug.LogWarning($"[IosFileSharingPostProcess] No se encontró Info.plist en '{plistPath}'; " +
                             "la carpeta Documents quedará oculta y el botón CARGAR no verá archivos.");
            return;
        }

        var plist = new PlistDocument();
        plist.ReadFromFile(plistPath);
        plist.root.SetBoolean("UIFileSharingEnabled", true);
        plist.root.SetBoolean("LSSupportsOpeningDocumentsInPlace", true);
        plist.WriteToFile(plistPath);

        Debug.Log("[IosFileSharingPostProcess] UIFileSharingEnabled + LSSupportsOpeningDocumentsInPlace " +
                  "activadas: la carpeta Documents de la app ya es visible desde Finder y la app Archivos.");
    }
}
#endif
