using UnityEngine;
        using UnityEngine.UI;
        
        /// <summary>
        /// Reads the world position of j6_link every frame and updates
        /// the X/Y/Z InputFields in SecCoord.
        /// Attach to any persistent GO (e.g. the SecCoord panel itself).
        /// </summary>
        public class J6LinkPositionReader : MonoBehaviour
        {
            [Header("Target")]
            public Transform j6Link;          // drag fr5v6/.../j6_link here, or resolved at Start
        
            [Header("Input Fields (auto-resolved)")]
            public InputField inputX;
            public InputField inputY;
            public InputField inputZ;
        
            // Capital L matters: the URDF importer keeps the link name verbatim, and
            // the URDF declares name="j6_Link" (frcobot_description/urdf/fr5v6.urdf:401).
            // This used to read "j6_link", and since both lookups below are
            // case-sensitive it never matched: Start() silently left j6Link null and
            // Update() then re-scanned the whole scene every single frame, forever,
            // while the X/Y/Z fields were never written to.
            const string TARGET_NAME  = "j6_Link";
            const string COORD_PATH_X = "Panels/RightPanel/Content/SecCoord/Body/Row_X/Input_X";
            const string COORD_PATH_Y = "Panels/RightPanel/Content/SecCoord/Body/Row_Y/Input_Y";
            const string COORD_PATH_Z = "Panels/RightPanel/Content/SecCoord/Body/Row_Z/Input_Z";
        
            void Start()
            {
                // Resolve j6_link by name search if not assigned
                if (j6Link == null)
                {
                    var found = FindObjectsOfType<Transform>();
                    foreach (var t in found)
                        if (t.name == TARGET_NAME) { j6Link = t; break; }
                }
        
                if (j6Link == null)
                    Debug.LogWarning($"[J6LinkPositionReader] {TARGET_NAME} not found yet — will retry in Update");
        
                // Resolve InputFields by path
                if (inputX == null) inputX = GameObject.Find(COORD_PATH_X)?.GetComponent<InputField>();
                if (inputY == null) inputY = GameObject.Find(COORD_PATH_Y)?.GetComponent<InputField>();
                if (inputZ == null) inputZ = GameObject.Find(COORD_PATH_Z)?.GetComponent<InputField>();
        
                if (inputX == null || inputY == null || inputZ == null)
                    Debug.LogWarning("[J6LinkPositionReader] One or more InputFields not found");
                else
                    Debug.Log("[J6LinkPositionReader] Inputs resolved OK");
            }
        
            void Update()
            {
                // Retry finding j6_link if not yet found (spawned at runtime)
                if (j6Link == null)
                {
                    j6Link = GameObject.Find(TARGET_NAME)?.transform;
                    if (j6Link == null) return;
                    Debug.Log($"[J6LinkPositionReader] {TARGET_NAME} found at runtime: {j6Link.position}");
                }
        
                if (inputX == null || inputY == null || inputZ == null) return;
        
                var pos = j6Link.position;
                inputX.text = pos.x.ToString("F3");
                inputY.text = pos.y.ToString("F3");
                inputZ.text = pos.z.ToString("F3");
            }
        }
        