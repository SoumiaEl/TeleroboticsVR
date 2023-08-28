using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MiroRGB3DCameraV7))]
public class MiroRBG3DCameraEditorV7 : Editor {

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        
        MiroRGB3DCameraV7 script = (MiroRGB3DCameraV7)target;

        /*
        if (GUILayout.Button("Enable RGB Feature"))
        {
            script.SwitchRGBCatcher(true);
        }

        if (GUILayout.Button("Disable RGB Feature"))
        {
            script.SwitchRGBCatcher(false);
        }
        */

        if (GUILayout.Button("Draw Extrema"))
        {
            script.DrawExtrema(true);
        }

        if (GUILayout.Button("Clear Extrema"))
        {
            script.DrawExtrema(false);
        }

        if (GUILayout.Button("Capture Points"))
        {
            Vector3[] pointCloud;
            Vector3[] pointCloud_color;

            script.CalcPointCloud(out pointCloud, out pointCloud_color);
            script.DrawLastCapturedPoints(true);
        }

        if (GUILayout.Button("Clear Last Captured Points"))
        {
            script.DrawLastCapturedPoints(false);
        }

        if (GUILayout.Button("Save Last Captured Points"))
        {

            string filename = EditorUtility.SaveFilePanel("Save pointcloud as PCD", "", "pointcloud.pcd", "pcd");
            if (filename != "")
            {
                script.CreatePCD(filename); ;
                Debug.Log("Pointcloud saved as " + filename);
            } else
            {
                Debug.LogWarning("Empty filename string, cannot save the pointcloud.");
            }

        }
    }

    
}
