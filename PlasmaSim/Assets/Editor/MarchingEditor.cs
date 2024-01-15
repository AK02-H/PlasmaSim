using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Marching))]
public class MarchingEditor : Editor
{
    public override void OnInspectorGUI()
    {
        
        base.OnInspectorGUI();
        Marching bPattern = (Marching)target;

        if (GUILayout.Button("Build"))
        {
            bPattern.Build();
        }

    }
}
