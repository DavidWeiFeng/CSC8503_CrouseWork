// Unity helper script to recreate the CSC8503 BuildSlopeScene layout from Assets/Data/LevelLayout.json
// Drop this into an Editor folder or attach to an empty GameObject and call LoadLevel().
// Requires Newtonsoft Json or UnityEngine.JsonUtility-compatible structures (this uses JsonUtility-friendly POCOs).

using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class LevelObject
{
    public string name;
    public string type;
    public float[] position;
    public float[] rotationEuler;
    public float[] scale;
    public float[] openOffset;
}

[Serializable]
public class LevelLayout
{
    public List<LevelObject> objects;
    public string notes;
}

public class UnityLevelLoader : MonoBehaviour
{
    public string jsonRelativePath = "Assets/Data/LevelLayout.json";
    public bool buildAtStart = true;
    public Material defaultMat;

    void Start()
    {
        if (buildAtStart)
        {
            LoadLevel();
        }
    }

    [ContextMenu("Load Level")]
    public void LoadLevel()
    {
        if (!File.Exists(jsonRelativePath))
        {
            Debug.LogError($"Level layout file not found: {jsonRelativePath}");
            return;
        }

        string json = File.ReadAllText(jsonRelativePath);
        LevelLayout layout = JsonUtility.FromJson<LevelLayout>(json);
        if (layout == null || layout.objects == null)
        {
            Debug.LogError("Failed to parse level layout.");
            return;
        }

        foreach (var obj in layout.objects)
        {
            Vector3 pos = ToVec3(obj.position);
            Quaternion rot = Quaternion.Euler(ToVec3(obj.rotationEuler));

            if (obj.type.Equals("Box", StringComparison.OrdinalIgnoreCase))
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.name = obj.name;
                go.transform.position = pos;
                go.transform.rotation = rot;
                go.transform.localScale = ToVec3(obj.scale);
                if (defaultMat != null)
                {
                    go.GetComponent<Renderer>().sharedMaterial = defaultMat;
                }
            }
            else if (obj.type.Equals("Marker", StringComparison.OrdinalIgnoreCase))
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = obj.name;
                go.transform.position = pos;
                go.transform.localScale = Vector3.one * 1.0f;
                if (defaultMat != null)
                {
                    go.GetComponent<Renderer>().sharedMaterial = defaultMat;
                }
            }
        }
    }

    private static Vector3 ToVec3(float[] arr)
    {
        if (arr == null || arr.Length < 3) return Vector3.zero;
        return new Vector3(arr[0], arr[1], arr[2]);
    }
}
