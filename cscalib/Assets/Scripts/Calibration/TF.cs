using UnityEngine;

public struct TF
{
    public TF(Quaternion _rot, Vector3 _pos)
    {
        rotation = _rot;
        position = _pos;
    }

    public Quaternion rotation;
    public Vector3 position;

    public void setToGameObject(ref GameObject unityTf)
    {
        unityTf.transform.position = position;
        unityTf.transform.rotation = rotation;
    }
}