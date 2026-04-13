using UnityEngine;

[ExecuteInEditMode]
public class DebugCoM : MonoBehaviour
{
    public bool showCoM = true;
    void OnDrawGizmos()
    {
        if (!showCoM) return;
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb)
        {
            Gizmos.color = Color.yellow;
            // Draw a sphere at the calculated World Center of Mass
            Gizmos.DrawSphere(rb.worldCenterOfMass, 0.005f);
        }
    }
}