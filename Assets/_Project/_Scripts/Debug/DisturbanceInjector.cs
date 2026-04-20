using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DisturbanceInjector : MonoBehaviour
{
    public float disturbanceTorque = 0.12f;
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Inject Pitch Disturbance
        if (Input.GetKeyDown(KeyCode.P))
        {
            // Applies torque around the local X-axis (Pitch)
            rb.AddRelativeTorque(Vector3.right * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("Pitch Disturbance Applied");
        }

        // Inject Roll Disturbance
        if (Input.GetKeyDown(KeyCode.R))
        {
            // Applies torque around the local Z-axis (Roll)
            rb.AddRelativeTorque(Vector3.forward * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("Roll Disturbance Applied");
        }
    }
}