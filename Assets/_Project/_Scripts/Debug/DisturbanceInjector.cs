using UnityEngine;
using UnityEngine.InputSystem; // Required for the new Input System

[RequireComponent(typeof(Rigidbody))]
public class DisturbanceInjector : MonoBehaviour
{
    public float disturbanceAngularVelocity = 5f; // Degrees per second
    private float inertia = 0.04547f;
    private float disturbanceTorque;
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        disturbanceTorque = inertia * disturbanceAngularVelocity * Mathf.Deg2Rad; // Convert to radians per second
    }

    void Update()
    {
        // Safety check to ensure a keyboard is connected
        if (Keyboard.current == null) return;

        // Inject Pitch Disturbance (P key)
        if (Keyboard.current.pKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.right * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("Pitch Disturbance Applied");
        }

        // Inject Roll Disturbance (R key)
        if (Keyboard.current.rKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.forward * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("Roll Disturbance Applied");
        }
    }
}