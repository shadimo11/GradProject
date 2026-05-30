using UnityEngine;
using UnityEngine.InputSystem; // Required for the new Input System

[RequireComponent(typeof(Rigidbody))]
public class DisturbanceInjector : MonoBehaviour
{
    [Header("Rotational Disturbance Parameters")]
    [Tooltip("Target angular velocity disturbance in degrees per second")]
    public float disturbanceAngularVelocity = 5f;
    private float inertia = 0.04547f;
    private float disturbanceTorque;


    private Rigidbody rb;

    // State flags for deterministic FixedUpdate execution
    private bool injectPitch;
    private bool injectRoll;
    private bool injectWind;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Convert desired angular velocity to required impulse torque (J = I * omega)
        disturbanceTorque = inertia * disturbanceAngularVelocity * Mathf.Deg2Rad;
    }

    void Update()
    {
        if (Keyboard.current == null) return;

        // Pitch Disturbance (X-Axis)
        if (Keyboard.current.fKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.right * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("[SIL] Pitch Forward Disturbance Applied");
        }
        else if (Keyboard.current.bKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.left * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("[SIL] Pitch Backward Disturbance Applied");
        }

        // Roll Disturbance (Z-Axis)
        if (Keyboard.current.lKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.forward * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("[SIL] Roll Left Disturbance Applied");
        }
        else if (Keyboard.current.rKey.wasPressedThisFrame)
        {
            rb.AddRelativeTorque(Vector3.back * disturbanceTorque, ForceMode.Impulse);
            Debug.Log("[SIL] Roll Right Disturbance Applied");
        }
    }

    void FixedUpdate()
    {
        // Execute physics exclusively in the deterministic fixed timestep loop

        if (injectPitch)
        {
            rb.AddRelativeTorque(Vector3.right * disturbanceTorque, ForceMode.Impulse);
            Debug.Log($"[SIL] Pitch Disturbance Applied: {disturbanceTorque} N.m.s");
            injectPitch = false;
        }

        if (injectRoll)
        {
            rb.AddRelativeTorque(Vector3.forward * disturbanceTorque, ForceMode.Impulse);
            Debug.Log($"[SIL] Roll Disturbance Applied: {disturbanceTorque} N.m.s");
            injectRoll = false;
        }
    }
}