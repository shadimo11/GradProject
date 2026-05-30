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

    [Header("Translational Wind Gust Parameters")]
    [Tooltip("Predetermined wind gust vector in m/s (World Space)")]
    public Vector3 windGustVelocity = new Vector3(5f, 0f, 0f);

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
        // Safety check to ensure a keyboard is connected
        if (Keyboard.current == null) return;

        // Poll inputs during the rendering frame
        if (Keyboard.current.pKey.wasPressedThisFrame) injectPitch = true;
        if (Keyboard.current.rKey.wasPressedThisFrame) injectRoll = true;
        if (Keyboard.current.wKey.wasPressedThisFrame) injectWind = true;
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

        if (injectWind)
        {
            // Apply a discrete translational velocity change in the World frame to simulate a lateral gust
            rb.AddForce(windGustVelocity, ForceMode.VelocityChange);
            Debug.Log($"[SIL] Wind Gust Applied: {windGustVelocity} m/s");
            injectWind = false;
        }
    }
}