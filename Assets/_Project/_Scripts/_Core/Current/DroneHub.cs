using System;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.InputSystem;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class DroneHub : MonoBehaviour
{
    [Header("Dependencies")]
    public TCPServer server;
    public Rigidbody rb;

    [Header("Sensors (Assign Child Objects)")]
    public IMU imu;
    public Barometer barometer;
    public Lidar lidar;
    public GPS gps;

    [Header("Motors")]
    [Tooltip("Assign motor transforms in the order: RR - RL - FR - FL")]
    public Transform[] motorTransforms;
    public float[] motorPwm;
    private float pwmMin = 0f;
    private float pwmMax = 255f;

    [Header("Physics Constants")]
    // Coefficients for PWM to Thrust conversion
    private float thrustA = 0.0106f;
    private float thrustB = 1.5276f;
    private float thrustC = 5.0422f;

    [Header("Manual Physics Overrides")]
    [Tooltip("Overrides the Rigidbody Center of Mass")]
    public Vector3 customCenterOfMass = new Vector3(-0.1532051f, 0.2461882f, -2.519418f);

    [Tooltip("Overrides the Rigidbody Inertia Tensor (Ixx, Iyy, Izz)")]
    public Vector3 customInertiaTensor = new Vector3(0.04547f, 0.04547f, 0.09094f);

    [Header("Setpoints (Virtual Pilot)")]
    public Transform targetTransform;
    private Vector3 positionSetpoint;
    private float yawSetpoint;

    // Internal Data Packet
    private readonly float[] feedbackFloats = new float[11];  // Number of variables sent back to Simulink
    private readonly byte[] sendBuffer = new byte[44];        // Number of bytes sent back to Simulink (floats * 4 bytes each)

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Awake()
    {
        if (motorTransforms != null)
            motorPwm = new float[motorTransforms.Length];

        // Manual Center of Mass & Inertia
        rb.centerOfMass = customCenterOfMass;
        rb.inertiaTensor = customInertiaTensor;
        rb.inertiaTensorRotation = Quaternion.identity;
    }

    void FixedUpdate()
    {
        // Update Setpoints from Unity Target (if it exists)
        if (targetTransform)
        {
            positionSetpoint = targetTransform.position;
            yawSetpoint = targetTransform.rotation.eulerAngles.y; // Simplified Yaw
        }

        // TCP: Get incoming Motor PWM from Simulink
        if (server != null && server.TryGetPwm(out float[] newPwm))
        {
            ApplyMotorCommands(newPwm);
        }

        // Physics: Apply Forces to Rigidbody
        ApplyPhysics();

        // TCP: Aggregate Sensor Data & Send to Simulink
        if (server != null)
        {
            PackSensorData();
            server.SendFeedback(sendBuffer);
        }
    }

    // ---------------------------------------------------------
    //                CENTRALIZED DATA AGGREGATION
    // ---------------------------------------------------------
    void PackSensorData()
    {
        // All feedback data are held here, default values are zeros if data is missing.
        float theta = 0, phi = 0, psi = 0;
        float baro_alt = 0, lidar_alt = 0;
        float pX = 0, pY = 0;
        float nSV = 0, fix = 0;

        // --- IMU AGGREGATION ---
        if (imu != null)
        {
            // Note: Preserving the mapping from the original script
            // Original: theta=x, phi=z, psi=y. 
            // Standard IMU.cs output is x=Pitch, y=Roll, z=Yaw.
            // This implies the old script swapped Roll/Yaw or used a specific frame.
            // We keep the old mapping to ensure Simulink compatibility.

            theta = imu.eulerDeg.x;
            phi = imu.eulerDeg.y;
            psi = imu.eulerDeg.z;
        }

        // --- GPS AGGREGATION ---
        if (gps != null)
        {
            pX = gps.positionX;
            pY = gps.positionY;
            nSV = gps.numSV;
            fix = gps.fixFlag;
        }
        else
        {
            // Fallback to ground truth if no GPS
            pX = 0;
            pY = 0;
        }

        // --- BAROMETER AGGREGATION ---
        if (barometer != null)
        {
            baro_alt = barometer.baroAltitude_m;
        }
        else
        {
            baro_alt = 0;
        }
        // --- LIDAR AGGREGATION ---    Skipped in early debugging and testing phase
        /*
        if (lidar != null)
        {
            lidar_alt = lidar.lidar_alt;
        }
        */

        // --- Disturbance Debugging ---
        float pulseSignal = 0f;
        if (Keyboard.current != null && (Keyboard.current.pKey.isPressed || Keyboard.current.rKey.isPressed))
        {
            pulseSignal = 1f;       // Outputs 1 to Simulink when 'P' or 'R' is held
        }   

        lidar_alt = pulseSignal;    // Hijack Lidar signal with pulse signal

        // --- FILL PACKET (Order MUST match Simulink Unpack) ---
        feedbackFloats[0] = theta;
        feedbackFloats[1] = phi;
        feedbackFloats[2] = psi;

        feedbackFloats[3] = pX;
        feedbackFloats[4] = pY;
        feedbackFloats[5] = baro_alt;

        feedbackFloats[6] = lidar_alt;
        feedbackFloats[7] = positionSetpoint.x;
        feedbackFloats[8] = positionSetpoint.z;
        feedbackFloats[9] = positionSetpoint.y;

        feedbackFloats[10] = yawSetpoint;

        // --- SERIALIZATION (Float[] -> Byte[]) ---
        Buffer.BlockCopy(feedbackFloats, 0, sendBuffer, 0, sendBuffer.Length);
    }

    // ---------------------------------------------------------
    //                      PHYSICS ENGINE
    // ---------------------------------------------------------
    void ApplyMotorCommands(float[] pwm)
    {
        if (pwm == null) return;
        for (int i = 0; i < Mathf.Min(pwm.Length, motorPwm.Length); i++)
        {
            motorPwm[i] = Mathf.Clamp(pwm[i], pwmMin, pwmMax);
        }
    }

    void ApplyPhysics()
    {
        if (motorTransforms == null) return;

        for (int i = 0; i < motorTransforms.Length; i++)
        {
            float pwm = motorPwm[i];
            // Polynomial Thrust Curve: F = A*x^2 + B*x + C
            float thrustGrams = (thrustA * pwm * pwm) + (thrustB * pwm) + thrustC;
            float thrustNewtons = (thrustGrams / 1000f) * 9.81f;

            if (pwm < 10f) thrustNewtons = 0f; // Deadzone

            Vector3 force = motorTransforms[i].up * thrustNewtons;
            rb.AddForceAtPosition(force, motorTransforms[i].position);
        }
    }
}