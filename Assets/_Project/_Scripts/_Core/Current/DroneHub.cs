using System;
using UnityEngine;
using UnityEngine.InputSystem;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class DroneHub : MonoBehaviour
{
    public enum InputMode { Controller, LogReplay }
    [Header("Operation Mode")]
    [Tooltip("Choose whether commands come from Simulink or a CSV log replay")]
    public InputMode operationMode = InputMode.Controller;

    [Header("Dependencies")]
    public TCPServer server;
    public Rigidbody rb;

    [Header("Sensors (Assign Child Objects)")]
    public IMU imu;
    public Barometer barometer;
    public Lidar lidar;
    public GPS gps;

    [Header("Battery & Propulsion Dynamics")]
    [Tooltip("The exact voltage at which thrustA, thrustB, thrustC were experimentally derived")]
    public float referenceVoltage = 12.0f;

    [Tooltip("The instantaneous voltage received from the Controller/Log")]
    public float currentVoltage = 12.0f;

    // Calculated dynamically each frame based on (V_batt / V_ref)^2
    public float voltageScalar = 1f;
    public float[] EfficiencyScalars = new float[] { 0.7761f, 0.7761f, 0.7761f, 0.7761f };

    [Header("Motors")]
    [Tooltip("Assign motor transforms in the order: RR - RL - FR - FL")]
    public Transform[] motorTransforms;
    private float motorKV = 1000f;
    [Tooltip("Rotation direction scalar (1 for CW, -1 for CCW). Mapped to RR, RL, FR, FL (QuadX Configuration)")]
    public float[] spinDirections = new float[] { 1f, -1f, -1f, 1f }; // RR, RL, FR, FL


    public float[] motorPwm; // Rendered in inspector for debugging


    private float pwmMin = 0f;
    private float pwmMax = 255f;

    [Header("Physics Constants")]
    // Coefficients for PWM to Thrust conversion at V_ref
    private float thrustA = 0.0106f;
    private float thrustB = 1.5276f;
    private float thrustC = 5.0422f;

    [Header("Manual Physics Overrides")]
    public Vector3 customCenterOfMass = new Vector3(-0.1532051f, 0.2461882f, -2.519418f);
    public Vector3 customInertiaTensor = new Vector3(0.04547f, 0.09094f, 0.04547f);

    [Header("Setpoints (Virtual Pilot)")]
    public Transform targetTransform;
    private Vector3 positionSetpoint;
    private float yawSetpoint;

    // Internal Data Packets (Publicly exposed for deterministic SIL Logger)
    public readonly float[] feedbackFloats = new float[13];  // Observation Vector z_k
    private readonly byte[] sendBuffer = new byte[52];       // 13 floats * 4 bytes

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Awake()
    {
        if (motorTransforms != null)
            motorPwm = new float[motorTransforms.Length];

        // Rigorous Body Kinematics initialization
        rb.centerOfMass = customCenterOfMass;
        rb.inertiaTensor = customInertiaTensor;
        rb.inertiaTensorRotation = Quaternion.identity;
    }

    void Update()
    {
        // Guard clause to prevent NullReferenceExceptions if models aren't assigned
        if (motorTransforms == null) return;

        // 1. Calculate Theoretical Max RPM based on instantaneous voltage
        float clampedVoltage = Mathf.Clamp(currentVoltage, 9.0f, 13.0f);
        float maxRPM = motorKV * clampedVoltage;

        // 2. Iterate through each motor and apply rotational kinematics
        for (int i = 0; i < motorTransforms.Length; i++)
        {
            // Normalize ESC PWM signal to a 0.0 - 1.0 Throttle Coefficient
            float throttle = Mathf.Clamp01(motorPwm[i] / pwmMax);

            // Calculate instantaneous RPM
            float currentRPM = throttle * maxRPM;

            // Convert RPM to Degrees per Second
            float degreesPerSecond = currentRPM * 6f;

            // Retrieve direction scalar safely
            float direction = (i < spinDirections.Length) ? spinDirections[i] : 1f;

            // Apply pure local-space rotation (around the Y-axis) tied to rendering framerate
            motorTransforms[i].Rotate(Vector3.up, direction * degreesPerSecond * Time.deltaTime, Space.Self);
        }
    }

    void FixedUpdate()
    {
        // 1. Update Setpoints
        if (targetTransform)
        {
            positionSetpoint = targetTransform.position;
            yawSetpoint = targetTransform.rotation.eulerAngles.y;
        }

        // 2. Data Ingestion: Get incoming augmented command vector u_k
        if (operationMode == InputMode.Controller && server != null && server.TryGetCommands(out float[] newCommands))
        {
            ApplySystemCommands(newCommands);
        }

        // 3. Actuation: Apply computed forces to Rigidbody
        ApplyPhysics();

        // 4. Observation: Aggregate Sensor Data & Transmit
        if (server != null)
        {
            PackSensorData();
            server.SendFeedback(sendBuffer);
        }
    }

    public void InjectLogCommands(float[] loggedCommands)
    {
        if (operationMode == InputMode.LogReplay)
        {
            ApplySystemCommands(loggedCommands);
        }
    }

    // ---------------------------------------------------------
    //                CENTRALIZED DATA AGGREGATION
    // ---------------------------------------------------------
    void PackSensorData()
    {
        float theta = 0, phi = 0, psi = 0;
        float baro_alt = 0;
        float pX = 0, pY = 0;

        // IMU AGGREGATION
        if (imu != null)
        {
            theta = imu.eulerDeg.x;
            phi = imu.eulerDeg.y;
            psi = imu.eulerDeg.z;
        }

        // GPS AGGREGATION
        if (gps != null)
        {
            pX = gps.positionX;
            pY = gps.positionY;
        }

        // BAROMETER AGGREGATION
        if (barometer != null) baro_alt = barometer.baroAltitude_m;

        // Disturbance Debugging & Lidar Pulse hijack
        float windMag = 0f;
        ContinuousWindInjector windInj = GetComponent<ContinuousWindInjector>();
        if (windInj != null) windMag = windInj.CurrentWindMagnitude;

        // 2. Fetch Discrete Disturbance Signals (+1/-1 logic)
        float pDist = 0f;
        float rDist = 0f;
        if (Keyboard.current != null)
        {
            if (Keyboard.current.fKey.isPressed) pDist = 1f;       // Forward
            else if (Keyboard.current.bKey.isPressed) pDist = -1f; // Backward

            if (Keyboard.current.lKey.isPressed) rDist = 1f;       // Left
            else if (Keyboard.current.rKey.isPressed) rDist = -1f; // Right
        }

        // MATRIX ALIGNMENT (z_k)
        feedbackFloats[0] = theta;
        feedbackFloats[1] = phi;
        feedbackFloats[2] = psi;
        feedbackFloats[3] = pX;
        feedbackFloats[4] = pY;
        feedbackFloats[5] = baro_alt;
        feedbackFloats[6] = windMag;           // New Index 6
        feedbackFloats[7] = pDist;             // New Index 7
        feedbackFloats[8] = rDist;             // New Index 8
        feedbackFloats[9] = positionSetpoint.x;  // Shifted to 9
        feedbackFloats[10] = positionSetpoint.z; // Shifted to 10
        feedbackFloats[11] = positionSetpoint.y; // Shifted to 11
        feedbackFloats[12] = yawSetpoint;

        // SERIALIZATION
        Buffer.BlockCopy(feedbackFloats, 0, sendBuffer, 0, sendBuffer.Length);
    }

    // ---------------------------------------------------------
    //                      PHYSICS ENGINE
    // ---------------------------------------------------------
    void ApplySystemCommands(float[] commands)
    {
        if (commands == null || commands.Length < 5) return;

        // Isolate PWMs
        for (int i = 0; i < Mathf.Min(4, motorPwm.Length); i++)
        {
            motorPwm[i] = Mathf.Clamp(commands[i], pwmMin, pwmMax);
        }

        // Isolate Instantaneous Voltage (Index 4)
        currentVoltage = commands[4];
    }

    void ApplyPhysics()
    {
        if (motorTransforms == null) return;

        // Boundary constraint for realistic LiPo limits (Prevents division by zero)
        float clampedVoltage = Mathf.Clamp(currentVoltage, 9.0f, 13.0f);

        // Derived voltage degradation scalar mapping
        voltageScalar = (clampedVoltage * clampedVoltage) / (referenceVoltage * referenceVoltage);

        for (int i = 0; i < motorTransforms.Length; i++)
        {
            float pwm = motorPwm[i];

            // Baseline polynomial thrust calculation
            float thrustGrams = (thrustA * pwm * pwm) + (thrustB * pwm) + thrustC;
            float thrustNewtons = (thrustGrams / 1000f) * 9.81f;

            // Apply degradation multipliers
            thrustNewtons *= voltageScalar;
            thrustNewtons *= EfficiencyScalars[i];

            if (pwm < 10f) thrustNewtons = 0f;

            Vector3 force = motorTransforms[i].up * thrustNewtons;
            rb.AddForceAtPosition(force, motorTransforms[i].position);
        }
    }


}