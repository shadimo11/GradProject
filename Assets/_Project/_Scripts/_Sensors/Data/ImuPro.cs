using System;
using UnityEngine;

/// IMU sensor with MPU-9250-like noise characteristics.
/// - Accelerometer: bias + optional slow drift + white noise
/// - Gyroscope:     bias + optional slow drift + white noise
/// - Magnetometer:  constant bias + white noise
///
/// Outputs:
///   accel_mps2  (body frame specific force)
///   gyro_radps  (body frame angular rate)
///   mag_uT      (body frame magnetic field)
///   worldQuat   (world orientation)
///   eulerDeg    (wrapped -180..180)
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ImuPro : MonoBehaviour
{
    [Header("Sampling")]
    [Tooltip("Nominal sample rate (used for noise density ? sigma).")]
    public float sampleRateHz = 100f;       // suggested 100 200 Hz
    [Tooltip("Gravity magnitude [m/s ].")]
    public float gravity = 9.80665f;

    // ================== ACCELEROMETER ================== //
    [Header("Accelerometer (MPU9250-like)")]
    [Tooltip("Noise density (m/s )/?Hz. MPU9250 ? 0.00294.")]
    public float accNoiseDensity = 0.00294f;

    [Tooltip("Initial accelerometer bias (m/s ).")]
    public Vector3 accBias0 = Vector3.zero;

    [Tooltip("Enable slow bias drift for accelerometer.")]
    public bool enableAccDrift = true;

    [Tooltip("Bias RMS amplitude (m/s ). Controls how big the drift can be.")]
    public float accBiasSigma = 0.03f;

    [Tooltip("Bias correlation time (s). Larger = slower drift. 300s is realistic.")]
    public float accBiasTau = 300f;

    // ================== GYROSCOPE ================== //
    [Header("Gyroscope (MPU9250-like)")]
    [Tooltip("Noise density (rad/s)/?Hz. MPU9250 gyro ? 0.0001745.")]
    public float gyroNoiseDensity = 0.0001745f;

    [Tooltip("Initial gyroscope bias (rad/s).")]
    public Vector3 gyroBias0 = Vector3.zero;

    [Tooltip("Enable slow bias drift for gyroscope.")]
    public bool enableGyroDrift = true;

    [Tooltip("Bias RMS amplitude (rad/s).")]
    public float gyroBiasSigma = 0.0025f;

    [Tooltip("Bias correlation time (s). 500s = very slow drift.")]
    public float gyroBiasTau = 500f;

    // ================== MAGNETOMETER ================== //
    [Header("Magnetometer (simple)")]
    [Tooltip("Earth magnetic field in WORLD frame [microTesla].")]
    public Vector3 earthMagWorld_uT = new Vector3(22f, 0f, 45f);

    [Tooltip("Constant hard-iron bias of magnetometer (uT).")]
    public Vector3 magBias0 = Vector3.zero;

    [Tooltip("Magnetometer white noise STD [microTesla].")]
    public float magNoiseStd_uT = 0.3f;

    // ================== OUTPUTS ================== //
    [Header("Outputs (read-only)")]
    public Vector3 accel_mps2;     // body frame specific force
    public Vector3 gyro_radps;     // body frame angular rate
    public Vector3 mag_uT;         // body frame magnetic field

    public Quaternion worldQuat;   // world orientation
    public Vector3 eulerDeg;       // roll, pitch, yaw in degrees (-180..180)

    // ================== INTERNAL STATE ================== //
    Rigidbody rb;
    Vector3 prevVel;
    float dt;

    System.Random rng;

    Vector3 accBiasState;
    Vector3 gyroBiasState;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        prevVel = rb.linearVelocity;

        rng = new System.Random();

        accBiasState = accBias0;
        gyroBiasState = gyroBias0;

        if (sampleRateHz <= 0f && Time.fixedDeltaTime > 0f)
        {
            sampleRateHz = 1f / Time.fixedDeltaTime;
        }
    }

    void FixedUpdate()
    {
        dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);

        // ========== TRUE PHYSICS ========== //
        Vector3 vNow = rb.linearVelocity;
        Vector3 aWorld = (vNow - prevVel) / dt;   // world-frame acceleration
        prevVel = vNow;

        Vector3 g = new Vector3(0f, -gravity, 0f);
        Vector3 specificForceWorld = aWorld - g;  // remove gravity (specific force)

        // Transform to body frame
        Vector3 fBody = transform.InverseTransformDirection(specificForceWorld);
        Vector3 gyroTrueBody = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 magBody = transform.InverseTransformDirection(earthMagWorld_uT);

        // ========== BIAS DRIFT (Gauss-Markov) ========== //
        // Accelerometer bias
        if (enableAccDrift && accBiasTau > 0f)
        {
            float phiAcc = Mathf.Exp(-dt / accBiasTau);
            float accW = accBiasSigma * Mathf.Sqrt(Mathf.Max(0f, 1f - phiAcc * phiAcc));
            accBiasState = phiAcc * accBiasState + accW * RandN3();
        }
        else
        {
            // no drift: bias ???? = accBias0
            accBiasState = accBias0;
        }

        // Gyroscope bias
        if (enableGyroDrift && gyroBiasTau > 0f)
        {
            float phiGyro = Mathf.Exp(-dt / gyroBiasTau);
            float gyroW = gyroBiasSigma * Mathf.Sqrt(Mathf.Max(0f, 1f - phiGyro * phiGyro));
            gyroBiasState = phiGyro * gyroBiasState + gyroW * RandN3();
        }
        else
        {
            // no drift: bias ???? = gyroBias0
            gyroBiasState = gyroBias0;
        }

        // ========== WHITE NOISE PER SAMPLE ========== //
        // ? = noiseDensity * sqrt(BW), with BW ? sampleRate/2
        float BW = Mathf.Max(sampleRateHz * 0.5f, 1f); // avoid 0
        float accSigma = accNoiseDensity * Mathf.Sqrt(BW);
        float gyroSigma = gyroNoiseDensity * Mathf.Sqrt(BW);

        Vector3 accNoisy = fBody + accBiasState + accSigma * RandN3();
        Vector3 gyroNoisy = gyroTrueBody + gyroBiasState + gyroSigma * RandN3();
        Vector3 magNoisy = magBody + magBias0 + magNoiseStd_uT * RandN3();

        // ========== OUTPUTS ========== //
        accel_mps2 = accNoisy;
        gyro_radps = gyroNoisy;
        mag_uT = magNoisy;

        worldQuat = transform.rotation;
        Vector3 e = transform.rotation.eulerAngles;
        eulerDeg = new Vector3(
            Wrap180(e.x),
            Wrap180(e.y),
            Wrap180(e.z)
        );
    }

    // ================== UTILS ================== //
    static float Wrap180(float a)
    {
        return Mathf.Repeat(a + 180f, 360f) - 180f;
    }

    Vector3 RandN3()
    {
        return new Vector3(RandN(), RandN(), RandN());
    }

    float RandN()
    {
        // Box-Muller
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
