using System;
using UnityEngine;

[DisallowMultipleComponent]
public class Barometer : MonoBehaviour
{
    [Header("Basic Barometer Settings")]
    [Tooltip("Sea-level pressure [hPa].")]
    public float seaLevelPressure_hPa = 1013.25f;

    [Tooltip("Sea-level temperature [C].")]
    public float seaLevelTemperature_C = 15f; // Used for ISA calc

    [Tooltip("RMS white noise on pressure [hPa].")]
    public float pressureNoiseStd_hPa = 0.5f;

    [Tooltip("Constant pressure bias [hPa].")]
    public float pressureBias_hPa = 0.0f;

    [Header("Altitude Source")]
    [Tooltip("If set, use this transform for altitude. Otherwise use this object.")]
    public Transform altitudeFrom;

    [Tooltip("Up direction in world space.")]
    public Vector3 worldUpAxis = Vector3.up;

    [Tooltip("Extra offset added to geometric altitude [m].")]
    public float altitudeOffsetM = 0f;

    [Header("Readings")]
    public float pressure_hPa;
    [Tooltip("The altitude calculated FROM the noisy pressure.")]
    public float baroAltitude_m;

    // Internal
    private System.Random rng;
    private const float L = 0.0065f;  // Temperature Lapse Rate (K/m)
    private const float R = 8.31447f; // Gas Constant
    private const float g = 9.80665f; // Gravity
    private const float M = 0.0289644f; // Molar Mass

    // Pre-calculated Exponents for ISA Model
    // Exponent = (g * M) / (R * L) ~= 5.255
    private const float Exponent = 5.25578f;
    private const float InvExponent = 1.0f / Exponent;

    void Awake()
    {
        rng = new System.Random();
    }

    void FixedUpdate()
    {
        // Get Ground Truth
        Transform src = altitudeFrom != null ? altitudeFrom : transform;
        Vector3 up = worldUpAxis.sqrMagnitude < 1e-6f ? Vector3.up : worldUpAxis.normalized;
        float trueAltitude = Vector3.Dot(src.position, up) + altitudeOffsetM;

        // Convert Truth to Temperature (Kelvin)
        // T = T0 - L * h
        float T0 = seaLevelTemperature_C + 273.15f;
        float currentTempK = T0 - (L * trueAltitude);

        // Convert Truth to Ideal Pressure (ISA Model)
        // P = P0 * (1 - L*h/T0) ^ (gM/RL)
        // Simplified: P = P0 * (Temp / T0) ^ Exponent
        float tempRatio = currentTempK / T0;
        float idealPressure = seaLevelPressure_hPa * Mathf.Pow(tempRatio, Exponent);

        // Inject Noise
        float noise = (float)NextGaussian(0.0, pressureNoiseStd_hPa);
        pressure_hPa = idealPressure + pressureBias_hPa + noise;

        // Calculate Altitude FROM the Noisy Pressure
        // h = (T0 / L) * (1 - (P / P0) ^ (1/Exponent))
        float pressureRatio = pressure_hPa / seaLevelPressure_hPa;

        // Safety check for negative pressure/NaN
        if (pressureRatio > 0)
        {
            float factor = 1.0f - Mathf.Pow(pressureRatio, InvExponent);
            baroAltitude_m = (T0 / L) * factor;
        }
    }

    // Optimization: Consider using the Box-Muller transform from previous response 
    // to avoid Sqrt/Log calls if performance is critical.
    double NextGaussian(double mean, double std)
    {
        if (std <= 0.0) return mean;
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        double r = Math.Sqrt(-2.0 * Math.Log(u1));
        double theta = 2.0 * Math.PI * u2;
        return mean + std * r * Math.Cos(theta);
    }
}