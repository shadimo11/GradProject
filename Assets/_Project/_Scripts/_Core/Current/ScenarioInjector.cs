using UnityEngine;

[DisallowMultipleComponent]
public class ScenarioInjector : MonoBehaviour
{
    [Header("Target")]
    public Rigidbody droneRb;
    public IMU imuSensor;
    public GPS gpsSensor;
    public Barometer baroSensor;
    public Lidar lidarSensor;
    public DroneHub droneHub;

    [Header("Scenario Trigger")]
    public float injectionStartTime = 5f;
    public float injectionDuration = 20f;

    // ── Category 1: Wind ──────────────────────────────────────
    [Header("Wind Disturbance")]
    public bool enableWind = false;
    public Vector3 windForceNewtons = Vector3.zero;
    public bool enableGust = false;
    public float gustPeakForce = 0f;
    public float gustDuration = 0.5f;
    public float gustInterval = 5f;
    public bool enableOscillatingWind = false;
    public float oscillationFrequency = 0.5f;

    // ── Category 2: Hardware Degradation ─────────────────────
    [Header("Hardware Degradation")]
    [Range(0f, 1f)] public float voltageScalar = 1f;
    [Range(0f, 1f)] public float motor1EfficiencyScalar = 1f;
    [Range(0f, 1f)] public float motor2EfficiencyScalar = 1f;
    [Range(0f, 1f)] public float motor3EfficiencyScalar = 1f;
    [Range(0f, 1f)] public float motor4EfficiencyScalar = 1f;

    // ── Category 3: Sensor Degradation ───────────────────────
    [Header("Sensor Degradation")]
    public float imuNoiseMultiplier = 1f;
    public float gpsNoiseMultiplier = 1f;
    public float baroNoiseMultiplier = 1f;
    public bool enableLidarOcclusion = false;

    // ── Category 4: Mass / CoM Variation ─────────────────────
    [Header("Payload")]
    public float addedMassKg = 0f;
    public Vector3 comShiftMeters = Vector3.zero;

    // ── Internal state ────────────────────────────────────────
    private float _nominalImuNoise;
    private float _nominalGpsNoise;
    private float _nominalBaroNoise;
    private float _nominalMass;
    private Vector3 _nominalCoM;

    private float _gustTimer = 0f;
    private bool _active = false;
    private bool _restored = false;

    // Baseline noise values from sensors
    void Start()
    {
        if (imuSensor != null) _nominalImuNoise = imuSensor.linAccNoise;
        if (gpsSensor != null) _nominalGpsNoise = gpsSensor.posNoiseMeters;
        if (baroSensor != null) _nominalBaroNoise = baroSensor.pressureNoiseStd_hPa;
        if (droneRb != null)
        {
            _nominalMass = droneRb.mass;
            _nominalCoM = droneRb.centerOfMass;
        }
    }

    void FixedUpdate()
    {
        float t = Time.time;
        bool shouldBeActive = (t >= injectionStartTime) &&
                              (t <= injectionStartTime + injectionDuration);

        // Activate
        if (shouldBeActive && !_active)
        {
            _active = true;
            _restored = false;
            ApplyStaticModifications();
        }

        // Deactivate and restore
        if (!shouldBeActive && _active && !_restored)
        {
            _active = false;
            _restored = true;
            RestoreNominal();
        }

        if (!_active) return;

        // ── Wind Forces (applied every physics step) ──────────
        if (enableWind && droneRb != null)
        {
            Vector3 force = windForceNewtons;

            if (enableOscillatingWind)
            {
                float oscillation = Mathf.Sin(2f * Mathf.PI * oscillationFrequency * t);
                force *= oscillation;
            }

            droneRb.AddForce(force, ForceMode.Force);
        }

        // ── Gust Logic ────────────────────────────────────────
        if (enableGust && droneRb != null)
        {
            _gustTimer += Time.fixedDeltaTime;
            if (_gustTimer >= gustInterval)
            {
                _gustTimer = 0f;
                Vector3 gustDir = windForceNewtons.normalized;
                if (gustDir == Vector3.zero) gustDir = Vector3.right;
                droneRb.AddForce(gustDir * gustPeakForce, ForceMode.Impulse);
            }
        }
    }

    // Applied once when scenario activates
    void ApplyStaticModifications()
    {
        // Sensor noise scaling
        if (imuSensor != null)
            imuSensor.linAccNoise = _nominalImuNoise * imuNoiseMultiplier;

        if (gpsSensor != null)
            gpsSensor.posNoiseMeters = _nominalGpsNoise * gpsNoiseMultiplier;

        if (baroSensor != null)
            baroSensor.pressureNoiseStd_hPa = _nominalBaroNoise * baroNoiseMultiplier;

        // Mass and CoM
        if (droneRb != null)
        {
            droneRb.mass = _nominalMass + addedMassKg;
            droneRb.centerOfMass = _nominalCoM + comShiftMeters;
        }
    }

    void RestoreNominal()
    {
        if (imuSensor != null) imuSensor.linAccNoise = _nominalImuNoise;
        if (gpsSensor != null) gpsSensor.posNoiseMeters = _nominalGpsNoise;
        if (baroSensor != null) baroSensor.pressureNoiseStd_hPa = _nominalBaroNoise;

        if (droneRb != null)
        {
            droneRb.mass = _nominalMass;
            droneRb.centerOfMass = _nominalCoM;
        }
    }
}