using UnityEngine;

/// <summary>
/// imu2.cs - Modified for ENU (Z-Up) Output
/// Outputs:
/// - Linear Accel: (x=East, y=North, z=Up)
/// - Euler Angles: (x=Pitch, y=Roll, z=Yaw)
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class imu2 : MonoBehaviour
{
    [Header("Sampling")]
    public float sampleRateHz = 100f;
    public float gravity = 9.80665f;

    [Header("Calibration State (Auto)")]
    [Range(0, 3)] public int calibSys = 0;

    [Header("Base Noise (at Calib=3)")]
    public float rpNoiseDeg = 0.2f;
    public float yawNoiseDeg = 0.5f;
    public float linAccNoise = 0.02f;

    [Header("Outputs (ENU Configuration)")]
    [Tooltip("Vector3(Pitch, Roll, Yaw) where Z is Yaw (Up)")]
    public Vector3 eulerDeg;
    [Tooltip("Vector3(East, North, Up)")]
    public Vector3 linearAccel_mps2;

    [Header("Debug")]
    public bool logToConsole = true;
    public float logEverySeconds = 0.5f;

    // ===== Internals =====
    Rigidbody rb;
    Vector3 velPrevWorld;

    float imuTimer;
    float imuDt;
    float logTimer;

    // Auto-calibration helpers
    float gyroTimer;
    float totalRotation;
    float lastYaw;
    int orientationPoints;
    Vector3 lastCheckUp;

    Vector3 currentWander;
    Vector3 wanderTarget;

    Ring<Vector3> eulerDelay;
    Ring<Vector3> accDelay;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        imuDt = 1f / Mathf.Max(1f, sampleRateHz);

        int delaySamples = Mathf.RoundToInt((15f / 1000f) * sampleRateHz);
        eulerDelay = new Ring<Vector3>(delaySamples);
        accDelay = new Ring<Vector3>(delaySamples);

        lastYaw = transform.eulerAngles.y;

#if UNITY_6000_0_OR_NEWER
        velPrevWorld = rb.linearVelocity;
#else
        velPrevWorld = rb.velocity;
#endif
    }

    void FixedUpdate()
    {
        imuTimer += Time.fixedDeltaTime;
        if (imuTimer < imuDt) return;
        imuTimer -= imuDt;

        float dt = imuDt;

        UpdateAutoCalibration(dt);

        float calibMult = Mathf.Lerp(5f, 1f, calibSys / 3f);

        // ------------------------------------------------------------
        // 1. Calculate Linear Acceleration (Unity Frame: Y-Up)
        // ------------------------------------------------------------
#if UNITY_6000_0_OR_NEWER
        Vector3 vNow = rb.linearVelocity;
#else
        Vector3 vNow = rb.velocity;
#endif
        Vector3 aWorld = (vNow - velPrevWorld) / dt;
        velPrevWorld = vNow;

        // Remove Gravity (Unity Gravity is on Y-axis)
        Vector3 linAccWorld = aWorld - new Vector3(0f, -gravity, 0f);
        Vector3 linAccBodyUnity = transform.InverseTransformDirection(linAccWorld);

        // Add Noise (Unity Frame)
        float vibration = linAccBodyUnity.magnitude * 0.05f * calibMult;
        linAccBodyUnity += ((linAccNoise * calibMult) + vibration) * RandN3();


        // ------------------------------------------------------------
        // 2. Calculate Euler Angles (Unity Frame)
        // ------------------------------------------------------------
        Vector3 eulTrue = transform.rotation.eulerAngles;
        // Unity Convention: .x=Pitch, .y=Yaw, .z=Roll

        if (Random.value < 0.02f)
            wanderTarget = RandN3() * 0.5f * calibMult;
        currentWander = Vector3.Lerp(currentWander, wanderTarget, dt);

        // Apply noise to raw Unity angles
        float rawPitch = Wrap180(eulTrue.x + currentWander.x + (rpNoiseDeg * calibMult + vibration) * RandN());
        float rawYaw = Mathf.Repeat(eulTrue.y + currentWander.y + (yawNoiseDeg * calibMult + vibration) * RandN(), 360f);
        float rawRoll = Wrap180(eulTrue.z + currentWander.z + (rpNoiseDeg * calibMult + vibration) * RandN());


        // ------------------------------------------------------------
        // 3. Convert to ENU (East-North-Up)
        // ------------------------------------------------------------
        // Unity (Y-Up, Z-Fwd, X-Right) -> ENU (Z-Up, Y-North, X-East)
        // Transformation: X->X, Y->Z, Z->Y

        // ACCEL: Swap Y and Z
        Vector3 linAccENU = new Vector3(linAccBodyUnity.x, linAccBodyUnity.z, linAccBodyUnity.y);

        // EULER: Map to (Pitch, Roll, Yaw) based on axis swap
        // X (Unity Pitch) -> X (ENU Pitch)
        // Z (Unity Roll)  -> Y (ENU Roll)
        // Y (Unity Yaw)   -> Z (ENU Yaw)
        Vector3 eulerENU = new Vector3(rawPitch, rawRoll, rawYaw);


        // ------------------------------------------------------------
        // 4. Output to Buffers
        // ------------------------------------------------------------
        eulerDeg = eulerDelay.PushAndGet(eulerENU);
        linearAccel_mps2 = accDelay.PushAndGet(linAccENU);

        if (logToConsole)
        {
            logTimer += dt;
            if (logTimer >= logEverySeconds)
            {
                logTimer = 0f;
                Debug.Log(
                    $"IMU2 (ENU) | Calib={calibSys} | " +
                    $"Euler(P,R,Y)=({eulerDeg.x:F2},{eulerDeg.y:F2},{eulerDeg.z:F2}) | " +
                    $"Acc(E,N,U)=({linearAccel_mps2.x:F3},{linearAccel_mps2.y:F3},{linearAccel_mps2.z:F3})"
                );
            }
        }
    }

    void UpdateAutoCalibration(float dt)
    {
        if (rb.angularVelocity.magnitude < 0.1f)
            gyroTimer += dt;
        else
            gyroTimer = 0f;

        float currentYaw = transform.eulerAngles.y;
        totalRotation += Mathf.Abs(Mathf.DeltaAngle(currentYaw, lastYaw));
        lastYaw = currentYaw;

        if (Vector3.Angle(transform.up, lastCheckUp) > 20f || lastCheckUp == Vector3.zero)
        {
            orientationPoints++;
            lastCheckUp = transform.up;
            if (logToConsole) Debug.Log($"<color=green>IMU2: New Orientation Point ({orientationPoints}/4)</color>");
        }

        int g = (gyroTimer >= 2.0f) ? 1 : 0;
        int m = (totalRotation >= 360f) ? 1 : 0;
        int a = (orientationPoints >= 4) ? 1 : 0;

        calibSys = g + m + a;
    }

    static float Wrap180(float deg)
    {
        return Mathf.Repeat(deg + 180f, 360f) - 180f;
    }

    static Vector3 RandN3()
    {
        return new Vector3(RandN(), RandN(), RandN());
    }

    static float RandN()
    {
        float u1 = Mathf.Clamp01(Random.value);
        float u2 = Mathf.Clamp01(Random.value);
        return Mathf.Sqrt(-2f * Mathf.Log(Mathf.Max(1e-6f, u1)))
             * Mathf.Cos(2f * Mathf.PI * u2);
    }

    // ===== Ring buffer =====
    public class Ring<T>
    {
        readonly T[] buf;
        int head;

        public Ring(int n)
        {
            buf = new T[Mathf.Max(1, n + 1)];
            head = 0;
        }

        public T PushAndGet(T x)
        {
            buf[head] = x;
            head = (head + 1) % buf.Length;
            return buf[head];
        }
    }
}