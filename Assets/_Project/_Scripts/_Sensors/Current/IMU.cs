using UnityEngine;

/// <summary>
/// IMUSensor.cs - Final Version for Unity 6 & BNO055 Alignment
/// Configured for: X=Forward, Y=Left, Z=Up (Right-Handed System)
/// </summary>
[DisallowMultipleComponent]
public class IMU : MonoBehaviour
{
    [Header("Sampling")]
    public float sampleRateHz = 100f;
    public float gravityMagnitude = 9.80665f;

    [Header("Noise & Bias (BNO055 Specs)")]
    [Range(0, 3)] public int calibSys = 3;
    public float rpNoiseDeg = 0.2f;
    public float yawNoiseDeg = 0.5f;
    public float linAccNoise = 0.02f;

    [Header("Outputs (BNO055 Hardware Frame)")]
    [Tooltip("Output Vector: (Pitch, Roll, Yaw)")]
    public Vector3 eulerDeg;
    [Tooltip("Output Vector: (Forward, Left, Up)")]
    public Vector3 linearAccel_mps2;

    [Header("Debug")]
    public bool logToConsole = true;
    public float logEverySeconds = 0.5f;

    [Tooltip("Drag the parent Drone root object here")]
    public Rigidbody rb;
    private Vector3 velPrevWorld;
    private float imuTimer;
    private float imuDt;
    private float logTimer;

    private RingBuffer<Vector3> eulerDelay;
    private RingBuffer<Vector3> accDelay;

    void Awake()
    {
        imuDt = 1f / Mathf.Max(1f, sampleRateHz);

        // Simulated hardware latency (15ms)
        int delaySamples = Mathf.RoundToInt(0.015f * sampleRateHz);
        eulerDelay = new RingBuffer<Vector3>(delaySamples);
        accDelay = new RingBuffer<Vector3>(delaySamples);

#if UNITY_6000_0_OR_NEWER
        velPrevWorld = (rb != null) ? rb.linearVelocity : Vector3.zero;
#else
        velPrevWorld = (rb != null) ? rb.velocity : Vector3.zero;
#endif
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        imuTimer += Time.fixedDeltaTime;
        if (imuTimer < imuDt) return;
        imuTimer -= imuDt;

        float dt = imuDt;

        // Kinematic Acceleration (Unity 6 Version Safety)
        Vector3 vNow;
#if UNITY_6000_0_OR_NEWER
        vNow = rb.linearVelocity;
#else
        vNow = rb.velocity;
#endif
        // aWorld is the pure linear acceleration (0 at hover)
        Vector3 aWorld = (vNow - velPrevWorld) / dt;
        velPrevWorld = vNow;

        // Transform to Body Frame
        Vector3 linAccBodyUnity = transform.InverseTransformDirection(aWorld);

        // Hardware Mapping (Unity LHS -> BNO055 RHS)
        // Fix for A: Using aWorld directly gives 0,0,0 when stationary
        // BNO X (Forward) = Unity Z
        // BNO Y (Left)    = -Unity X
        // BNO Z (Up)      = Unity Y
        Vector3 linAccHardware = new Vector3(linAccBodyUnity.z, -linAccBodyUnity.x, linAccBodyUnity.y);

        // Add Noise
        linAccHardware += (linAccNoise * Mathf.Lerp(5f, 1f, calibSys / 3f)) * RandGaussian3();

        // Euler Mapping & Sign Correction
        Vector3 eul = transform.rotation.eulerAngles;

        // Fix for B: Pitch-Up is Positive in BNO055 (Unity X-rotation is -ve for Pitch Up)
        float bnoPitch = -Wrap180(eul.x);

        // Fix for C & D: Mapping Roll to Y and Yaw to Z
        // Roll (Rotation around Forward/X): Unity Z-rotation. 
        // RHS Roll Right is positive (Unity Z-rotation Right is -ve)
        float bnoRoll = -Wrap180(eul.z);

        // Yaw (Rotation around Up/Z): Unity Y-rotation.
        // RHS Heading decreases Clockwise. (Unity Y-rotation Right is +ve)
        float bnoYaw = -Wrap180(eul.y);

        Vector3 eulerHardware = new Vector3(bnoPitch, bnoRoll, bnoYaw);

        // Delayed Outputs
        eulerDeg = eulerDelay.PushAndGet(eulerHardware);
        linearAccel_mps2 = accDelay.PushAndGet(linAccHardware);

        if (logToConsole && Time.time >= logTimer)
        {
            logTimer = Time.time + logEverySeconds;
            Debug.Log($"IMU | Acc: {linearAccel_mps2:F2} | Euler(P,R,Y): {eulerDeg:F2}");
        }
    }

    private float Wrap180(float angle) => Mathf.Repeat(angle + 180f, 360f) - 180f;
    private Vector3 RandGaussian3() => new Vector3(RandGaussian(), RandGaussian(), RandGaussian());
    private float RandGaussian()
    {
        float u1 = Mathf.Max(1e-6f, Random.value);
        float u2 = Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }

    private class RingBuffer<T>
    {
        private readonly T[] buf;
        private int head;
        public RingBuffer(int n) { buf = new T[Mathf.Max(1, n + 1)]; head = 0; }
        public T PushAndGet(T x) { buf[head] = x; head = (head + 1) % buf.Length; return buf[head]; }
    }
}