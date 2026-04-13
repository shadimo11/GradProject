using System;
using UnityEngine;

[DisallowMultipleComponent]
public class Lidar : MonoBehaviour
{
    [Header("Ray geometry")]
    [Tooltip("The 'true' measurement origin. If null, this object's transform is used.")]
    public Transform origin;
    [Tooltip("Nominal beam direction in LOCAL space of 'origin'.")]
    public Vector3 localDirection = Vector3.down;

    [Header("Range limits")]
    [Tooltip("Minimum valid range (meters). Hits closer than this are ignored or clamped.")]
    public float minRangeMeters = 0.20f;    // 20 cm
    [Tooltip("Maximum measurement range (meters). Beyond this => no hit.")]
    public float maxRangeMeters = 8.0f;     // 8 m

    [Header("Beam model")]
    [Tooltip("Full field of view (degrees). e.g., 2 = ±1° cone.")]
    [Range(0f, 10f)] public float fovDeg = 2.0f;      // full angle
    [Tooltip("How many sub-rays per sample (>=1). Take nearest hit.")]
    [Min(1)] public int raysPerSample = 5;
    [Tooltip("If true, sub-rays are randomized each sample; else use deterministic pattern.")]
    public bool randomizeSubRays = true;

    [Header("Sampling")]
    [Tooltip("Samples per second. 0 = every FixedUpdate.")]
    public float sampleRateHz = 100f;
    [Tooltip("Which layers to collide with.")]
    public LayerMask layerMask = ~0; // everything
    [Tooltip("Gizmo line for the central ray.")]
    public bool drawGizmo = true;

    [Header("Noise & Bias")]
    [Tooltip("Additive white noise on range [m] (RMS).")]
    public float noiseStd_m = 0;
    [Tooltip("Constant bias added to range [m].")]
    public float bias_m = 0f;
    [Tooltip("Clamp noisy final reading to [min,max]?")]
    public bool clampToRange = true;

    [Header("Logging")]
    public bool logToConsole = true;
    public float logEverySeconds = 0.25f;

    // -------- Outputs --------
    [NonSerialized] public float lidar_alt = Mathf.Infinity;    // noisy final
    [NonSerialized] public float rawDistance_m = Mathf.Infinity; // nearest raw among sub-rays
    [NonSerialized] public bool hit;
    [NonSerialized] public RaycastHit lastHit;

    public event Action<Lidar> OnSample;

    System.Random rng;
    double nextEmitTime;
    double nextLogTime;

    void Awake()
    {
        rng = new System.Random();
    }

    void OnEnable()
    {
        nextEmitTime = Time.timeAsDouble; // start immediately
        nextLogTime = Time.timeAsDouble + logEverySeconds;
    }

    void FixedUpdate()
    {
        bool emitNow = sampleRateHz <= 0f || (Time.timeAsDouble + 1e-9) >= nextEmitTime;
        if (!emitNow) return;
        if (sampleRateHz > 0f) nextEmitTime = Time.timeAsDouble + (1.0 / sampleRateHz);


        // 'measurementSrc' is the TRUE sensor position (your LIDAR object with the collider).
        //    This is assigned via the public 'origin' field.
        Transform measurementSrc = origin ? origin : transform;

        // 'raycastSrc' is THIS object (LIDAR_Sensor). We will move this
        //    just outside the drone's mesh.
        Transform raycastSrc = transform;

        // The ray's DIRECTION is based on the TRUE sensor's orientation.
        Vector3 dirNominalWorld = measurementSrc.TransformDirection(localDirection).normalized;

        // The ray's STARTING POSITION is from THIS object (outside the mesh).
        Vector3 posWorld = raycastSrc.position;

        float nearest = Mathf.Infinity;
        bool anyHit = false;
        RaycastHit bestHit = new RaycastHit();

        int rays = Mathf.Max(1, raysPerSample);
        float halfAngleRad = Mathf.Deg2Rad * Mathf.Clamp(fovDeg * 0.5f, 0f, 89.9f);

        for (int i = 0; i < rays; i++)
        {
            Vector3 dirWorld = (halfAngleRad <= 0.0f)
                ? dirNominalWorld
                : SampleDirectionInCone(dirNominalWorld, halfAngleRad, randomizeSubRays ? rng : null, i, rays);

            // Raycast (thin beam); for a thick beam you can add SphereCast here instead
            if (Physics.Raycast(posWorld, dirWorld, out RaycastHit hitInfo, maxRangeMeters, layerMask, QueryTriggerInteraction.Ignore))
            {
                // Instead of using hitInfo.distance (distance from raycastSrc),
                // we calculate the distance from the TRUE sensor (measurementSrc) to the hit point.
                float d = Vector3.Distance(measurementSrc.position, hitInfo.point);

                // Ignore hits closer than minRange
                if (d < minRangeMeters) continue;

                if (d < nearest)
                {
                    nearest = d;
                    bestHit = hitInfo;
                    anyHit = true;
                }
            }
        }

        // Raw distance: nearest valid hit among sub-rays (or Infinity)
        rawDistance_m = anyHit ? nearest : Mathf.Infinity;
        hit = anyHit;
        lastHit = bestHit;

        // Noise/bias + handling no-hit as max range
        float noisy;
        if (!anyHit)
            noisy = maxRangeMeters; // typical lidar behavior: return max on no return
        else
            noisy = rawDistance_m + bias_m + RandN(noiseStd_m);

        if (clampToRange)
            noisy = Mathf.Clamp(noisy, minRangeMeters, maxRangeMeters);

        lidar_alt = noisy;

        // Draw central gizmo (optional)
        if (drawGizmo)
        {
            // The gizmo ray still starts from the raycast origin (posWorld)
            float gizmoLen = anyHit ? Vector3.Distance(posWorld, bestHit.point) : maxRangeMeters;
            Debug.DrawRay(posWorld, dirNominalWorld * gizmoLen, anyHit ? Color.green : Color.red,
                            Mathf.Max(0.01f, 1f / Mathf.Max(1f, sampleRateHz)));
        }

        OnSample?.Invoke(this);

        if (logToConsole && Time.timeAsDouble >= nextLogTime)
        {
            nextLogTime = Time.timeAsDouble + Math.Max(0.05f, logEverySeconds);
            if (anyHit)
                Debug.Log($"[Lidar1D:{name}] hit {bestHit.collider.name} @ {lidar_alt:F3} m (raw {rawDistance_m:F3})");
            else
                Debug.Log($"[Lidar1D:{name}] no hit (> {maxRangeMeters:F2} m), reading={lidar_alt:F3} m");
        }
    }

    // --- Helpers ---

    // Sample a unit direction within a cone around 'axisWorld' (uniform over solid angle).
    // If rng==null we use a small deterministic ring pattern based on index.
    Vector3 SampleDirectionInCone(Vector3 axisWorld, float halfAngleRad, System.Random rngOrNull, int index, int total)
    {
        axisWorld = axisWorld.normalized;
        // Build an orthonormal basis (axisWorld, u, v)
        Vector3 u = Vector3.Cross(axisWorld, Math.Abs(axisWorld.y) < 0.99f ? Vector3.up : Vector3.right).normalized;
        Vector3 v = Vector3.Cross(axisWorld, u);

        float cosMax = Mathf.Cos(halfAngleRad);

        float cosTheta, sinTheta, phi;
        if (rngOrNull != null)
        {
            double r1 = rngOrNull.NextDouble();
            double r2 = rngOrNull.NextDouble();
            // Uniform on cone cap: cosθ in [cosMax,1], φ in [0,2π)
            cosTheta = (float)(cosMax + (1.0 - cosMax) * r1);
            sinTheta = Mathf.Sqrt(Mathf.Max(0f, 1f - cosTheta * cosTheta));
            phi = (float)(2.0 * Math.PI * r2);
        }
        else
        {
            // Deterministic ring sampling
            float t = (total <= 1) ? 0f : (index / (float)total);
            phi = 2f * Mathf.PI * t;
            cosTheta = Mathf.Lerp(cosMax, 1f, (total <= 1) ? 1f : 0.5f + 0.5f * Mathf.Sin(2f * Mathf.PI * t));
            sinTheta = Mathf.Sqrt(Mathf.Max(0f, 1f - cosTheta * cosTheta));
        }

        return (axisWorld * cosTheta) + (u * (Mathf.Cos(phi) * sinTheta)) + (v * (Mathf.Sin(phi) * sinTheta));
    }

    float RandN(float std)
    {
        if (std <= 0f) return 0f;
        // Box–Muller
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        double r = Math.Sqrt(-2.0 * Math.Log(u1));
        double th = 2.0 * Math.PI * u2;
        return (float)(std * r * Math.Cos(th));
    }
}