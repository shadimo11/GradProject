using UnityEngine;

public class Gps : MonoBehaviour
{
    [Header("Target Position")]
    public Transform target;

    [Header("NMEA Simulation")]
    public float numSV = 8;             // Number of satellites in view (for info only, not used in noise model)
    public float fixFlag = 1;           // 0=No Fix, 1=2D Fix, 2=3D Fix (for info only, not used in noise model)

    [Header("Sampling")]
    [Tooltip("Samples per second (GPS update rate).")]
    public float sampleRateHz = 10f;

    [Header("GPS Error")]
    [Tooltip("Maximum GPS error radius in meters.")]
    public float errorRadius_m = 5f;

    [Header("Debug Outputs")]
    public Vector3 perfectPosition;   // target's real position
    public Vector3 noisyPosition;     // GPS noisy reading
    public float positionX;           // output X
    public float positionY;           // output Y (using Z)

    float nextSampleTime;

    void Awake()
    {
        nextSampleTime = Time.time;
    }

    void Update()
    {
        if (target == null) return;

        if (Time.time >= nextSampleTime)
        {
            nextSampleTime += 1f / Mathf.Max(0.001f, sampleRateHz);
            SampleGPS();
        }
    }

    void SampleGPS()
    {
        // 1) perfect position = where the drone SHOULD be
        perfectPosition = target.position;

        // 2) GPS white noise (uniform in sphere)
        Vector3 noise = Random.insideUnitSphere * errorRadius_m;

        // 3) final noisy GPS reading
        noisyPosition = perfectPosition + noise;

        // 4) Output X,Y only (Z becomes Y for flat map)
        positionX = noisyPosition.x;
        positionY = noisyPosition.z;
    }
}
