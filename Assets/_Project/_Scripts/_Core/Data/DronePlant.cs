using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class DronePlant : MonoBehaviour
{
    // ================= PHYSICS & MOTORS =================

    [Header("Physics")]
    public Rigidbody rb;

    [Tooltip("Transforms for each motor (position + local up = thrust direction).")]
    public Transform[] motorTransforms;

    public float[] motorPwm;

    [Tooltip("Clamp PWM commands inside this range (for safety).")]
    public float pwmMin = 0f;
    public float pwmMax = 255f;

    private float thrustA = 0.0106f;
    private float thrustB = 1.5276f;
    private float thrustC = 5.0422f;

    [Header("Controller Enable")]
    public bool controllerEnabled = true;

    // ================= SENSORS =================

    [Header("Sensors")]
    public ImuPro imu;
    public ModifiedBarometer barometer;
    public Lidar1D lidar;
    public Gps gps;

    // ================= SETPOINTS =================

    [Header("Setpoints (Unity side)")]
    public Transform targetTransform;
    public Vector3 positionSetpoint;
    public float yawSetpoint;

    // ================= NETWORKING =================

    [Header("Networking")]
    public DroneTcpServer_Test tcpServer;


    // -----------------------------------------------------
    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Awake()
    {
        if (!rb) rb = GetComponent<Rigidbody>();
        if (motorTransforms != null &&
            (motorPwm == null || motorPwm.Length != motorTransforms.Length))
        {
            motorPwm = new float[motorTransforms.Length];
        }
        // --- AUTOMATIC BALANCING ---
        Vector3 averageMotorPos = Vector3.zero;
        foreach (Transform motor in motorTransforms)
        {
            // We use localPosition because CoM is in local space
            averageMotorPos += motor.localPosition;
        }
        averageMotorPos /= motorTransforms.Length;

        // Force the Center of Mass to this exact spot
        // We usually want the vertical CoM (Y) to be slightly lower for stability,
        // but for tuning, keeping it level with motors is safest.
        rb.centerOfMass = new Vector3(averageMotorPos.x, averageMotorPos.y, averageMotorPos.z);

        Debug.Log("DronePlant: Balanced CoM to " + rb.centerOfMass);
    }

    // -----------------------------------------------------
    void FixedUpdate()
    {
        UpdateSetpointsFromTarget();

        if (tcpServer != null)
        {
            if (tcpServer.TryGetLatestPwm(out var pwmFromSim) && pwmFromSim != null)
            {
                SetMotorPwm(pwmFromSim);
            }
        }

        if (controllerEnabled)
        {
            ApplyMotorForces();
        }

        if (tcpServer != null)
        {
            DroneFeedback fb = BuildFeedback();
            tcpServer.PushFeedback(fb);
        }
    }

    // =====================================================
    //                 PWM HANDLING & ACTUATION
    // =====================================================

    public void SetMotorPwm(float[] pwmArray)
    {
        if (pwmArray == null) return;

        if (motorPwm == null || motorPwm.Length != pwmArray.Length)
        {
            motorPwm = new float[pwmArray.Length];
        }

        for (int i = 0; i < pwmArray.Length; i++)
        {
            motorPwm[i] = Mathf.Clamp(pwmArray[i], pwmMin, pwmMax);
        }
    }

    void ApplyMotorForces()
    {
        if (rb == null || motorTransforms == null || motorTransforms.Length == 0)
            return;

        int n = Mathf.Min(motorTransforms.Length, motorPwm.Length);

        for (int i = 0; i < n; i++)
        {
            Transform m = motorTransforms[i];
            if (m == null) continue;

            float pwm = Mathf.Clamp(motorPwm[i], pwmMin, pwmMax);

            float thrust = PwmToThrust(pwm);

            Vector3 force = m.up * thrust;
            rb.AddForceAtPosition(force, m.position, ForceMode.Force);
        }
    }

    public float PwmToThrust(float pwm)
    {
        if (pwm < 10f)
            return 0f;

        float x = pwm;
        float thrust_Grams = (thrustA * x * x) + (thrustB * x) + thrustC;
        float thrust_Newtons = (thrust_Grams / 1000f) * 9.81f;
        return thrust_Newtons;
    }

    // =====================================================
    //                 SETPOINTS
    // =====================================================

    void UpdateSetpointsFromTarget()
    {
        if (targetTransform == null)
            return;

        positionSetpoint = targetTransform.position;

        Vector3 euler = targetTransform.rotation.eulerAngles;
        yawSetpoint = Mathf.Repeat(euler.y + 180f, 360f) - 180f;
    }

    // =====================================================
    //                 FEEDBACK BUILDING
    // =====================================================

    public DroneFeedback BuildFeedback()
    {
        DroneFeedback fb = new DroneFeedback();

        // ---- Position (Using GPS noisy X,Y if available) ----
        if (gps != null)
        {
            fb.position = new Vector3(
                gps.positionX,
                transform.position.y,
                gps.positionY
            );
        }
        else
        {
            fb.position = transform.position;
        }

        fb.rotation = transform.rotation;
        fb.velocity = rb ? rb.linearVelocity : Vector3.zero;
        fb.angularVelocity = rb ? rb.angularVelocity : Vector3.zero;

        // ---- IMU ----
        if (imu != null)
        {
            fb.imuAccel = imu.accel_mps2;
            fb.imuGyro = imu.gyro_radps;
            fb.imuEuler = imu.eulerDeg;
            fb.imuMag = imu.mag_uT;
        }

        // ---- Barometer ----
        if (barometer != null)
        {
            fb.baroPressure_hPa = barometer.pressure_hPa;
            fb.baroRawPressure_hPa = barometer.rawPressure_hPa;
            fb.baroGeoAltitude_m = barometer.altitude_m;
            fb.baroAltitude_m = barometer.baroAltitude_m;
            fb.baroTemperature_C = barometer.temperature_C;
        }

        // ---- LiDAR ----
        if (lidar != null)
        {
            fb.lidarDistance_m = lidar.distance_m;
            fb.lidarRawDistance_m = lidar.rawDistance_m;
            fb.lidarHit = lidar.hit;
        }

        // ---- GPS raw ----
        if (gps != null)
        {
            fb.gpsNoisyX = gps.positionX;
            fb.gpsNoisyY = gps.positionY;
            fb.gpsPerfectPosition = gps.perfectPosition;
            fb.gpsNoisyPosition = gps.noisyPosition;
            fb.numSV = gps.numSV; 
            fb.fixFlag = gps.fixFlag;
        }

        // ---- Setpoints ----
        fb.setpointX = positionSetpoint.x;
        fb.setpointY = positionSetpoint.z;
        fb.setpointAlt = positionSetpoint.y;
        fb.setpointYaw = yawSetpoint;

        return fb;
    }
}

// =============================================================
//                       DRONE FEEDBACK
// =============================================================

[System.Serializable]
public class DroneFeedback
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 velocity;
    public Vector3 angularVelocity;

    public Vector3 imuAccel;
    public Vector3 imuGyro;
    public Vector3 imuEuler;
    public Vector3 imuMag;

    public float baroPressure_hPa;
    public float baroRawPressure_hPa;
    public float baroGeoAltitude_m;
    public float baroAltitude_m;
    public float baroTemperature_C;

    public float lidarDistance_m;
    public float lidarRawDistance_m;
    public bool lidarHit;

    public float gpsNoisyX;
    public float gpsNoisyY;
    public Vector3 gpsPerfectPosition;
    public Vector3 gpsNoisyPosition;
    public float numSV;
    public float fixFlag;

    public float setpointX;
    public float setpointY;
    public float setpointAlt;
    public float setpointYaw;
}
