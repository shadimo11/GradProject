using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;

public class DroneFromLog : MonoBehaviour
{
    [Header("Dependencies")]
    public DroneHub droneHub;
    public DroneLogger droneLogger;

    [Header("Shadow Test Settings")]
    public string fileName = "physical_model_log.csv";
    [Tooltip("Distance in meters before a State-Reset is triggered")]
    public float positionErrorTolerance = 1.0f;
    [Tooltip("Difference in degrees before a State-Reset is triggered")]
    public float rotationErrorTolerance = 5.0f;

    private string filePath;
    private float startTime;
    private bool isPlaying = false;
    private int currentIndex = 0;

    // Expanded struct to hold the 11-column input
    private struct LogFrame
    {
        public float time;
        public float[] pwm;
        public Vector3 refPos;
        public Vector3 refRot;
    }

    private List<LogFrame> logData;

    void Start()
    {
        filePath = Path.Combine(Application.dataPath, "_Project/Logs/Physical_Model", fileName);
        LoadAndParseLog();
    }

    void LoadAndParseLog()
    {
        if (!File.Exists(filePath))
        {
            Debug.LogError($"[DroneFromLog] Error: Log file not found at {filePath}");
            return;
        }

        string[] lines = File.ReadAllLines(filePath);
        logData = new List<LogFrame>(lines.Length);

        // Expects 11 Columns: Time, 4xPWM, 3xPos, 3xRot
        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] cols = lines[i].Split(',');
            if (cols.Length < 11) continue;

            if (float.TryParse(cols[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float t) &&
                float.TryParse(cols[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float m1) &&
                float.TryParse(cols[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float m2) &&
                float.TryParse(cols[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float m3) &&
                float.TryParse(cols[4], NumberStyles.Float, CultureInfo.InvariantCulture, out float m4) &&
                float.TryParse(cols[5], NumberStyles.Float, CultureInfo.InvariantCulture, out float px) &&
                float.TryParse(cols[6], NumberStyles.Float, CultureInfo.InvariantCulture, out float py) &&
                float.TryParse(cols[7], NumberStyles.Float, CultureInfo.InvariantCulture, out float pz) &&
                float.TryParse(cols[8], NumberStyles.Float, CultureInfo.InvariantCulture, out float rx) &&
                float.TryParse(cols[9], NumberStyles.Float, CultureInfo.InvariantCulture, out float ry) &&
                float.TryParse(cols[10], NumberStyles.Float, CultureInfo.InvariantCulture, out float rz))
            {
                logData.Add(new LogFrame
                {
                    time = t,
                    pwm = new float[] { m1, m2, m3, m4 },
                    refPos = new Vector3(px, py, pz),
                    refRot = new Vector3(rx, ry, rz)
                });
            }
        }

        if (logData.Count > 0)
        {
            Debug.Log($"[DroneFromLog] Loaded {logData.Count} log frames. Shadow Test starting.");
            startTime = Time.time;
            isPlaying = true;
        }
    }

    void FixedUpdate()
    {
        if (!isPlaying || logData == null || currentIndex >= logData.Count || droneHub == null) return;

        float elapsedTime = Time.time - startTime;

        while (currentIndex < logData.Count - 1 && logData[currentIndex + 1].time <= elapsedTime)
        {
            currentIndex++;
        }

        LogFrame currentFrame = logData[currentIndex];

        // 1. Capture DT State BEFORE resetting
        Vector3 dtPos = droneHub.rb.position;
        Vector3 dtRot = droneHub.rb.rotation.eulerAngles;

        // 2. Calculate Residual Errors
        float posError = Vector3.Distance(currentFrame.refPos, dtPos);
        float rotError = Quaternion.Angle(droneHub.rb.rotation, Quaternion.Euler(currentFrame.refRot));

        // 3. Evaluate Tolerances
        int trigger = 0;
        if (posError > positionErrorTolerance || rotError > rotationErrorTolerance)
        {
            trigger = 1;
            ApplyStateReset(currentFrame);
        }

        // 4. Log the Frame (Send the DT state BEFORE the reset so we can analyze the error)
        if (droneLogger != null)
        {
            droneLogger.LogShadowTestFrame(
                elapsedTime, currentFrame.pwm,
                currentFrame.refPos, currentFrame.refRot,
                dtPos, dtRot,
                posError, rotError, trigger
            );
        }

        // 5. Inject PWM for the next physics tick
        droneHub.InjectLogPWM(currentFrame.pwm);
    }

    private void ApplyStateReset(LogFrame targetFrame)
    {
        // Snap Transform
        droneHub.rb.position = targetFrame.refPos;
        droneHub.rb.rotation = Quaternion.Euler(targetFrame.refRot);

        // Manually calculate required velocities to prevent instant drift post-reset
        if (currentIndex > 0)
        {
            LogFrame prevFrame = logData[currentIndex - 1];
            float deltaTime = targetFrame.time - prevFrame.time;

            if (deltaTime > 0.001f)
            {
                // Linear Velocity Derivation (Updated for Unity 2023.3+)
                Vector3 linearVel = (targetFrame.refPos - prevFrame.refPos) / deltaTime;
                droneHub.rb.linearVelocity = linearVel;

                // Angular Velocity Derivation
                Quaternion prevRotQ = Quaternion.Euler(prevFrame.refRot);
                Quaternion currRotQ = Quaternion.Euler(targetFrame.refRot);
                Quaternion deltaRot = currRotQ * Quaternion.Inverse(prevRotQ);

                deltaRot.ToAngleAxis(out float angle, out Vector3 axis);

                // Handle Unity's ToAngleAxis edge cases (zero rotation returns infinity axis)
                if (float.IsInfinity(axis.x) || float.IsNaN(axis.x)) axis = Vector3.zero;

                // Convert angle to Radians for Rigidbody.angularVelocity
                Vector3 angularVel = axis * (angle * Mathf.Deg2Rad) / deltaTime;
                droneHub.rb.angularVelocity = angularVel;
            }
        }
    }
}