using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;
public class DroneFromLog : MonoBehaviour
{
    [Header("Dependencies")]
    public DroneHub droneHub;

    [Header("Log Settings: Edit Before Test")]
    public string fileName = "physical_model_log.csv";

    private string filePath;
    private float startFixedTime;
    private bool isPlaying = false;
    private int currentIndex = 0;
    private float logStartTimeOffset = 0f;

    private struct LogFrame
    {
        public float time;
        public float[] commands;
    }

    private List<LogFrame> logData;

    void Start()
    {
        filePath = Path.Combine(Application.dataPath, "_Project/Logs/Physical_Model", fileName);
        LoadAndParseLog();
    }

    void LoadAndParseLog()
    {
        if (!File.Exists(filePath)) return;

        string[] lines = File.ReadAllLines(filePath);
        logData = new List<LogFrame>(lines.Length);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] cols = lines[i].Split(',');

            // Require at least 6 columns: Time, M1, M2, M3, M4, V_Batt
            if (cols.Length < 6) continue;

            // Strict parsing of the augmented control vector
            if (float.TryParse(cols[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float t) &&
                float.TryParse(cols[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float m1) &&
                float.TryParse(cols[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float m2) &&
                float.TryParse(cols[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float m3) &&
                float.TryParse(cols[4], NumberStyles.Float, CultureInfo.InvariantCulture, out float m4) &&
                float.TryParse(cols[5], NumberStyles.Float, CultureInfo.InvariantCulture, out float vBatt)) // 5th Dimension
            {
                // Injecting the complete R^5 vector
                logData.Add(new LogFrame
                {
                    time = t,
                    commands = new float[] { m1, m2, m3, m4, vBatt }
                });
            }
        }

        if (logData.Count > 0)
        {
            logStartTimeOffset = logData[0].time;
            startFixedTime = Time.fixedTime;
            isPlaying = true;

            // --- NEW: RIGOROUS INITIALIZATION ---
            // Pre-load the t=0 boundary conditions directly into the plant
            if (droneHub != null)
            {
                droneHub.InjectLogCommands(logData[0].commands);
            }
        }

    }

    void FixedUpdate()
    {
        if (!isPlaying || logData == null || currentIndex >= logData.Count - 1 || droneHub == null) return;

        float elapsedPhysicsTime = Time.fixedTime - startFixedTime;

        // Define a half-frame tolerance to prevent floating-point drift
        float epsilon = Time.fixedDeltaTime * 0.5f;

        while (currentIndex < logData.Count - 1)
        {
            float nextLogRelativeTime = logData[currentIndex + 1].time - logStartTimeOffset;

            // NEW: Evaluate with epsilon window
            if (nextLogRelativeTime <= (elapsedPhysicsTime + epsilon))
            {
                currentIndex++;
            }
            else
            {
                break;
            }
        }

        droneHub.InjectLogCommands(logData[currentIndex].commands);
    }
}