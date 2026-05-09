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
    private float startTime;
    private bool isPlaying = false;
    private int currentIndex = 0;

    // Struct to hold pre-parsed memory
    private struct LogFrame
    {
        public float time;
        public float[] pwm;
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

        // Start at 1 to skip the CSV header line
        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] cols = lines[i].Split(',');
            if (cols.Length < 5) continue; // We need at least Time + 4 PWMs

            // CultureInfo invariant ensures correct parsing regardless of PC region settings
            if (float.TryParse(cols[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float t) &&
                float.TryParse(cols[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float m1) &&
                float.TryParse(cols[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float m2) &&
                float.TryParse(cols[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float m3) &&
                float.TryParse(cols[4], NumberStyles.Float, CultureInfo.InvariantCulture, out float m4))
            {
                logData.Add(new LogFrame { time = t, pwm = new float[] { m1, m2, m3, m4 } });
            }
        }

        if (logData.Count > 0)
        {
            Debug.Log($"[DroneFromLog] Loaded {logData.Count} log frames. Playback starting.");
            startTime = Time.time;
            isPlaying = true;
        }
    }

    void FixedUpdate()
    {
        if (!isPlaying || logData == null || currentIndex >= logData.Count || droneHub == null) return;

        // Calculate how much actual physics time has passed since we started
        float elapsedTime = Time.time - startTime;

        // Fast-forward index to match elapsed time (Temporal Synchronization)
        while (currentIndex < logData.Count - 1 && logData[currentIndex + 1].time <= elapsedTime)
        {
            currentIndex++;
        }

        // Inject the PWM from the synchronized frame into the plant
        droneHub.InjectLogPWM(logData[currentIndex].pwm);
    }
}