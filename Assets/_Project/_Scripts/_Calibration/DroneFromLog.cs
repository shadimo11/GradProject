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
        if (!File.Exists(filePath)) return;

        string[] lines = File.ReadAllLines(filePath);
        logData = new List<LogFrame>(lines.Length);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] cols = lines[i].Split(',');
            if (cols.Length < 5) continue;

            if (float.TryParse(cols[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float t) &&
                float.TryParse(cols[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float m1) &&
                float.TryParse(cols[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float m2) &&
                float.TryParse(cols[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float m3) &&
                float.TryParse(cols[4], NumberStyles.Float, CultureInfo.InvariantCulture, out float m4))
            {
                // TODO: Verify your Simulink Motor Order here!
                // Assuming Simulink outputs M1=FR, M2=RL, M3=FL, M4=RR
                // Mapping to DroneHub order: RR (0), RL (1), FR (2), FL (3)
                logData.Add(new LogFrame { time = t, pwm = new float[] { m1, m2, m3, m4 } });
            }
        }

        if (logData.Count > 0)
        {
            // Capture the timestamp of the very first row to normalize the timeline
            logStartTimeOffset = logData[0].time;
            startFixedTime = Time.fixedTime;
            isPlaying = true;
        }
    }

    void FixedUpdate()
    {
        if (!isPlaying || logData == null || currentIndex >= logData.Count - 1 || droneHub == null) return;

        // Use strictly Fixed Time for physics determinism
        float elapsedPhysicsTime = Time.fixedTime - startFixedTime;

        // Normalize the CSV time so it starts at 0.0, matching the elapsed time
        while (currentIndex < logData.Count - 1)
        {
            float nextLogRelativeTime = logData[currentIndex + 1].time - logStartTimeOffset;

            if (nextLogRelativeTime <= elapsedPhysicsTime)
            {
                currentIndex++;
            }
            else
            {
                break;
            }
        }

        droneHub.InjectLogPWM(logData[currentIndex].pwm);
    }
}