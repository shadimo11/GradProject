using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;

public class DroneLogger : MonoBehaviour
{
    [Header("Drone Hub Reference")]
    [Tooltip("Required to read the injected motor PWM values, Battery Voltage, and EXACT feedback array")]
    public DroneHub droneHub;

    [Header("Log Settings: Edit Before Test")]
    public string fileName = "digital_twin_log.csv";

    private string directoryPath;
    private List<string> logBuffer;
    private bool isLogging = false;

    void Start()
    {
        directoryPath = Path.Combine(Application.dataPath, "_Project/Logs/Digital_Twin");

        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        // Pre-allocate memory to prevent GC spikes during SIL flight
        logBuffer = new List<string>(50000);

        // Header precisely mapped to [Time, u_k^T (R^5), z_k^T (R^11)]
        logBuffer.Add("Time,M1_RR,M2_RL,M3_FR,M4_FL,V_batt,Theta,Phi,Psi,PosX,PosY,BaroAlt,LidarPulse,SetX,SetZ,SetY,SetYaw");
        isLogging = true;
    }

    void FixedUpdate()
    {
        if (!isLogging || droneHub == null) return;

        // Extract Control Input Actuators
        float m1 = 0f, m2 = 0f, m3 = 0f, m4 = 0f;
        if (droneHub.motorPwm != null && droneHub.motorPwm.Length >= 4)
        {
            m1 = droneHub.motorPwm[0];
            m2 = droneHub.motorPwm[1];
            m3 = droneHub.motorPwm[2];
            m4 = droneHub.motorPwm[3];
        }

        // Extract the Augmented Input Parameter (Battery Voltage)
        float vBatt = droneHub.currentVoltage;

        // Extract State Observation (z_k) directly from the Hub to guarantee parity
        float[] fb = droneHub.feedbackFloats;

        // Guard clause to prevent IndexOutOfRange exceptions if Hub isn't initialized
        if (fb == null || fb.Length < 11) return;

        // Format string expanded to 17 parameters (Time + 5 Inputs + 11 States)
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1:F1},{2:F1},{3:F1},{4:F1},{5:F2},{6:F3},{7:F3},{8:F3},{9:F5},{10:F5},{11:F3},{12:F1},{13:F3},{14:F3},{15:F3},{16:F3}",
            Time.fixedTime,
            m1, m2, m3, m4, vBatt,             // u_k vector
            fb[0], fb[1], fb[2], fb[3], fb[4], fb[5], fb[6], fb[7], fb[8], fb[9], fb[10]); // z_k vector

        logBuffer.Add(line);
    }

    void OnDisable()
    {
        SaveLogToDisk();
    }

    private void SaveLogToDisk()
    {
        if (logBuffer == null || logBuffer.Count <= 1) return;

        string fullPath = Path.Combine(directoryPath, fileName);
        File.WriteAllLines(fullPath, logBuffer);
        Debug.Log($"[DroneLogger] SIL Telemetry saved to {fullPath}. Total discrete frames: {logBuffer.Count - 1}");

        logBuffer.Clear();
    }
}