using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;

public class DroneLogger : MonoBehaviour
{
    [Header("Sensors to Log")]
    public IMU imu;
    public GPS gps;
    public Barometer barometer;

    [Header("Drone Hub Reference")]
    [Tooltip("Required to read the injected motor PWM values")]
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

        logBuffer = new List<string>(50000);

        // Updated CSV Header exactly to your requested order
        logBuffer.Add("Time,M1,M2,M3,M4,Pitch,Roll,Yaw,PosX,PosY,Altitude");
        isLogging = true;
    }

    void FixedUpdate()
    {
        if (!isLogging) return;

        // Extract sensor data safely
        Vector3 euler = imu != null ? imu.eulerDeg : Vector3.zero;
        float pX = gps != null ? gps.positionX : 0f;
        float pY = gps != null ? gps.positionY : 0f;
        float alt = barometer != null ? barometer.baroAltitude_m : 0f;

        // Extract PWM data safely
        float m1 = 0f, m2 = 0f, m3 = 0f, m4 = 0f;
        if (droneHub != null && droneHub.motorPwm != null && droneHub.motorPwm.Length >= 4)
        {
            m1 = droneHub.motorPwm[0];
            m2 = droneHub.motorPwm[1];
            m3 = droneHub.motorPwm[2];
            m4 = droneHub.motorPwm[3];
        }

        // Format string matching the new header order
        // euler.x = Pitch, euler.y = Roll, euler.z = Yaw
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1:F1},{2:F1},{3:F1},{4:F1},{5:F3},{6:F3},{7:F3},{8:F5},{9:F5},{10:F3}",
            Time.fixedTime, m1, m2, m3, m4, euler.x, euler.y, euler.z, pX, pY, alt);

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
        Debug.Log($"[DroneLogger] Telemetry saved to {fullPath}. Total records: {logBuffer.Count - 1}");

        logBuffer.Clear();
    }
}