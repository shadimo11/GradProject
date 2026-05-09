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

    [Header("Log Settings: Edit Before Test")]
    public string fileName = "digital_twin_log.csv";

    private string directoryPath;
    private List<string> logBuffer;
    private bool isLogging = false;

    void Start()
    {
        // Routes to the requested specific folder inside the Editor
        directoryPath = Path.Combine(Application.dataPath, "_Project/Logs/Digital_Twin");

        // Ensure the directory exists
        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        // Pre-allocate memory for ~500 seconds of flight at 100Hz to prevent mid-flight GC allocation
        logBuffer = new List<string>(50000);
        logBuffer.Add("Time,Roll,Pitch,Yaw,PosX,PosY,Altitude"); // CSV Headers
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

        // CultureInfo.InvariantCulture ensures decimals are logged as '.' even on Arabic/European Windows locales
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1:F3},{2:F3},{3:F3},{4:F5},{5:F5},{6:F3}",
            Time.fixedTime, euler.x, euler.y, euler.z, pX, pY, alt);

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