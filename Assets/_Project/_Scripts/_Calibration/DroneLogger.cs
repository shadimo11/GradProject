using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Globalization;
using UnityEngine;

public class DroneLogger : MonoBehaviour
{
    public enum LogVariable
    {
        Time, M1_RR, M2_RL, M3_FR, M4_FL, V_Batt,
        Pitch, Roll, Yaw, PosX, PosY, Alt, Disturbance,
        SetX, SetZ, SetY, SetYaw
    }

    [Header("Drone Hub Reference")]
    public DroneHub droneHub;

    [Header("Log Settings: Edit Before Test")]
    public string fileName = "digital_twin_log.csv";

    [Header("Dynamic Observation Matrix (P)")]
    [Tooltip("Modify the order and selection of variables to match your physical CSV log")]
    public List<LogVariable> telemetryFormat = new List<LogVariable>
    {
        // Initialized to match validation_scenario.csv exactly
        LogVariable.Time,
        LogVariable.M1_RR, LogVariable.M2_RL, LogVariable.M3_FR, LogVariable.M4_FL,
        LogVariable.V_Batt,
        LogVariable.Pitch, LogVariable.Roll, LogVariable.Yaw,
        LogVariable.Alt
    };

    private string directoryPath;
    private List<string> logBuffer;
    private StringBuilder rowBuilder;
    private bool isLogging = false;

    void Start()
    {
        directoryPath = Path.Combine(Application.dataPath, "_Project/Logs/Digital_Twin");

        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        // Pre-allocate memory to prevent GC spikes
        logBuffer = new List<string>(50000);
        rowBuilder = new StringBuilder(256);

        // Dynamically build the CSV Header based on the Inspector selection
        GenerateHeader();

        isLogging = true;
    }

    private void GenerateHeader()
    {
        rowBuilder.Clear();
        for (int i = 0; i < telemetryFormat.Count; i++)
        {
            // Rename internal Alt to "Altitude" to exactly match your physical CSV header
            string headerName = telemetryFormat[i] == LogVariable.Alt ? "Altitude" : telemetryFormat[i].ToString();

            rowBuilder.Append(headerName);
            if (i < telemetryFormat.Count - 1) rowBuilder.Append(",");
        }
        logBuffer.Add(rowBuilder.ToString());
    }

    void FixedUpdate()
    {
        if (!isLogging || droneHub == null) return;

        float[] fb = droneHub.feedbackFloats;
        if (fb == null || fb.Length < 11) return;

        rowBuilder.Clear();

        // Dynamically extract the state vector y_k based on the permutation matrix P
        for (int i = 0; i < telemetryFormat.Count; i++)
        {
            float val = 0f;
            switch (telemetryFormat[i])
            {
                // Temporal
                case LogVariable.Time: val = (float)Time.fixedTimeAsDouble; break;

                // Actuator u_k
                case LogVariable.M1_RR: val = droneHub.motorPwm[0]; break;
                case LogVariable.M2_RL: val = droneHub.motorPwm[1]; break;
                case LogVariable.M3_FR: val = droneHub.motorPwm[2]; break;
                case LogVariable.M4_FL: val = droneHub.motorPwm[3]; break;
                case LogVariable.V_Batt: val = droneHub.currentVoltage; break;

                // State z_k (From DroneHub.cs feedbackFloats matrix alignment)
                case LogVariable.Pitch: val = fb[0]; break;
                case LogVariable.Roll: val = fb[1]; break;
                case LogVariable.Yaw: val = fb[2]; break;
                case LogVariable.PosX: val = fb[3]; break;
                case LogVariable.PosY: val = fb[4]; break;
                case LogVariable.Alt: val = fb[5]; break;
                case LogVariable.Disturbance: val = fb[6]; break;
                case LogVariable.SetX: val = fb[7]; break;
                case LogVariable.SetZ: val = fb[8]; break;
                case LogVariable.SetY: val = fb[9]; break;
                case LogVariable.SetYaw: val = fb[10]; break;
            }

            // Append with deterministic format precision
            rowBuilder.Append(val.ToString("F4", CultureInfo.InvariantCulture));
            if (i < telemetryFormat.Count - 1) rowBuilder.Append(",");
        }

        logBuffer.Add(rowBuilder.ToString());
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
        Debug.Log($"[DroneLogger] Dynamic SIL Telemetry saved to {fullPath}. Total records: {logBuffer.Count - 1}");

        logBuffer.Clear();
    }
}