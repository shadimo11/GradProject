using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;

public class DroneLogger : MonoBehaviour
{
    [Header("Log Settings")]
    public string fileName = "digital_twin_shadow_log.csv";

    private string directoryPath;
    private List<string> logBuffer;

    void Start()
    {
        directoryPath = Path.Combine(Application.dataPath, "_Project/Logs/Digital_Twin");

        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        logBuffer = new List<string>(500000);
        // The agreed 20-Column Schema
        logBuffer.Add("Timestamp,PWM_FL,PWM_FR,PWM_RL,PWM_RR,Ref_PosX,Ref_PosY,Ref_PosZ,Ref_RotX,Ref_RotY,Ref_RotZ,DT_PosX,DT_PosY,DT_PosZ,DT_RotX,DT_RotY,DT_RotZ,Pos_Error,Rot_Error,Reset_Trigger");
    }

    // Called precisely by DroneFromLog.cs to ensure data synchronization
    public void LogShadowTestFrame(float time, float[] pwm, Vector3 refPos, Vector3 refRot, Vector3 dtPos, Vector3 dtRot, float posErr, float rotErr, int trigger)
    {
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3}," +                               // Time
            "{1:F0},{2:F0},{3:F0},{4:F0}," +          // PWM
            "{5:F4},{6:F4},{7:F4}," +                 // Ref Pos
            "{8:F3},{9:F3},{10:F3}," +                // Ref Rot
            "{11:F4},{12:F4},{13:F4}," +              // DT Pos
            "{14:F3},{15:F3},{16:F3}," +              // DT Rot
            "{17:F4},{18:F3}," +                      // Errors
            "{19}",                                   // Trigger
            time,
            pwm[0], pwm[1], pwm[2], pwm[3],
            refPos.x, refPos.y, refPos.z,
            refRot.x, refRot.y, refRot.z,
            dtPos.x, dtPos.y, dtPos.z,
            dtRot.x, dtRot.y, dtRot.z,
            posErr, rotErr, trigger);

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
        Debug.Log($"[DroneLogger] Shadow Test Log saved to {fullPath}. Total records: {logBuffer.Count - 1}");

        logBuffer.Clear();
    }
}