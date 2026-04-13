using UnityEngine;

public class Script_Manager : MonoBehaviour
{
    [Header("Drone Sensors")]
    public IMU IMU;
    public Barometer Barometer;
    public Lidar Lidar1D;
    public GPS GPS;

    [Header("Visuals")]
    public CameraFollow Camera_Follower;

    [Header("Real-Time Telemetry Dashboard")]
    [Tooltip("Live sensor data updates here during Play mode.")]
    [TextArea(15, 20)]
    public string liveSensorData;

    void Update()
    {
        // Only update the string if we are in Play mode to save resources
        if (!Application.isPlaying) return;

        UpdateTelemetryDashboard();
    }

    private void UpdateTelemetryDashboard()
    {
        string data = "=== DRONE DIGITAL TWIN TELEMETRY ===\n\n";

        // IMU Data
        if (IMU != null)
        {
            data += "[IMU - BNO055]\n";
            data += $"Euler (P, R, Y): {IMU.eulerDeg.x:F2}°, {IMU.eulerDeg.y:F2}°, {IMU.eulerDeg.z:F2}°\n";
            data += $"Lin. Accel: {IMU.linearAccel_mps2.x:F2}, {IMU.linearAccel_mps2.y:F2}, {IMU.linearAccel_mps2.z:F2} m/s²\n\n";
        }
        else data += "[IMU] Not Assigned\n\n";

        // Barometer Data
        if (Barometer != null)
        {
            data += "[BAROMETER - BMP280]\n";
            data += $"Pressure: {Barometer.pressure_hPa:F2} hPa\n";
            data += $"Altitude (Baro): {Barometer.baroAltitude_m:F2} m\n\n";
        }
        else data += "[BAROMETER] Not Assigned\n\n";

        // Lidar Data
        if (Lidar1D != null)
        {
            data += "[LIDAR - TF-Luna]\n";
            data += $"Altitude (Lidar): {Lidar1D.lidar_alt:F2} m\n";
            data += $"Raw Target Dist: {Lidar1D.rawDistance_m:F2} m | Hit: {Lidar1D.hit}\n\n";
        }
        else data += "[LIDAR] Not Assigned\n\n";

        // GPS Data
        if (GPS != null)
        {
            data += "[GPS - NEO-M8N]\n";
            data += $"Position (X, Y): {GPS.positionX:F2}, {GPS.positionY:F2}\n";
            data += $"Satellites (nSV): {GPS.numSV} | Fix: {(GPS.fixFlag > 0 ? "Yes" : "No")}\n";
        }
        else data += "[GPS] Not Assigned\n";

        liveSensorData = data;
    }
}