using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

[DisallowMultipleComponent]
public class DroneTcpServer_Test : MonoBehaviour
{
    [Header("TCP Ports")]
    [SerializeField] private int pwmPort = 9000;      // Simulink -> Unity (4 floats)
    [SerializeField] private int feedbackPort = 9001; // Unity -> Simulink (10 floats)

    [SerializeField] private bool autoStart = true;
    [SerializeField] private bool debugLog = true;

    // Latest PWM received from Simulink
    float[] latestPwm = null;
    bool hasNewPwm = false;
    readonly object pwmLock = new object();

    // Latest feedback snapshot to send to Simulink
    FeedbackSnapshot latestFeedback;
    readonly object feedbackLock = new object();

    Thread pwmThread;
    Thread feedbackThread;
    volatile bool running;

    void Start()
    {
        if (autoStart)
            StartServer();
    }

    void OnDestroy()
    {
        StopServer();
    }

    // =========================== PUBLIC API ===========================

    /// <summary>
    /// Called from DronePlant each physics step.
    /// Copies real sensor values & setpoints into a feedback snapshot.
    /// </summary>
    public void PushFeedback(DroneFeedback fb)
    {
        if (fb == null) return;

        FeedbackSnapshot s = new FeedbackSnapshot();

        // IMU Euler values
        s.theta = fb.imuEuler.x;
        s.phi = fb.imuEuler.z;
        s.psi = fb.imuEuler.y;

        // Position & altitude
        s.posX = fb.position.x;
        s.posY = fb.position.z;
        s.alt = fb.baroAltitude_m;

        // Setpoints
        s.spX = fb.setpointX;
        s.spY = fb.setpointY;
        s.spAlt = fb.setpointAlt;
        s.spYaw = fb.setpointYaw;

        // NMEA Values
        s.numSV = fb.numSV;
        s.fixFlag = fb.fixFlag;

        lock (feedbackLock)
        {
            latestFeedback = s;
        }
    }

    /// <summary>
    /// Called by DronePlant to fetch latest PWM commands from Simulink.
    /// Returns true if new data was received.
    /// </summary>
    public bool TryGetLatestPwm(out float[] pwm)
    {
        lock (pwmLock)
        {
            if (!hasNewPwm || latestPwm == null)
            {
                pwm = null;
                return false;
            }

            pwm = (float[])latestPwm.Clone();
            hasNewPwm = false;
            return true;
        }
    }

    /// <summary>
    /// Starts both listener threads for PWM and Feedback.
    /// </summary>
    public void StartServer()
    {
        if (running) return;

        running = true;

        pwmThread = new Thread(PwmListenerLoop) { IsBackground = true };
        feedbackThread = new Thread(FeedbackListenerLoop) { IsBackground = true };

        pwmThread.Start();
        feedbackThread.Start();

        if (debugLog)
            Debug.Log($"[DroneTcpServer_Test] Starting PWM:{pwmPort}  Feedback:{feedbackPort}");
    }

    public void StopServer()
    {
        running = false;
    }

    // ======================== PWM LISTENER (IN) ========================

    void PwmListenerLoop()
    {
        TcpListener listener = null;

        try
        {
            listener = new TcpListener(IPAddress.Any, pwmPort);
            listener.Start();
            if (debugLog) Debug.Log("[DroneTcpServer_Test] Waiting for PWM client...");

            while (running)
            {
                using (TcpClient client = listener.AcceptTcpClient())
                using (NetworkStream ns = client.GetStream())
                {
                    if (debugLog) Debug.Log("[DroneTcpServer_Test] PWM client connected.");

                    byte[] buffer = new byte[16]; // 4 floats = 16 bytes

                    while (running && client.Connected)
                    {
                        int totalRead = 0;
                        while (totalRead < 16)
                        {
                            int read = ns.Read(buffer, totalRead, 16 - totalRead);
                            if (read == 0) throw new IOException("PWM client disconnected.");
                            totalRead += read;
                        }

                        float pwm1 = BitConverter.ToSingle(buffer, 0);
                        float pwm2 = BitConverter.ToSingle(buffer, 4);
                        float pwm3 = BitConverter.ToSingle(buffer, 8);
                        float pwm4 = BitConverter.ToSingle(buffer, 12);

                        if (debugLog)
                            Debug.Log($"[DroneTcpServer_Test] RX PWM: {pwm1}, {pwm2}, {pwm3}, {pwm4}");

                        lock (pwmLock)
                        {
                            latestPwm = new float[] { pwm1, pwm2, pwm3, pwm4 };
                            hasNewPwm = true;
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            if (debugLog) Debug.LogWarning("[DroneTcpServer_Test] PWM listener error: " + ex.Message);
        }
        finally
        {
            try { listener?.Stop(); } catch { }
        }
    }

    // ====================== FEEDBACK SENDER (OUT) ======================

    void FeedbackListenerLoop()
    {
        TcpListener listener = null;

        try
        {
            listener = new TcpListener(IPAddress.Any, feedbackPort);
            listener.Start();
            if (debugLog) Debug.Log("[DroneTcpServer_Test] Waiting for feedback client...");

            while (running)
            {
                using (TcpClient client = listener.AcceptTcpClient())
                using (NetworkStream ns = client.GetStream())
                {
                    if (debugLog) Debug.Log("[DroneTcpServer_Test] Feedback client connected.");

                    while (running && client.Connected)
                    {
                        byte[] payload = BuildFeedbackBytes();
                        ns.Write(payload, 0, payload.Length);
                        ns.Flush();

                        if (debugLog)
                            Debug.Log($"[DroneTcpServer_Test] Sent {payload.Length} bytes of feedback");

                        Thread.Sleep(10); // ~100 Hz
                    }
                }
            }
        }
        catch (Exception ex)
        {
            if (debugLog) Debug.LogWarning("[DroneTcpServer_Test] Feedback listener error: " + ex.Message);
        }
        finally
        {
            try { listener?.Stop(); } catch { }
        }
    }

    // ======================= FEEDBACK PACKING =======================

    /// <summary>
    /// Packs 10 float feedback values into a 40-byte array (little-endian).
    /// Expected by Simulink Unpack block.
    /// </summary>
    byte[] BuildFeedbackBytes()
    {
        FeedbackSnapshot fb;
        lock (feedbackLock)
        {
            fb = latestFeedback;
        }

        float[] arr = new float[]
        {
            fb.theta, fb.phi, fb.psi,
            fb.posX, fb.posY, fb.alt,
            fb.spX, fb.spY, fb.spAlt, fb.spYaw,
            fb.numSV, fb.fixFlag
        };

        byte[] bytes = new byte[arr.Length * 4];
        Buffer.BlockCopy(arr, 0, bytes, 0, bytes.Length);

        if (debugLog)
        {
            Debug.Log($"[DroneTcpServer_Test] Feedback floats: " +
                      $"{arr[0]}, {arr[1]}, {arr[2]}, {arr[3]}, {arr[4]}, " +
                      $"{arr[5]}, {arr[6]}, {arr[7]}, {arr[8]}, {arr[9]}");
        }

        return bytes;
    }

    // ========================== STRUCT ==========================

    struct FeedbackSnapshot
    {
        public float theta, phi, psi;
        public float posX, posY;
        public float alt;
        public float spX, spY, spAlt, spYaw;
        public float numSV;
        public float fixFlag;
    }
}

