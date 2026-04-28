using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

[DisallowMultipleComponent]
public class TCPServer : MonoBehaviour
{
    [Header("Configuration")]
    public int pwmPort = 9000;      // In: 4 floats (16 bytes)
    public int feedbackPort = 9001; // Out: 11 floats (44 bytes)
    public bool autoStart = true;
    public bool debugLog = false;

    // Threading & Networking Objects
    private Thread pwmThread;
    private Thread feedbackThread;
    private volatile bool running = false;

    private TcpListener pwmListener;
    private TcpListener feedbackListener;
    private TcpClient activePwmClient;
    private TcpClient activeFeedbackClient;

    // Data Locks
    private float[] latestPwm = new float[4];
    private bool hasNewPwm = false;
    private readonly object pwmLock = new object();

    private byte[] feedbackData = new byte[44]; // 11 floats * 4
    private readonly object feedbackLock = new object();

    void Start()
    {
        if (autoStart) StartServer();
    }

    void OnApplicationQuit()
    {
        StopServer();
    }

    void OnDestroy()
    {
        StopServer();
    }

    // ---------------- API FOR DRONE HUB ----------------

    public void SendFeedback(byte[] data)
    {
        if (data == null || data.Length != feedbackData.Length) return;
        lock (feedbackLock)
        {
            Array.Copy(data, feedbackData, data.Length);
        }
    }

    public bool TryGetPwm(out float[] pwm)
    {
        lock (pwmLock)
        {
            if (hasNewPwm)
            {
                pwm = (float[])latestPwm.Clone();
                hasNewPwm = false;
                return true;
            }
            pwm = null;
            return false;
        }
    }

    // ---------------- NETWORKING ----------------

    public void StartServer()
    {
        if (running) return;
        running = true;
        pwmThread = new Thread(PwmLoop) { IsBackground = true };
        feedbackThread = new Thread(FeedbackLoop) { IsBackground = true };
        pwmThread.Start();
        feedbackThread.Start();
        if (debugLog) Debug.Log("TCP Server Started.");
    }

    public void StopServer()
    {
        if (!running) return;
        running = false;

        // Forcefully close listeners and clients to break blocking Read/Write calls
        pwmListener?.Stop();
        feedbackListener?.Stop();
        activePwmClient?.Close();
        activeFeedbackClient?.Close();

        if (debugLog) Debug.Log("TCP Server Stopped and Sockets Cleared.");
    }

    void PwmLoop() // Receives Motor Commands
    {
        try
        {
            pwmListener = new TcpListener(IPAddress.Any, pwmPort);
            pwmListener.Start();
            byte[] buffer = new byte[16];

            while (running)
            {
                if (!pwmListener.Pending()) { Thread.Sleep(50); continue; }

                using (activePwmClient = pwmListener.AcceptTcpClient())
                using (NetworkStream ns = activePwmClient.GetStream())
                {
                    while (running && activePwmClient.Connected)
                    {
                        int read = 0;
                        while (read < 16 && running)
                        {
                            int r = ns.Read(buffer, read, 16 - read);
                            if (r == 0) break;
                            read += r;
                        }
                        if (read < 16) break;

                        float[] tempPwm = new float[4];
                        for (int i = 0; i < 4; i++) tempPwm[i] = BitConverter.ToSingle(buffer, i * 4);

                        lock (pwmLock)
                        {
                            latestPwm = tempPwm;
                            hasNewPwm = true;
                        }
                    }
                }
            }
        }
        catch (SocketException) { /* Expected during forced shutdown */ }
        catch (Exception e) { if (running && debugLog) Debug.LogWarning("PWM Server Error: " + e.Message); }
        finally { pwmListener?.Stop(); }
    }

    void FeedbackLoop() // Sends Sensor Data
    {
        try
        {
            feedbackListener = new TcpListener(IPAddress.Any, feedbackPort);
            feedbackListener.Start();
            byte[] buffer = new byte[44];

            while (running)
            {
                if (!feedbackListener.Pending()) { Thread.Sleep(50); continue; }

                using (activeFeedbackClient = feedbackListener.AcceptTcpClient())
                using (NetworkStream ns = activeFeedbackClient.GetStream())
                {
                    while (running && activeFeedbackClient.Connected)
                    {
                        lock (feedbackLock)
                        {
                            Array.Copy(feedbackData, buffer, 44);
                        }
                        ns.Write(buffer, 0, 44);
                        Thread.Sleep(10); // Send at ~100Hz
                    }
                }
            }
        }
        catch (SocketException) { /* Expected during forced shutdown */ }
        catch (Exception e) { if (running && debugLog) Debug.LogWarning("Feedback Server Error: " + e.Message); }
        finally { feedbackListener?.Stop(); }
    }
}