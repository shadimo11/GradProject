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
    public int commandPort = 9000;  // In: 5 floats (20 bytes) - [M1, M2, M3, M4, V_batt]
    public int feedbackPort = 9001; // Out: 11 floats (44 bytes) - State Observation Vector
    public bool autoStart = true;
    public bool debugLog = false;

    // Threading & Data
    private Thread commandThread;
    private Thread feedbackThread;
    private volatile bool running = false;

    // Command Vector (Expanded to R^5)
    private float[] latestCommands = new float[5];
    private bool hasNewCommands = false;
    private readonly object commandLock = new object();

    private byte[] feedbackData = new byte[44]; // 11 floats * 4 bytes
    private readonly object feedbackLock = new object();

    void Start()
    {
        if (autoStart) StartServer();
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

    // Renamed to reflect the expanded vector payload
    public bool TryGetCommands(out float[] commands)
    {
        lock (commandLock)
        {
            if (hasNewCommands)
            {
                commands = (float[])latestCommands.Clone();
                hasNewCommands = false;
                return true;
            }
            commands = null;
            return false;
        }
    }

    // ---------------- NETWORKING ----------------

    public void StartServer()
    {
        if (running) return;
        running = true;

        commandThread = new Thread(CommandLoop) { IsBackground = true };
        feedbackThread = new Thread(FeedbackLoop) { IsBackground = true };

        commandThread.Start();
        feedbackThread.Start();

        if (debugLog) Debug.Log("[TCP] SIL Telemetry Server Started.");
    }

    public void StopServer()
    {
        running = false;
    }

    void CommandLoop() // Receives Control Vector u_k
    {
        TcpListener listener = new TcpListener(IPAddress.Any, commandPort);
        listener.Start();
        byte[] buffer = new byte[20]; // Upgraded to 20 bytes for 5 floats

        try
        {
            while (running)
            {
                if (!listener.Pending()) { Thread.Sleep(10); continue; }

                using (TcpClient client = listener.AcceptTcpClient())
                using (NetworkStream ns = client.GetStream())
                {
                    while (running && client.Connected)
                    {
                        int read = 0;
                        while (read < 20)
                        {
                            int r = ns.Read(buffer, read, 20 - read);
                            if (r == 0) break;
                            read += r;
                        }
                        if (read < 20) break;

                        float[] tempCmd = new float[5];
                        for (int i = 0; i < 5; i++)
                        {
                            tempCmd[i] = BitConverter.ToSingle(buffer, i * 4);
                        }

                        lock (commandLock)
                        {
                            latestCommands = tempCmd;
                            hasNewCommands = true;
                        }
                    }
                }
            }
        }
        catch (Exception e) { Debug.LogWarning("[TCP] Command Server Error: " + e.Message); }
        finally { listener.Stop(); }
    }

    void FeedbackLoop() // Sends Observation Vector z_k
    {
        TcpListener listener = new TcpListener(IPAddress.Any, feedbackPort);
        listener.Start();
        byte[] buffer = new byte[44];

        try
        {
            while (running)
            {
                if (!listener.Pending()) { Thread.Sleep(10); continue; }

                using (TcpClient client = listener.AcceptTcpClient())
                using (NetworkStream ns = client.GetStream())
                {
                    while (running && client.Connected)
                    {
                        lock (feedbackLock)
                        {
                            Array.Copy(feedbackData, buffer, 44);
                        }
                        ns.Write(buffer, 0, 44);
                        Thread.Sleep(10); // Transmit at 100Hz (Ts = 0.01)
                    }
                }
            }
        }
        catch (Exception e) { Debug.LogWarning("[TCP] Feedback Server Error: " + e.Message); }
        finally { listener.Stop(); }
    }
}