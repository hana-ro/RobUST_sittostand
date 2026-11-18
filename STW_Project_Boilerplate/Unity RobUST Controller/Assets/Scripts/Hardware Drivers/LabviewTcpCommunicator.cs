using UnityEngine;
using Unity.Profiling;

using System;
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;
using System.Buffers.Text;

/// <summary>
/// Handles threaded TCP communication with LabVIEW, continuously sending the latest tension data.
/// Optimized for Zero-Allocation and "Soft" Real-Time on Windows.
/// </summary>
public class LabviewTcpCommunicator : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "Labview TCP Communicator Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "Labview TCP Communicator Send Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Network Settings")]
    public string serverAddress = "10.0.0.62";
    public int serverPort = 8053;

    // Network components
    private TcpClient tcpClient;
    private NetworkStream networkStream;
    
    // Threading
    private Thread sendThread;
    private double sendFrequency_Hz = 1000.0; 
    private volatile bool isRunning = false;
    private readonly object dataLock = new object();
    private volatile char controlModeCode = 'O';

    // Current data to send - pre-allocated during initialization
    private double[] goalTensions;

    // --- NEW: Zero-Allocation Buffers ---
    private byte[] sendBuffer;     // Raw bytes to send to TCP
    private char[] formatBuffer;   // Temp buffer for double->string conversion

    // Measured tensions received from LabVIEW
    private double[] measuredTensions;

    // TCP receive buffers
    private byte[] receiveBuffer;
    private byte[] receiveLineBuffer;
    private int receiveLineLength = 0;

    public bool IsConnected { get; private set; } = false;
    /// <summary>
    /// Initializes the TCP communicator with the cable configuration.
    /// Called by RobotController in the correct dependency order.
    /// </summary>
    public bool Initialize()
    {
        // Pre-allocate arrays based on cable count
        goalTensions = new double[14];

        // Size calc: 1 byte (Mode) + 14 * (1 comma + ~9 chars) + 2 newline = ~150 bytes. 
        // We give 512 for safety.
        sendBuffer = new byte[512];
        formatBuffer = new char[32]; // Enough for one double "0.123456"
        
        measuredTensions = new double[14];

        receiveBuffer = new byte[1024];
        receiveLineBuffer = new byte[512];
        receiveLineLength = 0;
        return true;
    }

    /// <summary>
    /// Universal update method (zero heap allocation). Accepts standard arrays, stackalloc spans, or list spans.
    /// </summary>
    public void UpdateTensionSetpoint(ReadOnlySpan<double> newTensions)
    {
        lock (dataLock)
        {
            // Note: 'tensions' array is implicitly converted to a Span<double> here.
            newTensions.CopyTo(goalTensions);
        }
    }

    public void GetMeasuredTensions(Span<double> destination)
    {
        lock (dataLock)
        {
            measuredTensions.AsSpan().CopyTo(destination);
        }
    }

    /// <summary>
    /// Switches outgoing packets to closed-loop control mode.
    /// </summary>
    public void SetClosedLoopControl()
    {
        controlModeCode = 'C';
    }

    /// <summary>
    /// Switches outgoing packets to open-loop control mode.
    /// </summary>
    public void SetOpenLoopControl()
    {
        controlModeCode = 'O';
    }

    public async void ConnectToServer()
    {
        try
        {
            tcpClient = new TcpClient();
            UnityEngine.Debug.Log($"Connecting to {serverAddress}:{serverPort}...");
            
            await tcpClient.ConnectAsync(serverAddress, serverPort);
            networkStream = tcpClient.GetStream();
            tcpClient.NoDelay = true; // Disable Nagle's algorithm
            
            IsConnected = true;
            isRunning = true;
            sendThread = new Thread(TcpLoop)
            {
                IsBackground = true,
                Name = "Labview TCP Thread",
                Priority = System.Threading.ThreadPriority.Highest
            };
            sendThread.Start();
            
            UnityEngine.Debug.Log("Connected to LabVIEW server.");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Connection failed: {e.Message}");
        }
    }

    /// <summary>
    /// Background thread that continuously sends data at precise frequency.
    /// </summary>
    private void TcpLoop()
    {
        if (networkStream == null || tcpClient == null || !tcpClient.Connected)
        {
            isRunning = false;
            IsConnected = false;
            return;
        }

        Span<double> sendTensions = stackalloc double[14];
        Span<double> parsedTensions = stackalloc double[14];

        // Timing constants
        double frequency = Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / frequency;
        long intervalTicks = (long)(frequency / sendFrequency_Hz);
        long nextTargetTime = Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = Stopwatch.GetTimestamp();
        
        while (isRunning)
        {
            long loopStartTick = Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            lock (dataLock)
            {
                goalTensions.AsSpan().CopyTo(sendTensions);
            }

            int bytesToSend = FillPacketBuffer(sendTensions, sendBuffer);
            try
            {
                networkStream.Write(sendBuffer, 0, bytesToSend);
                DrainIncomingTensions(parsedTensions);
            }
            catch (Exception)
            {
                isRunning = false;
                break;
            }

            s_WorkloadNs.Value = (long)((Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);

            while (Stopwatch.GetTimestamp() < nextTargetTime)
            {
                // BURN cycles to hold the core.
            }

            // TIMING ADVANCE & DRIFT CORRECTION
            nextTargetTime += intervalTicks;

            // If we are late (processing took > 2ms), reset the pacer
            long now = Stopwatch.GetTimestamp();
            if (now > nextTargetTime)
            {
                nextTargetTime = now + intervalTicks;
            }
        }
    }

    private void DrainIncomingTensions(Span<double> parsedTensions)
    {
        int available;

        while ((available = tcpClient.Available) > 0)
        {
            int bytesToRead = available < receiveBuffer.Length ? available : receiveBuffer.Length;
            int bytesRead = networkStream.Read(receiveBuffer, 0, bytesToRead);

            ProcessReceivedBytes(receiveBuffer.AsSpan(0, bytesRead), parsedTensions);
        }
    }

    /// <summary>
    /// Fills the byte array directly with ASCII data.
    /// Returns the number of bytes written.
    /// </summary>
    private int FillPacketBuffer(ReadOnlySpan<double> values, byte[] buffer)
    {
        int pos = 0;

        // A. Write Control Mode
        buffer[pos++] = (byte)controlModeCode;

        // B. Write Tensions
        for (int i = 0; i < values.Length; i++)
        {
            buffer[pos++] = (byte)',';

            // ZERO-ALLOC NUMBER FORMATTING
            // TryFormat writes the number into 'formatBuffer' (char[]) without creating a String.
            // Requirement: Unity Project Settings > Player > Api Compatibility Level = .NET Standard 2.1
            if (values[i].TryFormat(formatBuffer, out int charsWritten, "F6"))
            {
                // Manually copy chars to bytes (ASCII)
                for (int k = 0; k < charsWritten; k++)
                {
                    buffer[pos++] = (byte)formatBuffer[k];
                }
            }
            else
            {
                // Fallback (Should ideally never happen with a large enough formatBuffer)
                buffer[pos++] = (byte)'0';
            }
        }

        // C. Write Newline
        buffer[pos++] = (byte)'\r';
        buffer[pos++] = (byte)'\n';

        return pos;
    }

    /// <summary>
    /// Reads a Span of bytes and separates each number via comma delimiter and stores it in measured_Tensions.
    /// </summary>
    private void ProcessReceivedBytes(ReadOnlySpan<byte> bytes, Span<double> parsedTensions)
    {
        for (int i = 0; i < bytes.Length; i++)
        {
            byte b = bytes[i];

            if (b == (byte)'\r')
                continue;

            if (b != (byte)'\n')
            {
                receiveLineBuffer[receiveLineLength++] = b;
                continue;
            }

            int valueIndex = 0;
            int tokenStart = 0;
            bool parseOk = true;

            for (int j = 0; j <= receiveLineLength; j++)
            {
                bool atEnd = j == receiveLineLength;
                bool atComma = !atEnd && receiveLineBuffer[j] == (byte)',';

                if (!atEnd && !atComma)
                    continue;

                int tokenLength = j - tokenStart;

                if (valueIndex >= 14 ||
                    tokenLength <= 0 ||
                    !Utf8Parser.TryParse(
                        receiveLineBuffer.AsSpan(tokenStart, tokenLength),
                        out double value,
                        out int consumed) ||
                    consumed != tokenLength)
                {
                    parseOk = false;
                    break;
                }

                parsedTensions[valueIndex++] = value;
                tokenStart = j + 1;
            }

            if (parseOk && valueIndex == 14)
            {
                lock (dataLock)
                {
                    parsedTensions.CopyTo(measuredTensions);
                }
            }

            receiveLineLength = 0;
        }
    }

    public void Disconnect()
    {
        if (!IsConnected) return;
        
        isRunning = false;

        networkStream?.Close();
        tcpClient?.Close();

        sendThread?.Join(100); 

        IsConnected = false;
        
        UnityEngine.Debug.Log("Disconnected from LabVIEW server.");
    }

    void OnApplicationQuit()
    {
        Disconnect();
    }
}