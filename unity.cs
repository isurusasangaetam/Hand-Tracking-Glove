using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using System.Collections.Generic;

public class UDPHandReceiver : MonoBehaviour
{
    public int port = 4210;
    private UdpClient udpClient;
    private Thread receiveThread;

    public Transform palm;
    public Transform[] fingers = new Transform[5];         // Second bone (receives data)
    public Transform[] fingerBaseBones = new Transform[5]; // First bone (predicted)

    private Quaternion qPalm;
    private Quaternion[] qFingers = new Quaternion[5];

    private Quaternion calibrationOffset = Quaternion.identity;
    private Quaternion[] fingerCalibrationOffsets = new Quaternion[5];
    private Quaternion unityPalmInitialRot;
    private Quaternion[] unityFingerInitialRots = new Quaternion[5];
    private Quaternion[] unityBaseInitialRots = new Quaternion[5];

    private bool calibrationDone = false;
    private float startupTimer = 0f;
    private float calibrationTime = 5f;
    private float noiseFilterTime = 6f;
    private List<Quaternion> palmReadings = new List<Quaternion>();
    private List<Quaternion>[] fingerReadings = new List<Quaternion>[5];

    private bool requestCalibration = false;
    private List<Quaternion> palmResetReadings = new List<Quaternion>();
    private List<Quaternion>[] fingerResetReadings = new List<Quaternion>[5];
    private float resetTimer = 0f;
    private bool resetInProgress = false;
    private float resetCollectDuration = 1f;

    public float baseBoneInfluence = 0.6f; // Predicted base bone is 60% of middle joint rotation

    void Start()
    {
        udpClient = new UdpClient(port);
        receiveThread = new Thread(ReceiveData);
        receiveThread.IsBackground = true;
        receiveThread.Start();

        if (palm != null)
            unityPalmInitialRot = palm.localRotation;

        for (int i = 0; i < fingers.Length; i++)
        {
            if (fingers[i] != null)
                unityFingerInitialRots[i] = fingers[i].localRotation;

            if (fingerBaseBones[i] != null)
                unityBaseInitialRots[i] = fingerBaseBones[i].localRotation;

            fingerReadings[i] = new List<Quaternion>();
            fingerResetReadings[i] = new List<Quaternion>();
        }

        Debug.Log("Waiting for startup calibration...");
    }

    void ReceiveData()
    {
        while (true)
        {
            try
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, port);
                byte[] data = udpClient.Receive(ref remoteEP);
                string message = Encoding.ASCII.GetString(data);
                ParseData(message);
            }
            catch (System.Exception ex)
            {
                Debug.LogError("UDP Receive Error: " + ex.Message);
            }
        }
    }

    void ParseData(string message)
    {
        string[] lines = message.Split('\n');
        int qIndex = -1;

        if (message.Contains("RESET"))
        {
            requestCalibration = true;
            resetInProgress = true;
            resetTimer = 0f;
            palmResetReadings.Clear();
            for (int i = 0; i < 5; i++) fingerResetReadings[i].Clear();
            Debug.Log("Reset command received. Starting recalibration...");
            return;
        }

        for (int i = 0; i < lines.Length; i++)
        {
            string line = lines[i].Trim();
            if (line == "Palm") qIndex = -1;
            else if (line.StartsWith("Finger")) qIndex = int.Parse(line.Split(' ')[1]) - 1;
            else if (line.Contains(",") && line.EndsWith(";"))
            {
                string[] values = line.TrimEnd(';').Split(',');
                if (values.Length == 4)
                {
                    float w = float.Parse(values[0]);
                    float x = float.Parse(values[1]);
                    float y = float.Parse(values[2]);
                    float z = float.Parse(values[3]);
                    Quaternion q = new Quaternion(x, y, z, w); // Unity uses (x, y, z, w)

                    if (qIndex == -1)
                        qPalm = q;
                    else if (qIndex >= 0 && qIndex < 5)
                        qFingers[qIndex] = q;
                }
            }
        }
    }

    void Update()
    {
        startupTimer += Time.deltaTime;

        if (startupTimer <= noiseFilterTime)
        {
            palmReadings.Add(qPalm);
            for (int i = 0; i < 5; i++)
                fingerReadings[i].Add(qFingers[i]);
        }

        if (!calibrationDone && startupTimer >= calibrationTime)
        {
            qPalm = AverageQuaternions(palmReadings);
            calibrationOffset = unityPalmInitialRot * Quaternion.Inverse(qPalm);

            for (int i = 0; i < 5; i++)
            {
                qFingers[i] = AverageQuaternions(fingerReadings[i]);
                fingerCalibrationOffsets[i] = unityFingerInitialRots[i] * Quaternion.Inverse(qFingers[i]);
            }

            calibrationDone = true;
            Debug.Log("Calibration complete at startup.");
        }

        if (calibrationDone)
        {
            if (palm != null)
                palm.localRotation = calibrationOffset * qPalm;

            for (int i = 0; i < 5; i++)
            {
                // Apply to second joint
                if (fingers[i] != null)
                    fingers[i].localRotation = fingerCalibrationOffsets[i] * qFingers[i];

                // Predict first joint
                if (fingerBaseBones[i] != null)
                {
                    Quaternion secondRot = fingers[i].localRotation;
                    Vector3 euler = secondRot.eulerAngles;
                    euler.x = (euler.x > 180f) ? euler.x - 360f : euler.x;
                    float predictedX = euler.x * baseBoneInfluence;
                    Quaternion predictedBase = Quaternion.Euler(predictedX, 0f, 0f);
                    fingerBaseBones[i].localRotation = predictedBase;
                }
            }
        }

        if (resetInProgress)
        {
            resetTimer += Time.deltaTime;
            if (resetTimer < resetCollectDuration)
            {
                palmResetReadings.Add(qPalm);
                for (int i = 0; i < 5; i++)
                    fingerResetReadings[i].Add(qFingers[i]);
            }
            else
            {
                qPalm = AverageQuaternions(palmResetReadings);
                calibrationOffset = unityPalmInitialRot * Quaternion.Inverse(qPalm);

                for (int i = 0; i < 5; i++)
                {
                    qFingers[i] = AverageQuaternions(fingerResetReadings[i]);
                    fingerCalibrationOffsets[i] = unityFingerInitialRots[i] * Quaternion.Inverse(qFingers[i]);
                }

                resetInProgress = false;
                Debug.Log("Hand recalibrated to new neutral pose.");
            }
        }
    }

    Quaternion AverageQuaternions(List<Quaternion> quaternions)
    {
        if (quaternions.Count == 0)
            return Quaternion.identity;

        Quaternion cumulative = quaternions[0];
        for (int i = 1; i < quaternions.Count; i++)
            cumulative = Quaternion.Slerp(cumulative, quaternions[i], 1f / (i + 1));

        return cumulative;
    }

    void OnApplicationQuit()
    {
        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Abort();

        udpClient?.Close();
    }
}