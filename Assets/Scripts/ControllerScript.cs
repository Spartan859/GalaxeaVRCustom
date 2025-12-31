using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Net.Sockets;
using System;
using System.Threading.Tasks;
using System.Threading;
using System.Collections.Concurrent;

public class ControllerScript : MonoBehaviour
{
    [Header("Server Settings")]
    [Tooltip("服务器地址，例如: localhost 或 192.168.1.100")]
    public string serverHost = "localhost";
    
    [Tooltip("服务器端口")]
    public int serverPort = 50051;
    
    [Tooltip("发送数据的时间间隔(秒)")]
    public float sendInterval = 0.1f;
    
    [Tooltip("发送ping指令的时间间隔(秒)")]
    public float pingInterval = 1.0f;

    [Tooltip("ping响应超时时间(毫秒)")]
    public int pingTimeoutMs = 2000;
    
    [Header("Queue Settings")]
    [Tooltip("发送队列和接收队列的最大大小")]
    public int maxQueueSize = 100;
    
    [Header("Mode Settings")]
    [Tooltip("长按摇杆切换模式的时间(秒)")]
    public float modeToggleDuration = 1.0f;

    [Tooltip("在Reset模式下长按右摇杆发送reset指令的时间(秒)")]
    public float resetCommandDuration = 1.0f;

    [Header("Joystick Settings")]
    [Tooltip("摇杆死区阈值（小于此值的输入将被忽略）")]
    public float joystickDeadzone = 0.15f;
    
    [Header("Chassis Settings")]
    [Tooltip("底盘前后速度上限 vx（摇杆y轴，范围[-maxVx, maxVx]）")]
    public float maxVx = 0.2f;

    [Tooltip("底盘左右速度上限 vy（摇杆x轴，范围[-maxVy, maxVy]）")]
    public float maxVy = 0.2f;

    [Tooltip("底盘角速度上限 w（右摇杆x轴，范围[-maxW, maxW]）")]
    public float maxW = 1.0f;

    [Header("Gripper Settings")]
    [Tooltip("夹爪变化速率 (每秒变化量)")]
    public float gripperChangeSpeed = 50.0f;

    [Tooltip("夹爪短按判定时间 (秒)")]
    public float gripperShortPressTime = 0.3f;

    [Header("Torso Settings")]
    [Tooltip("躯干前后速度上限 vx（X和Y键，范围[-maxTorsoVx, maxTorsoVx]）")]
    public float maxTorsoVx = 0.2f;

    [Tooltip("躯干升降速度上限 vz（右摇杆y轴，范围[-maxTorsoVz, maxTorsoVz]）")]
    public float maxTorsoVz = 0.2f;

    [Tooltip("躯干俯仰角速度上限 w_pitch（A和B键，范围[-maxTorsoWPitch, maxTorsoWPitch]）")]
    public float maxTorsoWPitch = 0.5f;

    [Tooltip("躯干转向角速度上限 w_yaw（左G和右G键，范围[-maxTorsoWYaw, maxTorsoWYaw]）")]
    public float maxTorsoWYaw = 0.5f;
    
    private float timer = 0f;
    private float pingTimer = 0f;
    private float thumbstickPressTimer = 0f;
    private bool isThumbstickPressed = false;
    private bool modeToggleInThisPress = false;  // 当前按下周期中是否已经切换过一次模式
    private float currentLeftGripper = 100f;
    private float currentRightGripper = 100f;
    private float leftTriggerTimer = 0f;
    private float rightTriggerTimer = 0f;
    private bool lastAButtonDown = false;      // 上一帧A按键状态（俯仰+）
    private bool lastBButtonDown = false;      // 上一帧B按键状态（俯仰-）
    private bool lastXButtonDown = false;      // 上一帧X按键状态（前进）
    private bool lastYButtonDown = false;      // 上一帧Y按键状态（后退）
    private bool lastLeftGrip = false;         // 上一帧左G按键状态（yaw-）
    private bool lastRightGrip = false;        // 上一帧右G按键状态（yaw+）
    private TcpClient tcpClient;
    private NetworkStream stream;
    private bool isConnected = false;
    private bool isReconnecting = false;  // 防止并发重连
    private bool connectionFailed = false;  // 连接彻底失败标志
    private int reconnectAttempts = 0;
    private const int maxReconnectAttempts = 5;
    private readonly SemaphoreSlim sendLock = new SemaphoreSlim(1, 1);
    private readonly ConcurrentQueue<string> sendQueue = new ConcurrentQueue<string>();
    private CancellationTokenSource sendCts;
    private Task sendTask;
    private readonly SemaphoreSlim receiveLock = new SemaphoreSlim(1, 1);
    private readonly ConcurrentQueue<string> receiveQueue = new ConcurrentQueue<string>();
    private CancellationTokenSource receiveCts;
    private Task receiveTask;
    private readonly SemaphoreSlim receiveQueueSizeLock = new SemaphoreSlim(1, 1);  // 保护receiveQueue大小检查和删除
    private readonly SemaphoreSlim responseWaitLock = new SemaphoreSlim(1, 1);  // 保护 WaitForResponseAsync
    private OVRCameraRig ovrCameraRig;  // VR相机设备引用

    // 累计发送差量的上一帧基准
    private bool hasLastSend = false;
    private Vector3 lastLeftPos;
    private Vector3 lastRightPos;
    private Quaternion lastLeftRot;
    private Quaternion lastRightRot;
    private float lastLeftGripperSent = 100f;
    private float lastRightGripperSent = 100f;
    
    // 上层模式：Normal / Event
    private enum UpperControlMode
    {
        Normal,
        Event
    }

    private UpperControlMode currentUpperMode = UpperControlMode.Normal;

    // Teleop 事件枚举与包装类型
    public enum TeleopEventType
    {
        NONE = 0,
        SUCCESS = 1,
        RERECORD = 2,
        TERMINATE = 3
    }

    public struct MyTeleopEvent
    {
        public TeleopEventType Type;

        public MyTeleopEvent(TeleopEventType type)
        {
            Type = type;
        }

        public override string ToString()
        {
            switch (Type)
            {
                case TeleopEventType.SUCCESS: return "success";
                case TeleopEventType.RERECORD: return "rerecord";
                case TeleopEventType.TERMINATE: return "terminate";
                default: return "none";
            }
        }

        public static MyTeleopEvent Success => new MyTeleopEvent(TeleopEventType.SUCCESS);
        public static MyTeleopEvent Rerecord => new MyTeleopEvent(TeleopEventType.RERECORD);
        public static MyTeleopEvent Terminate => new MyTeleopEvent(TeleopEventType.TERMINATE);
        public static MyTeleopEvent None => new MyTeleopEvent(TeleopEventType.NONE);
    }
    
    async void Start()
    {
        // 获取OVRCameraRig组件
        ovrCameraRig = FindObjectOfType<OVRCameraRig>();
        if (ovrCameraRig == null)
        {
            Debug.LogError("未找到OVRCameraRig组件，手柄坐标转换将无法正常工作");
        }
        
        await ConnectToServer();
        StartSendLoop();
        StartReceiveLoop();
    }
    
    async Task ConnectToServer()
    {
        Debug.Log($"ConnectToServer() called. isReconnecting={isReconnecting}, reconnectAttempts={reconnectAttempts}");
        try
        {
            tcpClient = new TcpClient();
            await tcpClient.ConnectAsync(serverHost, serverPort);
            stream = tcpClient.GetStream();
            isConnected = true;
            
            // 只在初始连接时重置重连次数（非重连状态）
            if (!isReconnecting)
            {
                reconnectAttempts = 0;
            }
            
            Debug.Log($"已连接到服务器 {serverHost}:{serverPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"连接服务器失败: {e.Message}");
            isConnected = false;
            // 初始连接失败时也标记为彻底失败，需通过 A+X 手动重试或检查服务器
            connectionFailed = true;
            Debug.LogError("初始连接失败，已设置 connectionFailed=true。按 A+X 重试或检查服务器。");
        }
    }

    void StartSendLoop()
    {
        if (sendTask != null && !sendTask.IsCompleted) return;
        sendCts = new CancellationTokenSource();
        sendTask = Task.Run(() => ProcessSendQueue(sendCts.Token));
    }

    void StartReceiveLoop()
    {
        if (receiveTask != null && !receiveTask.IsCompleted) return;
        receiveCts = new CancellationTokenSource();
        receiveTask = Task.Run(() => ProcessReceiveQueue(receiveCts.Token));
    }
    
    // 简化的更新阶段机（把原来冗长的 Update 拆分为多个小方法）
    private enum UpdatePhase { ReconnectCheck, SampleTransforms, ApplyDeadzoneAndInputs, HandleModeAndControls, HandleGrippers, SendAndPing, UpdateLastButtons, Done }

    // 临时状态/输入字段（供拆分方法使用）
    private Vector2 _leftStick;
    private Vector2 _rightStick;
    private Vector3 _leftControllerPosition;
    private Quaternion _leftControllerRotation;
    private Vector3 _rightControllerPosition;
    private Quaternion _rightControllerRotation;
    private Vector3 _headPositionTemp;
    private Quaternion _headRotationTemp;
    private bool _aButtonDown;
    private bool _bButtonDown;
    private bool _xButtonDown;
    private bool _yButtonDown;
    private bool _leftGrip;
    private bool _rightGrip;
    private bool _leftTriggerDown;
    private bool _rightTriggerDown;
    private float _vx, _vy, _w, _torsoVz, _torsoVx, _torsoWPitch, _torsoWYaw;
    private MyTeleopEvent _eventToSend = MyTeleopEvent.None;
    private bool _isIntervention = false;
    private bool _lastLeftThumbBtn = false;
    private bool _lastRightThumbBtn = false;
    private bool _leftThumbButtonDown = false;
    private bool _rightThumbButtonDown = false;

    // OVR 输入统一采样
    void CaptureInputs()
    {
        // 按钮 / 摇杆 / 触发器
        _aButtonDown = OVRInput.Get(OVRInput.Button.One);
        _bButtonDown = OVRInput.Get(OVRInput.Button.Two);
        _xButtonDown = OVRInput.Get(OVRInput.Button.Three);
        _yButtonDown = OVRInput.Get(OVRInput.Button.Four);
        _leftGrip = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger);
        _rightGrip = OVRInput.Get(OVRInput.Button.SecondaryHandTrigger);
        _leftTriggerDown = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger);
        _rightTriggerDown = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger);
        _leftThumbButtonDown = OVRInput.Get(OVRInput.Button.PrimaryThumbstick);
        _rightThumbButtonDown = OVRInput.Get(OVRInput.Button.SecondaryThumbstick);
        _leftStick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick);
        _rightStick = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);

        // 头部与手柄位置/姿态（相对于头显）
        _headPositionTemp = Vector3.zero;
        _headRotationTemp = Quaternion.identity;
        if (ovrCameraRig != null && ovrCameraRig.centerEyeAnchor != null)
        {
            _headPositionTemp = ovrCameraRig.centerEyeAnchor.position;
            _headRotationTemp = ovrCameraRig.centerEyeAnchor.rotation;
        }

        Quaternion headRotationInverse = Quaternion.Inverse(_headRotationTemp);
        Vector3 leftControllerWorldPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
        Quaternion leftControllerWorldRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
        Vector3 rightControllerWorldPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        Quaternion rightControllerWorldRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);

        _leftControllerPosition = headRotationInverse * (leftControllerWorldPosition - _headPositionTemp);
        _leftControllerRotation = headRotationInverse * leftControllerWorldRotation;
        _rightControllerPosition = headRotationInverse * (rightControllerWorldPosition - _headPositionTemp);
        _rightControllerRotation = headRotationInverse * rightControllerWorldRotation;
    }

    void Update()
    {
        // 统一采样所有 OVR 输入（按钮、摇杆、触发器、位置姿态）
        CaptureInputs();

        // 公共：连接检查与快捷重连
        SampleReconnectKeys();
        if (connectionFailed) return;
        if (!isConnected) return;

        // 监听左右摇杆按钮组合（下降沿）以切换上层模式
        bool comboLast = _lastLeftThumbBtn && _lastRightThumbBtn;
        bool comboNow = _leftThumbButtonDown && _rightThumbButtonDown;
        if (comboLast && !comboNow)
        {
            currentUpperMode = currentUpperMode == UpperControlMode.Normal ? UpperControlMode.Event : UpperControlMode.Normal;
            hasLastSend = false; // 模式切换时重置增量基准
            Debug.Log($"上层模式切换为: {currentUpperMode}");
        }
        _lastLeftThumbBtn = _leftThumbButtonDown;
        _lastRightThumbBtn = _rightThumbButtonDown;

        if (currentUpperMode == UpperControlMode.Normal)
        {
            _isIntervention = true;
            _eventToSend = MyTeleopEvent.None;

            SampleHeadAndControllers();
            ApplyDeadzoneAndComputeSpeeds();
            HandleGripperInputs();
            HandleSendAndPing();
            UpdateLastButtonStates();
        }
        else // UpperControlMode.Event
        {
            _isIntervention = false;
            _eventToSend = MyTeleopEvent.None;

            // 清零运动量，避免误发送移动指令
            _vx = _vy = _w = 0f;
            _torsoVz = _torsoVx = _torsoWPitch = _torsoWYaw = 0f;

            CheckButtonAndSetEvent();
            HandleSendAndPing();
            UpdateLastButtonStates();
        }
    }

    // --------------- 拆分后的辅助方法 ---------------
    void SampleReconnectKeys()
    {
        bool aNowForReconnect = _aButtonDown;
        bool xNowForReconnect = _xButtonDown;

        if (connectionFailed && aNowForReconnect && xNowForReconnect)
        {
            if (!(lastAButtonDown && lastXButtonDown))
            {
                Debug.Log("A+X 边沿检测通过：重置 connectionFailed，重置 reconnectAttempts 并调用 ConnectToServer()");
                connectionFailed = false;
                reconnectAttempts = 0;
                try { _ = ConnectToServer(); } catch (Exception ex) { Debug.LogWarning($"调用 ConnectToServer() 时捕获异常: {ex.Message}"); }
            }
            else
            {
                Debug.Log("A+X 同时按下但已在上一帧保持，忽略重复触发");
            }
        }
    }

    void SampleHeadAndControllers()
    {
        // 已在 CaptureInputs 中采样
    }

    void ApplyDeadzoneAndComputeSpeeds()
    {
        if (Mathf.Abs(_leftStick.x) < joystickDeadzone) _leftStick.x = 0f;
        if (Mathf.Abs(_leftStick.y) < joystickDeadzone) _leftStick.y = 0f;
        if (Mathf.Abs(_rightStick.x) < joystickDeadzone) _rightStick.x = 0f;
        if (Mathf.Abs(_rightStick.y) < joystickDeadzone) _rightStick.y = 0f;

        _vx = Mathf.Clamp(_leftStick.y, -1f, 1f) * maxVx;
        _vy = Mathf.Clamp(_leftStick.x, -1f, 1f) * maxVy;
        _w = Mathf.Clamp(_rightStick.x, -1f, 1f) * maxW;

        _torsoVz = Mathf.Clamp(_rightStick.y, -1f, 1f) * maxTorsoVz;

        _torsoWPitch = 0f;
        if (_xButtonDown) _torsoWPitch -= maxTorsoWPitch;
        if (_yButtonDown) _torsoWPitch += maxTorsoWPitch;

        _torsoWYaw = 0f;
        if (_rightThumbButtonDown)
        {
            _torsoWYaw = Mathf.Clamp(_rightStick.x, -1f, 1f) * maxTorsoWYaw;
        }

        // 当按住右摇杆时，右摇杆左右推动只控制躯干偏航，不再用于底盘角速度
        if (_rightThumbButtonDown)
        {
            _w = 0f;
        }

        _torsoVx = 0f;
        if (_aButtonDown) _torsoVx -= maxTorsoVx;
        if (_bButtonDown) _torsoVx += maxTorsoVx;
    }

    void HandleGripperInputs()
    {
        if (_leftTriggerDown)
        {
            leftTriggerTimer += Time.deltaTime;
            currentLeftGripper -= gripperChangeSpeed * Time.deltaTime;
        }
        else
        {
            if (leftTriggerTimer > 0f && leftTriggerTimer < gripperShortPressTime)
            {
                currentLeftGripper = 100f;
            }
            leftTriggerTimer = 0f;
        }
        currentLeftGripper = Mathf.Clamp(currentLeftGripper, 0f, 100f);

        if (_rightTriggerDown)
        {
            rightTriggerTimer += Time.deltaTime;
            currentRightGripper -= gripperChangeSpeed * Time.deltaTime;
        }
        else
        {
            if (rightTriggerTimer > 0f && rightTriggerTimer < gripperShortPressTime)
            {
                currentRightGripper = 100f;
            }
            rightTriggerTimer = 0f;
        }
        currentRightGripper = Mathf.Clamp(currentRightGripper, 0f, 100f);
    }

    bool IsFallingEdge(bool current, bool last) => last && !current;

    void CheckButtonAndSetEvent()
    {
        // 按键按下定义为“下降沿”
        if (IsFallingEdge(_aButtonDown, lastAButtonDown))
        {
            _eventToSend = MyTeleopEvent.Success;
            return;
        }
        if (IsFallingEdge(_bButtonDown, lastBButtonDown))
        {
            _eventToSend = MyTeleopEvent.Rerecord;
            return;
        }
        if (IsFallingEdge(_xButtonDown, lastXButtonDown))
        {
            _eventToSend = MyTeleopEvent.Terminate;
            return;
        }
        // 其他按键或无下降沿则保持 NONE（在调用处已初始化）
    }

    void HandleSendAndPing()
    {
        timer += Time.deltaTime;
        if (timer >= sendInterval)
        {
            timer = 0f;
            Vector3 sendLeftPos = _leftControllerPosition;
            Quaternion sendLeftRot = _leftControllerRotation;
            Vector3 sendRightPos = _rightControllerPosition;
            Quaternion sendRightRot = _rightControllerRotation;

            Vector3 deltaPosL;
            Vector3 deltaPosR;
            Vector3 deltaEulerL;
            Vector3 deltaEulerR;
            float deltaGripperL;
            float deltaGripperR;

            ComputePoseAndDelta(sendLeftPos, sendLeftRot, sendRightPos, sendRightRot, currentLeftGripper, currentRightGripper,
                                out deltaPosL, out deltaPosR, out deltaEulerL, out deltaEulerR, out deltaGripperL, out deltaGripperR);

            // 只零化增量，不修改当前采样姿态
            if (currentUpperMode == UpperControlMode.Event)
            {
                deltaPosL = Vector3.zero;
                deltaPosR = Vector3.zero;
                deltaEulerL = Vector3.zero;
                deltaEulerR = Vector3.zero;
                deltaGripperL = 0f;
                deltaGripperR = 0f;
            }
            else
            {
                if (!_leftGrip)
                {
                    deltaPosL = Vector3.zero;
                    deltaEulerL = Vector3.zero;
                    // deltaGripperL = 0f;
                }
                if (!_rightGrip)
                {
                    deltaPosR = Vector3.zero;
                    deltaEulerR = Vector3.zero;
                    // deltaGripperR = 0f;
                }
            }

            _ = SendControllerDataAsync(deltaPosL, deltaEulerL,
                                      deltaPosR, deltaEulerR, deltaGripperL, deltaGripperR,
                                      _vx, _vy, _w, _torsoVx, _torsoVz, _torsoWPitch, _torsoWYaw, _isIntervention, _eventToSend);

            // 事件为一次性发送，发送后复位
            _eventToSend = MyTeleopEvent.None;
        }

        pingTimer += Time.deltaTime;
        if (pingTimer >= pingInterval)
        {
            pingTimer = 0f;
            _ = SendPingAsync();
        }
    }

    void UpdateLastButtonStates()
    {
        lastAButtonDown = _aButtonDown;
        lastBButtonDown = _bButtonDown;
        lastXButtonDown = _xButtonDown;
        lastYButtonDown = _yButtonDown;
    }
    
    // void ToggleMode()
    // {
    //     if (currentMode == ControlMode.Reset)
    //     {
    //         // 从Reset切换到BiManual
    //         currentMode = ControlMode.BiManual;
    //         Debug.Log("切换到 BiManual 模式");
    //         hasLastSend = false;
    //     }
    //     else
    //     {
    //         // 从BiManual切换到Reset
    //         currentMode = ControlMode.Reset;
    //         Debug.Log("切换到 Reset 模式");
    //         hasLastSend = false;
    //     }
    // }
    
    float NormalizeAngle(float angle)
    {
        while (angle > 180f) angle -= 360f;
        while (angle < -180f) angle += 360f;
        return angle;
    }

    // 统一处理姿态轴交换、基准更新与增量计算
    void ComputePoseAndDelta(
        Vector3 leftPos,
        Quaternion leftRot,
        Vector3 rightPos,
        Quaternion rightRot,
        float leftGripperValue,
        float rightGripperValue,
        out Vector3 deltaPosL,
        out Vector3 deltaPosR,
        out Vector3 deltaEulerL,
        out Vector3 deltaEulerR,
        out float deltaGripperL,
        out float deltaGripperR)
    {
        // 当前手柄位置（已做轴交换）
        Vector3 leftToSendSwapped = new Vector3(leftPos.z, -leftPos.x, leftPos.y);
        Vector3 rightToSendSwapped = new Vector3(rightPos.z, -rightPos.x, rightPos.y);

        Quaternion leftQuatSwapped = new Quaternion(-leftRot.y, leftRot.x, leftRot.z, leftRot.w);
        Quaternion rightQuatSwapped = new Quaternion(-rightRot.y, rightRot.x, rightRot.z, rightRot.w);

        // 再绕pitch轴（y轴）旋转-90度
        Quaternion pitchMinus90 = Quaternion.AngleAxis(-90f, Vector3.up);
        leftQuatSwapped = pitchMinus90 * leftQuatSwapped;
        rightQuatSwapped = pitchMinus90 * rightQuatSwapped;

        if (!hasLastSend)
        {
            // 首帧发送零增量，同时记录基准
            deltaPosL = Vector3.zero;
            deltaPosR = Vector3.zero;
            deltaEulerL = Vector3.zero;
            deltaEulerR = Vector3.zero;
            deltaGripperL = 0f;
            deltaGripperR = 0f;
            hasLastSend = true;
        }
        else
        {
            deltaPosL = leftToSendSwapped - lastLeftPos;
            deltaPosR = rightToSendSwapped - lastRightPos;

            Quaternion deltaRotL = Quaternion.Inverse(lastLeftRot) * leftQuatSwapped;
            Quaternion deltaRotR = Quaternion.Inverse(lastRightRot) * rightQuatSwapped;

            Vector3 eulerL = deltaRotL.eulerAngles;
            Vector3 eulerR = deltaRotR.eulerAngles;

            deltaEulerL = new Vector3(
                NormalizeAngle(eulerL.x) * Mathf.Deg2Rad,
                NormalizeAngle(eulerL.y) * Mathf.Deg2Rad,
                NormalizeAngle(eulerL.z) * Mathf.Deg2Rad
            );

            deltaEulerR = new Vector3(
                NormalizeAngle(eulerR.x) * Mathf.Deg2Rad,
                NormalizeAngle(eulerR.y) * Mathf.Deg2Rad,
                NormalizeAngle(eulerR.z) * Mathf.Deg2Rad
            );

            deltaGripperL = leftGripperValue - lastLeftGripperSent;
            deltaGripperR = rightGripperValue - lastRightGripperSent;
        }

        lastLeftPos = leftToSendSwapped;
        lastRightPos = rightToSendSwapped;
        lastLeftRot = leftQuatSwapped;
        lastRightRot = rightQuatSwapped;
        lastLeftGripperSent = leftGripperValue;
        lastRightGripperSent = rightGripperValue;
    }

    async Task SendControllerDataAsync(Vector3 deltaPosL, Vector3 deltaEulerL, Vector3 deltaPosR, Vector3 deltaEulerR, float deltaGripperL = 0f, float deltaGripperR = 0f, float vx = 0f, float vy = 0f, float w = 0f, float torsoVx = 0f, float torsoVz = 0f, float torsoWPitch = 0f, float torsoWYaw = 0f, bool sendIsIntervention = false, MyTeleopEvent sendEvent = default)
    {
        // 如果连接已失败，不再发送
        if (!isConnected || stream == null || connectionFailed) return;
        
        try
        {
            // 构造 send_action 命令，包含chassis_speed、torso_speed、gripper和可选reset
            // 注意：这里改为发送 droll, dpitch, dyaw (6个元素)
            if (sendEvent.Type != TeleopEventType.NONE)
            {
                Debug.Log($"SendControllerDataAsync: sendEvent={sendEvent}");
            }

            string jsonData = string.Format(
                "{{\"cmd\":\"send_action\",\"action\":{{\"left_ee_pose\":[{0},{1},{2},{3},{4},{5}],\"right_ee_pose\":[{6},{7},{8},{9},{10},{11}],\"left_gripper\":{12},\"right_gripper\":{13},\"chassis_speed\":[{14},{15},{16}],\"torso_speed\":[{17},{18},{19},{20}],\"isIntervention\":{21},\"event\":\"{22}\"}}}}",
                deltaPosL.x, deltaPosL.y, deltaPosL.z, deltaEulerL.x, deltaEulerL.y, deltaEulerL.z,
                deltaPosR.x, deltaPosR.y, deltaPosR.z, deltaEulerR.x, deltaEulerR.y, deltaEulerR.z,
                deltaGripperL, deltaGripperR,
                vx, vy, w,
                torsoVx, torsoVz, torsoWPitch, torsoWYaw,
                sendIsIntervention.ToString().ToLower(),
                sendEvent.ToString()
            );
            
            EnqueueMessage(jsonData);
            
            // 等待action响应，过滤出包含"action"的消息
            try
            {
                string response = await WaitForResponseAsync(pingTimeoutMs, msg => msg.Contains("action"));
                // Debug.Log($"[Action Response] {response}");  // 可选：取消注释以打印每次action响应
            }
            catch (OperationCanceledException)
            {
                Debug.LogWarning("action响应超时，触发重连");
                await ReconnectAsync();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"等待action响应异常: {e.Message}，触发重连");
                await ReconnectAsync();
            }
            
            return;
        }
        catch (Exception e)
        {
            Debug.LogError($"发送控制器数据失败: {e.Message}，触发重连");
            await ReconnectAsync();
        }
    }
    
    async Task ReconnectAsync()
    {
        // 如果连接已彻底失败，不再尝试
        if (connectionFailed) return;
        
        // 防止并发多个重连请求
        if (isReconnecting) return;
        isReconnecting = true;
        
        try
        {
            // 立即关闭连接，清空队列中的所有堆积消息
            if (stream != null)
            {
                stream.Close();
            }
            if (tcpClient != null)
            {
                tcpClient.Close();
            }
            
            isConnected = false;
            
            // 清空发送队列和接收队列，移除所有堆积的消息
            while (sendQueue.TryDequeue(out _)) { }
            
            await receiveQueueSizeLock.WaitAsync();
            try
            {
                while (receiveQueue.TryDequeue(out _)) { }
            }
            finally
            {
                receiveQueueSizeLock.Release();
            }
            
            reconnectAttempts++;
            
            if (reconnectAttempts < maxReconnectAttempts)
            {
                Debug.Log($"尝试重连... (第 {reconnectAttempts} 次)");
                await System.Threading.Tasks.Task.Delay(1000); // 等待1秒后重连
                await ConnectToServer();
            }
            else
            {
                // 重连失败次数达到上限，标记连接彻底失败
                connectionFailed = true;
                Debug.LogError($"重连失败超过 {maxReconnectAttempts} 次，停止重连。请检查服务器连接");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"重连异常: {e.Message}");
        }
        finally
        {
            isReconnecting = false;
        }
    }

    // 解析观测数据中的左右末端位置，只取 xyz
    bool TryParseObservationPositions(string json, out Vector3 leftPos, out Vector3 rightPos)
    {
        leftPos = Vector3.zero;
        rightPos = Vector3.zero;

        try
        {
            // 兼容形如 {"ok":true,"result":{...}} 的包装
            ObservationEnvelope envelope = JsonUtility.FromJson<ObservationEnvelope>(json);
            ObservationData data = null;

            if (envelope != null)
            {
                // 优先 result.observation
                if (envelope.result != null)
                {
                    if (envelope.result.observation != null)
                    {
                        data = envelope.result.observation;
                    }
                    else if (envelope.result.left_ee_pose != null || envelope.result.right_ee_pose != null)
                    {
                        data = new ObservationData
                        {
                            left_ee_pose = envelope.result.left_ee_pose,
                            right_ee_pose = envelope.result.right_ee_pose
                        };
                    }
                }

                // 回退 observation 或根级左右末端
                if (data == null)
                {
                    if (envelope.observation != null)
                    {
                        data = envelope.observation;
                    }
                    else if (envelope.left_ee_pose != null || envelope.right_ee_pose != null)
                    {
                        data = new ObservationData
                        {
                            left_ee_pose = envelope.left_ee_pose,
                            right_ee_pose = envelope.right_ee_pose
                        };
                    }
                }
            }

            if (data == null) return false;

            if (!TryGetVec3(data.left_ee_pose, out leftPos)) return false;
            if (!TryGetVec3(data.right_ee_pose, out rightPos)) return false;

            return true;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"观测解析异常: {e.Message}");
            return false;
        }
    }

    bool TryGetVec3(float[] arr, out Vector3 vec)
    {
        vec = Vector3.zero;
        if (arr == null || arr.Length < 3) return false;
        vec = new Vector3(arr[0], arr[1], arr[2]);
        return true;
    }

    [Serializable]
    class ObservationEnvelope
    {
        public ResultWrapper result;
        public ObservationData observation;
        public float[] left_ee_pose;
        public float[] right_ee_pose;
    }

    [Serializable]
    class ResultWrapper
    {
        public ObservationData observation;
        public float[] left_ee_pose;
        public float[] right_ee_pose;
    }

    [Serializable]
    class ObservationData
    {
        public float[] left_ee_pose;
        public float[] right_ee_pose;
    }
    
    async Task SendPingAsync()
    {
        if (!isConnected || stream == null) return;
        
        try
        {
            string jsonData = "{\"cmd\":\"ping\"}";
            EnqueueMessage(jsonData);
            
            // 等待ping响应，过滤出包含"pong"的消息
            try
            {
                string response = await WaitForResponseAsync(pingTimeoutMs, msg => msg.Contains("pong"));
                Debug.Log($"[Ping Response] {response}");
                if (string.IsNullOrEmpty(response) || (!response.Contains("\"ok\"") && !response.Contains("\"ok\":")))
                {
                    Debug.LogWarning($"ping响应异常: {response}，触发重连");
                    await ReconnectAsync();
                }
            }
            catch (OperationCanceledException)
            {
                Debug.LogWarning("ping响应超时，触发重连");
                await ReconnectAsync();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"等待ping响应异常: {e.Message}，触发重连");
                await ReconnectAsync();
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"发送ping失败: {e.Message}");
            await ReconnectAsync();
        }
    }

    async Task<string> WaitForResponseAsync(int timeoutMs = 2000, System.Func<string, bool> filter = null)
    {
        // 同一时刻只允许一个线程执行，防止竞态条件
        await responseWaitLock.WaitAsync();
        try
        {
            var deadline = System.DateTime.UtcNow.AddMilliseconds(timeoutMs);
            var tempQueue = new Queue<string>(); // 临时存储不匹配的消息
            
            while (System.DateTime.UtcNow < deadline)
            {
                // 如果连接已失败，立即返回
                if (connectionFailed)
                {
                    throw new OperationCanceledException("连接已失败");
                }
                
                // 先尝试从临时队列取
                if (tempQueue.Count > 0)
                {
                    var temp = tempQueue.Dequeue();
                    if (filter == null || filter(temp))
                    {
                        // 匹配，返回，并把临时队列剩余消息放回接收队列
                        while (tempQueue.Count > 0)
                        {
                            receiveQueue.Enqueue(tempQueue.Dequeue());
                        }
                        return temp;
                    }
                    // 不匹配，继续找
                }
                
                // 从接收队列取
                if (receiveQueue.TryDequeue(out var message))
                {
                    if (filter == null || filter(message))
                    {
                        // 匹配，返回，并把临时队列的消息放回接收队列
                        await receiveQueueSizeLock.WaitAsync();
                        try
                        {
                            while (tempQueue.Count > 0)
                            {
                                var msg = tempQueue.Dequeue();
                                receiveQueue.Enqueue(msg);
                                // 如果队列超过限制，丢弃最老的消息
                                while (receiveQueue.Count > maxQueueSize)
                                {
                                    receiveQueue.TryDequeue(out _);
                                }
                            }
                        }
                        finally
                        {
                            receiveQueueSizeLock.Release();
                        }
                        return message;
                    }
                    else
                    {
                        // 不匹配，放入临时队列
                        tempQueue.Enqueue(message);
                    }
                }
                
                await Task.Delay(10);
            }
            
            // 超时，把临时队列的消息放回接收队列
            await receiveQueueSizeLock.WaitAsync();
            try
            {
                while (tempQueue.Count > 0)
                {
                    var msg = tempQueue.Dequeue();
                    receiveQueue.Enqueue(msg);
                    // 如果队列超过限制，丢弃最老的消息
                    while (receiveQueue.Count > maxQueueSize)
                    {
                        receiveQueue.TryDequeue(out _);
                    }
                }
            }
            finally
            {
                receiveQueueSizeLock.Release();
            }
            
            throw new OperationCanceledException("等待响应超时");
        }
        finally
        {
            responseWaitLock.Release();
        }
    }

    void EnqueueMessage(string json)
    {
        sendQueue.Enqueue(json);
        // 如果队列超过限制，丢弃最老的消息
        while (sendQueue.Count > maxQueueSize)
        {
            sendQueue.TryDequeue(out _);
        }
    }
    
    async Task ProcessSendQueue(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                if (!isConnected || stream == null)
                {
                    await Task.Delay(50, token);
                    continue;
                }

                if (sendQueue.TryDequeue(out var message))
                {
                    await SendTcpMessageAsync(message);
                }
                else
                {
                    await Task.Delay(10, token);
                }
            }
            catch (OperationCanceledException)
            {
                // 关闭时正常退出
                break;
            }
            catch (Exception e)
            {
                Debug.LogWarning($"发送队列处理异常: {e.Message}，触发重连");
                await ReconnectAsync();
            }
        }
    }

    async Task ProcessReceiveQueue(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                if (!isConnected || stream == null)
                {
                    await Task.Delay(50, token);
                    continue;
                }

                try
                {
                    string message = await ReceiveTcpMessageAsync(token);
                    receiveQueue.Enqueue(message);
                    
                    // 如果队列超过限制，丢弃最老的消息（原子操作）
                    await receiveQueueSizeLock.WaitAsync();
                    try
                    {
                        while (receiveQueue.Count > maxQueueSize)
                        {
                            receiveQueue.TryDequeue(out _);
                        }
                    }
                    finally
                    {
                        receiveQueueSizeLock.Release();
                    }
                }
                catch (OperationCanceledException)
                {
                    // 正常超时或关闭
                    break;
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"接收队列处理异常: {e.Message}，触发重连");
                await ReconnectAsync();
            }
        }
    }
    
    async Task SendTcpMessageAsync(string jsonMessage)
    {
        await sendLock.WaitAsync();
        try
        {
            if (!isConnected || stream == null) return;

            // 计算消息长度（4字节长度前缀，网络字节序）
            byte[] messageBytes = Encoding.UTF8.GetBytes(jsonMessage);
            byte[] lengthPrefix = BitConverter.GetBytes((uint)messageBytes.Length);
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(lengthPrefix); // 转为网络字节序（big-endian）
            }
            
            // 发送长度前缀 + 消息体（串行化，避免并发交织）
            await stream.WriteAsync(lengthPrefix, 0, lengthPrefix.Length);
            await stream.WriteAsync(messageBytes, 0, messageBytes.Length);
            await stream.FlushAsync();
        }
        finally
        {
            sendLock.Release();
        }
    }
    
    async Task<string> ReceiveTcpMessageAsync(CancellationToken token = default)
    {
        // 确保只有一个线程读取 NetworkStream
        await receiveLock.WaitAsync(token);
        try
        {
            // 读取4字节长度前缀
            byte[] lengthBytes = new byte[4];
            int bytesRead = await stream.ReadAsync(lengthBytes, 0, 4, token);
            if (bytesRead != 4)
            {
                throw new Exception("无法读取消息长度");
            }
            
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(lengthBytes);
            }
            uint messageLength = BitConverter.ToUInt32(lengthBytes, 0);
            
            // 读取消息体
            byte[] messageBytes = new byte[messageLength];
            int totalRead = 0;
            while (totalRead < messageLength)
            {
                int read = await stream.ReadAsync(messageBytes, totalRead, (int)(messageLength - totalRead), token);
                if (read == 0)
                {
                    throw new Exception("连接断开");
                }
                totalRead += read;
            }
            
            return Encoding.UTF8.GetString(messageBytes);
        }
        finally
        {
            receiveLock.Release();
        }
    }
    
    void OnDestroy()
    {
        try
        {
            sendCts?.Cancel();
            receiveCts?.Cancel();
        }
        catch { }
        
        if (stream != null)
        {
            stream.Close();
        }
        if (tcpClient != null)
        {
            tcpClient.Close();
        }
    }
}
