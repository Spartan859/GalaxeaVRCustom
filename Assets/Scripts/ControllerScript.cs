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
    private float rightThumbstickPressTimer = 0f;  // 右摇杆按下的计时
    private bool isRightThumbstickPressed = false;  // 右摇杆是否被按下
    private bool resetCommandInThisPress = false;  // 当前按下周期中是否已经发送过reset指令
    private bool resetRequested = false;  // 是否请求了reset
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
    
    // 模式枚举
    private enum ControlMode
    {
        Reset,      // Reset模式，不发送坐标
        BiManual    // BiManual模式，发送控制器坐标
    }
    
    private ControlMode currentMode = ControlMode.Reset;
    
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
    
    void Update()
    {
        // 在断线/失败状态下仍需响应重连快捷键，先采样 A/X 用于边沿检测
        bool aNowForReconnect = OVRInput.Get(OVRInput.Button.One);
        bool xNowForReconnect = OVRInput.Get(OVRInput.Button.Three);
        // 少量信息性日志，便于确认按键采样与状态（仅在断线或有按键时打印，避免刷屏）
        if (connectionFailed || aNowForReconnect || xNowForReconnect)
        {
            Debug.Log($"A+X sample: A={aNowForReconnect}, X={xNowForReconnect}, connectionFailed={connectionFailed}, lastA={lastAButtonDown}, lastX={lastXButtonDown}, isReconnecting={isReconnecting}, isConnected={isConnected}");
        }

        if (connectionFailed && aNowForReconnect && xNowForReconnect)
        {
            if (!(lastAButtonDown && lastXButtonDown))
            {
                Debug.Log("A+X 边沿检测通过：重置 connectionFailed，重置 reconnectAttempts 并调用 ConnectToServer()");
                connectionFailed = false;
                reconnectAttempts = 0;
                try
                {
                    _ = ConnectToServer();
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"调用 ConnectToServer() 时捕获异常: {ex.Message}");
                }
            }
            else
            {
                Debug.Log("A+X 同时按下但已在上一帧保持，忽略重复触发");
            }
        }
        // 立即更新上一帧按键状态，避免重复触发
        lastAButtonDown = aNowForReconnect;
        lastXButtonDown = xNowForReconnect;

        // 如果连接已彻底失败，停止其他操作
        if (connectionFailed) return;

        if (!isConnected) return;
        
        // 获取头显（VR眼镜）的位置和旋转
        Vector3 headPosition = Vector3.zero;
        Quaternion headRotation = Quaternion.identity;
        
        if (ovrCameraRig != null && ovrCameraRig.centerEyeAnchor != null)
        {
            headPosition = ovrCameraRig.centerEyeAnchor.position;
            headRotation = ovrCameraRig.centerEyeAnchor.rotation;
        }
        
        Quaternion headRotationInverse = Quaternion.Inverse(headRotation);
        
        // 获取手柄在世界坐标系下的位置和旋转
        Vector3 leftControllerWorldPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
        Quaternion leftControllerWorldRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
        
        Vector3 rightControllerWorldPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        Quaternion rightControllerWorldRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);
        
        // 转换为相对于头显的位置和旋转
        Vector3 leftControllerPosition = headRotationInverse * (leftControllerWorldPosition - headPosition);
        Quaternion leftControllerRotation = headRotationInverse * leftControllerWorldRotation;
        
        Vector3 rightControllerPosition = headRotationInverse * (rightControllerWorldPosition - headPosition);
        Quaternion rightControllerRotation = headRotationInverse * rightControllerWorldRotation;
        
        // 计算欧拉角（弧度）
        Vector3 leftEulerRad = leftControllerRotation.eulerAngles * Mathf.Deg2Rad;
        Vector3 rightEulerRad = rightControllerRotation.eulerAngles * Mathf.Deg2Rad;
        
        // 转换为度数方便阅读
        Vector3 leftEulerDeg = leftControllerRotation.eulerAngles;
        Vector3 rightEulerDeg = rightControllerRotation.eulerAngles;
        
        // Debug.Log($"[当前手柄角度] 左手: Roll={leftEulerDeg.x:F2}° Pitch={leftEulerDeg.y:F2}° Yaw={leftEulerDeg.z:F2}° | 右手: Roll={rightEulerDeg.x:F2}° Pitch={rightEulerDeg.y:F2}° Yaw={rightEulerDeg.z:F2}°");
        
        // 检测左摇杆按下(OVRInput.Button.PrimaryThumbstick)
        bool thumbstickDown = OVRInput.Get(OVRInput.Button.PrimaryThumbstick);
        
        if (thumbstickDown)
        {
            if (!isThumbstickPressed)
            {
                // 刚按下，开始计时
                isThumbstickPressed = true;
                thumbstickPressTimer = 0f;
                modeToggleInThisPress = false;  // 重置本次按下的切换标志
            }
            else
            {
                // 持续按下，累加时间
                thumbstickPressTimer += Time.deltaTime;
                
                // 检查是否达到切换时长，且在本次按下中还未切换过
                if (thumbstickPressTimer >= modeToggleDuration && !modeToggleInThisPress)
                {
                    ToggleMode();
                    modeToggleInThisPress = true;  // 标记本次按下已经切换过
                }
            }
        }
        else
        {
            // 松开摇杆，重置状态
            isThumbstickPressed = false;
            thumbstickPressTimer = 0f;
            modeToggleInThisPress = false;  // 重置切换标志，为下一次按下做准备
        }
        
        // 获取左摇杆（PrimaryThumbstick）和右摇杆（SecondaryThumbstick）输入
        Vector2 leftStick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick);
        Vector2 rightStick = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);
        
        // 应用死区：如果某个轴的值小于死区阈值，将其设为0
        if (Mathf.Abs(leftStick.x) < joystickDeadzone) leftStick.x = 0f;
        if (Mathf.Abs(leftStick.y) < joystickDeadzone) leftStick.y = 0f;
        if (Mathf.Abs(rightStick.x) < joystickDeadzone) rightStick.x = 0f;
        if (Mathf.Abs(rightStick.y) < joystickDeadzone) rightStick.y = 0f;
        
        // 检测右摇杆按下(OVRInput.Button.SecondaryThumbstick)，仅在Reset模式下有效
        bool rightThumbstickDown = OVRInput.Get(OVRInput.Button.SecondaryThumbstick);
        
        if (currentMode == ControlMode.Reset && rightThumbstickDown)
        {
            if (!isRightThumbstickPressed)
            {
                // 刚按下，开始计时
                isRightThumbstickPressed = true;
                rightThumbstickPressTimer = 0f;
                resetCommandInThisPress = false;  // 重置本次按下的reset指令标志
            }
            else
            {
                // 持续按下，累加时间
                rightThumbstickPressTimer += Time.deltaTime;
                
                // 检查是否达到发送reset指令的时长，且在本次按下中还未发送过
                if (rightThumbstickPressTimer >= resetCommandDuration && !resetCommandInThisPress)
                {
                    resetRequested = true;  // 标记reset请求
                    resetCommandInThisPress = true;  // 标记本次按下已经发送过reset指令
                }
            }
        }
        else
        {
            // 松开摇杆或离开Reset模式，重置状态
            isRightThumbstickPressed = false;
            rightThumbstickPressTimer = 0f;
            resetCommandInThisPress = false;  // 重置reset指令标志，为下一次按下做准备
        }

        // chassis_speed: vx, vy, w
        float vx = Mathf.Clamp(leftStick.y, -1f, 1f) * maxVx; // 前后
        float vy = Mathf.Clamp(leftStick.x, -1f, 1f) * maxVy; // 左右
        float w = Mathf.Clamp(rightStick.x, -1f, 1f) * maxW; // 右手横向

        // torso_speed: vx, vz, w_pitch, w_yaw
        // vz: 右摇杆竖向（升降）
        float torsoVz = Mathf.Clamp(rightStick.y, -1f, 1f) * maxTorsoVz;
        
        // w_pitch: X和Y键（俯仰）
        bool yButtonDown = OVRInput.Get(OVRInput.Button.Four);     // X键：抬起
        bool xButtonDown = OVRInput.Get(OVRInput.Button.Three);    // Y键：低头
        float torsoWPitch = 0f;
        if (xButtonDown) torsoWPitch -= maxTorsoWPitch;
        if (yButtonDown) torsoWPitch += maxTorsoWPitch;
        
        // w_yaw: 左G和右G（转向）
        bool leftGrip = OVRInput.Get(OVRInput.Button.PrimaryHandTrigger);
        bool rightGrip = OVRInput.Get(OVRInput.Button.SecondaryHandTrigger);
        float torsoWYaw = 0f;
        if (leftGrip) torsoWYaw += maxTorsoWYaw;
        if (rightGrip) torsoWYaw -= maxTorsoWYaw;
        
        // vx: A和B键（前后）
        bool aButtonDown = OVRInput.Get(OVRInput.Button.One);      // A键：后退
        bool bButtonDown = OVRInput.Get(OVRInput.Button.Two);      // B键：前进
        float torsoVx = 0f;
        if (aButtonDown) torsoVx -= maxTorsoVx;
        if (bButtonDown) torsoVx += maxTorsoVx;



        // 检测LT/RT按键，控制夹爪
        bool leftTriggerDown = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger);
        bool rightTriggerDown = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger);
        
        // 左夹爪逻辑
        if (leftTriggerDown)
        {
            leftTriggerTimer += Time.deltaTime;
            // 长按：缓慢减小数值
            currentLeftGripper -= gripperChangeSpeed * Time.deltaTime;
        }
        else
        {
            // 松开瞬间判断是否为短按
            if (leftTriggerTimer > 0f && leftTriggerTimer < gripperShortPressTime)
            {
                currentLeftGripper = 100f; // 短按恢复
            }
            leftTriggerTimer = 0f;
        }
        currentLeftGripper = Mathf.Clamp(currentLeftGripper, 0f, 100f);

        // 右夹爪逻辑
        if (rightTriggerDown)
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

        // 按照设定的时间间隔发送数据
        timer += Time.deltaTime;
        if (timer >= sendInterval)
        {
            timer = 0f;
            if (currentMode == ControlMode.BiManual)
            {
                // BiManual模式下发送机械臂+底盘+躯干
                _ = SendControllerDataAsync(leftControllerPosition, leftControllerRotation, 
                                          rightControllerPosition, rightControllerRotation, vx, vy, w, currentLeftGripper, currentRightGripper,
                                          torsoVx, torsoVz, torsoWPitch, torsoWYaw, resetRequested);
            }
            else
            {
                // Reset模式下只发送底盘+躯干，机械臂数据为零
                _ = SendControllerDataAsync(Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity, vx, vy, w, currentLeftGripper, currentRightGripper,
                                          torsoVx, torsoVz, torsoWPitch, torsoWYaw, resetRequested);
            }
            resetRequested = false;  // 清除reset标志
        }
        
        // 按照设定的时间间隔发送ping指令
        pingTimer += Time.deltaTime;
        if (pingTimer >= pingInterval)
        {
            pingTimer = 0f;
            _ = SendPingAsync();
        }

        // 更新上一帧按钮状态（用于边沿检测）
        lastAButtonDown = aButtonDown;
        lastXButtonDown = xButtonDown;
    }
    
    void ToggleMode()
    {
        if (currentMode == ControlMode.Reset)
        {
            // 从Reset切换到BiManual
            currentMode = ControlMode.BiManual;
            Debug.Log("切换到 BiManual 模式");
            hasLastSend = false;
        }
        else
        {
            // 从BiManual切换到Reset
            currentMode = ControlMode.Reset;
            Debug.Log("切换到 Reset 模式");
            hasLastSend = false;
        }
    }
    
    float NormalizeAngle(float angle)
    {
        while (angle > 180f) angle -= 360f;
        while (angle < -180f) angle += 360f;
        return angle;
    }

    async Task SendControllerDataAsync(Vector3 leftPos, Quaternion leftRot, Vector3 rightPos, Quaternion rightRot, float vx = 0f, float vy = 0f, float w = 0f, float leftGripperValue = 0f, float rightGripperValue = 0f, float torsoVx = 0f, float torsoVz = 0f, float torsoWPitch = 0f, float torsoWYaw = 0f, bool sendReset = false)
    {
        // 如果连接已失败，不再发送
        if (!isConnected || stream == null || connectionFailed) return;
        
        try
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

            Vector3 deltaPosL;
            Vector3 deltaPosR;
            Vector3 deltaEulerL;
            Vector3 deltaEulerR;
            float deltaGripperL;
            float deltaGripperR;

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

            // 构造 send_action 命令，包含chassis_speed、torso_speed、gripper和可选reset
            // 注意：这里改为发送 droll, dpitch, dyaw (6个元素)
            string jsonData = string.Format(
                "{{\"cmd\":\"send_action\",\"action\":{{\"left_ee_pose\":[{0},{1},{2},{3},{4},{5}],\"right_ee_pose\":[{6},{7},{8},{9},{10},{11}],\"left_gripper\":{12},\"right_gripper\":{13},\"chassis_speed\":[{14},{15},{16}],\"torso_speed\":[{17},{18},{19},{20}],\"reset\":{21}}}}}",
                deltaPosL.x, deltaPosL.y, deltaPosL.z, deltaEulerL.x, deltaEulerL.y, deltaEulerL.z,
                deltaPosR.x, deltaPosR.y, deltaPosR.z, deltaEulerR.x, deltaEulerR.y, deltaEulerR.z,
                deltaGripperL, deltaGripperR,
                vx, vy, w,
                torsoVx, torsoVz, torsoWPitch, torsoWYaw,
                sendReset.ToString().ToLower()  // 转为小写 "true" 或 "false"
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
