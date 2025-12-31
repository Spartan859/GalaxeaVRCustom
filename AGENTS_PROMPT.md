modify code:
1. remove logic: hold left G or right G to spin torso
2. remove logic: 长按右摇杆 to reset
3. add logic: hold 右摇杆 and push it left or right to spin torso left/right. 
4. add logic: when holding 右摇杆, pushing it left or right will not spin 底座
5. remove logic: 同时按住XY/AB to copy "left to right" / "right to left".
6. add logic: 同时按住AB to reset.

modify code:
1. SendControllerDataAsync去除sendReset及相关处理逻辑。
2. 定义MyTeleopEvent，包含枚举类型TeleopEventType { SUCCESS = "success", RERECORD = "rerecord", TERMINATE = "terminate" , NONE = "none"}。, 
3. SendControllerDataAsync添加sendIsIntervention参数，传入boolean值表示是否为人工干预。如果是人工干预则为true，否则为false。在发送的tcp包中添加"isIntervention": sendIsIntervention字段
4. SendControllerDataAsync添加sendEvent参数，传入MyTeleopEvent类型的对象。在发送的tcp包中添加"event": sendEvent(转为字符串) 字段。

modify code:
1. 添加UpperControlMode，含有Normal和Event两个状态。
2. Normal状态下，sendIsIntervention为True。sendEvent为NONE
3. 按键按下定义为下降沿；按键弹起定义为上升沿。
4. Event状态下，sendIsIntervention为False，sendEvent默认为NONE；
5. 检测A键，若当前帧出现下降沿，当前帧sendEvent设为SUCCESS。再依次以相同逻辑检测B和X，分别对应RERECORD和TERMINATE。此逻辑称为CheckButtonAndSetEvent。
6. Normal状态下，保持现在状态机不变；Event状态下，使用另一个状态循环，只进行CheckButtonAndSetEvent和SendAndPing以及其他必要的操作。
7. Normal和Event通过监听“同时按下左右摇杆”这个组合键的下降沿来切换。切换时，需要打印日志。

modify code:
1. 去除ControlMode及相关处理逻辑，去除HandleModeSwitch
2. 直接在HandleSendAndPing中检查_leftGrip和_rightGrip的状态，若_leftGrip为False则设sendLeftPos为False，若_rightGrip为False则设sendRightPos为False。