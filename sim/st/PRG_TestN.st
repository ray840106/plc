(*
  模擬測試專用：以位元組 I/O 驅動 FB_ElevatorCtrl，樓層數由 %IW10 決定（2..8），
  用來測試 4 層以外的樓層數。實機不需要本程式。
    %IB10 車廂按鈕  %IB11 上呼  %IB12 下呼  %IB13 平層感測器   (bit0 = 1F)
    %QB12 車廂燈    %QB13 上呼燈 %QB14 下呼燈 %QB15 樓層燈
*)
PROGRAM PRG_TestN
VAR
    IB_Car          AT %IB10   : BYTE;
    IB_Up           AT %IB11   : BYTE;
    IB_Dn           AT %IB12   : BYTE;
    IB_Sensor       AT %IB13   : BYTE;
    I_DoorOpenLS    AT %IX14.0 : BOOL;
    I_DoorClosedLS  AT %IX14.1 : BOOL;
    I_DoorOpenBtn   AT %IX14.2 : BOOL;
    I_DoorCloseBtn  AT %IX14.3 : BOOL;
    I_DoorObstruct  AT %IX14.4 : BOOL;
    I_Overload      AT %IX14.5 : BOOL;
    I_SafetyOK      AT %IX14.6 : BOOL;
    I_UpLimitOK     AT %IX14.7 : BOOL;
    I_DnLimitOK     AT %IX15.0 : BOOL;
    I_FaultReset    AT %IX15.1 : BOOL;
    IW_NumFloors    AT %IW10   : INT;
END_VAR
VAR
    Q_MotorUp       AT %QX10.0 : BOOL;
    Q_MotorDown     AT %QX10.1 : BOOL;
    Q_LowSpeed      AT %QX10.2 : BOOL;
    Q_DoorOpen      AT %QX10.3 : BOOL;
    Q_DoorClose     AT %QX10.4 : BOOL;
    Q_DirUp         AT %QX10.5 : BOOL;
    Q_DirDown       AT %QX10.6 : BOOL;
    Q_FaultLamp     AT %QX10.7 : BOOL;
    Q_Buzzer        AT %QX11.0 : BOOL;
    Q_InService     AT %QX11.1 : BOOL;
    QB_CarLamp      AT %QB12   : BYTE;
    QB_UpLamp       AT %QB13   : BYTE;
    QB_DnLamp       AT %QB14   : BYTE;
    QB_FloorLamp    AT %QB15   : BYTE;
    QW_Floor        AT %QW10   : INT;
    QW_FaultCode    AT %QW11   : INT;
END_VAR
VAR
    fb              : FB_ElevatorCtrl;
    aCarBtn         : T_FloorBool;
    aHallUpBtn      : T_FloorBool;
    aHallDnBtn      : T_FloorBool;
    aFloorSensor    : T_FloorBool;
    bMask           : BYTE;
    i               : INT;
END_VAR

bMask := 16#01;
FOR i := 1 TO 8 DO
    aCarBtn[i]      := (IB_Car AND bMask) <> 16#00;
    aHallUpBtn[i]   := (IB_Up AND bMask) <> 16#00;
    aHallDnBtn[i]   := (IB_Dn AND bMask) <> 16#00;
    aFloorSensor[i] := (IB_Sensor AND bMask) <> 16#00;
    bMask := SHL(bMask, 1);
END_FOR;

fb(nNumFloors    := IW_NumFloors,
   xCarBtn       := aCarBtn,
   xHallUpBtn    := aHallUpBtn,
   xHallDnBtn    := aHallDnBtn,
   xDoorOpenBtn  := I_DoorOpenBtn,
   xDoorCloseBtn := I_DoorCloseBtn,
   xFaultReset   := I_FaultReset,
   xFloorSensor  := aFloorSensor,
   xDoorOpenLS   := I_DoorOpenLS,
   xDoorClosedLS := I_DoorClosedLS,
   xDoorObstruct := I_DoorObstruct,
   xOverload     := I_Overload,
   xSafetyOK     := I_SafetyOK,
   xUpLimitOK    := I_UpLimitOK,
   xDnLimitOK    := I_DnLimitOK);

Q_MotorUp   := fb.xMotorUp;
Q_MotorDown := fb.xMotorDown;
Q_LowSpeed  := fb.xLowSpeed;
Q_DoorOpen  := fb.xDoorOpen;
Q_DoorClose := fb.xDoorClose;
Q_DirUp     := fb.xDirUpLamp;
Q_DirDown   := fb.xDirDnLamp;
Q_FaultLamp := fb.xFaultLamp;
Q_Buzzer    := fb.xBuzzer;
Q_InService := fb.xInService;

QB_CarLamp := 16#00;
QB_UpLamp := 16#00;
QB_DnLamp := 16#00;
QB_FloorLamp := 16#00;
bMask := 16#01;
FOR i := 1 TO 8 DO
    IF fb.xCarCallLamp[i] THEN QB_CarLamp := QB_CarLamp OR bMask; END_IF;
    IF fb.xHallUpLamp[i] THEN QB_UpLamp := QB_UpLamp OR bMask; END_IF;
    IF fb.xHallDnLamp[i] THEN QB_DnLamp := QB_DnLamp OR bMask; END_IF;
    IF fb.xFloorLamp[i] THEN QB_FloorLamp := QB_FloorLamp OR bMask; END_IF;
    bMask := SHL(bMask, 1);
END_FOR;

QW_Floor     := fb.nFloor;
QW_FaultCode := fb.nFaultCode;

END_PROGRAM
