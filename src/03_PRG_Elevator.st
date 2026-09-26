(*
 ==============================================================================
  PRG_Elevator — 4 層電梯主程式：實體 I/O 對應 + 呼叫 FB_ElevatorCtrl
 ==============================================================================
  位址採 IEC 61131-3 格式（CODESYS / OpenPLC 等）。換成其他廠牌時，
  只要修改本程式的位址（例如三菱 X0/Y0、西門子 %I0.0/%Q0.0），
  FB_ElevatorCtrl 不需要改。I/O 對照表見 README.md。

  安全相關輸入（安全迴路、上下極限）請接 B 接點（常閉），
  斷線時 PLC 會視為動作而停機（失效安全）。
 ==============================================================================
*)
PROGRAM PRG_Elevator
VAR
    (* ---------------- 輸入 ---------------- *)
    I_Car1          AT %IX0.0 : BOOL;   (* 車廂按鈕 1F *)
    I_Car2          AT %IX0.1 : BOOL;   (* 車廂按鈕 2F *)
    I_Car3          AT %IX0.2 : BOOL;   (* 車廂按鈕 3F *)
    I_Car4          AT %IX0.3 : BOOL;   (* 車廂按鈕 4F *)
    I_Up1           AT %IX0.4 : BOOL;   (* 1F 上呼 *)
    I_Up2           AT %IX0.5 : BOOL;   (* 2F 上呼 *)
    I_Up3           AT %IX0.6 : BOOL;   (* 3F 上呼 *)
    I_Dn2           AT %IX0.7 : BOOL;   (* 2F 下呼 *)
    I_Dn3           AT %IX1.0 : BOOL;   (* 3F 下呼 *)
    I_Dn4           AT %IX1.1 : BOOL;   (* 4F 下呼 *)
    I_Floor1        AT %IX1.2 : BOOL;   (* 1F 平層感測器 *)
    I_Floor2        AT %IX1.3 : BOOL;   (* 2F 平層感測器 *)
    I_Floor3        AT %IX1.4 : BOOL;   (* 3F 平層感測器 *)
    I_Floor4        AT %IX1.5 : BOOL;   (* 4F 平層感測器 *)
    I_DoorOpenLS    AT %IX1.6 : BOOL;   (* 開門到位 *)
    I_DoorClosedLS  AT %IX1.7 : BOOL;   (* 關門到位（門鎖） *)
    I_DoorOpenBtn   AT %IX2.0 : BOOL;   (* 開門鈕 *)
    I_DoorCloseBtn  AT %IX2.1 : BOOL;   (* 關門鈕 *)
    I_DoorObstruct  AT %IX2.2 : BOOL;   (* 門口光電 / 安全觸邊（有障礙 = ON） *)
    I_Overload      AT %IX2.3 : BOOL;   (* 超載（超載 = ON） *)
    I_SafetyOK      AT %IX2.4 : BOOL;   (* 安全迴路 / 緊急停止（B 接點，正常 = ON） *)
    I_UpLimitOK     AT %IX2.5 : BOOL;   (* 上極限（B 接點，正常 = ON） *)
    I_DnLimitOK     AT %IX2.6 : BOOL;   (* 下極限（B 接點，正常 = ON） *)
    I_FaultReset    AT %IX2.7 : BOOL;   (* 故障復歸 *)
END_VAR
VAR
    (* ---------------- 輸出 ---------------- *)
    Q_MotorUp       AT %QX0.0 : BOOL;   (* 馬達上升 *)
    Q_MotorDown     AT %QX0.1 : BOOL;   (* 馬達下降 *)
    Q_LowSpeed      AT %QX0.2 : BOOL;   (* 低速 *)
    Q_DoorOpen      AT %QX0.3 : BOOL;   (* 開門 *)
    Q_DoorClose     AT %QX0.4 : BOOL;   (* 關門 *)
    Q_DirUp         AT %QX0.5 : BOOL;   (* 上行方向燈 *)
    Q_DirDown       AT %QX0.6 : BOOL;   (* 下行方向燈 *)
    Q_FaultLamp     AT %QX0.7 : BOOL;   (* 故障燈 *)
    Q_CarLamp1      AT %QX1.0 : BOOL;   (* 車廂按鈕燈 1F *)
    Q_CarLamp2      AT %QX1.1 : BOOL;   (* 車廂按鈕燈 2F *)
    Q_CarLamp3      AT %QX1.2 : BOOL;   (* 車廂按鈕燈 3F *)
    Q_CarLamp4      AT %QX1.3 : BOOL;   (* 車廂按鈕燈 4F *)
    Q_UpLamp1       AT %QX1.4 : BOOL;   (* 1F 上呼燈 *)
    Q_UpLamp2       AT %QX1.5 : BOOL;   (* 2F 上呼燈 *)
    Q_UpLamp3       AT %QX1.6 : BOOL;   (* 3F 上呼燈 *)
    Q_DnLamp2       AT %QX1.7 : BOOL;   (* 2F 下呼燈 *)
    Q_DnLamp3       AT %QX2.0 : BOOL;   (* 3F 下呼燈 *)
    Q_DnLamp4       AT %QX2.1 : BOOL;   (* 4F 下呼燈 *)
    Q_Buzzer        AT %QX2.2 : BOOL;   (* 超載蜂鳴器 *)
    Q_Seg_A         AT %QX2.3 : BOOL;   (* 樓層顯示 BCD 1（接 7 段解碼器 A） *)
    Q_Seg_B         AT %QX2.4 : BOOL;   (* 樓層顯示 BCD 2（B） *)
    Q_Seg_C         AT %QX2.5 : BOOL;   (* 樓層顯示 BCD 4（C） *)
    Q_Seg_D         AT %QX2.6 : BOOL;   (* 樓層顯示 BCD 8（D） *)
    Q_InService     AT %QX2.7 : BOOL;   (* 正常服務中 *)
END_VAR
VAR
    (* ---------------- 給人機介面（HMI）的數值 ---------------- *)
    MW_Floor        AT %MW0   : INT;    (* 目前樓層，0 = 位置不明 *)
    MW_FaultCode    AT %MW1   : INT;    (* 故障碼，0 = 正常 *)
END_VAR
VAR
    fbElevator      : FB_ElevatorCtrl;
    aCarBtn         : T_FloorBool;
    aHallUpBtn      : T_FloorBool;
    aHallDnBtn      : T_FloorBool;
    aFloorSensor    : T_FloorBool;
END_VAR

(* ---------- 輸入整理成陣列 ---------- *)
aCarBtn[1] := I_Car1;
aCarBtn[2] := I_Car2;
aCarBtn[3] := I_Car3;
aCarBtn[4] := I_Car4;

aHallUpBtn[1] := I_Up1;
aHallUpBtn[2] := I_Up2;
aHallUpBtn[3] := I_Up3;

aHallDnBtn[2] := I_Dn2;
aHallDnBtn[3] := I_Dn3;
aHallDnBtn[4] := I_Dn4;

aFloorSensor[1] := I_Floor1;
aFloorSensor[2] := I_Floor2;
aFloorSensor[3] := I_Floor3;
aFloorSensor[4] := I_Floor4;

(* ---------- 電梯控制 ---------- *)
fbElevator(
    nNumFloors    := 4,
    tDoorDwell    := T#3S,
    tDoorTimeout  := T#8S,
    tStartDelay   := T#500MS,
    tBrakeDelay   := T#500MS,
    tRunTimeout   := T#15S,
    tSlowTimeout  := T#30S,
    nDoorRetryMax := 3,
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

(* ---------- 輸出 ---------- *)
Q_MotorUp   := fbElevator.xMotorUp;
Q_MotorDown := fbElevator.xMotorDown;
Q_LowSpeed  := fbElevator.xLowSpeed;
Q_DoorOpen  := fbElevator.xDoorOpen;
Q_DoorClose := fbElevator.xDoorClose;
Q_DirUp     := fbElevator.xDirUpLamp;
Q_DirDown   := fbElevator.xDirDnLamp;
Q_FaultLamp := fbElevator.xFaultLamp;

Q_CarLamp1 := fbElevator.xCarCallLamp[1];
Q_CarLamp2 := fbElevator.xCarCallLamp[2];
Q_CarLamp3 := fbElevator.xCarCallLamp[3];
Q_CarLamp4 := fbElevator.xCarCallLamp[4];

Q_UpLamp1 := fbElevator.xHallUpLamp[1];
Q_UpLamp2 := fbElevator.xHallUpLamp[2];
Q_UpLamp3 := fbElevator.xHallUpLamp[3];

Q_DnLamp2 := fbElevator.xHallDnLamp[2];
Q_DnLamp3 := fbElevator.xHallDnLamp[3];
Q_DnLamp4 := fbElevator.xHallDnLamp[4];

Q_Buzzer    := fbElevator.xBuzzer;
Q_InService := fbElevator.xInService;

(* 樓層 → BCD（例如 3 樓：A=1、B=1、C=0、D=0） *)
Q_Seg_A := (fbElevator.nFloor MOD 2) = 1;
Q_Seg_B := ((fbElevator.nFloor / 2) MOD 2) = 1;
Q_Seg_C := ((fbElevator.nFloor / 4) MOD 2) = 1;
Q_Seg_D := ((fbElevator.nFloor / 8) MOD 2) = 1;

MW_Floor     := fbElevator.nFloor;
MW_FaultCode := fbElevator.nFaultCode;

END_PROGRAM
