(*
 ==============================================================================
  FB_ElevatorCtrl — 單台電梯控制功能塊（集選控制 Selective-Collective）
 ==============================================================================
  功能
    * 車廂呼叫 + 各樓上/下呼叫登記與按鈕燈
    * 集選控制：同方向順路停靠，該方向沒有呼叫才反向；待命時先去最近的呼叫
    * 門控制：開門停留、開門鈕/關門鈕、光電/安全觸邊重開、超載不關門、
              關門逾時自動重開重試
    * 安全：安全迴路、上下極限、門鎖（門未關妥不運轉）、運行時間限制、
            樓層感測器順序檢查、上下行互鎖、起動/停車延遲
    * 上電時位置不明 → 低速找樓層；停車超出平層區 → 低速再平層

  每次 PLC 掃描呼叫一次。所有動作都由單一狀態機 eState 決定，
  輸出在最後一段統一依狀態產生，並再經過一次安全互鎖。

  故障碼 nFaultCode
    1 安全迴路開路 / 緊急停止（恢復後自動重新初始化）
    2 運行逾時（馬達運轉過久仍未到達下一樓層）
    3 樓層感測器異常（多個同時 ON、樓層順序錯誤、再平層失敗）
    4 超越上/下極限開關
    5 門鎖 / 門位置訊號異常（運行中關門到位消失，或開門到位與關門到位同時 ON）
    6 開門失敗（開門逾時）
    7 關門失敗（重試 nDoorRetryMax 次仍關不上）
    2~7 需按故障復歸鈕 xFaultReset 才會重新初始化。
 ==============================================================================
*)
FUNCTION_BLOCK FB_ElevatorCtrl
VAR_INPUT
    (* ---------- 參數 ---------- *)
    nNumFloors      : INT  := 4;        (* 服務樓層數 2..8 *)
    tDoorDwell      : TIME := T#3S;     (* 開門停留時間 *)
    tDoorTimeout    : TIME := T#8S;     (* 開門 / 關門動作逾時 *)
    tStartDelay     : TIME := T#500MS;  (* 門關妥到馬達起動的延遲 *)
    tBrakeDelay     : TIME := T#500MS;  (* 馬達停止後的煞車穩定時間 *)
    tRunTimeout     : TIME := T#15S;    (* 高速：相鄰兩樓層間最長運行時間 *)
    tSlowTimeout    : TIME := T#30S;    (* 低速（找樓層 / 再平層）最長運行時間 *)
    nDoorRetryMax   : INT  := 3;        (* 關門失敗重試次數 *)

    (* ---------- 按鈕 ---------- *)
    xCarBtn         : T_FloorBool;      (* 車廂內樓層按鈕 *)
    xHallUpBtn      : T_FloorBool;      (* 各樓「上」呼叫按鈕（頂樓不用） *)
    xHallDnBtn      : T_FloorBool;      (* 各樓「下」呼叫按鈕（底樓不用） *)
    xDoorOpenBtn    : BOOL;             (* 開門鈕 *)
    xDoorCloseBtn   : BOOL;             (* 關門鈕 *)
    xFaultReset     : BOOL;             (* 故障復歸鈕 *)

    (* ---------- 感測器 ---------- *)
    xFloorSensor    : T_FloorBool;      (* 各樓平層感測器：車廂在該樓平層區 = TRUE *)
    xDoorOpenLS     : BOOL;             (* 開門到位 *)
    xDoorClosedLS   : BOOL;             (* 關門到位（門鎖） *)
    xDoorObstruct   : BOOL;             (* 門口光電 / 安全觸邊：TRUE = 有障礙 *)
    xOverload       : BOOL;             (* 超載：TRUE = 超載 *)
    xSafetyOK       : BOOL;             (* 安全迴路（B 接點串聯）：TRUE = 正常 *)
    xUpLimitOK      : BOOL;             (* 上極限開關（B 接點）：TRUE = 未動作 *)
    xDnLimitOK      : BOOL;             (* 下極限開關（B 接點）：TRUE = 未動作 *)
END_VAR
VAR_OUTPUT
    xMotorUp        : BOOL;             (* 馬達上升 *)
    xMotorDown      : BOOL;             (* 馬達下降 *)
    xLowSpeed       : BOOL;             (* 低速（找樓層 / 再平層時與上升或下降同時 ON） *)
    xDoorOpen       : BOOL;             (* 門機開門 *)
    xDoorClose      : BOOL;             (* 門機關門 *)
    xCarCallLamp    : T_FloorBool;      (* 車廂按鈕燈 *)
    xHallUpLamp     : T_FloorBool;      (* 上呼按鈕燈 *)
    xHallDnLamp     : T_FloorBool;      (* 下呼按鈕燈 *)
    xDirUpLamp      : BOOL;             (* 上行方向燈 *)
    xDirDnLamp      : BOOL;             (* 下行方向燈 *)
    xFloorLamp      : T_FloorBool;      (* 樓層指示燈（目前樓層那一個 ON） *)
    nFloor          : INT;              (* 目前樓層，0 = 位置不明 *)
    xBuzzer         : BOOL;             (* 超載蜂鳴器 *)
    xFaultLamp      : BOOL;             (* 故障燈 *)
    nFaultCode      : INT;              (* 故障碼，0 = 正常 *)
    xInService      : BOOL;             (* 正常服務中（可接受呼叫） *)
    eState          : E_ElevState;      (* 目前狀態（監控用） *)
    eDir            : E_Dir;            (* 目前服務方向 *)
END_VAR
VAR CONSTANT
    MAX_FLOORS      : INT  := 8;        (* 必須等於 T_FloorBool 的陣列上限 *)
    T_SETTLE        : TIME := T#1S;     (* 上電後等待輸入穩定的時間 *)
    RELEVEL_MAX     : INT  := 3;        (* 連續再平層次數上限 *)
    FLT_SAFETY      : INT  := 1;
    FLT_RUN_TIME    : INT  := 2;
    FLT_POSITION    : INT  := 3;
    FLT_LIMIT       : INT  := 4;
    FLT_DOOR_LOCK   : INT  := 5;
    FLT_DOOR_OPEN   : INT  := 6;
    FLT_DOOR_CLOSE  : INT  := 7;
END_VAR
VAR
    aCarCall        : T_FloorBool;      (* 已登記的車廂呼叫 *)
    aUpCall         : T_FloorBool;      (* 已登記的上呼 *)
    aDnCall         : T_FloorBool;      (* 已登記的下呼 *)
    nFloors         : INT;              (* 檢查過範圍的樓層數 *)
    nCurFloor       : INT;              (* 目前樓層，0 = 不明 *)
    nSensorFloor    : INT;              (* 平層感測器 ON 的樓層 *)
    nSensorCount    : INT;              (* 同時 ON 的平層感測器數量 *)
    xAtFloor        : BOOL;             (* 車廂在某一樓的平層區內 *)
    xNewFloor       : BOOL;             (* 本次掃描運行中抵達新樓層 *)
    xSeqError       : BOOL;             (* 樓層順序錯誤 *)
    nCallAbove      : INT;              (* 目前樓層以上最近的呼叫樓層，0 = 無 *)
    nCallBelow      : INT;              (* 目前樓層以下最近的呼叫樓層，0 = 無 *)
    xCallHere       : BOOL;             (* 本層有要立刻服務（開門）的呼叫 *)
    xStopHere       : BOOL;             (* 運行中：本層要停 *)
    eMoveDir        : E_Dir;            (* 本次掃描的馬達命令 *)
    eRunDir         : E_Dir;            (* 最近一次運轉方向 *)
    xSlow           : BOOL;             (* 本次掃描使用低速 *)
    xCloseCmd       : BOOL;             (* 找樓層前的關門命令 *)
    xOpenAfterStop  : BOOL;             (* 找樓層完成後開門讓乘客離開 *)
    xDwellHold      : BOOL;             (* 開門停留重新計時 *)
    nDoorRetry      : INT;              (* 關門失敗次數 *)
    nRelevelCnt     : INT;              (* 連續再平層次數 *)
    nNewFault       : INT;              (* 本次掃描偵測到的故障 *)
    nRunRefFloor    : INT;              (* 運行計時參考樓層 *)
    ePrevState      : E_ElevState;      (* 上一次掃描開始時的狀態 *)
    tInState        : TIME;             (* 已停留在目前狀態的時間 *)
    i               : INT;
    tonState        : TON;              (* 狀態停留計時 *)
    tonDwell        : TON;              (* 開門停留計時 *)
    tonRun          : TON;              (* 運行時間限制 *)
    rtReset         : R_TRIG;           (* 故障復歸上升緣 *)
END_VAR

(* ============================================================================
   1. 參數與計時器（每次掃描固定呼叫一次）
   ============================================================================ *)
nFloors := LIMIT(2, nNumFloors, MAX_FLOORS);

(* 狀態停留時間：狀態一改變就從 0 重新計時 *)
tonState(IN := (eState = ePrevState), PT := T#24H);
ePrevState := eState;
tInState := tonState.ET;

(* 開門停留：有人按開門鈕、遮擋、超載或本層再被呼叫時重新計時 *)
tonDwell(IN := (eState = ST_DOOR_OPEN) AND NOT xDwellHold, PT := tDoorDwell);

(* 運行時間限制：馬達運轉中每抵達一個新樓層就重新計時 *)
tonRun(IN := (xMotorUp OR xMotorDown) AND (nCurFloor = nRunRefFloor),
       PT := SEL(xLowSpeed, tRunTimeout, tSlowTimeout));
nRunRefFloor := nCurFloor;

rtReset(CLK := xFaultReset);

(* ============================================================================
   2. 平層感測器與樓層追蹤
   ============================================================================ *)
nSensorCount := 0;
nSensorFloor := 0;
FOR i := 1 TO nFloors DO
    IF xFloorSensor[i] THEN
        nSensorCount := nSensorCount + 1;
        nSensorFloor := i;
    END_IF;
END_FOR;
xAtFloor := (nSensorCount = 1);

xNewFloor := FALSE;
xSeqError := FALSE;
IF xAtFloor AND (nSensorFloor <> nCurFloor) THEN
    IF eState = ST_RUN THEN
        (* 運行中只能依序抵達上一層或下一層 *)
        IF ((eRunDir = DIR_UP) AND (nSensorFloor = nCurFloor + 1))
           OR ((eRunDir = DIR_DOWN) AND (nSensorFloor = nCurFloor - 1)) THEN
            nCurFloor := nSensorFloor;
            xNewFloor := TRUE;
        ELSE
            xSeqError := TRUE;
        END_IF;
    ELSIF (eState <> ST_INIT) AND (eState <> ST_HOMING) AND (eState <> ST_FAULT) THEN
        (* 停止中或再平層時出現別的樓層 *)
        xSeqError := TRUE;
    END_IF;
END_IF;

(* ============================================================================
   3. 故障偵測（優先順序由上而下）
   ============================================================================ *)
nNewFault := 0;
IF NOT xSafetyOK THEN
    nNewFault := FLT_SAFETY;
ELSIF ((xMotorUp OR xMotorDown) AND NOT xDoorClosedLS) OR (xDoorOpenLS AND xDoorClosedLS) THEN
    nNewFault := FLT_DOOR_LOCK;
ELSIF (xMotorUp AND NOT xUpLimitOK) OR (xMotorDown AND NOT xDnLimitOK) THEN
    nNewFault := FLT_LIMIT;
ELSIF (nSensorCount > 1) OR xSeqError THEN
    nNewFault := FLT_POSITION;
ELSIF tonRun.Q THEN
    nNewFault := FLT_RUN_TIME;
END_IF;

IF (nNewFault <> 0) AND (eState <> ST_FAULT) THEN
    nFaultCode := nNewFault;
    eState := ST_FAULT;
END_IF;

(* ============================================================================
   4. 呼叫登記（只在正常服務中接受）
   ============================================================================ *)
xInService := (eState <> ST_INIT) AND (eState <> ST_HOMING) AND (eState <> ST_FAULT);
IF xInService THEN
    FOR i := 1 TO nFloors DO
        IF xCarBtn[i] THEN
            aCarCall[i] := TRUE;
        END_IF;
        IF xHallUpBtn[i] AND (i < nFloors) THEN
            aUpCall[i] := TRUE;
        END_IF;
        IF xHallDnBtn[i] AND (i > 1) THEN
            aDnCall[i] := TRUE;
        END_IF;
    END_FOR;
END_IF;

(* ============================================================================
   5. 呼叫摘要：目前樓層以上 / 以下最近的呼叫
   ============================================================================ *)
nCallAbove := 0;
nCallBelow := 0;
IF nCurFloor >= 1 THEN
    FOR i := nFloors TO nCurFloor + 1 BY -1 DO
        IF aCarCall[i] OR aUpCall[i] OR aDnCall[i] THEN
            nCallAbove := i;
        END_IF;
    END_FOR;
    FOR i := 1 TO nCurFloor - 1 DO
        IF aCarCall[i] OR aUpCall[i] OR aDnCall[i] THEN
            nCallBelow := i;
        END_IF;
    END_FOR;
END_IF;

(* ============================================================================
   6. 停在樓層時：決定方向，並服務本層呼叫（清除呼叫 → 開門）
   ============================================================================ *)
xCallHere := FALSE;
IF ((eState = ST_IDLE) OR (eState = ST_DOOR_OPENING) OR (eState = ST_DOOR_OPEN)
    OR (eState = ST_DOOR_CLOSING) OR (eState = ST_START))
   AND xAtFloor AND (nCurFloor >= 1) THEN

    (* 門已關（待命）時才可以反向：原方向已無呼叫 → 反向或無方向。
       門開著時保持原方向，避免剛進車廂的乘客被載往反方向。 *)
    IF eState = ST_IDLE THEN
        IF eDir = DIR_UP THEN
            IF NOT ((nCallAbove > 0) OR aUpCall[nCurFloor]) THEN
                IF aDnCall[nCurFloor] OR (nCallBelow > 0) THEN
                    eDir := DIR_DOWN;
                ELSE
                    eDir := DIR_NONE;
                END_IF;
            END_IF;
        ELSIF eDir = DIR_DOWN THEN
            IF NOT ((nCallBelow > 0) OR aDnCall[nCurFloor]) THEN
                IF aUpCall[nCurFloor] OR (nCallAbove > 0) THEN
                    eDir := DIR_UP;
                ELSE
                    eDir := DIR_NONE;
                END_IF;
            END_IF;
        END_IF;
    END_IF;

    (* 無方向：本層的廳外呼叫優先，否則前往最近的呼叫（距離相同先往上） *)
    IF eDir = DIR_NONE THEN
        IF aUpCall[nCurFloor] THEN
            eDir := DIR_UP;
        ELSIF aDnCall[nCurFloor] THEN
            eDir := DIR_DOWN;
        ELSIF (nCallAbove > 0)
              AND ((nCallBelow = 0) OR ((nCallAbove - nCurFloor) <= (nCurFloor - nCallBelow))) THEN
            eDir := DIR_UP;
        ELSIF nCallBelow > 0 THEN
            eDir := DIR_DOWN;
        END_IF;
    END_IF;

    (* 本層要服務的呼叫：車廂呼叫 + 與目前方向相同的廳外呼叫 *)
    xCallHere := aCarCall[nCurFloor]
                 OR ((eDir = DIR_UP) AND aUpCall[nCurFloor])
                 OR ((eDir = DIR_DOWN) AND aDnCall[nCurFloor]);
    IF xCallHere THEN
        aCarCall[nCurFloor] := FALSE;
        IF eDir = DIR_UP THEN
            aUpCall[nCurFloor] := FALSE;
        ELSIF eDir = DIR_DOWN THEN
            aDnCall[nCurFloor] := FALSE;
        END_IF;
    END_IF;
END_IF;

(* ============================================================================
   7. 主狀態機
   ============================================================================ *)
eMoveDir := DIR_NONE;
xSlow := FALSE;
xCloseCmd := FALSE;
xDwellHold := FALSE;

CASE eState OF

ST_INIT:
    (* 等輸入穩定後，依平層感測器判斷位置 *)
    eDir := DIR_NONE;
    IF tInState >= T_SETTLE THEN
        nDoorRetry := 0;
        nRelevelCnt := 0;
        xOpenAfterStop := FALSE;
        IF xAtFloor THEN
            nCurFloor := nSensorFloor;
            IF xDoorClosedLS THEN
                eState := ST_IDLE;
            ELSE
                eState := ST_DOOR_OPEN;         (* 門沒關：停留後自動關門 *)
            END_IF;
        ELSE
            nCurFloor := 0;
            eRunDir := DIR_NONE;
            eState := ST_HOMING;
        END_IF;
    END_IF;

ST_HOMING:
    (* 位置不明：先關門，再低速往下找樓層（若已壓到下極限則往上） *)
    IF NOT xDoorClosedLS THEN
        xCloseCmd := NOT xDoorObstruct;         (* 不在平層區不可開門，有障礙就暫停關門 *)
        IF tInState >= tDoorTimeout THEN
            nFaultCode := FLT_DOOR_CLOSE;
            eState := ST_FAULT;
        END_IF;
    ELSIF xAtFloor THEN
        nCurFloor := nSensorFloor;
        xOpenAfterStop := TRUE;
        eState := ST_STOPPING;
    ELSE
        IF eRunDir = DIR_NONE THEN
            IF xDnLimitOK THEN
                eRunDir := DIR_DOWN;
            ELSE
                eRunDir := DIR_UP;
            END_IF;
        END_IF;
        eMoveDir := eRunDir;
        xSlow := TRUE;
    END_IF;

ST_IDLE:
    nDoorRetry := 0;
    IF NOT xAtFloor THEN
        (* 車廂離開平層區 → 再平層 *)
        nRelevelCnt := nRelevelCnt + 1;
        IF nRelevelCnt > RELEVEL_MAX THEN
            nFaultCode := FLT_POSITION;
            eState := ST_FAULT;
        ELSE
            IF eRunDir = DIR_DOWN THEN
                eRunDir := DIR_UP;
            ELSE
                eRunDir := DIR_DOWN;
            END_IF;
            eState := ST_RELEVEL;
        END_IF;
    ELSIF xCallHere OR xDoorOpenBtn OR xOverload OR xOpenAfterStop THEN
        xOpenAfterStop := FALSE;
        eState := ST_DOOR_OPENING;
    ELSIF NOT xDoorClosedLS THEN
        eState := ST_DOOR_CLOSING;              (* 門沒有關妥 → 再關一次 *)
    ELSIF eDir <> DIR_NONE THEN
        eState := ST_START;
    END_IF;

ST_DOOR_OPENING:
    IF xDoorOpenLS THEN
        eState := ST_DOOR_OPEN;
    ELSIF tInState >= tDoorTimeout THEN
        nFaultCode := FLT_DOOR_OPEN;
        eState := ST_FAULT;
    END_IF;

ST_DOOR_OPEN:
    nRelevelCnt := 0;
    xDwellHold := xDoorOpenBtn OR xDoorObstruct OR xOverload OR xCallHere;
    IF xDoorCloseBtn AND NOT xDwellHold THEN
        eState := ST_DOOR_CLOSING;              (* 關門鈕：提前關門 *)
    ELSIF tonDwell.Q AND NOT xDwellHold THEN
        eState := ST_DOOR_CLOSING;              (* 停留時間到 *)
    END_IF;

ST_DOOR_CLOSING:
    IF xDoorObstruct OR xDoorOpenBtn OR xOverload OR xCallHere THEN
        eState := ST_DOOR_OPENING;              (* 有障礙 / 按開門 / 超載 / 本層再呼叫 → 重新開門 *)
    ELSIF xDoorClosedLS THEN
        nDoorRetry := 0;
        eState := ST_IDLE;
    ELSIF tInState >= tDoorTimeout THEN
        nDoorRetry := nDoorRetry + 1;
        IF nDoorRetry >= nDoorRetryMax THEN
            nFaultCode := FLT_DOOR_CLOSE;
            eState := ST_FAULT;
        ELSE
            eState := ST_DOOR_OPENING;          (* 關門逾時 → 重新開門再試 *)
        END_IF;
    END_IF;

ST_START:
    IF xCallHere OR xDoorOpenBtn OR xOverload THEN
        eState := ST_DOOR_OPENING;              (* 起動前本層又有呼叫 → 開門 *)
    ELSIF (NOT xDoorClosedLS) OR (NOT xAtFloor) OR (eDir = DIR_NONE) THEN
        eState := ST_IDLE;
    ELSIF tInState >= tStartDelay THEN
        IF ((eDir = DIR_UP) AND NOT xUpLimitOK) OR ((eDir = DIR_DOWN) AND NOT xDnLimitOK) THEN
            nFaultCode := FLT_LIMIT;
            eState := ST_FAULT;
        ELSE
            eRunDir := eDir;
            nRelevelCnt := 0;
            eState := ST_RUN;
        END_IF;
    END_IF;

ST_RUN:
    eMoveDir := eRunDir;
    IF xNewFloor THEN
        (* 抵達新樓層：車廂呼叫、同方向廳外呼叫、或前方已無呼叫就停 *)
        xStopHere := aCarCall[nCurFloor]
                     OR ((eRunDir = DIR_UP) AND (aUpCall[nCurFloor] OR (nCallAbove = 0)))
                     OR ((eRunDir = DIR_DOWN) AND (aDnCall[nCurFloor] OR (nCallBelow = 0)));
        IF xStopHere THEN
            eMoveDir := DIR_NONE;
            eState := ST_STOPPING;
        END_IF;
    END_IF;

ST_STOPPING:
    IF tInState >= tBrakeDelay THEN
        IF xAtFloor THEN
            eState := ST_IDLE;
        ELSE
            (* 滑行超出平層區 → 反方向低速再平層 *)
            nRelevelCnt := nRelevelCnt + 1;
            IF nRelevelCnt > RELEVEL_MAX THEN
                nFaultCode := FLT_POSITION;
                eState := ST_FAULT;
            ELSE
                IF eRunDir = DIR_DOWN THEN
                    eRunDir := DIR_UP;
                ELSE
                    eRunDir := DIR_DOWN;
                END_IF;
                eState := ST_RELEVEL;
            END_IF;
        END_IF;
    END_IF;

ST_RELEVEL:
    IF xAtFloor THEN
        eState := ST_STOPPING;
    ELSE
        eMoveDir := eRunDir;
        xSlow := TRUE;
    END_IF;

ST_FAULT:
    IF nFaultCode = FLT_SAFETY THEN
        IF xSafetyOK THEN                       (* 安全迴路恢復 → 自動重新初始化 *)
            nFaultCode := 0;
            eState := ST_INIT;
        END_IF;
    ELSIF rtReset.Q AND xSafetyOK THEN          (* 其他故障 → 按復歸鈕 *)
        nFaultCode := 0;
        eState := ST_INIT;
    END_IF;

END_CASE;

(* 故障中（含本次掃描剛進入故障）：清除所有呼叫與方向 *)
IF eState = ST_FAULT THEN
    FOR i := 1 TO MAX_FLOORS DO
        aCarCall[i] := FALSE;
        aUpCall[i] := FALSE;
        aDnCall[i] := FALSE;
    END_FOR;
    eDir := DIR_NONE;
    xOpenAfterStop := FALSE;
END_IF;

(* ============================================================================
   8. 輸出（最後一道軟體互鎖；上下行接觸器另須硬體互鎖）
   ============================================================================ *)
xMotorUp   := (eMoveDir = DIR_UP) AND xSafetyOK AND xDoorClosedLS AND xUpLimitOK
              AND (eState <> ST_FAULT);
xMotorDown := (eMoveDir = DIR_DOWN) AND xSafetyOK AND xDoorClosedLS AND xDnLimitOK
              AND (eState <> ST_FAULT);
xLowSpeed  := xSlow AND (xMotorUp OR xMotorDown);

(* 開門：只在車廂停止且位於平層區時 *)
xDoorOpen  := (eState = ST_DOOR_OPENING) AND xAtFloor AND xSafetyOK
              AND NOT xMotorUp AND NOT xMotorDown AND NOT xDoorOpenLS;
xDoorClose := ((eState = ST_DOOR_CLOSING) OR xCloseCmd) AND xSafetyOK
              AND NOT xDoorOpen AND NOT xDoorClosedLS;

xInService := (eState <> ST_INIT) AND (eState <> ST_HOMING) AND (eState <> ST_FAULT);
FOR i := 1 TO MAX_FLOORS DO
    xCarCallLamp[i] := aCarCall[i];
    xHallUpLamp[i] := aUpCall[i];
    xHallDnLamp[i] := aDnCall[i];
    xFloorLamp[i] := (i = nCurFloor);
END_FOR;
nFloor := nCurFloor;
xDirUpLamp := xInService AND (eDir = DIR_UP);
xDirDnLamp := xInService AND (eDir = DIR_DOWN);
xBuzzer := xOverload AND xInService;
xFaultLamp := (eState = ST_FAULT);

END_FUNCTION_BLOCK
