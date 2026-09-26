(* 模擬測試組態：同時執行實機程式 PRG_Elevator（4 層）與測試用 PRG_TestN（2..8 層） *)
CONFIGURATION Config0
    RESOURCE Res0 ON PLC
        TASK MainTask(INTERVAL := T#10MS, PRIORITY := 0);
        PROGRAM Elevator WITH MainTask : PRG_Elevator;
        PROGRAM TestN WITH MainTask : PRG_TestN;
    END_RESOURCE
END_CONFIGURATION
