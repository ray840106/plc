(*
 ==============================================================================
  組態：10 ms 週期工作執行 PRG_Elevator
  （OpenPLC / Beremiz 需要本段；CODESYS、TIA Portal、GX Works 等請在
    軟體的「工作 / Task / OB1」設定中呼叫 PRG_Elevator，不需要本檔。）
 ==============================================================================
*)
CONFIGURATION Config0
    RESOURCE Res0 ON PLC
        TASK MainTask(INTERVAL := T#10MS, PRIORITY := 0);
        PROGRAM Elevator WITH MainTask : PRG_Elevator;
    END_RESOURCE
END_CONFIGURATION
