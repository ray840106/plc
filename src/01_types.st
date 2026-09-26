(*
 ==============================================================================
  電梯控制程式 — 資料型別
  Elevator control — data types                         IEC 61131-3 結構化文字
 ==============================================================================
*)
TYPE
    (* 每層一個 BOOL 的陣列。最多 8 層；超過 8 層請把上限 8 改大，
       並同步修改 FB_ElevatorCtrl 內的常數 MAX_FLOORS。 *)
    T_FloorBool : ARRAY[1..8] OF BOOL;

    (* 電梯主狀態 *)
    E_ElevState : (
        ST_INIT,            (* 上電初始化，等待輸入穩定後判斷位置 *)
        ST_HOMING,          (* 位置不明：低速尋找最近樓層 *)
        ST_IDLE,            (* 停在樓層、門已關、決定下一步 *)
        ST_DOOR_OPENING,    (* 開門中 *)
        ST_DOOR_OPEN,       (* 開門停留 *)
        ST_DOOR_CLOSING,    (* 關門中 *)
        ST_START,           (* 門已關妥，啟動前延遲 *)
        ST_RUN,             (* 運行中 *)
        ST_STOPPING,        (* 停車 / 煞車穩定延遲 *)
        ST_RELEVEL,         (* 停車位置超出平層區：低速再平層 *)
        ST_FAULT            (* 故障停機 *)
    );

    (* 方向 *)
    E_Dir : (
        DIR_NONE,           (* 無方向 *)
        DIR_UP,             (* 上 *)
        DIR_DOWN            (* 下 *)
    );
END_TYPE
