/*
 * 模擬用 PLC 執行環境：把 matiec (iec2c) 由 ST 產生的 C 程式包成共享函式庫，
 * 讓 Python 以固定掃描週期驅動，時間完全由模擬器控制（結果可重現）。
 */
#include "iec_std_lib.h"
#include "accessor.h"
#include "POUS.h"

TIME __CURRENT_TIME;

/* 為每一個有位址的變數 (%IX0.0 ...) 配置記憶體，Python 以 __store__IX0_0 存取 */
#define __LOCATED_VAR(type, name, ...) type __store##name; type *name = &__store##name;
#include "LOCATED_VARIABLES.h"
#undef __LOCATED_VAR

extern void config_init__(void);
extern void config_run__(unsigned long tick);
extern PRG_ELEVATOR_data__ RES0__ELEVATOR;
extern PRG_TESTN_data__ RES0__TESTN;

static unsigned long tick;

void plc_init(void)
{
    __CURRENT_TIME.tv_sec = 0;
    __CURRENT_TIME.tv_nsec = 0;
    tick = 0;
    config_init__();
}

/* 時間前進 ms 毫秒後執行一次掃描 */
void plc_scan(int ms)
{
    __CURRENT_TIME.tv_nsec += (long)ms * 1000000L;
    while (__CURRENT_TIME.tv_nsec >= 1000000000L) {
        __CURRENT_TIME.tv_nsec -= 1000000000L;
        __CURRENT_TIME.tv_sec += 1;
    }
    config_run__(tick++);
}

/* 監控用：FB 內部狀態（E_ElevState / E_Dir 的序號） */
int elevator_state(void) { return RES0__ELEVATOR.FBELEVATOR.ESTATE.value; }
int elevator_dir(void)   { return RES0__ELEVATOR.FBELEVATOR.EDIR.value; }
int testn_state(void)    { return RES0__TESTN.FB.ESTATE.value; }
int testn_dir(void)      { return RES0__TESTN.FB.EDIR.value; }
