"""電梯 PLC 模擬器。

以 ctypes 載入 build.sh 編譯出的 libelevator_sim.so（真正的 ST 程式經 matiec
轉成的 C），每 10 ms 掃描一次；同時用簡單的物理模型模擬車廂、門與各種感測器，
並在每次掃描檢查安全條件（上下互鎖、門未關不可運轉、不在平層區不可開門……）。
"""

import ctypes
import os
import subprocess

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
LIB_PATH = os.path.join(SIM_DIR, "build", "libelevator_sim.so")

SCAN_MS = 10
DT = SCAN_MS / 1000.0

# 與 src/01_types.st 中列舉的順序相同
STATES = ["INIT", "HOMING", "IDLE", "DOOR_OPENING", "DOOR_OPEN", "DOOR_CLOSING",
          "START", "RUN", "STOPPING", "RELEVEL", "FAULT"]
DIRS = ["NONE", "UP", "DOWN"]

_lib = None


def load_lib(rebuild=True):
    """編譯（若需要）並載入 PLC 共享函式庫。"""
    global _lib
    if _lib is None:
        if rebuild or not os.path.exists(LIB_PATH):
            subprocess.run([os.path.join(SIM_DIR, "build.sh")], check=True)
        _lib = ctypes.CDLL(LIB_PATH)
        for name in ("elevator_state", "elevator_dir", "testn_state", "testn_dir"):
            getattr(_lib, name).restype = ctypes.c_int
    return _lib


class InvariantError(AssertionError):
    pass


# ---------------------------------------------------------------------------
# I/O 對應
# ---------------------------------------------------------------------------
class MainIO:
    """實機程式 PRG_Elevator（src/03_PRG_Elevator.st）的 4 層 I/O 位址。"""

    n_floors = 4

    def __init__(self, lib):
        def x(addr):
            return ctypes.c_uint8.in_dll(lib, "__store__" + addr.replace(".", "_"))

        def w(addr):
            return ctypes.c_int16.in_dll(lib, "__store__" + addr)

        self.lib = lib
        self.car_btn = [x("IX0.0"), x("IX0.1"), x("IX0.2"), x("IX0.3")]
        self.up_btn = [x("IX0.4"), x("IX0.5"), x("IX0.6"), None]
        self.dn_btn = [None, x("IX0.7"), x("IX1.0"), x("IX1.1")]
        self.sensor = [x("IX1.2"), x("IX1.3"), x("IX1.4"), x("IX1.5")]
        self.i_bits = {
            "door_open_ls": x("IX1.6"), "door_closed_ls": x("IX1.7"),
            "door_open_btn": x("IX2.0"), "door_close_btn": x("IX2.1"),
            "obstruct": x("IX2.2"), "overload": x("IX2.3"),
            "safety_ok": x("IX2.4"), "up_limit_ok": x("IX2.5"),
            "dn_limit_ok": x("IX2.6"), "fault_reset": x("IX2.7"),
        }
        self.q_bits = {
            "motor_up": x("QX0.0"), "motor_down": x("QX0.1"), "low_speed": x("QX0.2"),
            "door_open": x("QX0.3"), "door_close": x("QX0.4"),
            "dir_up": x("QX0.5"), "dir_dn": x("QX0.6"), "fault_lamp": x("QX0.7"),
            "buzzer": x("QX2.2"), "in_service": x("QX2.7"),
        }
        self.car_lamp = [x("QX1.0"), x("QX1.1"), x("QX1.2"), x("QX1.3")]
        self.up_lamp = [x("QX1.4"), x("QX1.5"), x("QX1.6"), None]
        self.dn_lamp = [None, x("QX1.7"), x("QX2.0"), x("QX2.1")]
        self.bcd = [x("QX2.3"), x("QX2.4"), x("QX2.5"), x("QX2.6")]
        self.mw_floor = w("MW0")
        self.mw_fault = w("MW1")

    def write(self, inp):
        for i in range(self.n_floors):
            self.car_btn[i].value = inp["car"][i]
            if self.up_btn[i] is not None:
                self.up_btn[i].value = inp["up"][i]
            if self.dn_btn[i] is not None:
                self.dn_btn[i].value = inp["dn"][i]
            self.sensor[i].value = inp["sensor"][i]
        for k, v in self.i_bits.items():
            v.value = inp[k]

    def read(self):
        out = {k: bool(v.value) for k, v in self.q_bits.items()}
        n = self.n_floors
        out["car_lamp"] = [bool(self.car_lamp[i].value) for i in range(n)]
        out["up_lamp"] = [bool(self.up_lamp[i].value) if self.up_lamp[i] else False for i in range(n)]
        out["dn_lamp"] = [bool(self.dn_lamp[i].value) if self.dn_lamp[i] else False for i in range(n)]
        out["floor"] = self.mw_floor.value
        out["fault_code"] = self.mw_fault.value
        out["bcd_floor"] = sum(1 << b for b in range(4) if self.bcd[b].value)
        return out

    def state(self):
        return STATES[self.lib.elevator_state()]

    def direction(self):
        return DIRS[self.lib.elevator_dir()]


class TestNIO:
    """測試用程式 PRG_TestN（sim/st/PRG_TestN.st），樓層數 2..8。"""

    def __init__(self, lib, n_floors):
        def x(addr):
            return ctypes.c_uint8.in_dll(lib, "__store__" + addr.replace(".", "_"))

        def w(addr):
            return ctypes.c_int16.in_dll(lib, "__store__" + addr)

        self.lib = lib
        self.n_floors = n_floors
        self.ib_car, self.ib_up, self.ib_dn, self.ib_sensor = x("IB10"), x("IB11"), x("IB12"), x("IB13")
        self.iw_floors = w("IW10")
        self.i_bits = {
            "door_open_ls": x("IX14.0"), "door_closed_ls": x("IX14.1"),
            "door_open_btn": x("IX14.2"), "door_close_btn": x("IX14.3"),
            "obstruct": x("IX14.4"), "overload": x("IX14.5"),
            "safety_ok": x("IX14.6"), "up_limit_ok": x("IX14.7"),
            "dn_limit_ok": x("IX15.0"), "fault_reset": x("IX15.1"),
        }
        self.q_bits = {
            "motor_up": x("QX10.0"), "motor_down": x("QX10.1"), "low_speed": x("QX10.2"),
            "door_open": x("QX10.3"), "door_close": x("QX10.4"),
            "dir_up": x("QX10.5"), "dir_dn": x("QX10.6"), "fault_lamp": x("QX10.7"),
            "buzzer": x("QX11.0"), "in_service": x("QX11.1"),
        }
        self.qb_car, self.qb_up, self.qb_dn, self.qb_floor = x("QB12"), x("QB13"), x("QB14"), x("QB15")
        self.qw_floor = w("QW10")
        self.qw_fault = w("QW11")

    @staticmethod
    def _mask(bits):
        return sum(1 << i for i, b in enumerate(bits) if b)

    def write(self, inp):
        self.iw_floors.value = self.n_floors
        self.ib_car.value = self._mask(inp["car"])
        self.ib_up.value = self._mask(inp["up"])
        self.ib_dn.value = self._mask(inp["dn"])
        self.ib_sensor.value = self._mask(inp["sensor"])
        for k, v in self.i_bits.items():
            v.value = inp[k]

    def read(self):
        out = {k: bool(v.value) for k, v in self.q_bits.items()}
        n = self.n_floors
        out["car_lamp"] = [bool(self.qb_car.value >> i & 1) for i in range(n)]
        out["up_lamp"] = [bool(self.qb_up.value >> i & 1) for i in range(n)]
        out["dn_lamp"] = [bool(self.qb_dn.value >> i & 1) for i in range(n)]
        out["floor_lamp"] = [bool(self.qb_floor.value >> i & 1) for i in range(8)]
        out["floor"] = self.qw_floor.value
        out["fault_code"] = self.qw_fault.value
        return out

    def state(self):
        return STATES[self.lib.testn_state()]

    def direction(self):
        return DIRS[self.lib.testn_dir()]


# ---------------------------------------------------------------------------
# 受控體（車廂 / 門 / 感測器）
# ---------------------------------------------------------------------------
class Plant:
    """車廂位置單位 mm，1F 平層位置 = 0。"""

    def __init__(self, n_floors, start_pos, door=0.0, floor_height=3000.0,
                 hi_speed=1000.0, lo_speed=200.0, zone=30.0,
                 coast_hi=10.0, coast_lo=2.0, door_time=2.0, limit_margin=150.0):
        self.n = n_floors
        self.h = floor_height
        self.pos = float(start_pos)
        self.door = float(door)             # 0 = 全關，1 = 全開
        self.hi_speed = hi_speed
        self.lo_speed = lo_speed
        self.zone = zone                    # 平層區半寬
        self.coast_hi = coast_hi            # 馬達停止後滑行距離
        self.coast_lo = coast_lo
        self.door_time = door_time          # 開 / 關門全程時間 (s)
        self.limit_margin = limit_margin    # 極限開關位於端站外多少 mm
        self.coast_left = 0.0
        self.coast_dir = 0
        self.coast_speed = 0.0
        self.powered = False
        # ---- 故障注入 ----
        self.motor_stalled = False          # 馬達轉但車廂不動
        self.dead_sensors = set()           # 壞掉（永遠 OFF）的平層感測器
        self.stuck_sensors = set()          # 卡住（永遠 ON）的平層感測器
        self.door_jam_at = None             # 關門卡在此位置（例如 0.3）
        self.door_cannot_open = False       # 門打不開
        self.door_lock_broken = False       # 門鎖接點斷開（關門到位訊號消失）
        self.limit_tripped = set()          # 強制動作的極限開關 {"up", "dn"}
        self.open_ls_stuck = False          # 開門到位開關卡住（永遠 ON）

    def level(self, floor):
        return (floor - 1) * self.h

    def nearest_floor(self):
        return min(self.n, max(1, int(round(self.pos / self.h)) + 1))

    def in_zone(self):
        return abs(self.pos - self.level(self.nearest_floor())) <= self.zone

    def moving(self):
        return self.powered or self.coast_left > 0

    def sensors(self):
        return [((abs(self.pos - self.level(f)) <= self.zone and f not in self.dead_sensors)
                 or f in self.stuck_sensors) for f in range(1, self.n + 1)]

    def door_open_ls(self):
        return self.door >= 1.0 or self.open_ls_stuck

    def door_closed_ls(self):
        return self.door <= 0.0 and not self.door_lock_broken

    def up_limit_ok(self):
        return self.pos <= self.level(self.n) + self.limit_margin and "up" not in self.limit_tripped

    def dn_limit_ok(self):
        return self.pos >= self.level(1) - self.limit_margin and "dn" not in self.limit_tripped

    def step(self, out, dt):
        up, down = out["motor_up"], out["motor_down"]
        if up or down:
            d = 1 if up else -1
            speed = self.lo_speed if out["low_speed"] else self.hi_speed
            if not self.motor_stalled:
                self.pos += d * speed * dt
            self.powered = True
            self.coast_dir = d
            self.coast_speed = speed
            self.coast_left = 0.0 if self.motor_stalled else (
                self.coast_lo if out["low_speed"] else self.coast_hi)
        else:
            self.powered = False
            if self.coast_left > 0:
                move = min(self.coast_left, self.coast_speed * dt)
                self.pos += self.coast_dir * move
                self.coast_left -= move
        # 緩衝器（實體止擋）
        self.pos = max(self.level(1) - 2 * self.limit_margin,
                       min(self.level(self.n) + 2 * self.limit_margin, self.pos))

        if out["door_open"] and not self.door_cannot_open:
            self.door = min(1.0, self.door + dt / self.door_time)
        if out["door_close"]:
            lowest = self.door_jam_at if self.door_jam_at is not None else 0.0
            if self.door > lowest:
                self.door = max(lowest, self.door - dt / self.door_time)


# ---------------------------------------------------------------------------
# 模擬器
# ---------------------------------------------------------------------------
class Sim:
    def __init__(self, program="main", n_floors=4, start_floor=1, start_pos=None,
                 door=0.0, **plant_kw):
        lib = load_lib(rebuild=False)
        lib.plc_init()
        if program == "main":
            self.io = MainIO(lib)
            if n_floors != 4:
                raise ValueError("PRG_Elevator 固定 4 層")
        else:
            self.io = TestNIO(lib, n_floors)
        self.n = n_floors
        if start_pos is None:
            start_pos = (start_floor - 1) * plant_kw.get("floor_height", 3000.0)
        self.plant = Plant(n_floors, start_pos, door=door, **plant_kw)
        self.t = 0.0
        # 測試可直接控制的輸入
        self.level_inputs = {
            "door_open_btn": False, "door_close_btn": False, "obstruct": False,
            "overload": False, "safety_ok": True, "fault_reset": False,
        }
        self._presses = []              # [(kind, floor_or_None, release_time)]
        self.check_invariants = True
        self.check_travel_limits = True
        self.out = None
        self.state = "INIT"
        self.direction = "NONE"
        self.door_open_events = []      # (t, floor, dir_up_lamp, dir_dn_lamp)
        self.state_log = []             # (t, state)
        self.listeners = []             # 每次掃描後呼叫 fn(sim)
        self._was_open = self.plant.door_open_ls()
        self._last_state = None

    # ---- 按鈕 ----
    def press(self, kind, floor=None, duration=0.2):
        """kind: car / up / dn（需要 floor），door_open_btn / door_close_btn / fault_reset。"""
        self._presses.append((kind, floor, self.t + duration))

    def _inputs(self):
        n = self.n
        inp = {
            "car": [False] * n, "up": [False] * n, "dn": [False] * n,
            "sensor": self.plant.sensors(),
            "door_open_ls": self.plant.door_open_ls(),
            "door_closed_ls": self.plant.door_closed_ls(),
            "up_limit_ok": self.plant.up_limit_ok(),
            "dn_limit_ok": self.plant.dn_limit_ok(),
        }
        inp.update(self.level_inputs)
        still = []
        for kind, floor, until in self._presses:
            if floor is None:
                inp[kind] = True
            else:
                inp[kind][floor - 1] = True
            if self.t + DT < until:
                still.append((kind, floor, until))
        self._presses = still
        return inp

    # ---- 執行 ----
    def step(self):
        self.io.write(self._inputs())
        self.io.lib.plc_scan(SCAN_MS)
        out = self.io.read()
        self.out = out
        self.state = self.io.state()
        self.direction = self.io.direction()
        if self.state != self._last_state:
            self.state_log.append((round(self.t, 2), self.state))
            self._last_state = self.state
        if self.check_invariants:
            self._check(out)
        self.plant.step(out, DT)
        self.t += DT
        is_open = self.plant.door_open_ls()
        if is_open and not self._was_open:
            self.door_open_events.append(
                (round(self.t, 2), self.plant.nearest_floor(), out["dir_up"], out["dir_dn"]))
        self._was_open = is_open
        for fn in self.listeners:
            fn(self)

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()

    def run_until(self, cond, timeout, what="condition"):
        steps = int(round(timeout / DT))
        for _ in range(steps):
            self.step()
            if cond(self):
                return self.t
        raise AssertionError("%s not reached within %.1f s (t=%.2f, state=%s, pos=%.0f, floor=%s)"
                             % (what, timeout, self.t, self.state, self.plant.pos,
                                self.out and self.out["floor"]))

    def wait_state(self, state, timeout=60.0):
        return self.run_until(lambda s: s.state == state, timeout, "state " + state)

    def wait_door_open_at(self, floor, timeout=60.0):
        self.run_until(lambda s: s.plant.door_open_ls() and s.plant.nearest_floor() == floor,
                       timeout, "door open at %dF" % floor)
        return self.door_open_events[-1]

    def served_floors(self):
        return [e[1] for e in self.door_open_events]

    # ---- 安全檢查 ----
    def _check(self, out):
        p = self.plant
        motor = out["motor_up"] or out["motor_down"]
        errs = []
        if out["motor_up"] and out["motor_down"]:
            errs.append("上升與下降同時輸出")
        if out["door_open"] and out["door_close"]:
            errs.append("開門與關門同時輸出")
        if motor and p.door > 0.0:
            errs.append("門未關妥馬達卻在運轉 (door=%.2f)" % p.door)
        if motor and (out["door_open"] or out["door_close"]):
            errs.append("運轉中門機動作")
        if out["door_open"] and (p.moving() or not p.in_zone()):
            errs.append("車廂移動中或不在平層區卻開門 (pos=%.0f)" % p.pos)
        if out["dir_up"] and out["dir_dn"]:
            errs.append("上下方向燈同時亮")
        if self.check_travel_limits and not (p.up_limit_ok() and p.dn_limit_ok()):
            errs.append("車廂超出極限位置 (pos=%.0f)" % p.pos)
        if "bcd_floor" in out and out["bcd_floor"] != out["floor"]:
            errs.append("BCD 樓層顯示 %d 與樓層 %d 不符" % (out["bcd_floor"], out["floor"]))
        if "floor_lamp" in out:
            expect = [i + 1 == out["floor"] for i in range(8)]
            if out["floor_lamp"] != expect:
                errs.append("樓層燈與樓層不符")
        if errs:
            raise InvariantError("t=%.2f state=%s: %s" % (self.t, self.state, "; ".join(errs)))
