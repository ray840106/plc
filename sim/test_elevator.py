"""電梯 PLC 程式模擬測試。

4 層的測試同時跑兩個版本：ST 實機程式（…ST）與三菱 FX3U 階梯圖（…FX3U）。

執行：  python3 -m unittest -v        （在 sim/ 目錄下）
第一次執行會自動下載並編譯 matiec，約需 1~2 分鐘。
"""

import random
import unittest

import elevator_sim as es
import fx3u

H = 3000.0  # 樓高 (mm)


def setUpModule():
    es.load_lib(rebuild=True)


def idle_sim(**kw):
    s = es.Sim(**kw)
    s.wait_state("IDLE", 5)
    return s


def call_lamps_off(s):
    o = s.out
    return not (any(o["car_lamp"]) or any(o["up_lamp"]) or any(o["dn_lamp"]))


def settled(s):
    """所有呼叫都服務完、門已關、車廂停止。"""
    return s.state == "IDLE" and call_lamps_off(s) and s.plant.door <= 0.0


class Backend:
    """測試混入類別：PROGRAM 決定要測哪一個版本。"""

    PROGRAM = "main"

    def sim(self, **kw):
        return es.Sim(program=self.PROGRAM, **kw)

    def idle(self, **kw):
        s = self.sim(**kw)
        s.wait_state("IDLE", 5)
        return s


# ---------------------------------------------------------------------------
# 一般運轉
# ---------------------------------------------------------------------------
class NormalService(Backend):

    def test_power_up_at_floor(self):
        s = self.sim(start_floor=2)
        s.run(0.5)
        self.assertEqual(s.state, "INIT")
        self.assertFalse(s.out["in_service"])
        s.wait_state("IDLE", 2)
        self.assertEqual(s.out["floor"], 2)
        self.assertTrue(s.out["in_service"])
        self.assertEqual(s.out["fault_code"], 0)
        s.run(10)
        self.assertEqual(s.state, "IDLE")
        self.assertEqual(s.door_open_events, [])
        self.assertAlmostEqual(s.plant.pos, H)

    def test_car_call(self):
        s = self.idle()
        s.press("car", 4)
        s.run(0.05)
        self.assertTrue(s.out["car_lamp"][3])
        self.assertTrue(s.out["dir_up"])
        s.wait_door_open_at(4, 30)
        self.assertFalse(s.out["car_lamp"][3])
        s.run_until(settled, 20, "idle")
        self.assertEqual(s.served_floors(), [4])
        self.assertEqual(s.out["floor"], 4)

    def test_hall_down_call_announces_down(self):
        s = self.idle()
        s.press("dn", 4)
        _, floor, up, dn = s.wait_door_open_at(4, 30)
        self.assertEqual((floor, up, dn), (4, False, True))

    def test_hall_up_call_announces_up(self):
        s = self.idle(start_floor=3)
        s.press("up", 1)
        _, floor, up, dn = s.wait_door_open_at(1, 30)
        self.assertEqual((floor, up, dn), (1, True, False))

    def test_collective_up_trip(self):
        """上行途中：同方向的上呼順路停，反方向的下呼等回程才停。"""
        s = self.idle()
        s.press("car", 4)
        s.run_until(lambda s: s.state == "RUN" and s.plant.pos > 500, 10)
        s.press("up", 3)
        s.press("dn", 2)
        s.run_until(settled, 90, "all served")
        self.assertEqual(s.served_floors(), [3, 4, 2])
        ups = {e[1]: (e[2], e[3]) for e in s.door_open_events}
        self.assertEqual(ups[3], (True, False))   # 3F 開門顯示「上」
        self.assertEqual(ups[4], (False, True))   # 4F 之後要往下
        self.assertEqual(ups[2], (False, True))

    def test_collective_down_trip(self):
        s = self.idle(start_floor=4)
        s.press("car", 1)
        s.run_until(lambda s: s.state == "RUN" and s.plant.pos < 3 * H - 500, 10)
        s.press("dn", 2)
        s.press("up", 3)
        s.run_until(settled, 90, "all served")
        self.assertEqual(s.served_floors(), [2, 1, 3])
        ev = {e[1]: (e[2], e[3]) for e in s.door_open_events}
        self.assertEqual(ev[2], (False, True))
        self.assertEqual(ev[1], (True, False))
        self.assertEqual(ev[3], (True, False))

    def test_no_stop_without_call(self):
        """中間樓層沒有呼叫時直接通過。"""
        s = self.idle()
        s.press("car", 4)
        stops = []
        s.listeners.append(lambda s: s.state == "STOPPING" and (not stops or stops[-1] != s.out["floor"])
                           and stops.append(s.out["floor"]))
        s.wait_door_open_at(4, 30)
        self.assertEqual(stops, [4])

    def test_both_hall_calls_same_floor(self):
        """同一樓上下都有人：先服務上行，關門後反向再開門服務下行。"""
        s = self.idle()
        s.press("up", 2)
        s.press("dn", 2)
        s.run_until(settled, 60, "all served")
        self.assertEqual([(e[1], e[2], e[3]) for e in s.door_open_events],
                         [(2, True, False), (2, False, True)])

    def test_nearest_call_first(self):
        s = self.idle(start_floor=2)
        s.press("up", 1)
        s.press("dn", 4)
        s.run_until(settled, 90, "all served")
        self.assertEqual(s.served_floors(), [1, 4])

    def test_equal_distance_goes_up_first(self):
        s = self.idle(start_floor=2)
        s.press("up", 1)
        s.press("dn", 3)
        s.run_until(settled, 90, "all served")
        self.assertEqual(s.served_floors(), [3, 1])

    def test_call_at_current_floor_opens_door(self):
        s = self.idle(start_floor=2)
        s.press("up", 2)
        s.run(0.05)
        self.assertFalse(s.out["up_lamp"][1])     # 立即服務，燈不留著
        self.assertEqual(s.state, "DOOR_OPENING")
        _, floor, up, dn = s.wait_door_open_at(2, 5)
        self.assertEqual((floor, up), (2, True))
        self.assertEqual(s.plant.pos, H)          # 沒有移動

    def test_door_open_button_when_idle(self):
        s = self.idle(start_floor=3)
        s.press("door_open_btn")
        s.wait_door_open_at(3, 5)
        s.run_until(settled, 15, "door closed")

    def test_same_floor_car_button_reopens_closing_door(self):
        s = self.idle(start_floor=2)
        s.press("car", 2)
        s.wait_state("DOOR_CLOSING", 10)
        s.run(0.5)
        s.press("car", 2)
        s.run(0.05)
        self.assertEqual(s.state, "DOOR_OPENING")
        self.assertFalse(s.out["car_lamp"][1])

    def test_calls_blocked_until_in_service(self):
        s = self.sim(start_floor=1)
        s.press("car", 3, duration=0.5)
        s.run(0.6)
        self.assertFalse(s.out["car_lamp"][2])
        s.wait_state("IDLE", 2)
        s.run(3)
        self.assertEqual(s.state, "IDLE")
        self.assertEqual(s.plant.pos, 0.0)


# ---------------------------------------------------------------------------
# 門
# ---------------------------------------------------------------------------
class Door(Backend):

    def open_door(self, floor=1):
        s = self.idle(start_floor=floor)
        s.press("door_open_btn")
        s.wait_state("DOOR_OPEN", 5)
        return s

    def test_dwell_time(self):
        s = self.open_door()
        t_open = s.t
        t_close = s.wait_state("DOOR_CLOSING", 10)
        self.assertAlmostEqual(t_close - t_open, 3.0, delta=0.1)

    def test_close_button_skips_dwell(self):
        s = self.open_door()
        s.run(0.5)
        s.press("door_close_btn")
        t0 = s.t
        t_close = s.wait_state("DOOR_CLOSING", 2)
        self.assertLess(t_close - t0, 0.1)

    def test_open_button_holds_and_reopens(self):
        s = self.open_door()
        s.level_inputs["door_open_btn"] = True
        s.run(10)
        self.assertEqual(s.state, "DOOR_OPEN")
        s.level_inputs["door_open_btn"] = False
        s.wait_state("DOOR_CLOSING", 5)
        s.run(0.5)
        s.press("door_open_btn")
        s.run(0.05)
        self.assertEqual(s.state, "DOOR_OPENING")
        self.assertTrue(s.out["door_open"])

    def test_obstruction_reopens_and_holds(self):
        s = self.open_door()
        s.wait_state("DOOR_CLOSING", 5)
        s.run(0.5)
        s.level_inputs["obstruct"] = True
        s.run(0.05)
        self.assertEqual(s.state, "DOOR_OPENING")
        s.run(10)
        self.assertEqual(s.state, "DOOR_OPEN")
        s.level_inputs["obstruct"] = False
        t0 = s.t
        t_close = s.wait_state("DOOR_CLOSING", 5)
        self.assertAlmostEqual(t_close - t0, 3.0, delta=0.1)

    def test_overload_keeps_door_open(self):
        s = self.open_door()
        s.level_inputs["overload"] = True
        closing = []
        s.listeners.append(lambda s: s.level_inputs["overload"] and s.out["door_close"]
                           and closing.append(s.t))
        s.press("car", 3)
        s.run(20)
        self.assertEqual(s.state, "DOOR_OPEN")
        self.assertTrue(s.out["buzzer"])
        self.assertEqual(s.plant.pos, 0.0)
        s.press("door_close_btn")                 # 超載時關門鈕無效
        s.run(1)
        self.assertEqual(s.state, "DOOR_OPEN")
        self.assertEqual(closing, [])
        s.level_inputs["overload"] = False
        s.run(0.05)
        self.assertFalse(s.out["buzzer"])
        s.wait_door_open_at(3, 30)

    def test_overload_while_closing_reopens(self):
        s = self.open_door()
        s.wait_state("DOOR_CLOSING", 5)
        s.run(0.3)
        s.level_inputs["overload"] = True
        s.run(0.01)
        self.assertEqual(s.state, "DOOR_OPENING")
        s.run(10)
        self.assertEqual(s.state, "DOOR_OPEN")

    def test_close_timeout_retries_then_fault(self):
        s = self.open_door()
        s.plant.door_jam_at = 0.3
        s.press("car", 3)
        s.wait_state("FAULT", 60)
        self.assertEqual(s.out["fault_code"], 7)
        self.assertTrue(s.out["fault_lamp"])
        self.assertFalse(any(s.out["car_lamp"]))
        reopen = [st for _, st in s.state_log].count("DOOR_OPENING")
        self.assertEqual(reopen, 3)               # 第一次開門 + 重試 2 次
        s.plant.door_jam_at = None
        s.run(1)
        self.assertEqual(s.state, "FAULT")        # 未按復歸不會恢復
        s.press("fault_reset")
        s.wait_state("INIT", 1)
        s.run_until(settled, 20, "recovered")
        self.assertEqual(s.out["fault_code"], 0)

    def test_door_open_output_needs_floor_zone(self):
        """開門中平層訊號消失（感測器故障）→ 開門輸出立即停止（輸出段的互鎖）。"""
        s = self.idle(start_floor=2)
        s.press("up", 2)
        s.run_until(lambda s: s.out["door_open"] and s.plant.door > 0.3, 5)
        s.plant.dead_sensors = {2}
        s.run(0.01)
        self.assertFalse(s.out["door_open"])
        s.run(1)
        self.assertFalse(s.out["door_open"])
        self.assertLess(s.plant.door, 1.0)

    def test_close_button_ignored_while_obstructed(self):
        s = self.open_door()
        s.level_inputs["obstruct"] = True
        closing = []
        s.listeners.append(lambda s: s.out["door_close"] and closing.append(s.t))
        s.press("door_close_btn", duration=1.0)
        s.run(5)
        self.assertEqual(closing, [])             # 連一瞬間都不可以關門
        self.assertEqual(s.state, "DOOR_OPEN")

    def test_open_timeout_fault(self):
        s = self.idle(start_floor=2)
        s.plant.door_cannot_open = True
        s.press("up", 2)
        s.wait_state("FAULT", 15)
        self.assertEqual(s.out["fault_code"], 6)


# ---------------------------------------------------------------------------
# 安全與故障
# ---------------------------------------------------------------------------
class Safety(Backend):

    def test_safety_circuit_open_during_run(self):
        s = self.idle()
        s.press("car", 4)
        s.run_until(lambda s: s.state == "RUN" and s.plant.pos > 1.5 * H, 20)
        s.level_inputs["safety_ok"] = False
        s.run(0.01)
        self.assertFalse(s.out["motor_up"] or s.out["motor_down"])
        self.assertEqual(s.state, "FAULT")
        self.assertEqual(s.out["fault_code"], 1)
        self.assertFalse(any(s.out["car_lamp"]))
        s.press("car", 4)
        s.run(3)
        self.assertEqual(s.state, "FAULT")
        self.assertFalse(any(s.out["car_lamp"]))
        s.level_inputs["safety_ok"] = True       # 安全迴路恢復 → 自動找樓層
        s.wait_state("HOMING", 3)
        s.run(0.05)
        self.assertTrue(s.out["motor_down"] and s.out["low_speed"])
        _, floor, _, _ = s.wait_door_open_at(2, 20)
        self.assertEqual(floor, 2)
        self.assertEqual(s.out["fault_code"], 0)
        s.run_until(settled, 20, "idle")

    def test_power_up_between_floors_homes_down(self):
        s = self.sim(start_pos=1.5 * H)
        s.wait_state("HOMING", 2)
        s.press("car", 4)
        s.run(0.5)
        self.assertTrue(s.out["motor_down"] and s.out["low_speed"])
        self.assertFalse(s.out["car_lamp"][3])    # 找樓層期間不接受呼叫
        s.wait_door_open_at(2, 20)
        self.assertEqual(s.out["floor"], 2)
        s.run_until(settled, 20, "idle")

    def test_power_up_between_floors_with_door_open(self):
        s = self.sim(start_pos=2.5 * H, door=0.5)
        s.wait_state("HOMING", 2)
        s.run(0.05)
        self.assertTrue(s.out["door_close"])
        self.assertFalse(s.out["motor_down"])
        s.level_inputs["obstruct"] = True         # 有障礙：暫停關門但不會開門
        s.run(0.5)
        self.assertFalse(s.out["door_close"] or s.out["door_open"])
        s.level_inputs["obstruct"] = False
        s.wait_door_open_at(3, 30)

    def test_power_up_at_floor_with_door_open(self):
        s = self.sim(start_floor=3, door=1.0)
        s.wait_state("DOOR_OPEN", 2)
        s.run_until(settled, 15, "door closed")

    def test_homing_up_from_bottom_limit(self):
        s = self.sim(start_pos=-200.0)
        s.check_travel_limits = False
        s.wait_state("HOMING", 2)
        s.run(0.05)
        self.assertTrue(s.out["motor_up"] and s.out["low_speed"])
        s.wait_door_open_at(1, 10)

    def test_run_timeout(self):
        s = self.idle()
        s.plant.motor_stalled = True
        s.press("car", 3)
        t_run = s.wait_state("RUN", 5)
        t_fault = s.wait_state("FAULT", 20)
        self.assertEqual(s.out["fault_code"], 2)
        self.assertAlmostEqual(t_fault - t_run, 15.0, delta=0.1)
        s.plant.motor_stalled = False
        s.press("fault_reset")
        s.wait_state("IDLE", 3)
        self.assertEqual(s.out["floor"], 1)

    def test_run_timer_restarts_each_floor(self):
        """整趟 1F→4F 超過 tRunTimeout 也不會誤判，因為每到一層就重新計時。"""
        s = self.idle(hi_speed=250.0)              # 每層 12 秒，整趟 36 秒
        s.press("car", 4)
        s.wait_door_open_at(4, 60)
        self.assertEqual(s.out["fault_code"], 0)

    def test_skipped_floor_sensor(self):
        s = self.idle()
        s.plant.dead_sensors = {3}
        s.press("car", 4)
        s.wait_state("FAULT", 30)
        self.assertEqual(s.out["fault_code"], 3)
        self.assertFalse(s.out["motor_up"])

    def test_two_floor_sensors_on(self):
        s = self.idle()
        s.plant.stuck_sensors = {3}
        s.run(0.05)
        self.assertEqual(s.state, "FAULT")
        self.assertEqual(s.out["fault_code"], 3)

    def test_overtravel_top_limit(self):
        s = self.idle(start_floor=3)
        s.plant.dead_sensors = {4}
        s.check_travel_limits = False
        s.press("car", 4)
        s.wait_state("FAULT", 20)
        self.assertEqual(s.out["fault_code"], 4)
        self.assertFalse(s.out["motor_up"])
        self.assertLess(s.plant.pos, 3 * H + 200)

    def test_door_lock_lost_during_run(self):
        s = self.idle()
        s.press("car", 3)
        s.run_until(lambda s: s.state == "RUN" and s.plant.pos > 1000, 20)
        s.plant.door_lock_broken = True
        s.run(0.02)
        self.assertEqual(s.state, "FAULT")
        self.assertEqual(s.out["fault_code"], 5)
        self.assertFalse(s.out["motor_up"])

    def test_fault_priority_safety_first(self):
        """同一次掃描發生兩種故障時記錄優先權高的（安全迴路 → 可自動恢復）。"""
        s = self.idle()
        s.press("car", 3)
        s.run_until(lambda s: s.state == "RUN" and s.plant.pos > 1000, 20)
        s.level_inputs["safety_ok"] = False
        s.plant.door_lock_broken = True
        s.run(0.01)
        self.assertEqual(s.out["fault_code"], 1)
        s.plant.door_lock_broken = False
        s.level_inputs["safety_ok"] = True
        s.wait_state("HOMING", 3)

    def test_door_limit_switches_both_on(self):
        s = self.idle()
        s.plant.open_ls_stuck = True
        s.run(0.02)
        self.assertEqual(s.state, "FAULT")
        self.assertEqual(s.out["fault_code"], 5)

    def test_homing_timeout_uses_slow_limit(self):
        s = self.sim(start_pos=1.5 * H)
        s.plant.motor_stalled = True
        t_home = s.wait_state("HOMING", 2)
        t_fault = s.wait_state("FAULT", 40)
        self.assertEqual(s.out["fault_code"], 2)
        self.assertAlmostEqual(t_fault - t_home, 30.0, delta=0.1)

    def test_limit_switch_blocks_start(self):
        s = self.idle(start_floor=2)
        s.plant.limit_tripped = {"up"}            # 上極限開關誤動作
        s.check_travel_limits = False
        s.press("car", 4)
        s.wait_state("FAULT", 10)
        self.assertEqual(s.out["fault_code"], 4)
        self.assertEqual(s.plant.pos, H)          # 沒有起動

    def test_relevel_after_overshoot(self):
        s = self.idle(coast_hi=80.0)               # 滑行超出平層區（±30 mm）
        s.press("car", 3)
        s.wait_door_open_at(3, 30)
        states = [st for _, st in s.state_log]
        self.assertIn("RELEVEL", states)
        self.assertLessEqual(abs(s.plant.pos - 2 * H), 30.0)

    def test_relevel_gives_up(self):
        s = self.idle(coast_hi=80.0, coast_lo=80.0)
        s.press("car", 3)
        s.wait_state("FAULT", 60)
        self.assertEqual(s.out["fault_code"], 3)
        self.assertEqual([st for _, st in s.state_log].count("RELEVEL"), 3)


# ---------------------------------------------------------------------------
# 其他樓層數（測試用程式 PRG_TestN）
# ---------------------------------------------------------------------------
class TestFloorCount(unittest.TestCase):

    def test_round_trip(self):
        for n in (2, 3, 5, 8):
            with self.subTest(floors=n):
                s = idle_sim(program="testn", n_floors=n)
                s.press("car", n)
                s.wait_door_open_at(n, 20 + 5 * n)
                s.press("dn", n)                  # 頂樓：下呼有效
                s.press("up", 1)
                s.run_until(settled, 30 + 5 * n, "served")
                self.assertEqual(s.served_floors()[-1], 1)
                self.assertEqual(s.out["floor"], 1)

    def test_invalid_buttons_ignored(self):
        s = idle_sim(program="testn", n_floors=5)
        s.press("up", 5)                          # 頂樓沒有上呼
        s.press("dn", 1)                          # 底樓沒有下呼
        s.run(1)
        self.assertTrue(call_lamps_off(s))
        self.assertEqual(s.state, "IDLE")


# ---------------------------------------------------------------------------
# 隨機乘客壓力測試
# ---------------------------------------------------------------------------
class Passenger:
    def __init__(self, t, origin, dest):
        self.t_arrive = t
        self.origin = origin
        self.dest = dest
        self.up = dest > origin
        self.state = "waiting"
        self.t_board = None
        self.t_done = None
        self.press_car_at = None


class Traffic:

    def run_traffic(self, program, n_floors, minutes, seed, rate_per_min):
        rng = random.Random(seed)
        s = idle_sim(program=program, n_floors=n_floors)
        people = []
        prev = {"up": [False] * n_floors, "dn": [False] * n_floors, "car": [False] * n_floors}
        end_arrivals = minutes * 60.0
        p_arrival = rate_per_min / 60.0 * es.DT

        def check_lamp_off(s):
            """呼叫燈熄滅時，車廂必須停在該樓、方向正確、門正在開或已開。"""
            o = s.out
            for kind, lamp in (("up", "up_lamp"), ("dn", "dn_lamp"), ("car", "car_lamp")):
                for i in range(n_floors):
                    if prev[kind][i] and not o[lamp][i]:
                        self.assertEqual(s.plant.nearest_floor(), i + 1, "%s%d 燈熄但車廂不在該樓" % (kind, i + 1))
                        self.assertTrue(s.plant.in_zone() and not s.plant.moving())
                        self.assertIn(s.state, ("DOOR_OPENING", "DOOR_OPEN", "DOOR_CLOSING"))
                        if kind == "up":
                            self.assertTrue(o["dir_up"])
                        if kind == "dn":
                            self.assertTrue(o["dir_dn"])
                    prev[kind][i] = o[lamp][i]

        s.listeners.append(check_lamp_off)
        while True:
            # 新乘客
            if s.t < end_arrivals and rng.random() < p_arrival:
                a = rng.randint(1, n_floors)
                b = rng.choice([f for f in range(1, n_floors + 1) if f != a])
                p = Passenger(s.t, a, b)
                people.append(p)
                s.press("up" if p.up else "dn", a, duration=rng.uniform(0.1, 0.5))
            door_open = s.plant.door_open_ls()
            here = s.plant.nearest_floor()
            for p in people:
                if p.state == "waiting":
                    if door_open and here == p.origin and (
                            (p.up and s.out["dir_up"]) or (not p.up and s.out["dir_dn"])
                            or not (s.out["dir_up"] or s.out["dir_dn"])):
                        p.state = "riding"
                        p.t_board = s.t
                        p.press_car_at = s.t + rng.uniform(0.2, 1.0)
                        if rng.random() < 0.3:
                            s.press("door_close_btn", duration=0.1)
                    elif (not s.out["up_lamp" if p.up else "dn_lamp"][p.origin - 1]
                          and not (door_open and here == p.origin)):
                        s.press("up" if p.up else "dn", p.origin, duration=0.2)   # 重按
                elif p.state == "riding":
                    if p.press_car_at is not None and s.t >= p.press_car_at:
                        s.press("car", p.dest, duration=0.2)
                        p.press_car_at = None
                    if door_open and here == p.dest:
                        p.state = "done"
                        p.t_done = s.t
            # 門口偶爾有人擋門 / 按開門
            if s.state == "DOOR_CLOSING" and rng.random() < 0.002:
                s.level_inputs["obstruct"] = True
            elif s.level_inputs["obstruct"] and rng.random() < 0.05:
                s.level_inputs["obstruct"] = False
            if s.state == "DOOR_CLOSING" and rng.random() < 0.0005:
                s.press("door_open_btn", duration=0.1)
            s.step()
            self.assertNotEqual(s.state, "FAULT", "fault %d at t=%.1f" % (s.out["fault_code"], s.t))
            if s.t >= end_arrivals and all(p.state == "done" for p in people) and settled(s):
                break
            self.assertLess(s.t, end_arrivals + 600, "乘客未能全部送達")

        waits = [p.t_board - p.t_arrive for p in people]
        trips = [p.t_done - p.t_arrive for p in people]
        return len(people), max(waits), max(trips), sum(trips) / len(trips)


class RandomTraffic4(Backend, Traffic):

    def test_random_traffic_4_floors(self):
        n, max_wait, max_trip, avg = self.run_traffic(self.PROGRAM, 4, minutes=30, seed=1, rate_per_min=3)
        self.assertGreater(n, 50)
        self.assertLess(max_trip, 240)


class TestRandomTraffic7Floors(Traffic, unittest.TestCase):

    def test_random_traffic_7_floors(self):
        n, max_wait, max_trip, avg = self.run_traffic("testn", 7, minutes=20, seed=7, rate_per_min=2)
        self.assertGreater(n, 25)
        self.assertLess(max_trip, 300)


# ---------------------------------------------------------------------------
# FX3U 指令表檢查
# ---------------------------------------------------------------------------
class TestLadderLint(unittest.TestCase):

    def setUp(self):
        with open(es.FX3U_PATH, encoding="utf-8") as f:
            self.text = f.read()

    def problems(self, extra):
        text = self.text.replace("\nEND", "\n" + extra + "\nEND")
        return fx3u.lint(fx3u.parse(text))

    def test_program_is_clean(self):
        self.assertEqual(fx3u.lint(fx3u.parse(self.text)), [])

    def test_double_coil_detected(self):
        self.assertTrue(any("雙重線圈 Y0" in p for p in self.problems("LD X0\nOUT Y0")))

    def test_typo_device_detected(self):
        self.assertTrue(any("M233" in p for p in self.problems("LD M233\nOUT Y26")))

    def test_latched_range_rejected(self):
        self.assertTrue(any("M500" in p for p in self.problems("LD X0\nSET M500")))

    def test_octal_numbers(self):
        self.assertTrue(any("8 進位" in p for p in self.problems("LD X8\nOUT M399")))


    def test_gx_import_csv_up_to_date(self):
        """fx3u/gx-works2、fx3u/gx-developer 的 CSV 必須和指令表一致（build() 內也會讀回逐條比對）。"""
        import export_gx
        for path, data in export_gx.build().items():
            with open(path, "rb") as f:
                self.assertEqual(f.read(), data, "%s 已過時，請執行 python3 export_gx.py" % path)

    def test_rendered_ladder_up_to_date(self):
        """fx3u/ladder.html 與 fx3u/ladder/*.svg 必須和指令表一致。"""
        import render_ladder
        for path, content in render_ladder.render().items():
            with open(path, encoding="utf-8") as f:
                self.assertEqual(f.read(), content,
                                 "%s 已過時，請執行 python3 render_ladder.py" % path)


# ---------------------------------------------------------------------------
# 同一組 4 層測試：ST 程式與 FX3U 階梯圖各跑一次
# ---------------------------------------------------------------------------
class TestNormalServiceST(NormalService, unittest.TestCase):
    PROGRAM = "main"


class TestNormalServiceFX3U(NormalService, unittest.TestCase):
    PROGRAM = "fx3u"


class TestDoorST(Door, unittest.TestCase):
    PROGRAM = "main"


class TestDoorFX3U(Door, unittest.TestCase):
    PROGRAM = "fx3u"


class TestSafetyST(Safety, unittest.TestCase):
    PROGRAM = "main"


class TestSafetyFX3U(Safety, unittest.TestCase):
    PROGRAM = "fx3u"


class TestRandomTrafficST(RandomTraffic4, unittest.TestCase):
    PROGRAM = "main"


class TestRandomTrafficFX3U(RandomTraffic4, unittest.TestCase):
    PROGRAM = "fx3u"


if __name__ == "__main__":
    unittest.main()
