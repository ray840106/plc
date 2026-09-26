"""三菱 FX3U 指令表（IL）模擬器。

只實作本專案用到的指令，用來在電腦上驗證 fx3u/elevator_fx3u.txt：

    接點   LD LDI AND ANI OR ORI ANB ORB
    比較   LD= LD<> LD> LD< LD>= LD<=（AND□、OR□ 相同）
    輸出   OUT（Y/M/T/C）SET RST PLS
    應用   MOV ADD SUB ZRST
    END

掃描方式與 PLC 相同：輸入 X 在掃描開始時讀入，程式由上而下執行一次，
輸出 Y 在 END 後送出。X/Y 編號為 8 進位；T0~T199 為 100 ms 計時器、
T200~T245 為 10 ms 計時器，計時器接點在執行 OUT T 時更新；C0~C99 為
16 位元加算計數器；M8000 = RUN 中常 ON，M8002 = 第一次掃描 ON。

程式先經過 lint() 檢查（雙重線圈、軟元件範圍、梯級結構、沒有被寫入過就
讀取的軟元件……），再轉成一個 Python 函式執行，速度約為逐條直譯的 10 倍。
"""

import re

LOAD_OPS = {"LD", "LDI"}
CONTACT_OPS = {"AND", "ANI", "OR", "ORI"}
BLOCK_OPS = {"ANB", "ORB"}
OUTPUT_OPS = {"OUT", "SET", "RST", "PLS"}
APPLY_OPS = {"MOV": 2, "ADD": 3, "SUB": 3, "ZRST": 2}
CMP_RE = re.compile(r"^(LD|AND|OR)(<>|>=|<=|=|>|<)$")
CMP_PY = {"=": "==", "<>": "!=", ">": ">", "<": "<", ">=": ">=", "<=": "<="}

# FX3U 一般用（非停電保持）軟元件範圍；本程式刻意只用這些
LIMITS = {"X": 0o27, "Y": 0o27, "M": 499, "T": 199, "C": 99, "D": 199}
SPECIAL_M = {8000: "True", 8002: "fs"}


class ILError(Exception):
    pass


class Instr:
    def __init__(self, op, args, lineno, comment):
        self.op = op
        self.args = args
        self.lineno = lineno
        self.comment = comment

    def kind(self):
        if self.op in LOAD_OPS or (CMP_RE.match(self.op) and self.op.startswith("LD")):
            return "load"
        if self.op in CONTACT_OPS or CMP_RE.match(self.op):
            return "contact"
        if self.op in BLOCK_OPS:
            return "block"
        if self.op in OUTPUT_OPS:
            return "output"
        if self.op in APPLY_OPS:
            return "apply"
        if self.op == "END":
            return "end"
        raise ILError("第 %d 行：不支援的指令 %s" % (self.lineno, self.op))

    def __repr__(self):
        return " ".join([self.op] + self.args)


class Rung:
    def __init__(self, number, section, comments):
        self.number = number
        self.section = section
        self.comments = comments
        self.instrs = []


# ---------------------------------------------------------------------------
# 軟元件
# ---------------------------------------------------------------------------
class Dev:
    """位元軟元件 X/Y/M/T/C 或字元軟元件 D、常數 K、位數指定 K1M300。"""

    def __init__(self, kind, num, digits=0):
        self.kind = kind        # "X" "Y" "M" "T" "C" "D" "K" 或位數指定的位元種類
        self.num = num
        self.digits = digits    # K1M300 → digits = 1

    def name(self):
        if self.kind == "K":
            return "K%d" % self.num
        n = format(self.num, "o") if self.kind in "XY" else str(self.num)
        return ("K%d" % self.digits if self.digits else "") + self.kind + n


def parse_dev(tok, lineno):
    tok = tok.upper()
    m = re.match(r"^K(-?\d+)$", tok)
    if m:
        return Dev("K", int(m.group(1)))
    m = re.match(r"^K([1-4])([XYM])(\d+)$", tok)
    if m:
        d = parse_dev(m.group(2) + m.group(3), lineno)
        return Dev(d.kind, d.num, int(m.group(1)))
    m = re.match(r"^([XYMTCD])(\d+)$", tok)
    if not m:
        raise ILError("第 %d 行：無法辨識的軟元件 %s" % (lineno, tok))
    kind, digits = m.group(1), m.group(2)
    if kind in "XY":
        if re.search(r"[89]", digits):
            raise ILError("第 %d 行：%s 為 8 進位，不可有 8、9" % (lineno, tok))
        return Dev(kind, int(digits, 8))
    return Dev(kind, int(digits))


def bit_expr(d):
    if d.kind == "X":
        return "X[%d]" % d.num
    if d.kind == "Y":
        return "Y[%d]" % d.num
    if d.kind == "M":
        return SPECIAL_M.get(d.num, "M[%d]" % d.num)
    if d.kind == "T":
        return "TQ[%d]" % d.num
    if d.kind == "C":
        return "CQ[%d]" % d.num
    raise ILError("%s 不是位元軟元件" % d.name())


def word_expr(d):
    if d.kind == "K":
        return str(d.num)
    if d.kind == "D":
        return "D[%d]" % d.num
    if d.digits:
        arr = {"X": "X", "Y": "Y", "M": "M"}[d.kind]
        return "(" + " + ".join("%d * %s[%d]" % (1 << i, arr, d.num + i)
                                for i in range(4 * d.digits)) + ")"
    raise ILError("%s 不是字元軟元件" % d.name())


def word_assign(d, value, indent):
    if d.kind == "D":
        return ["%sD[%d] = w16(%s)" % (indent, d.num, value)]
    if d.digits:
        lines = ["%s_v = %s" % (indent, value)]
        for i in range(4 * d.digits):
            lines.append("%s%s[%d] = bool(_v & %d)" % (indent, d.kind, d.num + i, 1 << i))
        return lines
    raise ILError("無法寫入 %s" % d.name())


def w16(v):
    return ((v + 0x8000) & 0xFFFF) - 0x8000


# ---------------------------------------------------------------------------
# 解析
# ---------------------------------------------------------------------------
def parse(text):
    """指令表文字 → 梯級清單。空行只是排版；新梯級由「輸出之後的 LD」開始。"""
    rungs = []
    section = ""
    comments = []
    started = False
    rung = None
    after_output = True
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith(";;"):
            section = line[2:].strip()
            comments = []
            started = True
            continue
        if line.startswith(";"):
            if started:
                comments.append(line[1:].strip())
            continue
        code, _, comment = line.partition(";")
        tokens = code.split()
        ins = Instr(tokens[0].upper(), tokens[1:], lineno, comment.strip())
        kind = ins.kind()
        if kind == "end":
            return rungs
        if kind == "load" and after_output:
            rung = Rung(len(rungs) + 1, section, comments)
            rungs.append(rung)
            comments = []
        elif rung is None or (after_output and kind in ("contact", "block")):
            raise ILError("第 %d 行：梯級必須以 LD / LDI 開始（%s）" % (lineno, ins))
        rung.instrs.append(ins)
        after_output = kind in ("output", "apply")
    raise ILError("缺少 END")


def rung_parts(rung):
    """把梯級分成接點部分與輸出部分，並檢查堆疊平衡。"""
    depth = 0
    contacts, outputs = [], []
    for i, ins in enumerate(rung.instrs):
        kind = ins.kind()
        if kind in ("output", "apply"):
            outputs.append(ins)
            continue
        if outputs:
            raise ILError("第 %d 行：輸出之後不可再接接點（本程式不使用分支輸出）" % ins.lineno)
        if kind == "load" and i > 0:
            depth += 1
        elif kind == "block":
            depth -= 1
            if depth < 0:
                raise ILError("第 %d 行：%s 沒有對應的 LD" % (ins.lineno, ins.op))
        contacts.append(ins)
    if depth != 0:
        raise ILError("梯級 %d：LD 與 ANB/ORB 數量不平衡" % rung.number)
    if not outputs:
        raise ILError("梯級 %d：沒有輸出" % rung.number)
    return contacts, outputs


# ---------------------------------------------------------------------------
# 檢查
# ---------------------------------------------------------------------------
def lint(rungs):
    """回傳問題清單（空清單 = 通過）。"""
    problems = []
    coils = {}
    written = set()
    read = set()

    def check_range(d, ins):
        if d.kind == "M" and d.num in SPECIAL_M:
            return
        span = 4 * d.digits - 1 if d.digits else 0
        lim = LIMITS.get(d.kind)
        if lim is not None and not 0 <= d.num + span <= lim:
            problems.append("第 %d 行：%s 超出 FX3U 一般用範圍（上限 %s）"
                            % (ins.lineno, d.name(), format(lim, "o") if d.kind in "XY" else lim))

    for rung in rungs:
        try:
            contacts, outputs = rung_parts(rung)
            for ins in rung.instrs:
                for tok in ins.args:
                    parse_dev(tok, ins.lineno)
        except ILError as e:
            problems.append(str(e))
            continue
        for ins in contacts:
            if ins.kind() == "block":
                continue
            for tok in ins.args:
                d = parse_dev(tok, ins.lineno)
                check_range(d, ins)
                if d.kind in ("Y", "M", "T", "C", "D") and not (d.kind == "M" and d.num in SPECIAL_M):
                    read.add(d.name())
        for ins in outputs:
            devs = [parse_dev(t, ins.lineno) for t in ins.args]
            for d in devs:
                check_range(d, ins)
            if ins.op in ("OUT", "SET", "RST", "PLS"):
                d = devs[0]
                if d.kind == "M" and d.num in SPECIAL_M:
                    problems.append("第 %d 行：不可寫入特殊 M%d" % (ins.lineno, d.num))
                if d.kind == "X":
                    problems.append("第 %d 行：不可寫入輸入 %s" % (ins.lineno, d.name()))
                if ins.op in ("OUT", "PLS"):
                    if d.name() in coils:
                        problems.append("第 %d 行：雙重線圈 %s（第 %d 行已使用）"
                                        % (ins.lineno, d.name(), coils[d.name()]))
                    coils[d.name()] = ins.lineno
                if ins.op == "OUT" and d.kind in "TC" and (len(devs) != 2 or devs[1].kind != "K"):
                    problems.append("第 %d 行：%s 需要設定值 K" % (ins.lineno, d.name()))
                written.add(d.name())
            elif ins.op == "ZRST":
                # 只是清除，不算寫入：只被 ZRST 過就被讀取的軟元件一定是打錯編號
                a, b = devs
                if a.kind != b.kind or a.num > b.num:
                    problems.append("第 %d 行：ZRST 範圍錯誤" % ins.lineno)
            else:
                if len(devs) != APPLY_OPS[ins.op]:
                    problems.append("第 %d 行：%s 運算元數量錯誤" % (ins.lineno, ins.op))
                dst = devs[-1]
                if dst.digits:
                    for i in range(4 * dst.digits):
                        written.add(Dev(dst.kind, dst.num + i).name())
                else:
                    written.add(dst.name())
                for d in devs[:-1]:
                    if d.kind == "D":
                        read.add(d.name())
                    elif d.digits:
                        for i in range(4 * d.digits):
                            read.add(Dev(d.kind, d.num + i).name())
    for name in sorted(read - written):
        problems.append("%s 被讀取但從未被寫入（打錯編號？）" % name)
    for name, line in sorted(coils.items()):
        if name.startswith("M") and name not in read:
            problems.append("第 %d 行：%s 輸出後沒有被使用" % (line, name))
    return problems


# ---------------------------------------------------------------------------
# 轉成 Python
# ---------------------------------------------------------------------------
def generate(rungs):
    out = ["def scan(X, Y, M, TQ, TA, CQ, CV, D, E, fs, dt):"]
    edge = 0
    for rung in rungs:
        contacts, outputs = rung_parts(rung)
        out.append("    # 梯級 %d" % rung.number)
        depth = 0
        for i, ins in enumerate(contacts):
            op = ins.op
            m = CMP_RE.match(op)
            if m:
                a, b = (parse_dev(t, ins.lineno) for t in ins.args)
                expr = "(%s %s %s)" % (word_expr(a), CMP_PY[m.group(2)], word_expr(b))
                op = m.group(1)
            elif op in BLOCK_OPS:
                depth -= 1
                out.append("    a = s%d %s a" % (depth, "and" if op == "ANB" else "or"))
                continue
            else:
                expr = bit_expr(parse_dev(ins.args[0], ins.lineno))
            if op in ("LDI", "ANI", "ORI"):
                expr = "not " + expr
            if op in ("LD", "LDI"):
                if i > 0:
                    out.append("    s%d = a" % depth)
                    depth += 1
                out.append("    a = %s" % expr)
            elif op in ("AND", "ANI"):
                out.append("    a = a and %s" % expr)
            else:
                out.append("    a = a or %s" % expr)
        for ins in outputs:
            devs = [parse_dev(t, ins.lineno) for t in ins.args]
            d = devs[0]
            if ins.op == "OUT" and d.kind == "T":
                unit = 100 if d.num < 200 else 10
                preset = devs[1].num * unit
                out += ["    if a:",
                        "        TA[%d] += dt" % d.num,
                        "        if TA[%d] >= %d:" % (d.num, preset),
                        "            TA[%d] = %d" % (d.num, preset),
                        "            TQ[%d] = True" % d.num,
                        "    else:",
                        "        TA[%d] = 0" % d.num,
                        "        TQ[%d] = False" % d.num]
            elif ins.op == "OUT" and d.kind == "C":
                out += ["    if a and not E[%d] and not CQ[%d]:" % (edge, d.num),
                        "        CV[%d] += 1" % d.num,
                        "        CQ[%d] = CV[%d] >= %d" % (d.num, d.num, devs[1].num),
                        "    E[%d] = a" % edge]
                edge += 1
            elif ins.op == "OUT":
                out.append("    %s = a" % bit_expr(d))
            elif ins.op == "PLS":
                out += ["    %s = a and not E[%d]" % (bit_expr(d), edge),
                        "    E[%d] = a" % edge]
                edge += 1
            elif ins.op in ("SET", "RST"):
                if ins.op == "RST" and d.kind == "T":
                    out.append("    if a: TA[%d] = 0; TQ[%d] = False" % (d.num, d.num))
                elif ins.op == "RST" and d.kind == "C":
                    out.append("    if a: CV[%d] = 0; CQ[%d] = False" % (d.num, d.num))
                elif ins.op == "RST" and d.kind == "D":
                    out.append("    if a: D[%d] = 0" % d.num)
                else:
                    out.append("    if a: %s = %s" % (bit_expr(d), ins.op == "SET"))
            elif ins.op == "ZRST":
                lo, hi = d.num, devs[1].num
                arr = {"M": "M", "Y": "Y", "D": "D"}.get(d.kind)
                if d.kind in ("T", "C"):
                    q, v = ("TQ", "TA") if d.kind == "T" else ("CQ", "CV")
                    out.append("    if a: %s[%d:%d] = [False] * %d; %s[%d:%d] = [0] * %d"
                               % (q, lo, hi + 1, hi - lo + 1, v, lo, hi + 1, hi - lo + 1))
                else:
                    fill = "0" if arr == "D" else "False"
                    out.append("    if a: %s[%d:%d] = [%s] * %d" % (arr, lo, hi + 1, fill, hi - lo + 1))
            else:
                src = [word_expr(x) for x in devs[:-1]]
                value = src[0] if ins.op == "MOV" else "%s %s %s" % (
                    src[0], "+" if ins.op == "ADD" else "-", src[1])
                out.append("    if a:")
                out += word_assign(devs[-1], value, "        ")
    out.append("    return %d" % edge)
    return "\n".join(out) + "\n", edge


class Program:
    def __init__(self, text):
        self.rungs = parse(text)
        problems = lint(self.rungs)
        if problems:
            raise ILError("\n".join(problems))
        self.source, self.n_edges = generate(self.rungs)
        ns = {"w16": w16}
        exec(compile(self.source, "<fx3u>", "exec"), ns)
        self.scan_fn = ns["scan"]


class FX3U:
    """一台 FX3U：軟元件記憶體 + 已編譯的程式。"""

    def __init__(self, program):
        self.program = program
        self.power_on()

    def power_on(self):
        self.X = [False] * 0o30
        self.Y = [False] * 0o30
        self.M = [False] * 512
        self.TQ = [False] * 200
        self.TA = [0] * 200
        self.CQ = [False] * 100
        self.CV = [0] * 100
        self.D = [0] * 200
        self.E = [False] * self.program.n_edges
        self.first_scan = True

    def scan(self, dt_ms):
        self.program.scan_fn(self.X, self.Y, self.M, self.TQ, self.TA, self.CQ, self.CV,
                             self.D, self.E, self.first_scan, dt_ms)
        self.first_scan = False
