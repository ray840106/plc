"""把 FX3U 指令表轉成 GX Developer 可匯入的 CSV（專案 → 讀取其他格式 → TEXT/CSV）。

    python3 export_gx_developer.py      （在 sim/ 目錄下執行）

產生 fx3u/gx-developer/：
    MAIN.csv       程式（清單格式；只有 ASCII，任何語系的 Windows 都能讀）
    COMMENT.csv    軟元件註解（Big5 編碼，繁體中文 Windows 的 GX Developer 用）

清單格式和 GX Developer 匯出的一樣：每列一個指令，第一個運算元放在
「I/O(Device)」欄，其餘運算元各自放在下一列（步號、指令欄空白）。
X/Y 寫成三位數 8 進位（X000、Y027），和 GX Developer 的顯示方式相同。
產生後會把 CSV 讀回來，確認和原本的指令表逐條相同。
"""

import csv
import io
import os

import fx3u

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
IL_PATH = os.path.join(ROOT, "fx3u", "elevator_fx3u.txt")
DEVICES_PATH = os.path.join(ROOT, "fx3u", "devices.csv")
OUT_DIR = os.path.join(ROOT, "fx3u", "gx-developer")
MAIN_PATH = os.path.join(OUT_DIR, "MAIN.csv")
COMMENT_PATH = os.path.join(OUT_DIR, "COMMENT.csv")

HEADER = ["Step No.", "Line Statement", "Instruction", "I/O(Device)", "Blank", "PI Statement", "Note"]
COMMENT_ENCODING = "cp950"   # Big5


def gx_device(tok):
    """X0 → X000、Y17 → Y017；其他軟元件與常數不變。"""
    tok = tok.upper()
    if tok[0] in "XY" and tok[1:].isdigit():
        return tok[0] + tok[1:].zfill(3)
    return tok


def step_count(ins):
    """FX3U 各指令佔用的步數（本程式用到的指令）。"""
    op = ins.op
    if fx3u.CMP_RE.match(op):
        return 5
    if op in ("MOV", "ZRST"):
        return 5
    if op in ("ADD", "SUB"):
        return 7
    if op == "PLS":
        return 2
    kind = ins.args[0][0].upper() if ins.args else ""
    if op == "OUT" and kind in "TC":
        return 3
    if op == "RST" and kind in "TC":
        return 2
    if op == "RST" and kind == "D":
        return 3
    return 1


def program_rows(rungs):
    rows = []
    step = 0
    for rung in rungs:
        for ins in rung.instrs:
            args = [gx_device(a) for a in ins.args]
            rows.append([str(step), "", ins.op, args[0] if args else "", "", "", ""])
            for a in args[1:]:
                rows.append(["", "", "", a, "", "", ""])
            step += step_count(ins)
    rows.append([str(step), "", "END", "", "", "", ""])
    return rows, step + 1


def to_csv(rows):
    buf = io.StringIO()
    csv.writer(buf, quoting=csv.QUOTE_ALL, lineterminator="\r\n").writerows(rows)
    return buf.getvalue()


def read_back(text):
    """像 GX Developer 一樣把清單 CSV 讀回指令：(指令, [運算元…])。"""
    rows = list(csv.reader(io.StringIO(text)))
    start = rows.index(HEADER) + 1
    out = []
    for row in rows[start:]:
        if row[2]:
            out.append((row[2], [row[3]] if row[3] else []))
        elif row[3]:
            out[-1][1].append(row[3])
    return out


def build():
    with open(IL_PATH, encoding="utf-8") as f:
        rungs = fx3u.parse(f.read())
    rows, total_steps = program_rows(rungs)
    main = to_csv([["MAIN"], ["PLC Type:FX3U(C)"], HEADER] + rows)

    # 讀回來逐條比對
    expect = [(i.op, [gx_device(a) for a in i.args]) for r in rungs for i in r.instrs] + [("END", [])]
    if read_back(main) != expect:
        raise AssertionError("MAIN.csv 讀回後與指令表不一致")
    main.encode("ascii")

    with open(DEVICES_PATH, encoding="utf-8") as f:
        devices = list(csv.DictReader(f))
    comment_rows = [["Device name", "Comment"]]
    comment_rows += [[gx_device(d["Device"]), d["Comment"]] for d in devices]
    comment = to_csv(comment_rows)
    comment.encode(COMMENT_ENCODING)       # 確認每個字都能用 Big5 表示
    return {MAIN_PATH: (main, "ascii"), COMMENT_PATH: (comment, COMMENT_ENCODING)}, total_steps


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    files, total = build()
    for path, (content, enc) in files.items():
        with open(path, "wb") as f:
            f.write(content.encode(enc))
        print("寫入", os.path.relpath(path, ROOT), "(%s)" % enc)
    print("程式共 %d 步" % total)


if __name__ == "__main__":
    main()
