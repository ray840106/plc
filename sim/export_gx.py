"""把 FX3U 指令表轉成三菱軟體可以匯入的 CSV。

    python3 export_gx.py        （在 sim/ 目錄下執行）

產生：
    fx3u/gx-works2/MAIN.csv       GX Works2：專案樹 MAIN 按右鍵 → 從 CSV 檔讀取
    fx3u/gx-works2/COMMENT.csv    GX Works2：全域軟元件註解按右鍵 → 從 CSV 檔讀取
    fx3u/gx-developer/MAIN.csv    GX Developer：Project → Import file → TEXT, CSV（需要 GX Converter）
    fx3u/gx-developer/COMMENT.csv GX Developer 軟元件註解（Big5）

兩種格式的共同點（和三菱軟體自己匯出的一樣）：一列一個指令，第一個運算元放在
I/O 欄，其餘運算元各自放在下一列；X/Y 寫成三位數 8 進位（X000、Y027）；
步號依 FX3U 各指令的步數計算。

    GX Works2：UTF-16（含 BOM）、Tab 分隔、全部加引號、CRLF；前三列為標題、
               PLC 資訊、欄位名稱（依 GX Works2 簡體中文版實際匯出的檔案），
               梯級說明放在「行間聲明」欄，每則最多 64 位元組。
    GX Developer：只有 ASCII、逗號分隔（不含說明，避免編碼問題）。

產生後會把每個檔案讀回來，確認和原本的指令表逐條相同。
"""

import csv
import io
import os
import unicodedata

import fx3u

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
IL_PATH = os.path.join(ROOT, "fx3u", "elevator_fx3u.txt")
DEVICES_PATH = os.path.join(ROOT, "fx3u", "devices.csv")
GXW2_DIR = os.path.join(ROOT, "fx3u", "gx-works2")
GXDEV_DIR = os.path.join(ROOT, "fx3u", "gx-developer")

GXW2_HEADER = ["步号", "行间声明", "指令", "I/O(软元件)", "空白栏", "PI声明", "注解"]
GXW2_PLC_INFO = ["PLC信息:", "FXCPU FX3U/FX3UC"]
GXW2_COMMENT_HEADER = ["软元件名", "注释"]
GXDEV_HEADER = ["Step No.", "Line Statement", "Instruction", "I/O(Device)", "Blank", "PI Statement", "Note"]
STATEMENT_MAX_BYTES = 64


def gx_device(tok):
    """X0 → X000、Y17 → Y017；其他軟元件與常數不變。"""
    tok = tok.upper()
    if tok[0] in "XY" and tok[1:].isdigit():
        return tok[0] + tok[1:].zfill(3)
    return tok


def is_special_m(tok):
    return tok.upper().startswith("M") and tok[1:].isdigit() and int(tok[1:]) >= 8000


def step_count(ins):
    """FX3U 各指令佔用的步數（本程式用到的指令）。

    應用指令與比較接點（16 位元）= 1 + 每個運算元 2 步；
    特殊輔助繼電器 M8000 以後的接點 / 線圈 = 2 步。
    """
    op = ins.op
    if fx3u.CMP_RE.match(op) or op in ("MOV", "ZRST", "ADD", "SUB"):
        return 1 + 2 * len(ins.args)
    if op == "PLS":
        return 2
    dev = ins.args[0].upper() if ins.args else ""
    if is_special_m(dev):
        return 2
    if op == "OUT" and dev[:1] in "TC":
        return 3
    if op == "RST" and dev[:1] in "TC":
        return 2
    if op == "RST" and dev[:1] == "D":
        return 3
    return 1


def statement_of(rung, first_in_section):
    text = " ".join(rung.comments) or (rung.section if first_in_section else "")
    text = unicodedata.normalize("NFKC", text).strip()
    if len(text.encode("gb18030")) > STATEMENT_MAX_BYTES:
        raise ValueError("梯級 %d 的說明超過 %d 位元組，請在指令表縮短：%s"
                         % (rung.number, STATEMENT_MAX_BYTES, text))
    return text


def listing_rows(rungs, statements):
    rows = []
    step = 0
    section = None
    for rung in rungs:
        if statements:
            text = statement_of(rung, rung.section != section)
            if text:
                rows.append([str(step), text, "", "", "", "", ""])
        section = rung.section
        for ins in rung.instrs:
            args = [gx_device(a) for a in ins.args]
            rows.append([str(step), "", ins.op, args[0] if args else "", "", "", ""])
            for a in args[1:]:
                rows.append(["", "", "", a, "", "", ""])
            step += step_count(ins)
    rows.append([str(step), "", "END", "", "", "", ""])
    return rows, step + 1


def to_csv(rows, delimiter):
    buf = io.StringIO()
    csv.writer(buf, delimiter=delimiter, quoting=csv.QUOTE_ALL, lineterminator="\r\n").writerows(rows)
    return buf.getvalue()


def read_back(text, header, delimiter):
    """像三菱軟體一樣讀回清單：跳過說明列，把接續列的運算元併回前一個指令。"""
    rows = list(csv.reader(io.StringIO(text), delimiter=delimiter))
    start = rows.index(header) + 1
    out = []
    for row in rows[start:]:
        if row[2]:
            out.append((row[2], [row[3]] if row[3] else []))
        elif row[3]:
            out[-1][1].append(row[3])
    return out


def build():
    """回傳 {路徑: 檔案位元組}。"""
    with open(IL_PATH, encoding="utf-8") as f:
        rungs = fx3u.parse(f.read())
    with open(DEVICES_PATH, encoding="utf-8") as f:
        devices = [(gx_device(d["Device"]), d["Comment"]) for d in csv.DictReader(f)]
    expect = [(i.op, [gx_device(a) for a in i.args]) for r in rungs for i in r.instrs] + [("END", [])]
    files = {}

    # ---- GX Works2 ----
    rows, _ = listing_rows(rungs, statements=True)
    main = to_csv([["MAIN"], GXW2_PLC_INFO, GXW2_HEADER] + rows, "\t")
    comment = to_csv([["COMMENT"], GXW2_COMMENT_HEADER] + [list(d) for d in devices], "\t")
    if read_back(main, GXW2_HEADER, "\t") != expect:
        raise AssertionError("GX Works2 MAIN.csv 讀回後與指令表不一致")
    for name, text in (("MAIN.csv", main), ("COMMENT.csv", comment)):
        files[os.path.join(GXW2_DIR, name)] = b"\xff\xfe" + text.encode("utf-16-le")

    # ---- GX Developer ----
    rows, _ = listing_rows(rungs, statements=False)
    main = to_csv([["MAIN"], ["PLC Type:FX3U(C)"], GXDEV_HEADER] + rows, ",")
    comment = to_csv([["Device name", "Comment"]] + [list(d) for d in devices], ",")
    if read_back(main, GXDEV_HEADER, ",") != expect:
        raise AssertionError("GX Developer MAIN.csv 讀回後與指令表不一致")
    files[os.path.join(GXDEV_DIR, "MAIN.csv")] = main.encode("ascii")
    files[os.path.join(GXDEV_DIR, "COMMENT.csv")] = comment.encode("cp950")   # Big5
    return files


def total_steps():
    with open(IL_PATH, encoding="utf-8") as f:
        return listing_rows(fx3u.parse(f.read()), statements=False)[1]


def main():
    for path, data in build().items():
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(data)
        print("寫入", os.path.relpath(path, ROOT))
    print("程式共 %d 步" % total_steps())


if __name__ == "__main__":
    main()
