"""把 FX3U 指令表畫成階梯圖。

    python3 render_ladder.py            （在 sim/ 目錄下執行）

產生：
    fx3u/ladder.html        全部梯級（瀏覽器開啟，可列印）
    fx3u/ladder/*.svg       各段落一張圖（GitHub 上可直接看）
    fx3u/LADDER.md          依序顯示各段落的圖
圖面和 GX Works2 相同：左母線、接點上方是軟元件、下方是註解，線圈與應用指令
靠右母線。指令表 → 圖形的轉換和 sim/fx3u.py 模擬時用的是同一個解析器。
"""

import csv
import html
import os
import re

import fx3u

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
FX_DIR = os.path.join(ROOT, "fx3u")
IL_PATH = os.path.join(FX_DIR, "elevator_fx3u.txt")
CSV_PATH = os.path.join(FX_DIR, "devices.csv")
HTML_PATH = os.path.join(FX_DIR, "ladder.html")
SVG_DIR = os.path.join(FX_DIR, "ladder")
MD_PATH = os.path.join(FX_DIR, "LADDER.md")

CW = 84           # 一格寬
RH = 58           # 一列高
LINE = 26         # 導線在列中的高度
LEFT = 58         # 左邊（梯級編號）
OUT_CELLS = 3     # 輸出區寬（格）
STMT_H = 20       # 一行說明的高度
FONT = "'Microsoft JhengHei','PingFang TC','Noto Sans CJK TC','Noto Sans TC',sans-serif"
MONO = "Consolas,'DejaVu Sans Mono',monospace"

INK = "#1d2430"
WIRE = "#3a4556"
COMMENT = "#5d6b80"
STMT = "#2f7d4f"
BOX = "#1f4e8c"


# ---------------------------------------------------------------------------
# 接點網路 → 串並聯樹
# ---------------------------------------------------------------------------
class Leaf:
    def __init__(self, kind, text, devs):
        self.kind = kind          # "no" a 接點、"nc" b 接點、"cmp" 比較
        self.text = text
        self.devs = devs
        self.w = 2 if kind == "cmp" else 1
        self.h = 1


class Series:
    def __init__(self, items):
        self.items = items
        self.w = sum(i.w for i in items)
        self.h = max(i.h for i in items)


class Parallel:
    def __init__(self, items):
        self.items = items
        self.w = max(i.w for i in items)
        self.h = sum(i.h for i in items)


def series(a, b):
    items = (a.items if isinstance(a, Series) else [a]) + (b.items if isinstance(b, Series) else [b])
    return Series(items)


def parallel(a, b):
    items = (a.items if isinstance(a, Parallel) else [a]) + (b.items if isinstance(b, Parallel) else [b])
    return Parallel(items)


def build_expr(contacts):
    stack, acc = [], None
    for i, ins in enumerate(contacts):
        m = fx3u.CMP_RE.match(ins.op)
        if ins.op in fx3u.BLOCK_OPS:
            b = stack.pop()
            acc = series(b, acc) if ins.op == "ANB" else parallel(b, acc)
            continue
        if m:
            base = m.group(1)
            leaf = Leaf("cmp", "%s %s %s" % (m.group(2), ins.args[0], ins.args[1]), ins.args)
        else:
            base = {"LD": "LD", "LDI": "LD", "AND": "AND", "ANI": "AND", "OR": "OR", "ORI": "OR"}[ins.op]
            leaf = Leaf("nc" if ins.op in ("LDI", "ANI", "ORI") else "no", ins.args[0], ins.args)
        if base == "LD":
            if i > 0:
                stack.append(acc)
            acc = leaf
        elif base == "AND":
            acc = series(acc, leaf)
        else:
            acc = parallel(acc, leaf)
    return acc


# ---------------------------------------------------------------------------
# SVG
# ---------------------------------------------------------------------------
def load_comments():
    with open(CSV_PATH, encoding="utf-8") as f:
        return {row["Device"].upper(): row["Comment"] for row in csv.DictReader(f)}


def esc(s):
    return html.escape(s, quote=True)


def text(x, y, s, size=12, color=INK, anchor="middle", family=FONT, weight="normal"):
    return ('<text x="%.1f" y="%.1f" font-size="%d" fill="%s" text-anchor="%s" '
            'font-family="%s" font-weight="%s">%s</text>'
            % (x, y, size, color, anchor, family, weight, esc(s)))


def line(x1, y1, x2, y2, color=WIRE, width=1.4):
    return ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%.1f"/>'
            % (x1, y1, x2, y2, color, width))


def wrap(s, n=7):
    return [s[i:i + n] for i in range(0, len(s), n)][:2] if s else []


class Drawer:
    def __init__(self, comments, x_out):
        self.comments = comments
        self.x_out = x_out                    # 輸出區起點（px）
        self.x_right = x_out + OUT_CELLS * CW  # 右母線

    def comment_of(self, dev):
        return self.comments.get(dev.upper(), "")

    def comment_lines(self, parts, cx, ly, s, n=7):
        for k, t in enumerate(wrap(s, n)):
            parts.append(text(cx, ly + 24 + 13 * k, t, 11, COMMENT))

    def leaf(self, e, x, y, parts):
        ly = y + LINE
        if e.kind == "cmp":
            x1, x2 = x + 6, x + 2 * CW - 6
            parts.append(line(x, ly, x1, ly))
            parts.append(line(x2, ly, x + 2 * CW, ly))
            parts.append('<rect x="%.1f" y="%.1f" width="%.1f" height="22" fill="#fff" stroke="%s" '
                         'stroke-width="1.3" rx="2"/>' % (x1, ly - 11, x2 - x1, BOX))
            parts.append(text((x1 + x2) / 2, ly + 4.5, e.text, 12, BOX, family=MONO))
            notes = [self.comment_of(d) for d in e.devs if d.upper().startswith("D")]
            self.comment_lines(parts, (x1 + x2) / 2, ly, "、".join(n for n in notes if n), 13)
            return
        cx = x + CW / 2
        parts.append(line(x, ly, cx - 8, ly))
        parts.append(line(cx + 8, ly, x + CW, ly))
        parts.append(line(cx - 8, ly - 11, cx - 8, ly + 11, INK, 1.8))
        parts.append(line(cx + 8, ly - 11, cx + 8, ly + 11, INK, 1.8))
        if e.kind == "nc":
            parts.append(line(cx - 11, ly + 10, cx + 11, ly - 10, INK, 1.5))
        parts.append(text(cx, ly - 16, e.text, 12, INK, family=MONO, weight="bold"))
        self.comment_lines(parts, cx, ly, self.comment_of(e.text))

    def expr(self, e, x, y, parts):
        if isinstance(e, Leaf):
            self.leaf(e, x, y, parts)
        elif isinstance(e, Series):
            for it in e.items:
                self.expr(it, x, y, parts)
                x += it.w * CW
        else:
            width = e.w * CW
            cy = y
            last = y
            for it in e.items:
                self.expr(it, x, cy, parts)
                if it.w < e.w:
                    parts.append(line(x + it.w * CW, cy + LINE, x + width, cy + LINE))
                last = cy
                cy += it.h * RH
            parts.append(line(x, y + LINE, x, last + LINE))
            parts.append(line(x + width, y + LINE, x + width, last + LINE))

    def output(self, ins, y, parts):
        ly = y + LINE
        args = ins.args
        dev = args[0].upper()
        is_coil = ins.op == "OUT"
        if is_coil:
            cx = self.x_right - CW * 0.55
            parts.append(line(self.x_out, ly, cx - 12, ly))
            parts.append(line(cx + 12, ly, self.x_right, ly))
            parts.append('<path d="M%.1f %.1f Q%.1f %.1f %.1f %.1f M%.1f %.1f Q%.1f %.1f %.1f %.1f" '
                         'fill="none" stroke="%s" stroke-width="1.8"/>'
                         % (cx - 5, ly - 12, cx - 15, ly, cx - 5, ly + 12,
                            cx + 5, ly - 12, cx + 15, ly, cx + 5, ly + 12, INK))
            label = dev + (" " + args[1].upper() if len(args) > 1 else "")
            parts.append(text(cx, ly - 17, label, 12, INK, family=MONO, weight="bold"))
            self.comment_lines(parts, cx, ly, self.comment_of(dev))
            return
        x1, x2 = self.x_out + CW * 0.35, self.x_right - 8
        parts.append(line(self.x_out, ly, x1, ly))
        parts.append(line(x2, ly, self.x_right, ly))
        parts.append('<rect x="%.1f" y="%.1f" width="%.1f" height="24" fill="#fff" stroke="%s" '
                     'stroke-width="1.3" rx="2"/>' % (x1, ly - 12, x2 - x1, BOX))
        parts.append(text((x1 + x2) / 2, ly + 4.5, " ".join([ins.op] + [a.upper() for a in args]),
                          12, BOX, family=MONO, weight="bold"))
        if ins.op == "ZRST":
            note = "%s~%s 清除" % (args[0].upper(), args[1].upper())
        elif ins.op in ("MOV", "ADD", "SUB"):
            note = "→ " + self.comment_of(args[-1]) if self.comment_of(args[-1]) else ""
        else:
            note = self.comment_of(dev)
        parts.append(text((x1 + x2) / 2, ly + 24, note, 11, COMMENT))

    def rung(self, rung, y, parts):
        """畫一個梯級，回傳高度（px）。"""
        contacts, outputs = fx3u.rung_parts(rung)
        e = build_expr(contacts)
        rows = max(e.h, len(outputs))
        height = rows * RH
        parts.append(line(LEFT, y, LEFT, y + height, INK, 2.4))
        parts.append(line(self.x_right, y, self.x_right, y + height, INK, 2.4))
        parts.append(text(LEFT - 10, y + LINE + 4, str(rung.number), 12, COMMENT, anchor="end", family=MONO))
        self.expr(e, LEFT, y, parts)
        parts.append(line(LEFT + e.w * CW, y + LINE, self.x_out, y + LINE))
        for k, ins in enumerate(outputs):
            self.output(ins, y + k * RH, parts)
        if len(outputs) > 1:
            parts.append(line(self.x_out, y + LINE, self.x_out, y + (len(outputs) - 1) * RH + LINE))
        return height


def svg_doc(parts, width, height):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">'
            '<rect width="100%%" height="100%%" fill="#ffffff"/>%s</svg>\n'
            % (width, height, width, height, "".join(parts)))


# ---------------------------------------------------------------------------
# 輸出檔案
# ---------------------------------------------------------------------------
def sections(rungs):
    out = []
    for r in rungs:
        if not out or out[-1][0] != r.section:
            out.append((r.section, []))
        out[-1][1].append(r)
    return out


def slug(i, title):
    return "%02d" % (i + 1)


def render():
    with open(IL_PATH, encoding="utf-8") as f:
        rungs = fx3u.parse(f.read())
    comments = load_comments()
    max_w = max(build_expr(fx3u.rung_parts(r)[0]).w for r in rungs)
    drawer = Drawer(comments, LEFT + max_w * CW)
    width = int(drawer.x_right + 24)
    files = {}

    # ---- 各段落 SVG（含說明文字）----
    groups = sections(rungs)
    md = ["# 4 層電梯 — 三菱 FX3U 階梯圖", "",
          "由 `sim/render_ladder.py` 依 [`elevator_fx3u.txt`](elevator_fx3u.txt) 自動產生，"
          "請勿手動修改。完整說明見 [README](README.md)，也可以下載 "
          "[ladder.html](ladder.html) 用瀏覽器開啟。", ""]
    for gi, (title, rs) in enumerate(groups):
        parts = [text(16, 30, title, 17, INK, anchor="start", weight="bold")]
        y = 48
        for r in rs:
            for c in r.comments:
                parts.append(text(LEFT, y + 14, "＊" + c, 12, STMT, anchor="start"))
                y += STMT_H
            y += 4 + drawer.rung(r, y + 4, parts) + 14
        name = "%s.svg" % slug(gi, title)
        files[os.path.join(SVG_DIR, name)] = svg_doc(parts, width, y + 8)
        md += ["## " + title, "", "![%s](ladder/%s)" % (title, name), ""]
    files[MD_PATH] = "\n".join(md)

    # ---- HTML ----
    body = []
    toc = []
    for gi, (title, rs) in enumerate(groups):
        anchor = "s%02d" % (gi + 1)
        toc.append('<a href="#%s">%s</a>' % (anchor, esc(title)))
        body.append('<section id="%s"><h2>%s</h2>' % (anchor, esc(title)))
        for r in rs:
            parts = []
            h = drawer.rung(r, 6, parts)
            stmt = "".join("<p class=\"stmt\">＊%s</p>" % esc(c) for c in r.comments)
            il = "\n".join("%-6s %s" % (i.op, " ".join(i.args)) for i in r.instrs)
            body.append('<div class="rung">%s<div class="scroll">%s</div>'
                        '<details><summary>指令表</summary><pre>%s</pre></details></div>'
                        % (stmt, svg_doc(parts, width, h + 12).strip(), esc(il)))
        body.append("</section>")
    n_steps = sum(len(r.instrs) for r in rungs)
    files[HTML_PATH] = HTML_TEMPLATE % {
        "rungs": len(rungs), "instrs": n_steps, "toc": "\n".join(toc),
        "body": "\n".join(body), "width": width,
    }
    return files


HTML_TEMPLATE = """<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>電梯 FX3U 階梯圖</title>
<style>
  :root { color-scheme: light; --ink: #1d2430; --muted: #5d6b80; --line: #d9dee6; --paper: #ffffff; --bg: #eef1f5; --accent: #1f4e8c; --stmt: #2f7d4f; }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--ink);
         font-family: 'Microsoft JhengHei','PingFang TC','Noto Sans TC',system-ui,sans-serif; line-height: 1.55; }
  header { background: var(--paper); border-bottom: 1px solid var(--line); padding: 20px 16px 14px; }
  header h1 { margin: 0 0 4px; font-size: 22px; }
  header p { margin: 4px 0; color: var(--muted); font-size: 14px; }
  .legend { display: flex; flex-wrap: wrap; gap: 6px 18px; margin-top: 10px; font-size: 13px; color: var(--muted); }
  .legend code { color: var(--ink); background: var(--bg); padding: 1px 6px; border-radius: 4px; }
  nav { position: sticky; top: 0; z-index: 2; background: var(--paper); border-bottom: 1px solid var(--line);
        padding: 8px 16px; display: flex; gap: 6px 14px; overflow-x: auto; white-space: nowrap; font-size: 13px; }
  nav a { color: var(--accent); text-decoration: none; }
  main { max-width: calc(%(width)dpx + 34px); margin: 0 auto; padding: 8px 16px 48px; }
  h2 { font-size: 17px; margin: 28px 0 8px; padding-bottom: 4px; border-bottom: 2px solid var(--accent); scroll-margin-top: 48px; }
  .rung { background: var(--paper); border: 1px solid var(--line); border-radius: 6px; margin: 8px 0; padding: 6px 0 0; }
  .stmt { margin: 0 12px; color: var(--stmt); font-size: 13px; }
  .scroll { overflow-x: auto; }
  .scroll svg { display: block; min-width: %(width)dpx; width: 100%%; height: auto; }
  details { margin: 0 12px 8px; font-size: 13px; color: var(--muted); }
  summary { cursor: pointer; }
  pre { margin: 4px 0 0; padding: 8px 10px; background: var(--bg); border-radius: 4px; color: var(--ink);
        font: 13px/1.5 Consolas,'DejaVu Sans Mono',monospace; overflow-x: auto; }
  @media print { nav, details { display: none; } body { background: #fff; } .rung { break-inside: avoid; border: none; } }
</style>
</head>
<body>
<header>
  <h1>4 層電梯 — 三菱 FX3U 階梯圖</h1>
  <p>共 %(rungs)d 個梯級、%(instrs)d 個指令。由 elevator_fx3u.txt 自動產生；左邊數字為梯級編號（不是 GX Works2 的步號）。</p>
  <div class="legend">
    <span><code>┤├</code> a 接點</span><span><code>┤/├</code> b 接點</span>
    <span><code>[= D0 K1]</code> 比較接點</span><span><code>( )</code> 線圈</span>
    <span><code>[SET M200]</code> 指令</span><span>上方：軟元件　下方：註解　綠字：說明</span>
  </div>
</header>
<nav>%(toc)s</nav>
<main>
%(body)s
</main>
</body>
</html>
"""


def main():
    os.makedirs(SVG_DIR, exist_ok=True)
    for path, content in render().items():
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        print("寫入", os.path.relpath(path, ROOT))


if __name__ == "__main__":
    main()
