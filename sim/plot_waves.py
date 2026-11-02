"""Рендер осцилограм із VCD у PNG (вигляд логічного аналізатора / GTKWave).

Читає build/sim/tb_<f>.vcd, знаходить транзакції за сигналом busy і малює
вибрані вікна часу з розкодуванням шини I2C (START/STOP, біти, байти, ACK/NACK).

Використання: python sim/plot_waves.py  ->  docs/img/wave_*.png
"""
from __future__ import annotations

import bisect
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "build" / "sim"
IMG = ROOT / "docs" / "img"

BG, PANEL, GRID = "#0b0f0c", "#1a1f1b", "#26302a"
GREEN, YELLOW, CYAN, RED, GRAY = "#38e36b", "#f2d64b", "#5fd7ff", "#ff6464", "#9aa3a0"
STATES = {0: "IDLE", 1: "START", 2: "ADDR", 3: "WRITE", 4: "READ", 5: "STOP", 6: "BUF"}


# ----------------------------------------------------------------- VCD parser
def parse_vcd(path: Path, wanted: set[str]) -> dict[str, tuple[list[float], list[str]]]:
    """Повертає {повне_ім'я: (часи_мкс, значення)} лише для потрібних сигналів."""
    scope: list[str] = []
    id2names: dict[str, list[str]] = {}
    data: dict[str, tuple[list[float], list[str]]] = {}
    timescale_ps = 1.0
    t = 0.0
    with path.open() as f:
        it = iter(f)
        for line in it:
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "$timescale":
                spec = " ".join(tok[1:]) if len(tok) > 1 else next(it).strip()
                spec = spec.replace("$end", "").strip()
                num = "".join(c for c in spec if c.isdigit()) or "1"
                unit = spec[len(num):].strip()
                timescale_ps = int(num) * {"s": 1e12, "ms": 1e9, "us": 1e6, "ns": 1e3, "ps": 1, "fs": 1e-3}[unit]
            elif tok[0] == "$scope":
                scope.append(tok[2])
            elif tok[0] == "$upscope":
                scope.pop()
            elif tok[0] == "$var":
                name = ".".join(scope + [tok[4]])
                if name in wanted:
                    id2names.setdefault(tok[3], []).append(name)
                    data[name] = ([], [])
            elif tok[0] == "$enddefinitions":
                break
        for line in it:
            c = line[:1]
            if c == "#":
                t = int(line[1:]) * timescale_ps / 1e6  # -> мкс
            elif c in "01xzXZ":
                ident = line[1:].strip()
                for n in id2names.get(ident, ()):
                    data[n][0].append(t)
                    data[n][1].append(c.lower())
            elif c in "br":
                val, ident = line[1:].split()
                for n in id2names.get(ident, ()):
                    data[n][0].append(t)
                    data[n][1].append(val.lower())
    return data


def value_at(sig, t):
    ts, vs = sig
    i = bisect.bisect_right(ts, t) - 1
    return vs[i] if i >= 0 else "x"


def edges(sig, rising=True):
    ts, vs = sig
    out = []
    for i in range(1, len(ts)):
        if rising and vs[i - 1] == "0" and vs[i] == "1":
            out.append(ts[i])
        if not rising and vs[i - 1] == "1" and vs[i] == "0":
            out.append(ts[i])
    return out


# ------------------------------------------------------------- I2C decoding
def decode_i2c(scl, sda, t0, t1):
    """Події шини у вікні: ('S'|'P', t), ('bit', t, v, idx), ('byte', t_start, t_end, val, ack)."""
    ev = []
    sda_t, sda_v = sda
    for i in range(1, len(sda_t)):
        t = sda_t[i]
        if not (t0 <= t <= t1):
            continue
        if value_at(scl, t) == "1":
            if sda_v[i - 1] == "1" and sda_v[i] == "0":
                ev.append(("S", t))
            elif sda_v[i - 1] == "0" and sda_v[i] == "1":
                ev.append(("P", t))
    marks = sorted([e[1] for e in ev])
    rises = [t for t in edges(scl) if t0 <= t <= t1]
    out = list(ev)
    for si, ts in enumerate(m for m in marks if any(e == ("S", m) for e in ev)):
        nxt = [m for m in marks if m > ts]
        tend = nxt[0] if nxt else t1
        rs = [r for r in rises if ts < r < tend]
        for b in range(len(rs) // 9):
            grp = rs[b * 9:(b + 1) * 9]
            val = 0
            for k, r in enumerate(grp[:8]):
                v = 1 if value_at(sda, r + 1e-4) == "1" else 0
                val = (val << 1) | v
                out.append(("bit", r, v, k))
            ack = value_at(sda, grp[8] + 1e-4) == "0"
            out.append(("ackbit", grp[8], ack))
            t_end = grp[8]
            out.append(("byte", grp[0], t_end, val, ack, b))
    return out


# ------------------------------------------------------------------ drawing
def draw(data, rows, t0, t1, title, out, decode=True, height_per_row=0.55, notes=()):
    n = len(rows)
    fig_h = 0.9 + n * height_per_row + (0.35 if decode else 0)
    fig, ax = plt.subplots(figsize=(13, fig_h), dpi=150)
    fig.patch.set_facecolor(PANEL)
    ax.set_facecolor(BG)
    span = t1 - t0
    ax.set_xlim(t0, t1)
    ax.set_ylim(-0.6, n + (0.6 if decode else 0.2))
    ax.set_yticks([n - 1 - i + 0.4 for i in range(n)])
    ax.set_yticklabels([r[0] for r in rows], color="#e6e6e6", fontsize=9, family="monospace")
    ax.tick_params(axis="x", colors=GRAY, labelsize=8)
    ax.tick_params(axis="y", length=0)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(axis="x", color=GRID, lw=0.6)
    ax.set_xlabel("час, мкс", color=GRAY, fontsize=9)
    ax.set_title(title, color="#ffffff", fontsize=11, loc="left", pad=6)

    for i, (label, name, kind) in enumerate(rows):
        y = n - 1 - i
        sig = data[name]
        ts, vs = sig
        # точки в межах вікна + значення на початку
        pts = [(t0, value_at(sig, t0))] + [(t, v) for t, v in zip(ts, vs) if t0 < t < t1]
        pts.append((t1, pts[-1][1]))
        if kind == "bit":
            xs, ys = [], []
            for (ta, va), (tb, _) in zip(pts, pts[1:]):
                lvl = {"1": 0.8, "0": 0.0}.get(va, 0.4)
                xs += [ta, tb]
                ys += [y + lvl, y + lvl]
            ax.plot(xs, ys, color=GREEN, lw=1.1, solid_joinstyle="miter")
            for (ta, va), (tb, _) in zip(pts, pts[1:]):
                if va not in "01":
                    ax.fill_between([ta, tb], y, y + 0.8, color=RED, alpha=0.25, lw=0)
        else:  # шина: шестикутники зі значенням
            for (ta, va), (tb, _) in zip(pts, pts[1:]):
                if tb - ta <= 0:
                    continue
                d = min(span * 0.002, (tb - ta) / 3)
                ax.plot([ta, ta + d, tb - d, tb, tb - d, ta + d, ta],
                        [y + 0.4, y + 0.8, y + 0.8, y + 0.4, y, y, y + 0.4], color=GREEN, lw=0.9)
                txt = fmt_bus(va, kind)
                if (tb - ta) > span * 0.012 * max(1, len(txt) / 2):
                    ax.text((ta + tb) / 2, y + 0.4, txt, color=YELLOW, fontsize=7.5,
                            ha="center", va="center", family="monospace")

    if decode:
        scl, sda = data["tb_i2c_master.scl"], data["tb_i2c_master.sda"]
        ytop = n + 0.15
        sda_row = n - 1 - [r[1] for r in rows].index("tb_i2c_master.sda")
        for e in decode_i2c(scl, sda, t0, t1):
            if e[0] in ("S", "P"):
                ax.axvline(e[1], color=CYAN if e[0] == "S" else RED, lw=0.8, ls="--", alpha=0.8)
                ax.text(e[1], ytop + 0.2, e[0], color=CYAN if e[0] == "S" else RED, fontsize=9,
                        ha="center", va="center", weight="bold")
            elif e[0] == "bit" and span < 400:
                ax.text(e[1], sda_row + 0.4, str(e[2]), color=CYAN, fontsize=7, ha="center", va="center")
            elif e[0] == "ackbit" and span < 400:
                ax.text(e[1], sda_row + 0.4, "A" if e[2] else "N", color=GREEN if e[2] else RED,
                        fontsize=7, ha="center", va="center", weight="bold")
            elif e[0] == "byte":
                _, ta, tb, val, ack, idx = e
                lab = f"0x{val:02X} {'ACK' if ack else 'NACK'}"
                if idx == 0 and span < 2000:
                    lab = f"addr 0x{val >> 1:02X} {'R' if val & 1 else 'W'} {'ACK' if ack else 'NACK'}"
                ax.annotate("", xy=(tb, ytop), xytext=(ta, ytop),
                            arrowprops=dict(arrowstyle="<->", color=YELLOW, lw=0.7))
                ax.text((ta + tb) / 2, ytop + 0.2, lab, color=YELLOW if ack else RED, fontsize=7.5,
                        ha="center", va="center", family="monospace")
    for (tx, txt) in notes:
        ax.text(tx, -0.45, txt, color=CYAN, fontsize=7.5, ha="left", va="center")
    fig.tight_layout()
    fig.savefig(out, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("saved", out.relative_to(ROOT))


def fmt_bus(v: str, kind: str) -> str:
    if v.startswith(("x", "z")) or "x" in v:
        return "x"
    n = int(v, 2)
    if kind == "state":
        return STATES.get(n, str(n))
    if kind == "dec":
        return str(n)
    return f"{n:02X}"


# --------------------------------------------------------------------- main
TB = "tb_i2c_master"
CORE = f"{TB}.dut.u_core"
SIGS = {
    "rst_n": f"{TB}.rst_n", "cmd_start": f"{TB}.cmd_start", "busy": f"{TB}.busy", "done": f"{TB}.done",
    "ack_err": f"{TB}.ack_err", "tx_req": f"{TB}.tx_req", "tx_data": f"{TB}.tx_data",
    "rx_valid": f"{TB}.rx_valid", "rx_data": f"{TB}.rx_data", "scl": f"{TB}.scl", "sda": f"{TB}.sda",
    "state": f"{CORE}.state", "phase": f"{CORE}.phase", "qtick": f"{CORE}.qtick",
    "scl_oe": f"{CORE}.scl_oe", "sda_oe": f"{CORE}.sda_oe", "stretch": f"{CORE}.stretch",
    "s_scl_low": f"{TB}.slave.scl_low", "s_sda_low": f"{TB}.slave.sda_low",
    "tx_shreg": f"{CORE}.u_tx.shreg", "rx_shreg": f"{CORE}.u_rx.shreg",
}


def R(label, key, kind="bit"):
    return (label, SIGS[key], kind)


def main(freqs=("100000", "400000")):
    IMG.mkdir(parents=True, exist_ok=True)
    for f in freqs:
        vcd = SIM / f"tb_{f}.vcd"
        d = parse_vcd(vcd, set(SIGS.values()))
        busy = d[SIGS["busy"]]
        tr = list(zip(edges(busy, True), edges(busy, False)))   # 8 транзакцій
        khz = int(f) // 1000
        pad = (tr[0][1] - tr[0][0]) * 0.02
        if f == "100000":
            # 1. повна транзакція запису (T1)
            a, b = tr[0]
            draw(d, [R("rst_n", "rst_n"), R("cmd_start", "cmd_start"), R("busy", "busy"),
                     R("state", "state", "state"), R("tx_data", "tx_data", "hex"), R("tx_req", "tx_req"),
                     R("SCL", "scl"), R("SDA", "sda"), R("ack_err", "ack_err"), R("done", "done")],
                 a - pad, b + 3, f"Рис. А — T1: запис 5 байтів (ptr 0x02, DE AD BE EF) у пристрій 0x50, SCL = {khz} кГц",
                 IMG / "wave_write.png")
            # 2. START + адресний байт (зум)
            draw(d, [R("state", "state", "state"), R("phase", "phase", "dec"), R("qtick", "qtick"),
                     R("u_tx.shreg", "tx_shreg", "hex"), R("scl_oe (DUT)", "scl_oe"), R("sda_oe (DUT)", "sda_oe"),
                     R("SCL", "scl"), R("SDA", "sda"), R("slave sda_low", "s_sda_low")],
                 a - 3, a + 102, f"Рис. Б — START, адреса 0x50 + W і ACK веденого (фази 0–3 кожного такту SCL)",
                 IMG / "wave_start_addr.png")
            # 3. читання (T2, друга транзакція пари)
            a, b = tr[2]
            draw(d, [R("busy", "busy"), R("state", "state", "state"), R("SCL", "scl"), R("SDA", "sda"),
                     R("slave sda_low", "s_sda_low"), R("sda_oe (DUT)", "sda_oe"), R("rx_valid", "rx_valid"),
                     R("rx_data", "rx_data", "hex"), R("done", "done")],
                 a - 3, b + 3, f"Рис. В — T2: читання 4 байтів із 0x50 (майстер: ACK, ACK, ACK, NACK)",
                 IMG / "wave_read.png")
            # 4. NACK (T4)
            a, b = tr[5]
            draw(d, [R("cmd_start", "cmd_start"), R("busy", "busy"), R("state", "state", "state"),
                     R("SCL", "scl"), R("SDA", "sda"), R("ack_err", "ack_err"), R("done", "done")],
                 a - 3, b + 3, "Рис. Г — T4: звернення до відсутньої адреси 0x23 -> NACK, ack_err = 1, STOP",
                 IMG / "wave_nack.png")
            # 5. clock stretching (T5): перший такт після ACK адреси
            a, b = tr[6]
            rises = [t for t in edges(d[SIGS["scl"]]) if a < t < b]
            t_ack = rises[8]
            draw(d, [R("state", "state", "state"), R("phase", "phase", "dec"), R("scl_oe (DUT)", "scl_oe"),
                     R("slave scl_low", "s_scl_low"), R("stretch", "stretch"), R("SCL", "scl"), R("SDA", "sda")],
                 t_ack - 8, t_ack + 40, "Рис. Д — T5: clock stretching — ведений утримує SCL = 0 10 мкс, генератор фаз чекає",
                 IMG / "wave_stretch.png", notes=[(t_ack + 6, "stretch = 1: qtick заблоковано")])
        else:
            a, b = tr[2]
            draw(d, [R("busy", "busy"), R("state", "state", "state"), R("SCL", "scl"), R("SDA", "sda"),
                     R("rx_valid", "rx_valid"), R("rx_data", "rx_data", "hex")],
                 a - 1, b + 1, f"Рис. Е — Fast-mode {khz} кГц: читання 4 байтів (DE AD BE EF)",
                 IMG / "wave_read_400k.png")


if __name__ == "__main__":
    main(tuple(sys.argv[1:]) or ("100000", "400000"))
