"""Діаграми станів (FSM) I2C Master -> docs/img/fsm_master.png, docs/img/fsm_tx.png."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch  # noqa: E402

IMG = Path(__file__).resolve().parent / "img"
INK, ACC, MUTED, FILL, FILL2, RED = "#1d2733", "#1f6feb", "#5b6675", "#eef4ff", "#fff6e0", "#c0392b"
R = 0.62


def node(ax, xy, name, sub="", fill=FILL, initial=False, r=R):
    ax.add_patch(Circle(xy, r, fc=fill, ec=INK, lw=1.6, zorder=3))
    if initial:
        ax.add_patch(Circle(xy, r - 0.08, fc="none", ec=INK, lw=1.0, zorder=4))
    ax.text(xy[0], xy[1] + (0.12 if sub else 0), name, ha="center", va="center",
            fontsize=11, weight="bold", color=INK, zorder=5)
    if sub:
        ax.text(xy[0], xy[1] - 0.22, sub, ha="center", va="center", fontsize=7.5, color=MUTED, zorder=5)


def edge(ax, a, b, label, rad=0.0, lx=0.0, ly=0.0, color=ACC, fs=8, r=R):
    # відступ стрілки від центру вузла = радіус кола в пунктах
    pt = r * ax.figure.get_figwidth() * 72 / (ax.get_xlim()[1] - ax.get_xlim()[0]) + 2
    ax.add_patch(FancyArrowPatch(a, b, connectionstyle=f"arc3,rad={rad}", arrowstyle="-|>",
                                 mutation_scale=14, lw=1.3, color=color, shrinkA=pt, shrinkB=pt, zorder=2))
    mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    ax.text(mx + lx, my + ly, label, ha="center", va="center", fontsize=fs, color=INK,
            bbox=dict(fc="white", ec="none", pad=1.2), zorder=6)


def loop(ax, xy, label, side="right", color=ACC, fs=8):
    x, y = xy
    if side == "right":
        a, b, t = (x + 0.45, y + 0.42), (x + 0.45, y - 0.42), (x + 1.55, y)
    else:
        a, b, t = (x - 0.45, y - 0.42), (x - 0.45, y + 0.42), (x - 1.6, y)
    ax.add_patch(FancyArrowPatch(a, b, connectionstyle="arc3,rad=-1.6", arrowstyle="-|>",
                                 mutation_scale=12, lw=1.3, color=color, zorder=2))
    ax.text(*t, label, ha="center", va="center", fontsize=fs, color=INK,
            bbox=dict(fc="white", ec="none", pad=1.2), zorder=6)


def setup(w, h, xlim, ylim):
    fig, ax = plt.subplots(figsize=(w, h), dpi=170)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor("white")
    return fig, ax


# ------------------------------------------------------------ головний FSM
def master():
    fig, ax = setup(11.5, 7.0, (-0.8, 15.4), (-1.0, 7.9))
    P = {"IDLE": (1.0, 6.5), "START": (5.5, 6.5), "ADDR": (10.0, 6.5), "WRITE": (6.6, 3.3),
         "READ": (13.4, 3.3), "STOP": (10.0, 0.2), "BUF": (5.5, 0.2)}
    sub = {"IDLE": "SCL=SDA=1", "START": "SDA↓, SCL=1", "ADDR": "i2c_tx", "WRITE": "i2c_tx",
           "READ": "i2c_rx", "STOP": "SDA↑, SCL=1", "BUF": "t_BUF"}
    for k, v in P.items():
        node(ax, v, k, sub[k], fill=FILL2 if k in ("WRITE", "READ", "ADDR") else FILL, initial=(k == "IDLE"))
    ax.annotate("rst_n = 0", xy=(0.72, 5.93), xytext=(-0.5, 4.9), fontsize=8, color=MUTED,
                arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.1))
    edge(ax, P["IDLE"], P["START"], "cmd_start\n(addr, rw, len)", ly=0.5)
    edge(ax, P["START"], P["ADDR"], "phase 3: SCL↓\ntx_go({addr,rw})", ly=0.5)
    edge(ax, P["ADDR"], P["WRITE"], "ACK & rw=0\n& len>0", lx=-0.8, ly=0.2)
    edge(ax, P["ADDR"], P["READ"], "ACK & rw=1\n& len>0", lx=0.85, ly=0.2)
    edge(ax, P["ADDR"], P["STOP"], "NACK\n(ack_err=1)\nабо len=0", ly=0.6, color=RED)
    edge(ax, P["WRITE"], P["STOP"], "NACK | last", lx=-0.6, ly=-0.3, color=RED)
    edge(ax, P["READ"], P["STOP"], "last\n(NACK видано)", lx=0.8, ly=-0.3)
    loop(ax, P["WRITE"], "ACK & !last\ntx_req, tx_go", side="left")
    loop(ax, P["READ"], "!last (ACK)\nrx_valid,\nrx_go", side="right")
    edge(ax, P["STOP"], P["BUF"], "phase 3: SDA↑", ly=0.3)
    edge(ax, P["BUF"], P["IDLE"], "phase 3\ndone = 1", lx=-0.9, ly=-0.1)
    ax.text(1.9, 1.0, "START / STOP / BUF — 1 такт SCL (4 фази qtick)\n"
                      "ADDR / WRITE / READ — 9 тактів SCL\n(8 біт + ACK) у підмодулі i2c_tx / i2c_rx;\n"
                      "переходи з ADDR / WRITE — по tx_done,\nз READ — по rx_done",
            ha="center", va="center", fontsize=7.8, color=MUTED, style="italic")
    fig.tight_layout()
    fig.savefig(IMG / "fsm_master.png", facecolor="white")
    plt.close(fig)


# ------------------------------------------------------------ FSM передавача
def tx():
    fig, ax = setup(11, 5.4, (-1.8, 12.4), (-1.3, 5.4))
    r = 0.7
    P = {"IDLE": (0.2, 4.0), "PH0": (4.6, 4.0), "PH1": (9.4, 4.0), "PH2": (9.4, 0.0), "PH3": (4.6, 0.0),
         "DONE": (0.2, 0.0)}
    sub = {"IDLE": "busy=0", "PH0": "SDA ← біт", "PH1": "SCL ↑", "PH2": "середина SCL=1",
           "PH3": "SCL ↓, зсув", "DONE": "done=1"}
    for k, v in P.items():
        node(ax, v, k, sub[k], fill=FILL2 if k.startswith("PH") else FILL, initial=(k == "IDLE"), r=r)
    edge(ax, P["IDLE"], P["PH0"], "start: shreg←din,\nbitn←0, SCL=0", ly=0.5, r=r)
    edge(ax, P["PH0"], P["PH1"], "qtick:\nsda_oe ← bitn<8 ? ~shreg[7] : 0", ly=0.5, r=r)
    edge(ax, P["PH1"], P["PH2"], "qtick: scl_oe ← 0\n(лічильник стоїть,\nпоки SCL=0 — stretch)", lx=1.4, r=r)
    edge(ax, P["PH2"], P["PH3"], "qtick:\nbitn=8 ? ack ← ~sda_i", ly=-0.5, r=r)
    edge(ax, P["PH3"], P["PH0"], "qtick & bitn<8:\nscl_oe←1, shreg<<1,\nbitn←bitn+1", lx=1.5, r=r)
    edge(ax, P["PH3"], P["DONE"], "qtick & bitn=8:\nscl_oe←1", ly=-0.5, r=r)
    edge(ax, P["DONE"], P["IDLE"], "наступний\nтакт clk", lx=-1.0, r=r)
    fig.tight_layout()
    fig.savefig(IMG / "fsm_tx.png", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    IMG.mkdir(parents=True, exist_ok=True)
    master()
    tx()
    print("saved fsm_master.png, fsm_tx.png")
