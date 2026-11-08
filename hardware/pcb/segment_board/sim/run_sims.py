#!/usr/bin/env python3
"""SPICE checks of the segment board's analog circuits (ngspice, batch mode).

Each test writes a netlist to sim/cir/, runs `ngspice -b`, reads the `wrdata` output
and draws a plot in sim/results/. A pass/fail summary goes to sim/results/summary.txt.

Values are copied from gen/build.py (connectors_sheet, i2c_sheet). Part models are
generic approximations (see MODELS below) - the goal is to check the operating points
and margins, not the exact datasheet curves.

Usage:  python3 sim/run_sims.py
Needs:  ngspice (apt install ngspice), matplotlib
"""
import pathlib
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
CIR = HERE / "cir"
OUT = HERE / "results"

V33 = 3.3
LM339_VOS = 5e-3          # LM339 worst-case input offset (datasheet max 5 mV at 25 C)

MODELS = """
* SS34 Schottky (approx.: ~0.36 V at 0.8 A)
.model SS34 D(Is=5u N=1.05 Rs=0.04 Cjo=200p)
* 0603 LEDs: InGaN green (~2.6 V @ 1 mA) and AlInGaP yellow-green / yellow (~1.9 V @ 1 mA)
.model LED_GREEN_INGAN D(Is=1e-22 N=2.2 Rs=10)
.model LED_YG D(Is=1e-18 N=2.0 Rs=8)
* open-collector output stage of one LM339 unit: switch on when V(-) > V(+) + offset
.model OC SW(Vt=0 Vh=0.0001 Ron=40 Roff=1e9)
"""

results = []


def run(name, body, analysis, vectors):
    CIR.mkdir(exist_ok=True)
    OUT.mkdir(exist_ok=True)
    data = OUT / f"{name}.dat"
    net = f"""* {name}
{MODELS}
{body}
.control
set wr_singlescale
set wr_vecnames
{analysis}
wrdata {data} {' '.join(vectors)}
quit
.endc
.end
"""
    (CIR / f"{name}.cir").write_text(net)
    p = subprocess.run(["ngspice", "-b", str(CIR / f"{name}.cir")], capture_output=True, text=True)
    if p.returncode != 0 or not data.exists():
        raise RuntimeError(f"{name}: ngspice failed\n{p.stdout}\n{p.stderr}")
    lines = data.read_text().split("\n")
    head = lines[0].split()
    rows = [list(map(float, l.split())) for l in lines[1:] if l.strip()]
    cols = {h: [r[i] for r in rows] for i, h in enumerate(head)}
    data.unlink()
    return cols


def check(test, ok, detail):
    results.append((test, "PASS" if ok else "FAIL", detail))


def interp_x(xs, ys, y0):
    """first x where ys crosses y0"""
    for i in range(1, len(xs)):
        if (ys[i - 1] - y0) * (ys[i] - y0) <= 0 and ys[i] != ys[i - 1]:
            return xs[i - 1] + (y0 - ys[i - 1]) * (xs[i] - xs[i - 1]) / (ys[i] - ys[i - 1])
    return None


def comparator(minus, plus, out, offset=0.0):
    """one LM339 unit, open-collector output to GND. offset > 0 makes it trip later."""
    return f"S_{out} {out} 0 {minus} cmp_{out} OC\nV_os_{out} cmp_{out} {plus} {offset}\n"


# ------------------------------------------------------------------------------------------
def touch():
    """ZD10 touch sensor -> 6.8k divider -> ADS1115 AIN0 and LM339 (ref 0.3 V)."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    body = f"""
V1 3v3 0 {V33}
Rsens 3v3 touch 1k
Rdiv touch 0 6.8k
Cf touch 0 100n
Rr1 3v3 vref 100k
Rr2 vref 0 10k
{comparator('vref', 'touch', 'k')}
Rled 3v3 a 1k
Dled a k LED_YG
"""
    d = run("touch_dc", body, "dc Rsens 500 200k 250", ["v(touch)", "v(vref)", "i(V1)"])
    # ngspice can't sweep a resistor through 'dc' in all versions -> use .step style below if needed
    rs = d["res-sweep"] if "res-sweep" in d else None
    if rs is None:
        rs = list(range(len(d["v(touch)"])))
    vt = d["v(touch)"]
    ax[0].semilogx([r / 1e3 for r in rs], vt, label="V(TOUCH) = ADS1115 AIN")
    ax[0].axhline(d["v(vref)"][0], color="r", ls="--", label=f"LED threshold {d['v(vref)'][0]:.2f} V")
    for r, lab in ((2e3, "pressed 2k"), (17e3, "idle 17k")):
        v = V33 * 6.8e3 / (6.8e3 + r)
        ax[0].plot(r / 1e3, v, "o")
        ax[0].annotate(f"{lab}: {v:.2f} V", (r / 1e3, v), textcoords="offset points", xytext=(5, 5))
    ax[0].set_xlabel("sensor resistance (kΩ)")
    ax[0].set_ylabel("V")
    ax[0].set_title("Touch input vs sensor resistance")
    ax[0].grid(True, which="both", alpha=.3)
    ax[0].legend()
    v_idle = V33 * 6.8 / (6.8 + 17)
    v_press = V33 * 6.8 / (6.8 + 2)
    r_detect = interp_x(rs, vt, d["v(vref)"][0])
    check("touch: idle level", 0.8 < v_idle < 1.1, f"idle (17k) -> {v_idle:.2f} V, ADS gain ±4.096 V -> code {v_idle / 4.096 * 32767:.0f}")
    check("touch: pressed level", v_press < V33 - 0.3, f"pressed (2k) -> {v_press:.2f} V, code {v_press / 4.096 * 32767:.0f}; span {v_press - v_idle:.2f} V")
    check("touch: LED detects sensor", r_detect is not None and r_detect > 17e3 * 2,
          f"LED on for sensor R < {r_detect / 1e3:.0f} kΩ (idle 17k -> margin x{r_detect / 17e3:.1f}); open -> 0 V -> LED off")

    # step response of the 100 nF filter (pressed)
    body2 = f"""
V1 3v3 0 {V33}
Vp p 0 PULSE(0 1 1m 1u 1u 20m 40m)
Rsens 3v3 touch 17k
Rp 3v3 x 2.43k
Sp x touch p 0 PSW
.model PSW SW(Vt=0.5 Ron=1 Roff=1e12)
Rdiv touch 0 6.8k
Cf touch 0 100n
"""
    d2 = run("touch_step", body2, "tran 10u 40m", ["v(touch)"])
    t = [x * 1e3 for x in d2["time"]]
    ax[1].plot(t, d2["v(touch)"])
    ax[1].set_xlabel("ms")
    ax[1].set_ylabel("V(TOUCH)")
    ax[1].set_title("Press / release (17k -> 2k), 100 nF filter")
    ax[1].grid(True, alpha=.3)
    v = d2["v(touch)"]
    lo, hi = min(v), max(v)
    t90 = interp_x(d2["time"], v, lo + 0.9 * (hi - lo))
    check("touch: response time", t90 is not None and (t90 - 1e-3) < 5e-3, f"10-90% rise {(t90 - 1e-3) * 1e3:.2f} ms (ADS1115 at 128 SPS = 7.8 ms/sample)")
    fig.tight_layout()
    fig.savefig(OUT / "touch.png", dpi=110)
    plt.close(fig)


# ------------------------------------------------------------------------------------------
def detector(name, title, rsense, rtop, rbot, i_max, i_marks, diode=False, led="LED_YG"):
    """current through the connector ground return -> sense voltage -> LM339 -> LED."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    vref = V33 * rbot / (rtop + rbot)
    curves = {}
    for tag, off in (("typ", 0.0), ("+5mV", LM339_VOS), ("-5mV", -LM339_VOS)):
        body = f"""
V1 3v3 0 {V33}
Iload 0 sense 0
Rs sense 0 {rsense}
{'Dc sense 0 SS34' if diode else ''}
Rr1 3v3 vref {rtop}
Rr2 vref 0 {rbot}
{comparator('sense', 'vref', 'k', off)}
Rled 3v3 a 1k
Dled a k {led}
Vm k k2 0
"""
        # the LED cathode goes to the switch through Vm so the LED current can be read
        body = body.replace(comparator('sense', 'vref', 'k', off), comparator('sense', 'vref', 'k2', off))
        d = run(f"{name}_{tag}".replace("+", "p").replace("-", "m"), body,
                f"dc Iload 0 {i_max} {i_max / 20000}", ["v(sense)", "i(Vm)"])
        curves[tag] = d
    d = curves["typ"]
    i = [x * 1e3 for x in d["v-sweep"]] if "v-sweep" in d else [x * 1e3 for x in d["i-sweep"]]
    ax[0].plot(i, [v * 1e3 for v in d["v(sense)"]], label="V(sense)")
    ax[0].axhline(vref * 1e3, color="r", ls="--", label=f"VREF {vref * 1e3:.1f} mV")
    ax[0].axhspan((vref - LM339_VOS) * 1e3, (vref + LM339_VOS) * 1e3, color="r", alpha=.12, label="± LM339 offset")
    for x, lab in i_marks:
        ax[0].axvline(x * 1e3, color="g", ls=":")
        ax[0].text(x * 1e3, ax[0].get_ylim()[1] * 0.9, lab, rotation=90, va="top", fontsize=8)
    ax[0].set_xlabel("connector current (mA)")
    ax[0].set_ylabel("mV")
    ax[0].set_title(f"{title}: sense voltage")
    ax[0].grid(True, alpha=.3)
    ax[0].legend(fontsize=8)
    thr = {}
    for tag, dd in curves.items():
        il = [x * 1e3 for x in dd["i(Vm)"]]
        ax[1].plot(i, il, label=f"LM339 offset {tag}")
        thr[tag] = interp_x(i, il, max(il) / 2)
    ax[1].set_xlabel("connector current (mA)")
    ax[1].set_ylabel("LED current (mA)")
    ax[1].set_title(f"{title}: LED")
    ax[1].grid(True, alpha=.3)
    ax[1].legend(fontsize=8)
    ax[1].set_xlim(0, min(i_max * 1e3, max(t for t in thr.values() if t) * 3))
    fig.tight_layout()
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)
    return vref, thr, max(curves["typ"]["i(Vm)"]) * 1e3, d


def encoder():
    vref, thr, i_led, _ = detector("encoder_detect", "Encoder (10 Ω, ref 33 mV)", 10, 100e3, 1e3, 20e-3,
                                   [(6.5e-3, "AS5600 6.5 mA")])
    hi = max(t for t in thr.values() if t)
    check("encoder: detects AS5600", hi < 6.5 * 0.75,
          f"LED turns on at {min(thr.values()):.2f}-{hi:.2f} mA (AS5600 ~6.5 mA, margin {6.5 / hi:.1f}x); LED current {i_led:.2f} mA")
    check("encoder: ground lift", 6.5e-3 * 10 < 0.1, f"AS5600 ground sits {6.5e-3 * 10 * 1e3:.0f} mV above board GND (I2C VIL 0.3·VDD = 0.99 V -> negligible)")


def servo():
    vref, thr, i_led, d = detector("servo_detect", "Servo (4.7 Ω ‖ SS34, ref 10 mV)", 4.7, 330e3, 1e3, 1.0,
                                   [(0.4, "0.4 A run"), (0.8, "0.8 A peak")], diode=True)
    lo, hi = min(thr.values()), max(thr.values())
    check("servo: detect threshold", True,
          f"LED turns on at {lo:.2f}-{hi:.2f} mA -> servo idle current must be > {hi:.1f} mA (TBD: measure it)")
    vs = d["v(sense)"]
    isw = d["i-sweep"] if "i-sweep" in d else d["v-sweep"]
    v08 = vs[min(range(len(isw)), key=lambda k: abs(isw[k] - 0.8))]
    v04 = vs[min(range(len(isw)), key=lambda k: abs(isw[k] - 0.4))]
    check("servo: ground lift at 0.4 / 0.8 A", v08 < 0.5,
          f"{v04 * 1e3:.0f} mV / {v08 * 1e3:.0f} mV (SS34 clamps; without it 4.7 Ω x 0.8 A = 3.76 V)")
    check("servo: signal seen by servo", V33 - v08 > 2.0,
          f"PWM high relative to servo ground = {V33 - v08:.2f} V at 0.8 A (servo logic threshold typ. ~1.5-2 V)")
    check("servo: sense resistor power", 0.36 ** 2 / 4.7 < 0.1,
          f"resistor {v08 ** 2 / 4.7 * 1e3:.0f} mW, SS34 {v08 * 0.8 * 1e3:.0f} mW at 0.8 A (0603 = 100 mW, SMA ~1 W)")


def led_brightness():
    """1k from 3.3 V through the LED into an LM339 output (Vsat ~0.15 V)"""
    out = []
    for m, lab in (("LED_YG", "yellow-green AlInGaP"), ("LED_GREEN_INGAN", "green InGaN")):
        d = run(f"led_{m}", f"V1 3v3 0 {V33}\nR1 3v3 a 1k\nD1 a k {m}\nVsat k 0 0.15\n", "op", ["i(Vsat)"])
        out.append((lab, abs(d["i(Vsat)"][0]) * 1e3))
    detail = ", ".join(f"{lab}: {i:.2f} mA" for lab, i in out)
    check("connection LED current", min(i for _, i in out) > 0.5,
          detail + " -> choose a low-Vf (AlInGaP, ~2 V) LED or lower the resistor for InGaN")


def main():
    for f in (touch, encoder, servo, led_brightness):
        f()
    w = max(len(t) for t, _, _ in results)
    lines = [f"{s}  {t:<{w}}  {d}" for t, s, d in results]
    (OUT / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
