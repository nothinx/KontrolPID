"""Compile & jalankan simulasi.cpp (kode KontrolPID asli), lalu render grafik ke ../gambar/.

Jalankan dari folder ini: python gambar.py   (butuh g++ dan matplotlib)
"""
import glob
import os
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "figure.figsize": (8, 3.6), "figure.dpi": 100, "savefig.bbox": "tight", "savefig.pad_inches": 0.15,
    "figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
    "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": "#9ca3af",
    "axes.grid": True, "grid.color": "#e5e7eb", "grid.linewidth": 0.8,
    "legend.frameon": False, "svg.fonttype": "path", "svg.hashsalt": "nothinx",
    "lines.linewidth": 1.8,
})
WARNA = {"utama": "#2563eb", "pembanding": "#dc2626", "ketiga": "#16a34a", "keempat": "#9333ea",
         "kelima": "#ea580c", "mentah": "#9ca3af", "target": "#111827"}
KELUAR = os.path.join("..", "gambar")


def jalankan():
    with tempfile.TemporaryDirectory() as tmp:
        exe = os.path.join(tmp, "sim.exe" if os.name == "nt" else "sim")
        subprocess.run(["g++", "-std=c++11", "-O2", "-I../test", "-I../../src", "simulasi.cpp",
                        *glob.glob("../../src/*.cpp"), "-o", exe], check=True)
        teks = subprocess.run([exe], check=True, capture_output=True, text=True).stdout
    data, nama = {}, None
    for baris in teks.splitlines():
        if baris.startswith("# "):
            nama = baris[2:]
            data[nama] = []
        elif baris:
            data[nama].append([float(v) for v in baris.split(",")])
    return {k: list(zip(*v)) for k, v in data.items()}  # per kolom


def koma(x, n=1):
    return f"{x:.{n}f}".replace(".", ",")


def simpan(fig, nama):
    fig.savefig(os.path.join(KELUAR, nama), format="svg", metadata={"Date": None})
    plt.close(fig)


def respon_pemanas(d):
    varian = [("pemanas_naif", "Tanpa anti-windup", "pembanding"),
              ("pemanas_pidv1", "Algoritma PID_v1", "kelima"),
              ("pemanas_kontrolpid", "KontrolPID", "utama")]
    os_ = {k: max(d[k][1]) - 100 for k, _, _ in varian}
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(8, 5), sharex=True, gridspec_kw={"height_ratios": [3, 1.3]})
    ax.axhline(100, color=WARNA["target"], ls="--", lw=1.2)
    ax.text(59, 101.5, "target 100°", ha="right", va="bottom", color=WARNA["target"])
    for k, label, w in varian:
        t, y, u = d[k]
        ax.plot(t, y, color=WARNA[w], label=f"{label} (overshoot {koma(os_[k])}°)")
        bx.plot(t, u, color=WARNA[w], lw=1.4)
    ax.set_title(f"Anti-windup memangkas overshoot dari {koma(os_['pemanas_naif'])}° "
                 f"menjadi {koma(os_['pemanas_kontrolpid'])}°")
    ax.set_ylabel("Suhu (°C)")
    ax.set_ylim(0, 130)
    ax.legend(loc="lower right")
    bx.set_ylabel("PWM")
    bx.set_ylim(-10, 270)
    bx.set_yticks([0, 255])
    bx.text(59, 240, "keluaran mentok 255 saat awal (saturasi)", ha="right", va="top", fontsize=9)
    bx.set_xlabel("Waktu (detik)")
    bx.set_xlim(0, 60)
    simpan(fig, "respon-pemanas.svg")


def loop_tidak_rata(d):
    t, acuan = d["loop_acuan"]
    selisih = {k: max(abs(a - b) for a, b in zip(acuan, d[k][1])) for k in ("loop_kontrolpid", "loop_pidv1")}
    fig, ax = plt.subplots()
    ax.axhline(100, color=WARNA["target"], ls="--", lw=1.2)
    ax.plot(t, acuan, color=WARNA["mentah"], lw=5, alpha=0.6, label="Rancangan (hitung() tiap 100 ms tepat)")
    ax.plot(t, d["loop_pidv1"][1], color=WARNA["pembanding"],
            label=f"Algoritma PID_v1, SampleTime 100 ms (menyimpang maks {koma(selisih['loop_pidv1'])}°)")
    ax.plot(t, d["loop_kontrolpid"][1], color=WARNA["utama"],
            label=f"KontrolPID, dt dari micros() (menyimpang maks {koma(selisih['loop_kontrolpid'])}°)")
    ax.set_title(f"Loop tidak rata (100–400 ms): KontrolPID menyimpang {koma(selisih['loop_kontrolpid'])}°, "
                 f"PID_v1 {koma(selisih['loop_pidv1'])}°")
    ax.set_xlabel("Waktu (detik)")
    ax.set_ylabel("Suhu (°C)")
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 125)
    ax.legend(loc="lower right")
    simpan(fig, "loop-tidak-rata.svg")


def derivative_kick(d):
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(8, 4.6), sharex=True, gridspec_kw={"height_ratios": [1.3, 2]})
    t, _, _, _, target = d["kick_kontrolpid"]
    ax.plot(t, target, color=WARNA["target"], ls="--", lw=1.2)
    ax.text(2.05, 142, "target 100 → 140 rpm", va="bottom", color=WARNA["target"])
    puncak = {}
    for k, label, w in [("kick_bukuteks", "D dari error (rumus buku teks)", "pembanding"),
                        ("kick_kontrolpid", "KontrolPID: D dari perubahan nilai", "utama")]:
        _, rpm, u, dd, _ = d[k]
        puncak[k] = max(abs(v) for v in dd[200:])
        ax.plot(t, rpm, color=WARNA[w])
        bx.plot(t, u, color=WARNA[w], label=label)
    ax.set_ylabel("Kecepatan (rpm)")
    ax.set_ylim(90, 155)
    ax.set_title(f"Ganti target: D dari error melonjak ke {koma(puncak['kick_bukuteks'], 0)}, "
                 f"D dari nilai hanya {koma(puncak['kick_kontrolpid'], 0)}")
    bx.axhline(255, color=WARNA["mentah"], lw=1, ls=":")
    bx.text(1.52, 250, "batas 255", va="top", color="#6b7280", fontsize=9)
    bx.set_ylabel("Keluaran (PWM)")
    bx.set_ylim(80, 270)
    bx.set_xlabel("Waktu (detik)")
    bx.set_xlim(1.5, 3)
    bx.legend(loc="upper right")
    simpan(fig, "derivative-kick.svg")


if __name__ == "__main__":
    os.makedirs(KELUAR, exist_ok=True)
    d = jalankan()
    respon_pemanas(d)
    loop_tidak_rata(d)
    derivative_kick(d)
