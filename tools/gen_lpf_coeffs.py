#!/usr/bin/env python3
"""Генератор коэффициентов ФНЧ Баттерворта для ExtDrivers/NirsDSP.cpp.

    python3 tools/gen_lpf_coeffs.py [fs_hz] [order] [fc_hz ...]
    python3 tools/gen_lpf_coeffs.py 100 4 5 8 10 12 15 20

Вывод - готовый C++ фрагмент в формате CMSIS-DSP arm_biquad_cascade_df2T_f32:
на каждую секцию {b0, b1, b2, -a1, -a2} (знаки a инвертированы!). Коэффициент
усиления разнесён по секциям так, чтобы у каждой усиление на постоянном токе
было 1 (как в исходной таблице для Fs = 1000 Гц). Требуется scipy.
"""
import sys
import numpy as np
from scipy import signal

fs = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
order = int(sys.argv[2]) if len(sys.argv) > 2 else 4
fcs = [float(x) for x in sys.argv[3:]] or [5, 8, 10, 12, 15, 20]

print(f"// Фильтры Баттерворта {order}-го порядка, Fs = {fs:g} Гц (сгенерировано tools/gen_lpf_coeffs.py)")
for fc in fcs:
    sos = signal.butter(order, fc, btype="low", fs=fs, output="sos")
    rows = []
    for s in sos:
        b, a = s[:3].copy(), s[3:]
        b /= b.sum() / a.sum()                       # усиление секции на DC = 1
        rows.append([b[0], b[1], b[2], -a[1], -a[2]])
    w, h = signal.sosfreqz(sos, worN=[2 * np.pi * 1e-4])
    gd = signal.group_delay(signal.sos2tf(sos), w=[1e-3])[1][0] / fs * 1e3
    print(f"// {fc:g} Гц: задержка на низких частотах ~{gd:.0f} мс")
    print(f"{{ // {fc:g} Гц")
    for i, r in enumerate(rows):
        sep = "," if i < len(rows) - 1 else ""
        print("    " + ", ".join(f"{np.float32(v):.9g}f" for v in r) + sep)
    print("},")
