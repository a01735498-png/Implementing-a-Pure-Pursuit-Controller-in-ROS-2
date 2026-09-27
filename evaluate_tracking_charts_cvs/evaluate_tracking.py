#!/usr/bin/env python3
"""Analisis de desempeno del controlador Pure Pursuit (Step 3).

Compara la trayectoria de referencia (waypoints.csv) contra la trayectoria
que el auto realmente siguio en modo autonomo (actual_trajectory.csv):
  - calcula el error de cross-track (distancia lateral minima de cada punto
    de la trayectoria ejecutada a la ruta de referencia), reportando su
    media, maximo y RMS.
  - genera una grafica de ambas trayectorias superpuestas.

Uso:
    python3 evaluate_tracking.py waypoints.csv actual_trajectory.csv

    (o con nombres de archivo explicitos)
    python3 evaluate_tracking.py --ref waypoints.csv --actual actual_trajectory.csv \
        --out reporte_tracking

Requiere: pandas, numpy, matplotlib
    pip install pandas numpy matplotlib --break-system-packages   # si hace falta
"""
import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_xy(path):
    df = pd.read_csv(path)
    cols = {c.lower(): c for c in df.columns}
    if "x" not in cols or "y" not in cols:
        raise ValueError(f"'{path}' no tiene columnas 'x' e 'y'. Columnas encontradas: {list(df.columns)}")
    return df[[cols["x"], cols["y"]]].to_numpy(dtype=float)


def point_to_segment_distance(p, a, b):
    """Distancia minima del punto p al segmento a-b (proyeccion, clamp en [0,1])."""
    ab = b - a
    ab_len2 = np.dot(ab, ab)
    if ab_len2 < 1e-12:
        return np.linalg.norm(p - a)
    t = np.dot(p - a, ab) / ab_len2
    t = np.clip(t, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(p - proj)


def cross_track_errors(actual, ref):
    """Para cada punto de 'actual', la distancia lateral minima a la polilinea 'ref'."""
    errs = np.empty(len(actual))
    for i, p in enumerate(actual):
        best = np.inf
        for j in range(len(ref) - 1):
            d = point_to_segment_distance(p, ref[j], ref[j + 1])
            if d < best:
                best = d
        errs[i] = best
    return errs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ref", nargs="?", default="waypoints.csv", help="CSV de la trayectoria de referencia")
    ap.add_argument("actual", nargs="?", default="actual_trajectory.csv", help="CSV de la trayectoria ejecutada")
    ap.add_argument("--ref", dest="ref_kw", help="alternativa con --ref")
    ap.add_argument("--actual", dest="actual_kw", help="alternativa con --actual")
    ap.add_argument("--out", default="tracking_report", help="prefijo para los archivos de salida")
    args = ap.parse_args()

    ref_path = args.ref_kw or args.ref
    actual_path = args.actual_kw or args.actual

    ref = load_xy(ref_path)
    actual = load_xy(actual_path)

    if len(ref) < 2:
        sys.exit(f"'{ref_path}' tiene menos de 2 puntos, no se puede comparar.")
    if len(actual) < 1:
        sys.exit(f"'{actual_path}' esta vacio.")

    errs = cross_track_errors(actual, ref)

    mean_err = errs.mean()
    max_err = errs.max()
    rms_err = np.sqrt((errs ** 2).mean())
    worst_idx = int(errs.argmax())

    print(f"Referencia:          {ref_path}  ({len(ref)} puntos)")
    print(f"Trayectoria ejecutada: {actual_path}  ({len(actual)} puntos)")
    print(f"--- Error de cross-track ---")
    print(f"Media (mean):  {mean_err:.4f} m")
    print(f"RMS:           {rms_err:.4f} m")
    print(f"Maximo:        {max_err:.4f} m  (punto #{worst_idx}, "
          f"x={actual[worst_idx,0]:.2f}, y={actual[worst_idx,1]:.2f})")

    # Guarda el error por punto, por si se quiere graficar error vs distancia recorrida
    out_csv = f"{args.out}_errors.csv"
    pd.DataFrame({
        "x": actual[:, 0], "y": actual[:, 1], "cross_track_error_m": errs,
    }).to_csv(out_csv, index=False)
    print(f"Errores por punto guardados en: {out_csv}")

    # --- Grafica de superposicion ---
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(ref[:, 0], ref[:, 1], "-", color="0.4", lw=2.5, label="Referencia (waypoints.csv)")
    ax.plot(actual[:, 0], actual[:, 1], "-", color="C3", lw=1.5, label="Ejecutada (actual_trajectory.csv)")
    ax.plot(ref[0, 0], ref[0, 1], "go", ms=9, label="Inicio")
    ax.plot(ref[-1, 0], ref[-1, 1], "ks", ms=8, label="Meta")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Referencia vs. trayectoria ejecutada\n"
                 f"error medio = {mean_err:.3f} m | error max = {max_err:.3f} m")
    ax.legend()
    fig.tight_layout()
    out_png = f"{args.out}.png"
    fig.savefig(out_png, dpi=130)
    print(f"Grafica guardada en: {out_png}")


if __name__ == "__main__":
    main()
