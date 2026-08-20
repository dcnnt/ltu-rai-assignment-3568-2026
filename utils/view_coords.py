#!/usr/bin/env python3
"""
Visor de coordenadas del mapa: mundo (metros) vs pixel (fila, columna).

Relacion entre los dos sistemas (convencion ROS map_server):
    - pixel (0,0) = esquina SUPERIOR-IZQUIERDA de la imagen (fila 0 arriba, columna 0 izquierda)
    - origin_x, origin_y (del map.yaml) = posicion en el mundo del pixel INFERIOR-IZQUIERDO
    - el eje Y del mundo crece hacia ARRIBA, el eje "fila" de pixel crece hacia ABAJO

    col = (world_x - origin_x) / resolution
    row = height - 1 - (world_y - origin_y) / resolution

    world_x = origin_x + col * resolution
    world_y = origin_y + (height - 1 - row) * resolution

Uso:
    python3 view_coords.py --pgm warehouse_demo.pgm

Controles:
    mover el raton   -> la barra de herramientas (abajo) muestra world=(x,y) m  y  pixel=(col,row)
    click izquierdo  -> marca un punto; al segundo click imprime el bbox [xmin,xmax,ymin,ymax]
                        en world y en pixel, y limpia para la siguiente pareja de puntos
    click derecho     -> descarta el punto marcado sin imprimir nada
"""

import argparse
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

DEFAULT_RESOLUTION = 0.03
DEFAULT_ORIGIN = (-15.1, -25.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", required=True)
    ap.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    ap.add_argument("--origin-x", type=float, default=DEFAULT_ORIGIN[0])
    ap.add_argument("--origin-y", type=float, default=DEFAULT_ORIGIN[1])
    ap.add_argument("--grid-step-m", type=float, default=2.0, help="paso de la rejilla en metros")
    args = ap.parse_args()

    res = args.resolution
    ox, oy = args.origin_x, args.origin_y

    img = Image.open(args.pgm)
    w_px, h_px = img.size
    extent = [ox, ox + w_px * res, oy, oy + h_px * res]

    def world_to_pixel(x, y):
        col = (x - ox) / res
        row = h_px - 1 - (y - oy) / res
        return col, row

    def pixel_to_world(col, row):
        x = ox + col * res
        y = oy + (h_px - 1 - row) * res
        return x, y

    fig, ax = plt.subplots(figsize=(9, 13))
    ax.imshow(np.array(img), cmap="gray", extent=extent, origin="upper")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]  (mundo)")
    ax.set_ylabel("y [m]  (mundo)")

    # rejilla en metros, redondeada al grid-step
    import math
    x0 = math.ceil(extent[0] / args.grid_step_m) * args.grid_step_m
    x1 = math.floor(extent[1] / args.grid_step_m) * args.grid_step_m
    y0 = math.ceil(extent[2] / args.grid_step_m) * args.grid_step_m
    y1 = math.floor(extent[3] / args.grid_step_m) * args.grid_step_m
    ax.set_xticks(np.arange(x0, x1 + 1e-6, args.grid_step_m))
    ax.set_yticks(np.arange(y0, y1 + 1e-6, args.grid_step_m))
    ax.grid(True, color="cyan", alpha=0.3, linewidth=0.6)

    # eje secundario arriba/derecha en pixeles
    def x_world2px(x):
        return (np.asarray(x) - ox) / res

    def x_px2world(c):
        return ox + np.asarray(c) * res

    def y_world2px(y):
        return h_px - 1 - (np.asarray(y) - oy) / res

    def y_px2world(r):
        return oy + (h_px - 1 - np.asarray(r)) * res

    secax_x = ax.secondary_xaxis("top", functions=(x_world2px, x_px2world))
    secax_x.set_xlabel("columna [px]")
    secax_y = ax.secondary_yaxis("right", functions=(y_world2px, y_px2world))
    secax_y.set_ylabel("fila [px]")

    # coordenadas duales en la barra de herramientas al mover el raton
    def format_coord(x, y):
        col, row = world_to_pixel(x, y)
        return f"world=({x:6.2f}, {y:6.2f}) m   |   pixel=(col={col:6.1f}, row={row:6.1f})"

    ax.format_coord = format_coord

    ax.set_title(f"Mapa {w_px}x{h_px} px   |   origin=({ox}, {oy})   resolution={res} m/px")

    marked = []

    def on_click(event):
        if event.inaxes != ax or event.xdata is None:
            return
        if event.button == 3:  # click derecho: descartar
            marked.clear()
            print("Punto descartado.")
            return
        if event.button != 1:
            return
        x, y = event.xdata, event.ydata
        col, row = world_to_pixel(x, y)
        marked.append((x, y))
        ax.plot(x, y, "r+", markersize=12, markeredgewidth=2)
        fig.canvas.draw_idle()
        print(f"Punto {len(marked)}: world=({x:.2f}, {y:.2f}) m   pixel=(col={col:.0f}, row={row:.0f})")

        if len(marked) == 2:
            (x0_, y0_), (x1_, y1_) = marked
            xmin, xmax = sorted([x0_, x1_])
            ymin, ymax = sorted([y0_, y1_])
            col0, row0 = world_to_pixel(xmin, ymax)
            col1, row1 = world_to_pixel(xmax, ymin)
            print()
            print(f"  bbox world (xmin, xmax, ymin, ymax) = [{xmin:.2f}, {xmax:.2f}, {ymin:.2f}, {ymax:.2f}]")
            print(f"  bbox pixel (col_min, col_max, row_min, row_max) = [{col0:.0f}, {col1:.0f}, {row0:.0f}, {row1:.0f}]")
            print()
            marked.clear()

    fig.canvas.mpl_connect("button_press_event", on_click)

    print(__doc__)
    plt.show()


if __name__ == "__main__":
    main()
