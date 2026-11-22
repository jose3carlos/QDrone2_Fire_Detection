"""Printability check for STL files.

Usage: python check_print.py part1.stl [part2.stl ...]
Reports watertightness, bounding box (mm), volume, approximate mass in PLA,
and the area of downward-facing surfaces steeper than the overhang limit that
are not resting on the print bed (z = min z). STL files must already be in
their PRINT orientation.
"""
import sys
import numpy as np
import trimesh

OVERHANG_DEG = 50.0      # measured from vertical; >50 deg needs support
BED_TOL = 0.25           # mm, faces this close to z_min count as on-bed
PLA_DENSITY = 1.24e-3    # g/mm^3
K2_PLUS_BED = (350, 350, 350)


def check(path):
    m = trimesh.load(path, force="mesh")
    ext = m.bounds[1] - m.bounds[0]
    zmin = m.bounds[0][2]
    n = m.face_normals
    a = m.area_faces
    cos_lim = np.cos(np.radians(90.0 - OVERHANG_DEG + 90.0 - 90.0))
    # face points down more steeply than the limit: normal z < -sin(90-limit)
    down = n[:, 2] < -np.cos(np.radians(OVERHANG_DEG))
    on_bed = m.triangles[:, :, 2].max(axis=1) <= zmin + BED_TOL
    overhang = down & ~on_bed
    oh_area = float(a[overhang].sum())
    # bridges: perfectly flat down faces above the bed (often fine if short)
    flat = n[:, 2] < -0.999
    bridge_area = float(a[flat & ~on_bed].sum())
    fits = all(e <= b for e, b in zip(sorted(ext), sorted(K2_PLUS_BED)))
    vol = float(m.volume) if m.is_watertight else float("nan")
    print(f"== {path}")
    print(f"   watertight={m.is_watertight} bodies={len(m.split(only_watertight=False))}")
    print(f"   bbox mm = {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f}  fits K2 Plus={fits}")
    print(f"   volume = {vol/1000:.1f} cm3  ~{vol*PLA_DENSITY:.0f} g PLA (solid)")
    print(f"   bed contact area = {float(a[on_bed & (n[:,2] < -0.999)].sum()):.0f} mm2")
    print(f"   overhang >{OVERHANG_DEG:.0f}deg (not on bed) = {oh_area:.0f} mm2, of which flat bridges = {bridge_area:.0f} mm2")
    return dict(watertight=m.is_watertight, overhang=oh_area, bridge=bridge_area, ext=ext)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        check(p)
