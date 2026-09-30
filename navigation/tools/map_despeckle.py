#!/usr/bin/env python3
"""Remove isolated speckle from the 2D navigation grid (map.pgm).

Conservative by design: a cluster is turned free only when it is tiny
(<= --max-cells), and a moat of verified-free cells fully surrounds it, so
nothing connected to walls or touching unknown space is ever modified. The
3D prior map (map.pcd) used by ICP localization is not touched.

Modes: default is a read-only preview (preview PNG + audit JSON, no map
change); --apply additionally backs up map.pgm and writes the cleaned grid.
"""
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

ROOT = Path(__file__).resolve().parents[1]
MAPDIR = ROOT / 'maps/floor_1789552084236'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--max-cells', type=int, default=4,
                   help='Cluster size ceiling in cells (4 cells = 20 cm at 5 cm resolution)')
    p.add_argument('--moat', type=int, default=3,
                   help='Free-cell moat width around a cluster (3 cells = 15 cm)')
    p.add_argument('--apply', action='store_true', help='Write the cleaned map (default: preview only)')
    a = p.parse_args()

    cfg = yaml.safe_load((MAPDIR / 'map.yaml').read_text())
    img_path = MAPDIR / cfg['image']
    raw = np.array(Image.open(img_path))
    grid = np.flipud(raw)
    occ = grid == 0
    free = grid == 254

    struct8 = np.ones((3, 3), dtype=bool)
    labels, n = ndimage.label(occ, structure=struct8)
    sizes = ndimage.sum(occ, labels, range(1, n + 1))

    removed_mask = np.zeros_like(occ)
    removed = []
    for idx in range(1, n + 1):
        if sizes[idx - 1] > a.max_cells:
            continue
        cluster = labels == idx
        moat_ring = ndimage.binary_dilation(cluster, structure=struct8, iterations=a.moat) & ~cluster
        if not moat_ring.any():  # degenerate: cluster would cover the whole map
            continue
        # Isolation test: no other occupied cell within the moat. The ring may
        # contain free or unknown cells; unknown adjacency is fine because the
        # planner never routes through unknown space anyway.
        if not occ[moat_ring].any():
            removed_mask |= cluster
            ys, xs = np.where(cluster)
            wx = cfg['origin'][0] + xs * cfg['resolution']
            wy = cfg['origin'][1] + ys * cfg['resolution']
            removed.append({'size_cells': int(sizes[idx - 1]),
                            'x': round(float(wx.mean()), 2), 'y': round(float(wy.mean()), 2)})

    stamp = time.strftime('%Y%m%d_%H%M%S')
    audit_path = ROOT / 'log' / f'map_despeckle_{stamp}.json'
    sha = lambda b: hashlib.sha256(b).hexdigest()[:16]
    audit = {'mode': 'apply' if a.apply else 'preview',
             'map': str(img_path), 'max_cells': a.max_cells, 'moat_cells': a.moat,
             'occupied_before': int(occ.sum()), 'occupied_after': int((occ & ~removed_mask).sum()),
             'removed_clusters': len(removed),
             'removed_cells': int(removed_mask.sum()),
             'sha_before': sha((MAPDIR / cfg['image']).read_bytes()),
             'clusters': removed}

    preview = np.flipud(np.where(removed_mask, 160, grid)).astype(np.uint8)
    Image.fromarray(preview).save(ROOT / 'log' / f'map_despeckle_{stamp}_preview.png')

    if a.apply and removed_mask.any():
        backup = MAPDIR / (cfg['image'] + f'.bak_{stamp}')
        backup.write_bytes((MAPDIR / cfg['image']).read_bytes())
        cleaned = np.where(removed_mask, 254, grid).astype(np.uint8)
        Image.fromarray(np.flipud(cleaned)).save(img_path)
        audit['sha_after'] = sha(img_path.read_bytes())
        audit['backup'] = str(backup)
        print(f"APPLIED: {len(removed)} clusters ({int(removed_mask.sum())} cells) -> free. "
              f"Backup: {backup}")
    elif a.apply:
        print('Nothing matched the criteria; map unchanged.')
    else:
        print(f"PREVIEW ONLY: would remove {len(removed)} clusters "
              f"({int(removed_mask.sum())} cells). Map NOT modified.")
    audit_path.write_text(json.dumps(audit, indent=1))
    print(f"Audit: {audit_path}")
    print(f"Preview: {ROOT / 'log' / f'map_despeckle_{stamp}_preview.png'}")


if __name__ == '__main__':
    main()
