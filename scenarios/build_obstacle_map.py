#!/usr/bin/env python3
"""Deriva Sakhir con cinco obstáculos de ocupación, conservando el mapa original."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'results/Sakhir_map.yaml')
    parser.add_argument('--config', type=Path, default=ROOT / 'scenarios/static_obstacles.yaml')
    parser.add_argument('--out', type=Path, default=ROOT / 'results/Sakhir_obstacles_5')
    args = parser.parse_args()
    source = args.source.expanduser().resolve()
    config = yaml.safe_load(args.config.read_text())
    meta = yaml.safe_load(source.read_text())
    source_image = (source.parent / meta['image']).resolve()
    original_sha = sha(source_image)
    if original_sha != config['source_image_sha256']:
        raise ValueError('El PNG no coincide con el Sakhir original del curso.')
    resolution = float(meta['resolution'])
    ox, oy, yaw = meta['origin']
    if yaw != 0 or meta.get('negate', 0) != 0:
        raise ValueError('Este escenario requiere origen sin rotación y negate=0.')
    image = np.asarray(Image.open(source_image).convert('L')).copy()
    height, width = image.shape
    rows, cols = np.indices(image.shape)
    xs = ox + (cols + .5) * resolution
    ys = oy + (height - rows - .5) * resolution
    changed = np.zeros(image.shape, dtype=bool)
    obstacles = config['obstacles']
    if len(obstacles) != 5:
        raise ValueError('La prueba estática requiere exactamente cinco obstáculos.')

    def pixel(x, y):
        return height - 1 - math.floor((y - oy) / resolution), math.floor((x - ox) / resolution)

    original_free = image >= 205
    original_components, _ = ndimage.label(original_free)
    start = pixel(*config['start_xy'])
    track_component = original_components[start]
    if not track_component:
        raise ValueError('La salida no está en espacio libre.')
    for obstacle in obstacles:
        x, y, radius = (float(obstacle[k]) for k in ('x', 'y', 'radius'))
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0:
            raise ValueError('Geometría de obstáculo inválida.')
        mask = (xs - x) ** 2 + (ys - y) ** 2 <= radius ** 2
        if not mask.any() or not np.all(original_free[mask]) or np.any(changed & mask):
            raise ValueError(f'{obstacle["id"]}: fuera de espacio libre, toca pared o solapa otro.')
        if not np.all(original_components[mask] == track_component):
            raise ValueError(f'{obstacle["id"]}: no está dentro del carril de salida.')
        changed |= mask
    result = image.copy()
    result[changed] = 0
    components, number = ndimage.label(changed)
    if number != 5:
        raise ValueError('Se esperaban cinco componentes ocupadas separadas.')
    # Incluye una aproximación circular conservadora del vehículo en el paso local.
    clearance = ndimage.distance_transform_edt(result >= 205) * resolution
    radius = float(config['vehicle_clearance_radius'])
    safe = clearance > radius
    local_checks = []
    roi_radius = math.ceil(3.5 / resolution)
    for obstacle in obstacles:
        x, y, heading = (float(obstacle[k]) for k in ('x', 'y', 'heading'))
        cr, cc = pixel(x, y)
        r0, r1 = max(0, cr - roi_radius), min(height, cr + roi_radius + 1)
        c0, c1 = max(0, cc - roi_radius), min(width, cc + roi_radius + 1)
        labels, _ = ndimage.label(safe[r0:r1, c0:c1])
        ends = [pixel(x + d * math.cos(heading), y + d * math.sin(heading)) for d in (-2., 2.)]
        a, b = [labels[r-r0, c-c0] for r, c in ends]
        if not (a > 0 and a == b):
            raise ValueError(f'{obstacle["id"]}: no queda paso local para radio {radius} m.')
        local_checks.append({'id': obstacle['id'], 'local_bypass_connected': True})
    stem = args.out.expanduser().resolve()
    targets = [Path(str(stem) + suffix) for suffix in ('.png', '.yaml', '.json')]
    if any(p.exists() for p in targets) or source_image in targets or source in targets:
        raise FileExistsError('Elige una salida nueva; no se sobrescribe ningún mapa.')
    stem.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(result).save(targets[0])
    out_meta = dict(meta, image=targets[0].name)
    targets[1].write_text(yaml.safe_dump(out_meta, sort_keys=False))
    report = {'source_yaml': str(source), 'source_png_sha256': original_sha,
              'output_png_sha256': sha(targets[0]), 'obstacles': obstacles,
              'changed_cells': int(changed.sum()), 'new_occupied_components': number,
              'resolution_unchanged': True, 'origin_unchanged': True,
              'original_unchanged': sha(source_image) == original_sha,
              'vehicle_clearance_radius_m': radius, 'local_bypass_checks': local_checks,
              'scope': 'Geometría y paso local. No acredita diez vueltas sin colisión.'}
    targets[2].write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'map': str(targets[1]), 'obstacles': number,
                      'changed_cells': int(changed.sum()), 'local_bypasses': len(local_checks)}, indent=2))


if __name__ == '__main__':
    main()
