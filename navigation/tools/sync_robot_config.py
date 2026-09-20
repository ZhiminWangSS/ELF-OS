#!/usr/bin/env python3
"""Copy measured geometry to Nav2. Does not mark geometry as verified."""
from pathlib import Path
import math,yaml,json
ROOT=Path(__file__).resolve().parents[1]
robot=yaml.safe_load((ROOT/'config/robot.yaml').read_text());footprint=robot['footprint']
if len(footprint)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in footprint):raise SystemExit('Invalid footprint')
manifest=json.loads((ROOT/'maps/floor_1789552084236/manifest.json').read_text())
if [robot['obstacle_min_height'],robot['obstacle_max_height']]!=manifest['obstacle_height_band_m']:
    raise SystemExit('Height band changed: regenerate and review the navigation map first')
params=yaml.safe_load((ROOT/'config/nav2.yaml').read_text())
for key in ['local_costmap','global_costmap']:
    config=params[key][key]['ros__parameters'];config['footprint']=json.dumps(footprint)
    config['obstacle_layer']['cloud']['min_obstacle_height']=robot['obstacle_min_height']
    config['obstacle_layer']['cloud']['max_obstacle_height']=robot['obstacle_max_height']
(ROOT/'config/nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
print('Nav2 footprint synchronized; geometry verification status:',robot['verified'])
