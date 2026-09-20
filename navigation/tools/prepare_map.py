#!/usr/bin/env python3
"""Level a single-floor session and conservatively project observed lidar rays.

Original session is immutable. Output coordinates are defined by map_from_saved.
This creates a candidate map, not certification of floor coverage/traversability.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
from pcd import read_pcd, write_pcd


def level_rotation(normal):
    normal = normal / np.linalg.norm(normal)
    x = np.array([1., 0., 0.]); x -= normal * (x @ normal); x /= np.linalg.norm(x)
    return np.stack([x, np.cross(normal, x), normal])


def fit_floor(points, trajectory):
    _, _, v = np.linalg.svd(trajectory - trajectory.mean(0), full_matrices=False)
    normal = v[-1] * np.sign(v[-1, 2])
    height = points @ normal
    sensor_height = np.median(trajectory @ normal)
    bins = np.arange(sensor_height - 1.2, sensor_height - .15, .025)
    counts, edges = np.histogram(height, bins=bins)
    peak = edges[np.argmax(counts)] + .0125
    selected = np.abs(height - peak) < .12
    # Robustly refine the dominant below-sensor plane; reject multi-floor or warped maps.
    for _ in range(5):
        cloud = points[selected]; center = np.median(cloud, axis=0)
        _, _, v = np.linalg.svd(cloud - center, full_matrices=False)
        normal = v[-1] * np.sign(v[-1, 2]); offset = np.median(cloud @ normal)
        residual = points @ normal - offset
        selected = (np.abs(residual) < .10) & (np.abs(height - peak) < .2)
    floor_residual = points[selected] @ normal - offset
    clearance = trajectory @ normal - offset
    if selected.sum() < 1000 or np.quantile(np.abs(floor_residual), .95) > .10:
        raise ValueError('No well-supported floor plane')
    if np.quantile(clearance, .05) < .15 or np.quantile(clearance, .95) > 1.0:
        raise ValueError('Trajectory height inconsistent with a single floor; inspect source map')
    return normal, float(offset), {'floor_support_points': int(selected.sum()),
        'floor_residual_p95_m': float(np.quantile(np.abs(floor_residual), .95)),
        'sensor_height_quantiles_m': np.quantile(clearance, [0,.05,.5,.95,1]).tolist()}


def ray_cells(start, end):
    """Supercover-ish sampling at < half a cell; endpoints included."""
    steps = int(np.ceil(np.max(np.abs(end-start))*2)) + 1
    return np.floor(np.linspace(start, end, steps)).astype(int)


def prepare(session, output, resolution=.05, obstacle_min=.15, obstacle_max=1.2):
    if output.exists(): raise ValueError('Output exists; choose a new directory to preserve prior artifacts')
    cloud = read_pcd(session/'map.pcd')
    rows = np.genfromtxt(session/'trajectory.csv', delimiter=',', names=True)
    trajectory = np.stack([rows[k] for k in ('x','y','z')], axis=1)
    normal, offset, quality = fit_floor(cloud[:,:3], trajectory)
    rotation = level_rotation(normal); transform = np.eye(4)
    transform[:3,:3] = rotation; transform[2,3] = -offset
    leveled = cloud.copy(); leveled[:,:3] = cloud[:,:3] @ rotation.T + transform[:3,3]
    path = trajectory @ rotation.T + transform[:3,3]
    origin = np.floor((leveled[:,:2].min(0)-1)/resolution)*resolution
    size = np.ceil((leveled[:,:2].max(0)+1-origin)/resolution).astype(int)+1
    if np.prod(size) > 25000000: raise ValueError('Map bounds too large')
    grid = np.full((size[1],size[0]),205,dtype=np.uint8)
    selected = (leveled[:,2]>=obstacle_min)&(leveled[:,2]<=obstacle_max)
    cells = np.floor((leveled[selected,:2]-origin)/resolution).astype(int)
    occupied = np.zeros_like(grid,dtype=bool); occupied[cells[:,1],cells[:,0]]=True
    # Carve only measured rays from saved body-frame keyframes, never the whole bounding box.
    rays = 0
    for i,row in enumerate(rows):
        points = read_pcd(session/'keyframes'/f'{int(row["id"])}.pcd')[:,:3]
        q = [row[k] for k in ('qx','qy','qz','qw')]
        world = (points @ Rotation.from_quat(q).as_dcm().T + trajectory[i]) @ rotation.T + transform[:3,3]
        delta = world[:,:2]-path[i,:2]; distance = np.linalg.norm(delta,axis=1)
        valid = (world[:,2]>=obstacle_min)&(world[:,2]<=obstacle_max)&(distance>.3)&(distance<=8.)
        if not obstacle_min <= path[i,2] <= obstacle_max: continue
        endpoints = world[valid,:2]; delta = delta[valid]; distance = distance[valid]
        angles = np.floor((np.arctan2(delta[:,1],delta[:,0])+np.pi)/(2*np.pi)*720).astype(int)
        order = np.argsort(distance); _, nearest = np.unique(angles[order],return_index=True)
        for end in endpoints[order[nearest]]:
            indices = ray_cells((path[i,:2]-origin)/resolution,(end-origin)/resolution)
            inside = (indices[:,0]>=0)&(indices[:,0]<size[0])&(indices[:,1]>=0)&(indices[:,1]<size[1])
            indices = indices[inside]
            # A prior-map obstacle always blocks clearing, including beyond the first hit.
            hits = np.flatnonzero(occupied[indices[:,1],indices[:,0]])
            if len(hits): indices = indices[:hits[0]]
            grid[indices[:,1],indices[:,0]]=254; rays+=1
    grid[occupied]=0
    output.mkdir(parents=True)
    write_pcd(output/'map.pcd',leveled)
    Image.fromarray(np.flipud(grid)).save(output/'map.pgm')
    (output/'map.yaml').write_text(f'image: map.pgm\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    preview = Image.fromarray(np.flipud(grid)).convert('RGB'); draw=ImageDraw.Draw(preview)
    pixels=(path[:,:2]-origin)/resolution; pixels[:,1]=grid.shape[0]-1-pixels[:,1]
    draw.line([tuple(p) for p in pixels],fill=(0,120,255),width=2)
    preview.save(output/'preview.png')
    np.savetxt(output/'trajectory_xyz.csv',path,delimiter=',',header='x,y,z',comments='')
    report = dict(source_session=str(session.resolve()), source_sha256=hashlib.sha256((session/'map.pcd').read_bytes()).hexdigest(),
        map_from_saved=transform.tolist(), frame_id='map', resolution=resolution, origin_xy=origin.tolist(),
        width=int(size[0]),height=int(size[1]),obstacle_height_band_m=[obstacle_min,obstacle_max],
        tilt_deg=float(np.degrees(np.arccos(normal[2]))), ray_count=rays,
        cells={str(v):int((grid==v).sum()) for v in [0,205,254]},quality=quality,
        readiness='candidate_only: inspect walls, doors, stairs, footprint and localization before motion')
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();prepare(a.session,a.output)
