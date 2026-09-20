"""Strict binary XYZ(I) PCD I/O, independent of ROS."""
from pathlib import Path
import numpy as np

def read_pcd(path):
    with Path(path).open('rb') as f:
        h = {}
        while True:
            line = f.readline()
            if not line: raise ValueError('Incomplete PCD header')
            parts = line.decode('ascii').strip().split()
            if not parts or parts[0].startswith('#'): continue
            h[parts[0]] = parts[1:]
            if parts[0] == 'DATA': break
        fields = h['FIELDS']
        if (h['DATA'] != ['binary'] or fields not in [['x','y','z'], ['x','y','z','intensity']]
                or h['SIZE'] != ['4'] * len(fields) or h['TYPE'] != ['F'] * len(fields)
                or h.get('COUNT', ['1'] * len(fields)) != ['1'] * len(fields)):
            raise ValueError('Expected uncompressed binary float32 XYZ or XYZI PCD')
        data = np.frombuffer(f.read(), dtype='<f4')
        if data.size != int(h['POINTS'][0]) * len(fields): raise ValueError('PCD size mismatch')
        points = data.reshape(-1, len(fields)).copy()
        if not len(points) or not np.isfinite(points).all(): raise ValueError('Empty or nonfinite cloud')
        return points

def write_pcd(path, points):
    fields = 'x y z' if points.shape[1] == 3 else 'x y z intensity'
    n, k = points.shape
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS {fields}\nSIZE {" ".join(["4"]*k)}\n'
              f'TYPE {" ".join(["F"]*k)}\nCOUNT {" ".join(["1"]*k)}\nWIDTH {n}\nHEIGHT 1\n'
              f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n')
    with Path(path).open('wb') as f:
        f.write(header.encode('ascii')); f.write(np.asarray(points, dtype='<f4').tobytes())
