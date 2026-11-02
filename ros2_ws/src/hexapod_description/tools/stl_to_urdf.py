#!/usr/bin/env python3
"""
stl_to_urdf.py

Builds hexapod_description from a SolidWorks STL export of
Hexapod_full_assembly (one STL per component, "Do not translate STL
output data to positive space" ticked, millimetres).

    python3 tools/stl_to_urdf.py <stl_folder> [<package_folder>]

What it does
  1. Loads every component; merges the sub-parts of the electronics.
  2. Splits the parts into rigid links by contact: a servo and its horn
     (or holder) meet at a joint, every other touching pair is bolted.
  3. Measures each joint axis from its servo horn.
  4. Re-aligns the CAD so that +x = front (the ESP-EYE side), +z = up,
     with the body plate level and the legs left/right symmetric.
  5. Writes one STL per link and colour into meshes/, and
     urdf/hexapod.urdf.xacro with the measured joint positions.

The CAD pose is taken as the firmware's all-90 stance (every joint 0).
Needs numpy and scipy.
"""

import glob
import os
import re
import struct
import sys
from collections import defaultdict

import numpy as np
from scipy.spatial import cKDTree

STL_DIR = sys.argv[1]
PKG = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREFIX = 'Hexapod_full_assembly_stl - '


# ------------------------------------------------------------------ STL io

def load_stl(path):
    b = open(path, 'rb').read()
    if len(b) >= 84:
        n = struct.unpack('<I', b[80:84])[0]
        if 84 + 50 * n == len(b):
            dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
            return np.frombuffer(b[84:], dtype=dt, count=n)['v'].astype(np.float64)
    v = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b)
    return np.array(v, dtype=np.float64).reshape(-1, 3, 3)


def save_stl(path, tri):
    tri = np.asarray(tri, dtype=np.float32)
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = nrm / np.where(ln == 0, 1, ln)
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    a = np.zeros(len(tri), dtype=dt)
    a['n'] = nrm
    a['v'] = tri
    with open(path, 'wb') as f:
        f.write(b'hexapod_description'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tri)))
        f.write(a.tobytes())


def decimate(tri, cell):
    """Vertex clustering: snap vertices to a grid, drop collapsed triangles."""
    v = tri.reshape(-1, 3)
    _, idx = np.unique(np.round(v / cell).astype(np.int64), axis=0, return_inverse=True)
    idx = idx.ravel()
    centres = np.zeros((idx.max() + 1, 3))
    np.add.at(centres, idx, v)
    centres /= np.bincount(idx)[:, None]
    f = idx.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return centres[f[np.sort(keep)]]


# ------------------------------------------------------------------ parts

parts = defaultdict(list)
for p in sorted(glob.glob(os.path.join(STL_DIR, '*.STL')) + glob.glob(os.path.join(STL_DIR, '*.stl'))):
    name = os.path.basename(p)[:-4].replace(PREFIX, '')
    t = load_stl(p)
    if len(t) == 0:
        continue
    key = name.split('.step')[0] if '.step' in name else name
    parts[key].append(t)
parts = {k: np.concatenate(v) for k, v in parts.items()}

seen = {}
for k in sorted(parts, key=lambda s: (len(s), s)):
    v = parts[k].reshape(-1, 3)
    sig = (len(v),) + tuple(np.round(np.r_[v.min(0), v.max(0), v.mean(0)], 2))
    if sig in seen:
        print(f'  duplicate {k} == {seen[sig]}, dropped')
        del parts[k]
    else:
        seen[sig] = k


def kind(k):
    return re.sub(r'-\d+$', '', k)


names = list(parts)
servos = [k for k in names if kind(k).startswith('Perfect_MG996r')]
horns = [k for k in names if kind(k) == 'horn']
holders = [k for k in names if kind(k) == 'holder']
print(f'{len(names)} parts, {len(servos)} servos, {len(horns)} horns')


def cloud(t):
    return np.vstack([t.reshape(-1, 3), t.mean(1),
                      (t[:, 0] + t[:, 1]) / 2, (t[:, 1] + t[:, 2]) / 2, (t[:, 0] + t[:, 2]) / 2])


trees = {k: cKDTree(cloud(t)) for k, t in parts.items()}
lo = {k: t.reshape(-1, 3).min(0) for k, t in parts.items()}
hi = {k: t.reshape(-1, 3).max(0) for k, t in parts.items()}


def dist(a, b):
    if np.any(lo[a] > hi[b] + 5) or np.any(lo[b] > hi[a] + 5):
        return 1e9
    d, _ = trees[a].query(cloud(parts[b]), k=1, distance_upper_bound=5)
    return d.min()


horn_of = {s: min(horns, key=lambda h: dist(s, h)) for s in servos}
holder_of = {s: min(holders, key=lambda h: dist(s, h)) for s in servos} if holders else {}
assert len(set(horn_of.values())) == len(servos), 'could not pair every servo with a horn'

# rigid bodies = connected parts, cutting servo-horn and servo-bracket contacts
TOUCH = 1.05
nodes = [k for k in names if k not in holders]
adj = {k: set() for k in nodes}
for i, a in enumerate(nodes):
    for b in nodes[i + 1:]:
        if horn_of.get(a) == b or horn_of.get(b) == a:
            continue
        if (a in servos and kind(b) == 'bracket') or (b in servos and kind(a) == 'bracket'):
            continue
        if dist(a, b) < TOUCH:
            adj[a].add(b)
            adj[b].add(a)
comp = {}
for k in nodes:
    if k in comp:
        continue
    stack, c = [k], len(set(comp.values()))
    comp[k] = c
    while stack:
        x = stack.pop()
        for y in adj[x]:
            if y not in comp:
                comp[y] = c
                stack.append(y)
for s, h in holder_of.items():
    comp[h] = comp[horn_of[s]]          # a holder is the far-side pivot of the horn's link

lidar = next(k for k in names if k.startswith('RPLIDAR'))
camera = next((k for k in names if k.startswith('ESP-EYE')), None)
base_c = comp[lidar]


# ------------------------------------------------------------------ frames

def horn_axis(h, R):
    v = parts[h].reshape(-1, 3)
    c = v.mean(0)
    w, V = np.linalg.eigh(np.cov((v - c).T))
    proj = v @ V
    ctr = V @ ((proj.min(0) + proj.max(0)) / 2)
    return R @ V[:, 0], R @ ctr


def rot_to(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), np.dot(a, b)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# coarse: front = towards the camera, up = away from the feet
allv = np.vstack([t.reshape(-1, 3) for t in parts.values()])
centre_cad = (allv.min(0) + allv.max(0)) / 2
# up = from the feet towards the lidar (the lidar sits on top)
ll = np.vstack([parts[k].reshape(-1, 3) for k in names if kind(k) == 'long_leg'])
ld = parts[lidar].reshape(-1, 3)
up_vec = (ld.min(0) + ld.max(0)) / 2 - (ll.min(0) + ll.max(0)) / 2
up_axis = int(np.argmax(np.abs(up_vec)))
up_cad = np.sign(up_vec)
cam_c = parts[camera].reshape(-1, 3).mean(0) if camera else centre_cad + np.eye(3)[(up_axis + 1) % 3]
fwd = cam_c - centre_cad
fwd[up_axis] = 0
fax = np.argmax(np.abs(fwd))
X = np.zeros(3); X[fax] = np.sign(fwd[fax])
Z = np.zeros(3); Z[up_axis] = up_cad[up_axis]
Y = np.cross(Z, X)
R = np.vstack([X, Y, Z])                       # CAD -> body (coarse)

# level the body plate
plate = max((k for k in names if comp.get(k) == base_c and 'base' in k.lower()),
            key=lambda k: np.ptp(parts[k].reshape(-1, 3), axis=0).prod(), default=None)
if plate:
    t = parts[plate]
    nr = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ar = np.linalg.norm(nr, axis=1)
    nr = (R @ (nr / np.maximum(ar, 1e-12)[:, None]).T).T
    m = np.abs(nr[:, 2]) > 0.9
    n = (nr[m] * np.sign(nr[m][:, 2])[:, None] * ar[m][:, None]).sum(0)
    R = rot_to(n, np.array([0, 0, 1.])) @ R
    print(f'  body plate tilted {np.degrees(np.arccos(n[2] / np.linalg.norm(n))):.2f} deg in CAD, levelled')

coxa_servos = [s for s in servos if comp[s] == base_c]
assert len(coxa_servos) == 6, f'expected 6 coxa servos on the body, found {len(coxa_servos)}'

# yaw: make the coxa mounts mirror-symmetric about the x axis
cp = np.array([horn_axis(horn_of[s], R)[1] for s in coxa_servos])
c0 = cp.mean(0)
ang = np.arctan2(cp[:, 1] - c0[1], cp[:, 0] - c0[0])
def mirror_error(d):
    # mirror pairs about the x axis satisfy (a - d) = -(b - d)
    wrap = lambda x: (x + np.pi) % (2 * np.pi) - np.pi
    return sum(min(abs(wrap(a + b - 2 * d)) for b in ang) for a in ang)


best = min(np.radians(np.arange(-15, 15.001, 0.02)), key=mirror_error)
R = rz(-best) @ R
print(f'  body yawed {np.degrees(best):.2f} deg in CAD, straightened')


def to_body(p):
    return (R @ np.asarray(p).reshape(-1, 3).T).T


# ------------------------------------------------------------------ legs

legs = []
for cs in coxa_servos:
    coxa_c = comp[horn_of[cs]]
    fs = [s for s in servos if comp[horn_of[s]] == coxa_c and s != cs]
    assert len(fs) == 1, (cs, fs)
    femur_c = comp[fs[0]]
    ts = [s for s in servos if comp[s] == femur_c and s != fs[0]]
    assert len(ts) == 1, (cs, ts)
    legs.append(dict(servo=(cs, fs[0], ts[0]),
                     comp=dict(coxa=coxa_c, femur=femur_c, tibia=comp[horn_of[ts[0]]])))

pts = {}
for L in legs:
    _, cpt = horn_axis(horn_of[L['servo'][0]], R)
    fa, fpt = horn_axis(horn_of[L['servo'][1]], R)
    ta, tpt = horn_axis(horn_of[L['servo'][2]], R)
    L['raw'] = (cpt, fa, fpt, ta, tpt)
origin = np.r_[np.mean([L['raw'][0][:2] for L in legs], axis=0),
               np.mean([L['raw'][2][2] for L in legs])]


def line_plane(p, a, n, q):
    return p + np.dot(q - p, n) / np.dot(a, n) * a


for L in legs:
    cpt, fa, fpt, ta, tpt = L['raw']
    u = np.cross([0, 0, 1.], fa)
    u[2] = 0
    u /= np.linalg.norm(u)
    if np.dot(u[:2], cpt[:2] - origin[:2]) < 0:
        u = -u
    n = np.cross([0, 0, 1.], u)
    Oc = np.r_[cpt[:2], fpt[2]]                          # coxa axis is vertical in the body frame
    Of = line_plane(fpt, fa, n, Oc)
    Ot = line_plane(tpt, ta, n, Oc)
    Rl = np.c_[u, n, [0, 0, 1.]]
    tib = to_body(np.vstack([parts[k].reshape(-1, 3) for k in names if comp.get(k) == L['comp']['tibia']]))
    foot = tib[tib[:, 2].argmin()]
    rel = Oc - origin
    pos = 1 if abs(rel[0]) < 0.25 * np.abs([l['raw'][0][0] - origin[0] for l in legs]).max() else (0 if rel[0] > 0 else 2)
    num = pos + (3 if rel[1] > 0 else 0)
    L.update(n=num, Rl=Rl, Oc=Oc, Of=Of, Ot=Ot, foot=foot, yaw=np.arctan2(u[1], u[0]))
legs.sort(key=lambda L: L['n'])
assert [L['n'] for L in legs] == list(range(6)), 'leg numbering failed'

# ---- make the six legs identical (left/right mirror images) ----
# The CAD legs differ by a millimetre or two (loose mates), e.g. one foot
# sitting 2 mm higher. In simulation that one leg then carries a different
# load. Average the measurements over all six legs (mirroring the left side)
# so every leg has the same geometry; meshes stay where the CAD put them.
SYMMETRIC = True
if SYMMETRIC:
    def local(L):
        Rl = L['Rl']
        return (Rl.T @ (L['Of'] - L['Oc']), Rl.T @ (L['Ot'] - L['Of']), Rl.T @ (L['foot'] - L['Ot']))
    mir = np.array([1, -1, 1])
    side = {L['n']: (1 if L['n'] < 3 else -1) for L in legs}
    parts_avg = [np.mean([local(L)[k] * (mir if side[L['n']] < 0 else 1) for L in legs], axis=0) for k in range(3)]
    z_avg = np.mean([L['Oc'][2] for L in legs])
    for k in range(3):                       # pairs: 0-3 front, 1-4 middle, 2-5 back
        R_, L_ = legs[k], legs[k + 3]
        x = (R_['Oc'][0] + L_['Oc'][0]) / 2
        y = (abs(R_['Oc'][1] - origin[1]) + abs(L_['Oc'][1] - origin[1])) / 2
        yaw = (abs(R_['yaw']) + abs(L_['yaw'])) / 2
        for leg, sgn in ((R_, -1), (L_, 1)):
            leg['yaw'] = sgn * yaw
            c, s_ = np.cos(leg['yaw']), np.sin(leg['yaw'])
            leg['Rl'] = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
            m = mir if sgn > 0 else np.ones(3)
            leg['Oc'] = np.array([x, origin[1] + sgn * y, z_avg])
            leg['Of'] = leg['Oc'] + leg['Rl'] @ (parts_avg[0] * m)
            leg['Ot'] = leg['Of'] + leg['Rl'] @ (parts_avg[1] * m)
            leg['foot'] = leg['Ot'] + leg['Rl'] @ (parts_avg[2] * m)
    print('  legs made identical (left/right mirrored average of the six CAD legs)')

ground = min(L['foot'][2] for L in legs)
stand_h = (origin[2] - ground) / 1000.0


# ------------------------------------------------------------------ links and meshes

def material(k):
    kk = kind(k).lower()
    if kk.startswith('perfect_mg996r'):
        return 'servo'
    if kk == 'horn':
        return 'horn'
    if kk.startswith('rplidar'):
        return 'lidar'
    if kk.startswith('raspberry') or kk.startswith('shield'):
        return 'board'
    if kk.startswith('pca9685'):
        return 'pca'
    if kk.startswith('esp-eye'):
        return 'camera'
    if 'base' in kk or kk in ('box', 'd', 'roof'):
        return 'frame'
    return 'print'


link_of, frame_of = {}, {'base_link': (origin, np.eye(3))}
for L in legs:
    for j, O in (('coxa', L['Oc']), ('femur', L['Of']), ('tibia', L['Ot'])):
        ln = f"leg{L['n']}_{j}_link"
        frame_of[ln] = (O, L['Rl'])
        for k in names:
            if comp.get(k) == L['comp'][j]:
                link_of[k] = ln
for k in names:
    link_of.setdefault(k, 'base_link')

DENS = {'print': 1.24 * 0.45, 'frame': 1.24 * 0.45}         # PLA g/cm3 x ~45 % solid
MASS = {'servo': 0.055, 'horn': 0.003, 'lidar': 0.110, 'pca': 0.009, 'camera': 0.010}
BOARD = {'raspberry': 0.046, 'shield': 0.025}


def part_mass(k, tri):
    m = material(k)
    if m in MASS:
        return MASS[m]
    if m == 'board':
        return next(v for kk, v in BOARD.items() if kind(k).lower().startswith(kk))
    vol = abs(np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()) / 6.0  # mm3
    return max(vol / 1000.0 * DENS[m] / 1000.0, 0.002)


mesh_dir = os.path.join(PKG, 'meshes')
os.makedirs(mesh_dir, exist_ok=True)
for f in glob.glob(os.path.join(mesh_dir, '*.stl')):
    os.remove(f)

links = defaultdict(lambda: dict(vis=defaultdict(list), m=0.0, mc=np.zeros(3), I=np.zeros((3, 3)),
                                 lo=np.full(3, 1e9), hi=np.full(3, -1e9), parts=[]))
for k in names:
    ln = link_of[k]
    O, Rl = frame_of[ln]
    t = to_body(parts[k]).reshape(-1, 3, 3)
    loc = ((t.reshape(-1, 3) - O) @ Rl).reshape(-1, 3, 3) / 1000.0     # metres, link frame
    L = links[ln]
    L['vis'][material(k)].append(loc)
    L['parts'].append(k)
    m = part_mass(k, parts[k])
    v = loc.reshape(-1, 3)
    plo, phi = v.min(0), v.max(0)
    c, s = (plo + phi) / 2, phi - plo
    Ib = m / 12 * np.diag([s[1]**2 + s[2]**2, s[0]**2 + s[2]**2, s[0]**2 + s[1]**2])
    L['parts_inertia'] = L.get('parts_inertia', []) + [(m, c, Ib)]
    L['m'] += m
    L['mc'] += m * c
    if material(k) not in ('lidar', 'camera'):
        L['lo'] = np.minimum(L['lo'], plo)
        L['hi'] = np.maximum(L['hi'], phi)

total_tris = 0
for ln, L in links.items():
    com = L['mc'] / L['m']
    I = np.zeros((3, 3))
    for m, c, Ib in L['parts_inertia']:
        d = c - com
        I += Ib + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    L['com'], L['I'] = com, I
    files = {}
    for mat, tris in L['vis'].items():
        tri = np.concatenate(tris)
        if len(tri) > 15000:
            cell = 0.0006
            while len(tri) > 15000 and cell < 0.004:
                tri = decimate(np.concatenate(tris), cell)
                cell *= 1.4
        total_tris += len(tri)
        fn = f'{ln.replace("_link", "")}_{mat}.stl'
        save_stl(os.path.join(mesh_dir, fn), tri)
        files[mat] = fn
    L['files'] = files
print(f'  wrote {sum(len(L["files"]) for L in links.values())} meshes, {total_tris} triangles')


# ------------------------------------------------------------------ sensors

lid = to_body(parts[lidar]) - origin
lidar_xy = (lid.min(0)[:2] + lid.max(0)[:2]) / 2
lidar_bottom, lidar_top = lid[:, 2].min(), lid[:, 2].max()
laser_z = lidar_bottom + 0.72 * (lidar_top - lidar_bottom)        # approx. C1 scan plane
if camera:
    cam = to_body(parts[camera]) - origin
    cam_xyz = np.r_[cam[:, 0].max(), (cam.min(0)[1:] + cam.max(0)[1:]) / 2]
else:
    cam_xyz = np.array([0.12, 0, 0.04]) * 1000
base_lo, base_hi = links['base_link']['lo'], links['base_link']['hi']


# ------------------------------------------------------------------ URDF

def f(x, nd=5):
    return ' '.join(f'{v:.{nd}f}'.rstrip('0').rstrip('.') if abs(v) > 1e-9 else '0' for v in np.atleast_1d(x))


COLORS = dict(print='0.95 0.50 0.10 1', frame='0.30 0.30 0.32 1', servo='0.08 0.08 0.08 1',
              horn='0.92 0.92 0.92 1', lidar='0.10 0.10 0.12 1', board='0.10 0.45 0.25 1',
              pca='0.20 0.30 0.70 1', camera='0.05 0.05 0.05 1')


FOOT_R = 0.008    # rubber-tip contact sphere at each foot


def link_xml(ln, foot=None):
    L = links[ln]
    out = [f'  <link name="{ln}">']
    size = L['hi'] - L['lo']
    ctr = (L['hi'] + L['lo']) / 2
    out.append('    <xacro:if value="${use_meshes}">')
    for mat, fn in L['files'].items():
        out.append(f'      <visual><geometry><mesh filename="package://hexapod_description/meshes/{fn}"/></geometry>'
                   f'<material name="{mat}"/></visual>')
    out.append('    </xacro:if>')
    out.append('    <xacro:unless value="${use_meshes}">')
    out.append(f'      <visual><origin xyz="{f(ctr)}"/><geometry><box size="{f(size)}"/></geometry>'
               f'<material name="{"frame" if ln == "base_link" else "print"}"/></visual>')
    out.append('    </xacro:unless>')
    if foot is None:
        out.append(f'    <collision><origin xyz="{f(ctr)}"/><geometry><box size="{f(size)}"/></geometry></collision>')
    else:
        # tibia: box stops short of the tip, a sphere is the only thing touching the floor
        lo = L['lo'].copy()
        lo[2] += 0.02
        out.append(f'    <collision><origin xyz="{f((L["hi"] + lo) / 2)}"/>'
                   f'<geometry><box size="{f(L["hi"] - lo)}"/></geometry></collision>')
        out.append(f'    <collision name="{ln.replace("_tibia_link", "")}_foot_collision">'
                   f'<origin xyz="{f(np.asarray(foot) + [0, 0, FOOT_R])}"/>'
                   f'<geometry><sphere radius="{FOOT_R}"/></geometry></collision>')
    I = L['I']
    out.append(f'    <inertial><origin xyz="{f(L["com"])}"/><mass value="{L["m"]:.4f}"/>'
               f'<inertia ixx="{I[0,0]:.3e}" ixy="{I[0,1]:.3e}" ixz="{I[0,2]:.3e}" '
               f'iyy="{I[1,1]:.3e}" iyz="{I[1,2]:.3e}" izz="{I[2,2]:.3e}"/></inertial>')
    out.append('  </link>')
    return '\n'.join(out)


def joint(name, parent, child, xyz, rpy, axis, extra=''):
    return (f'  <joint name="{name}" type="revolute">\n'
            f'    <parent link="{parent}"/><child link="{child}"/>\n'
            f'    <origin xyz="{f(xyz)}" rpy="{f(rpy)}"/>\n'
            f'    <axis xyz="{f(axis)}"/>\n'
            f'    <limit lower="${{-joint_limit}}" upper="${{joint_limit}}" effort="${{servo_effort}}" velocity="${{servo_speed}}"/>\n'
            f'  </joint>')


xml = []
w = xml.append
w('<?xml version="1.0"?>')
w('''<!--
  hexapod.urdf.xacro  -  GENERATED by tools/stl_to_urdf.py from the SolidWorks
  assembly (Hexapod_full_assembly). Re-run that script after changing the CAD
  instead of editing the numbers here.

  Leg numbers and joint signs match hexapod_gripper.ino:

              front (+x)
          0 =========== 3
          |             |
          1    base     4        +y is LEFT
          |             |
          2 =========== 5
              back

  joint = (firmware logical angle - 90 deg) in radians; 0 everywhere = stance (h)
    coxa  +  leg swings toward the front
    femur +  foot lifts up
    tibia +  foot curls in toward the body
  gripper_joint: 0 = open (150 deg), upper limit = closed (100 deg)

  Not in the CAD, so placed by hand:  gripper, imu_link, battery mass.
-->''')
w('<robot name="hexapod" xmlns:xacro="http://www.ros.org/wiki/xacro">')
w('''
  <xacro:arg name="use_meshes" default="true"/>
  <xacro:property name="use_meshes" value="$(arg use_meshes)"/>

  <!-- lidar mounting turn in radians (3.14159 if the scan shows up backwards) -->
  <xacro:arg name="laser_yaw" default="0.0"/>

  <!-- firmware clamps every joint to 25..155 deg = +-65 deg from neutral -->
  <xacro:property name="joint_limit"  value="${radians(65)}"/>
  <!-- servo torque limit, N m (MG996R ~1.1 at 6 V); Gazebo can raise it -->
  <xacro:arg name="servo_effort" default="1.2"/>
  <xacro:property name="servo_effort" value="$(arg servo_effort)"/>
  <xacro:property name="servo_speed"  value="5.0"/>   <!-- rad/s -->
''')
for mat, c in COLORS.items():
    w(f'  <material name="{mat}"><color rgba="{c}"/></material>')

w('\n  <!-- ============================== BODY ============================== -->\n')
w('  <link name="base_footprint"/>')
w(f'  <joint name="base_joint" type="fixed">\n    <parent link="base_footprint"/><child link="base_link"/>\n'
  f'    <origin xyz="0 0 {stand_h:.4f}"/>\n  </joint>')
w(link_xml('base_link'))

for L in legs:
    n = L['n']
    w(f'\n  <!-- ============================== LEG {n} ============================== -->\n')
    coxa_xyz = (L['Oc'] - origin) / 1000
    fem_xyz = L['Rl'].T @ (L['Of'] - L['Oc']) / 1000
    tib_xyz = L['Rl'].T @ (L['Ot'] - L['Of']) / 1000
    foot_xyz = L['Rl'].T @ (L['foot'] - L['Ot']) / 1000
    side = -np.sign(np.sin(L['yaw']))                 # + coxa moves the foot forward
    w(joint(f'leg{n}_coxa_joint', 'base_link', f'leg{n}_coxa_link', coxa_xyz, [0, 0, L['yaw']], [0, 0, side]))
    w(link_xml(f'leg{n}_coxa_link'))
    w(joint(f'leg{n}_femur_joint', f'leg{n}_coxa_link', f'leg{n}_femur_link', fem_xyz, [0, 0, 0], [0, -1, 0]))
    w(link_xml(f'leg{n}_femur_link'))
    w(joint(f'leg{n}_tibia_joint', f'leg{n}_femur_link', f'leg{n}_tibia_link', tib_xyz, [0, 0, 0], [0, 1, 0]))
    w(link_xml(f'leg{n}_tibia_link', foot=foot_xyz))
    w(f'  <link name="leg{n}_foot"/>')
    w(f'  <joint name="leg{n}_foot_joint" type="fixed">\n    <parent link="leg{n}_tibia_link"/><child link="leg{n}_foot"/>\n'
      f'    <origin xyz="{f(foot_xyz)}"/>\n  </joint>')

gx = base_hi[0] + 0.012
gz = base_lo[2] + 0.010
w(f'''
  <!-- ============================= SENSORS ============================= -->

  <!-- RPLIDAR C1, frame on the (approximate) scan plane -->
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="laser"/>
    <origin xyz="{f(np.r_[lidar_xy, laser_z] / 1000)}" rpy="0 0 $(arg laser_yaw)"/>
  </joint>

  <!-- ESP-EYE camera on the front, looking forward -->
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="{f(cam_xyz / 1000)}"/>
  </joint>
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/><child link="camera_optical_frame"/>
    <origin rpy="${{-pi/2}} 0 ${{-pi/2}}"/>
  </joint>

  <!-- MPU6050: NOT IN THE CAD - move this to where the board really is -->
  <link name="imu_link">
    <visual><geometry><box size="0.021 0.016 0.003"/></geometry><material name="board"/></visual>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/>
    <origin xyz="0 0 {base_lo[2]:.4f}"/>
  </joint>

  <!-- ============================= GRIPPER ============================= -->
  <!-- SG90 gripper: NOT IN THE CAD - simple placeholder at the front -->
  <xacro:property name="jaw_travel" value="${{radians(50)/2}}"/>

  <link name="gripper_base">
    <visual><geometry><box size="0.023 0.033 0.012"/></geometry><material name="servo"/></visual>
    <collision><geometry><box size="0.023 0.033 0.012"/></geometry></collision>
  </link>
  <joint name="gripper_mount" type="fixed">
    <parent link="base_link"/><child link="gripper_base"/>
    <origin xyz="{gx:.4f} 0 {gz:.4f}"/>
  </joint>

  <xacro:macro name="jaw" params="name">
    <link name="${{name}}">
      <visual><origin xyz="0.020 0 0"/><geometry><box size="0.040 0.004 0.012"/></geometry><material name="print"/></visual>
      <collision><origin xyz="0.020 0 0"/><geometry><box size="0.040 0.004 0.012"/></geometry></collision>
      <inertial><origin xyz="0.020 0 0"/><mass value="0.005"/>
        <inertia ixx="1e-7" ixy="0" ixz="0" iyy="7e-7" iyz="0" izz="7e-7"/></inertial>
    </link>
  </xacro:macro>
  <xacro:jaw name="gripper_left_jaw"/>
  <xacro:jaw name="gripper_right_jaw"/>

  <joint name="gripper_joint" type="revolute">
    <parent link="gripper_base"/><child link="gripper_left_jaw"/>
    <origin xyz="0.012 0.012 0" rpy="0 0 ${{jaw_travel}}"/>
    <axis xyz="0 0 -1"/>
    <limit lower="0" upper="${{jaw_travel}}" effort="0.18" velocity="5.0"/>
  </joint>
  <joint name="gripper_right_joint" type="revolute">
    <parent link="gripper_base"/><child link="gripper_right_jaw"/>
    <origin xyz="0.012 -0.012 0" rpy="0 0 ${{-jaw_travel}}"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="${{jaw_travel}}" effort="0.18" velocity="5.0"/>
    <mimic joint="gripper_joint" multiplier="1" offset="0"/>
  </joint>

</robot>''')

os.makedirs(os.path.join(PKG, 'urdf'), exist_ok=True)
with open(os.path.join(PKG, 'urdf', 'hexapod.urdf.xacro'), 'w') as fh:
    fh.write('\n'.join(xml) + '\n')

# ------------------------------------------------------------------ report
print(f'  stand height {stand_h * 1000:.1f} mm, total mass {sum(L["m"] for L in links.values()):.3f} kg (no battery)')
for L in legs:
    print(f"  leg{L['n']}  yaw {np.degrees(L['yaw']):7.2f}  coxa {np.linalg.norm((L['Of'] - L['Oc'])[:2]):5.1f}"
          f"  femur {np.linalg.norm(L['Ot'] - L['Of']):5.1f}  tibia {np.linalg.norm(L['foot'] - L['Ot']):5.1f} mm"
          f"  servos {', '.join(s.replace('Perfect_MG996r-', '#') for s in L['servo'])}")
print('  written', os.path.join(PKG, 'urdf', 'hexapod.urdf.xacro'))
