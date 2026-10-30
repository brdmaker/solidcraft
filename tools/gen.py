# Generates the CNC build stages as MCP `batch` requests (stageX.json) plus expected numbers.
import json, math

M, Z, PA, BACKLASH = 2.0, 16, math.radians(20), 0.15
P = math.pi * M              # circular pitch 6.2832
R = M * Z / 2                # pitch radius 16
RA, RF = R + M, R - 1.25 * M  # 18, 13.5
TAN = math.tan(PA)

def box(x0, x1, y0, y1, z0, z1, op="join", body=None, targets=None, name=None):
    p = {"length": x1 - x0, "width": y1 - y0, "height": z1 - z0, "corner": [x0, y0, z0], "operation": op}
    if body: p["body_name"] = body
    if targets: p["targets"] = targets
    if name: p["name"] = name
    return {"command": "solid.box", "params": p}

def cyl(d, h, base, axis, op, targets=None, body=None, name=None):
    p = {"diameter": d, "height": h, "base": base, "axis": axis, "operation": op}
    if targets: p["targets"] = targets
    if body: p["body_name"] = body
    if name: p["name"] = name
    return {"command": "solid.cylinder", "params": p}

def r(v): return round(v, 4)

# ---- gear geometry -------------------------------------------------------
def inv(a): return math.tan(a) - a

def pinion_outline():
    """Closed involute outline, tooth k centred at 11.25 + k*22.5 deg (a gap at 0/90/180/270)."""
    rb = R * math.cos(PA)
    psi = (P / 2 - BACKLASH) / (2 * R)          # half tooth angle at the pitch circle
    def flank(rad):                              # angle of the flank from the tooth centre
        if rad <= rb: return psi + inv(PA)
        return psi + inv(PA) - inv(math.acos(rb / rad))
    radii = [RF, rb] + [rb + (RA - rb) * i / 7 for i in range(1, 8)]
    pts = []
    for k in range(Z):
        c = math.radians(11.25 + 22.5 * k)
        half_pitch = math.pi / Z
        # root arc from previous gap centre to this tooth's root
        pts.append((c - half_pitch, RF))
        for rad in radii:                        # leading flank, root -> tip
            pts.append((c - flank(rad), rad))
        for rad in reversed(radii):              # trailing flank, tip -> root
            pts.append((c + flank(rad), rad))
    out = []
    for a, rad in pts:
        q = [r(rad * math.cos(a)), r(rad * math.sin(a))]
        if not out or q != out[-1]: out.append(q)
    return out

def rack_outline(u0, u1, hb, centre=0.0, sign=1.0):
    """Rack along u in [u0,u1]; base 0..hb, teeth hb..hb+2.25m; a tooth centred at `centre`.
    `sign` maps height v to sketch y (the XZ plane has sketch y = -Z)."""
    t = (P / 2 - BACKLASH) / 2
    w_root, w_tip = t + 1.25 * M * TAN, t - M * TAN
    k0 = math.ceil((u0 + w_root - centre) / P); k1 = math.floor((u1 - w_root - centre) / P)
    pts = [(u0, 0), (u1, 0), (u1, hb)]
    for k in range(k1, k0 - 1, -1):
        c = centre + k * P
        pts += [(c + w_root, hb), (c + w_tip, hb + 2.25 * M), (c - w_tip, hb + 2.25 * M), (c - w_root, hb)]
    pts.append((u0, hb))
    return [[r(u), r(sign * v)] for u, v in pts], k1 - k0 + 1

def shoelace(pts):
    return abs(sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))) / 2

# ---- placements: canonical parts are built at offset T, then moved -------
def matrix(cols, t, T):
    """cols: images of local x,y,z; t: world position of the canonical origin; T: build offset."""
    Rm = [[cols[j][i] for j in range(3)] for i in range(3)]
    RT = [sum(Rm[i][j] * T[j] for j in range(3)) for i in range(3)]
    return [[Rm[i][0], Rm[i][1], Rm[i][2], r(t[i] - RT[i])] for i in range(3)]

T_MOTOR, T_PINION, T_SWITCH = [0, 0, -500], [200, 0, -500], [400, 0, -500]
X, Y, Zv = (1, 0, 0), (0, 1, 0), (0, 0, 1)
neg = lambda v: tuple(-c for c in v)
PLACE = {
    "motor": [  # flange face at local z=0, shaft along +z
        ("Y motor R", [Zv, Y, neg(X)], [276, 0, 30.5]),
        ("Y motor L", [neg(Zv), Y, X], [-276, 0, 30.5]),
        ("X motor",   [X, Zv, neg(Y)], [0, 30, 223.5]),
        ("Z motor",   [X, neg(Y), neg(Zv)], [0, -65, 308]),
    ],
    "pinion": [  # gear z 0..10 along its axis
        ("Y pinion R", [Zv, Y, neg(X)], [266, 0, 30.5]),
        ("Y pinion L", [neg(Zv), Y, X], [-266, 0, 30.5]),
        ("X pinion",   [X, Zv, neg(Y)], [0, 20, 223.5]),
    ],
    "switch": [  # body x -10..10, y -3.2..3.2, z 0..10.2, lever on +z
        ("Y limit", [neg(Y), X, Zv], [-198, 213, 18]),
        ("X limit", [Y, neg(Zv), neg(X)], [210, -10, 205.5]),
        ("Z limit", [X, neg(Y), neg(Zv)], [24, -46.5, 291]),
    ],
}

stages = {}
# ---- A: base frame -------------------------------------------------------
yr0, yr1 = -296, 250
rack_pts, n_yteeth = rack_outline(yr0, yr1, 12)
A = [
    box(205, 245, yr0, yr1, 0, 40, "new", body="Rail R"),
    box(204, 211, yr0 - 1, yr1 + 1, 26, 34, "cut", targets=["Rail R"], name="Rail R inner groove"),
    box(239, 246, yr0 - 1, yr1 + 1, 26, 34, "cut", targets=["Rail R"], name="Rail R outer groove"),
    {"command": "sketch.create", "params": {"plane": "YZ", "offset": 245, "name": "Rack R profile"}},
    {"command": "sketch.line", "params": {"points": rack_pts, "closed": True, "infer": False}},
    {"command": "sketch.finish", "params": {}},
    {"command": "solid.extrude", "params": {"distance": 20, "operation": "new", "body_name": "Rack R", "name": "Rack R"}},
    box(-205, 205, -296, -256, 0, 18, "new", body="Cross front"),
    box(-205, 205, 210, 250, 0, 18, "new", body="Cross back"),
    box(-200, -196, 210, 224, 13, 19, "cut", targets=["Cross back"], name="Y limit wire pocket"),
]
for i, (cx, cy) in enumerate([(-80, -166), (80, -166), (-80, -6), (80, -6)]):
    A.append(box(cx - 80, cx + 80, cy - 80, cy + 80, 0, 20, "new", body=f"Bed tile {i + 1}"))
A += [{"command": "solid.mirror", "params": {"bodies": ["Rail R", "Rack R"], "plane": "YZ", "name": "Base mirror"}}]
stages["A"] = A
rack_area = shoelace(rack_pts)
exp = {"rail": 40 * 40 * 546 - 2 * 6 * 8 * 546, "rack": rack_area * 20, "rack_teeth": n_yteeth,
       "cross": 410 * 40 * 18, "cross_back": 410 * 40 * 18 - 4 * 13 * 5, "tile": 160 * 160 * 20}

# ---- B: gantry -----------------------------------------------------------
xr_pts, n_xteeth = rack_outline(-210, 210, 5, sign=-1.0)
xr_pts = [[u, r(v - 200)] for u, v in xr_pts]   # sketch y = -z, base starts at z=200
B = [
    box(195, 255, -50, 50, 20, 55, "new", body="Shoe R"),
    box(204, 246, -51, 51, 19, 41, "cut", targets=["Shoe R"], name="Shoe R channel"),
    box(204, 210, -50, 50, 27, 33, name="Shoe R inner lip", targets=["Shoe R"]),
    box(240, 246, -50, 50, 27, 33, name="Shoe R outer lip", targets=["Shoe R"]),
    box(250, 276, -36, 36, 52, 62, name="Y motor bridge R", targets=["Shoe R"]),
    box(268, 276, -36, 36, 2, 62, name="Y motor plate R", targets=["Shoe R"]),
    cyl(38.5, 10, [267, 0, 30.5], [1, 0, 0], "cut", ["Shoe R"], name="Y motor pilot R"),
]
for dy in (-23.57, 23.57):
    for dz in (-23.57, 23.57):
        B.append(cyl(5.5, 10, [267, dy, 30.5 + dz], [1, 0, 0], "cut", ["Shoe R"]))
B += [
    box(210, 240, -20, 20, 55, 215, "new", body="Upright R"),
    box(209, 215, -19, -1, 203, 208, "cut", targets=["Upright R"], name="X limit wire pocket"),
    {"command": "solid.mirror", "params": {"bodies": ["Shoe R", "Upright R"], "plane": "YZ", "name": "Gantry mirror"}},
    box(-210, 210, -20, 20, 140, 200, "new", body="Beam"),
    box(-211, 211, -10, -2, 194, 201, "cut", targets=["Beam"], name="Beam top groove"),
    box(-211, 211, -10, -2, 139, 146, "cut", targets=["Beam"], name="Beam bottom groove"),
    {"command": "sketch.create", "params": {"plane": "XZ", "offset": 6, "name": "X rack profile"}},
    {"command": "sketch.line", "params": {"points": xr_pts, "closed": True, "infer": False}},
    {"command": "sketch.finish", "params": {}},
    {"command": "solid.extrude", "params": {"distance": 14, "operation": "new", "body_name": "X rack", "name": "X rack"}},
]
stages["B"] = B
shoe = 60 * 100 * 35 - 42 * 100 * 21 + 2 * 6 * 100 * 6 + 26 * 72 * 10 - 5 * 72 * 3 + 8 * 72 * 60 - 8 * 72 * 10 \
       - (math.pi * 19.25 ** 2 + 4 * math.pi * 2.75 ** 2) * 8
exp.update({"shoe": shoe, "upright": 30 * 40 * 160 - 5 * 18 * 5, "beam": 420 * 40 * 60 - 2 * 420 * 8 * 6,
            "xrack": shoelace(xr_pts) * 14, "xrack_teeth": n_xteeth})

# ---- C: X carriage, Z carriage, Z screw ------------------------------------
C = [
    box(-50, 50, -32, -21, 130, 308, "new", body="X carriage"),
    box(-50, 50, -21, 0, 201, 210, targets=["X carriage"], name="X top flange"),
    box(-50, 50, -9, -3, 195, 201, targets=["X carriage"], name="X top lip"),
    box(-50, 50, -21, 0, 130, 139, targets=["X carriage"], name="X bottom flange"),
    box(-50, 50, -9, -3, 139, 145, targets=["X carriage"], name="X bottom lip"),
    box(-35, 35, -21, -11, 210, 263, targets=["X carriage"], name="X motor tower"),
    box(-35, 35, -21, 30, 255, 263, targets=["X carriage"], name="X motor arm"),
    box(-35, 35, 22, 30, 195, 263, targets=["X carriage"], name="X motor plate"),
    cyl(38.5, 10, [0, 21, 223.5], [0, 1, 0], "cut", ["X carriage"], name="X motor pilot"),
]
for dx in (-23.57, 23.57):
    for dz in (-23.57, 23.57):
        C.append(cyl(5.5, 10, [dx, 21, 223.5 + dz], [0, 1, 0], "cut", ["X carriage"]))
C += [
    box(-50, -38, -40, -31.5, 122, 290, targets=["X carriage"], name="Z rail L"),
    box(38, 50, -40, -31.5, 122, 290, targets=["X carriage"], name="Z rail R"),
    box(45, 51, -38, -34, 121, 291, "cut", targets=["X carriage"], name="Z rail R groove"),
    box(-51, -45, -38, -34, 121, 291, "cut", targets=["X carriage"], name="Z rail L groove"),
    box(-40, 40, -96, -32, 300, 308, targets=["X carriage"], name="Z motor plate"),
    cyl(38.5, 10, [0, -65, 299], [0, 0, 1], "cut", ["X carriage"], name="Z motor pilot"),
]
for dx in (-23.57, 23.57):
    for dy in (-23.57, 23.57):
        C.append(cyl(5.5, 10, [dx, -65 + dy, 299], [0, 0, 1], "cut", ["X carriage"]))
C += [
    box(12, 36, -51, -42, 291, 300.5, targets=["X carriage"], name="Z limit block"),
    box(15, 33, -48, -45, 290, 295, "cut", targets=["X carriage"], name="Z limit wire pocket"),
    # Z carriage, modelled at the bottom of its travel
    box(-56, 56, -52, -41, 50, 180, "new", body="Z carriage"),
    box(51, 56, -41, -33, 50, 180, targets=["Z carriage"], name="Z skirt R"),
    box(-56, -51, -41, -33, 50, 180, targets=["Z carriage"], name="Z skirt L"),
    box(46, 51, -37.5, -34.5, 50, 180, targets=["Z carriage"], name="Z lip R"),
    box(-51, -46, -37.5, -34.5, 50, 180, targets=["Z carriage"], name="Z lip L"),
    box(-15, 15, -80, -52, 155, 180, targets=["Z carriage"], name="Z nut block"),
    cyl(16, 37, [0, -65, 149], [0, 0, 1], "cut", ["Z carriage"], name="Z nut bore"),
    box(-20, 20, -60, -52, 50, 90, targets=["Z carriage"], name="Holder web"),
    cyl(72, 40, [0, -86, 50], [0, 0, 1], "join", ["Z carriage"], name="Tool holder ring"),
    cyl(52, 42, [0, -86, 49], [0, 0, 1], "cut", ["Z carriage"], name="Tool holder bore"),
    box(-2, 2, -125, -110, 49, 91, "cut", targets=["Z carriage"], name="Tool holder clamp slot"),
    # Z lead screw with its motor coupler, one printed part
    cyl(16, 133, [0, -65, 150], [0, 0, 1], "new", body="Z screw", name="Z screw"),
    cyl(22, 16, [0, -65, 283], [0, 0, 1], "join", ["Z screw"], name="Z coupler"),
    cyl(6.35, 13, [0, -65, 287], [0, 0, 1], "cut", ["Z screw"], name="Z coupler bore"),
]
stages["C"] = C

# ---- D: purchased parts and the pinion, built at their offsets --------------
mx, my, mz = T_MOTOR
D = [
    box(mx - 28.5, mx + 28.5, my - 28.5, my + 28.5, mz - 76, mz, "new", body="NEMA 23", name="Motor body"),
]
for sx in (-1, 1):
    for sy in (-1, 1):
        D.append({"command": "solid.chamfer", "params": {"edges": [[mx + 28.5 * sx, my + 28.5 * sy, mz - 38]], "distance": 4}})
D += [cyl(38.1, 1.6, [mx, my, mz], [0, 0, 1], "join", ["NEMA 23"], name="Motor pilot boss"),
      cyl(6.35, 20.6, [mx, my, mz], [0, 0, 1], "join", ["NEMA 23"], name="Motor shaft")]
for dx in (-23.57, 23.57):
    for dy in (-23.57, 23.57):
        D.append(cyl(4.5, 6, [mx + dx, my + dy, mz - 5], [0, 0, 1], "cut", ["NEMA 23"]))
px, py, pz = T_PINION
gear = [[r(x + px), r(y + py)] for x, y in pinion_outline()]
D += [
    {"command": "sketch.create", "params": {"plane": "XY", "offset": pz, "name": "Pinion profile"}},
    {"command": "sketch.line", "params": {"points": gear, "closed": True, "infer": False}},
    {"command": "sketch.circle.center", "params": {"center": [px, py], "radius": 6.35 / 2}},
    {"command": "sketch.finish", "params": {}},
    {"command": "solid.extrude", "params": {"distance": 10, "operation": "new", "body_name": "Pinion", "name": "Pinion"}},
]
sx_, sy_, sz_ = T_SWITCH
D += [
    box(sx_ - 10, sx_ + 10, sy_ - 3.2, sy_ + 3.2, sz_, sz_ + 10.2, "new", body="Limit switch", name="Switch body"),
    box(sx_ - 3, sx_ - 1, sy_ - 1.5, sy_ + 1.5, sz_ + 10.2, sz_ + 11, targets=["Limit switch"], name="Switch plunger"),
    box(sx_ - 9, sx_ - 7, sy_ - 2, sy_ + 2, sz_ + 10.2, sz_ + 12, targets=["Limit switch"], name="Lever pivot"),
    box(sx_ - 9, sx_ + 14, sy_ - 2, sy_ + 2, sz_ + 11, sz_ + 12, targets=["Limit switch"], name="Switch lever"),
]
for tx in (-7.6, 0, 7.6):
    D.append(box(sx_ + tx - 1.6, sx_ + tx + 1.6, sy_ - 0.25, sy_ + 0.25, sz_ - 4, sz_, targets=["Limit switch"]))
for tx in (-4.75, 4.75):
    D.append(cyl(2.4, 8, [sx_ + tx, sy_ - 4, sz_ + 5.1], [0, 1, 0], "cut", ["Limit switch"]))
stages["D"] = D
gear_area = shoelace(gear) - math.pi * (6.35 / 2) ** 2
exp["pinion"] = gear_area * 10
exp["switch"] = 20 * 6.4 * 10.2 + 2 * 3 * 0.8 + (2 * 4 * 1.8 - 2 * 4 * 1) + 23 * 4 * 1 + 3 * 3.2 * 0.5 * 4 - 2 * math.pi * 1.2 ** 2 * 6.4
exp["motor"] = 57 * 57 * 76 - 4 * 8 * 76 + math.pi * 19.05 ** 2 * 1.6 + math.pi * 3.175 ** 2 * (20.6 - 1.6) \
               - 4 * math.pi * 2.25 ** 2 * 5
exp["placements"] = {k: [(n, matrix(c, t, {"motor": T_MOTOR, "pinion": T_PINION, "switch": T_SWITCH}[k])) for n, c, t in v]
                     for k, v in PLACE.items()}
for k, v in stages.items():
    json.dump({"tool": "batch", "args": {"verbose": False, "commands": v}}, open(f"stage{k}.json", "w"))
json.dump(exp, open("expected.json", "w"), indent=1)
print({k: len(v) for k, v in stages.items()}, "y teeth", n_yteeth, "x teeth", n_xteeth, "gear pts", len(gear))
print({k: round(v, 1) for k, v in exp.items() if isinstance(v, float)})
