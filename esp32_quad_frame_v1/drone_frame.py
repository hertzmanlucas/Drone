"""
ESP32 quad frame - parametric CadQuery model.

Coordinate system: +X = forward (camera end), +Y = left, +Z = up.
Origin: centre of the bottom plate, on its underside.

Parts produced:
  frame_bottom      unibody plate with arms, battery strap slots, ToF tail, camera interface
  deck              electronics deck on the 30.5x30.5 stack (FC perfboard + radio holder)
  camera_mount_19   optional bolt-on bracket for 19 mm "micro" FPV cameras
  camera_mount_14   optional bolt-on bracket for 14 mm "nano" FPV cameras
  landing_foot      optional TPU foot, bolts on with the motor screws (print 4)
  drone_assembly    everything incl. dummy electronics, for checking fit (STEP only)

Change the numbers in the PARAMETERS block and re-run to regenerate.
"""
import math
import os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------- PARAMETERS
PROP_D       = 76.2    # 3" props. Frame also takes 2.5" (63.5)
PROP_MARGIN  = 3.0     # min gap between prop tip and anything at prop height

PLATE_T      = 4.0     # bottom plate / arm thickness
ARM_W        = 10.0    # arm width
PAD_D        = 17.0    # motor pad diameter

MOTOR_D      = 14.0    # 1104 bell diameter
MOTOR_H      = 14.0    # 1104 height
MOTOR_HOLE_R = (4.25, 4.75)  # M2 slot covers 8.5 - 9.5 mm bolt circle
MOTOR_HOLE_W = 2.2
MOTOR_CENTER_HOLE = 5.5

STACK        = 30.5    # ESC / deck bolt pattern
STACK_HOLE   = 3.3     # M3 clearance
ESC_L, ESC_W, ESC_H = 43.0, 37.0, 6.0   # 4in1 60A board (check yours)
ESC_Z0       = PLATE_T + 3.0            # ESC sits on 3 mm nylon nuts / grommets
DECK_Z0      = PLATE_T + 16.0           # deck underside height (16 mm standoffs)
DECK_T       = 2.5
DECK_X       = (-33.0, 33.0)
DECK_Y       = 18.5                     # half width

FC_BOARD     = (59.0, 34.0)             # cut-to-size perfboard (L x W)
FC_X0        = -26.0                    # rear edge of FC board on the deck
FC_POST_H    = 6.0                      # clears M3 nuts + bolt tips on top of the deck

ESP32        = (51.5, 28.5)             # DOIT DevKit V1 30-pin (DevKitC: 54.4 x 27.9)
RADIO        = (28.5, 15.0, 1.6)        # 2.85 x 1.5 cm board, stood upright
BATTERY      = (57.0, 31.0, 12.0)       # 2S 650 mAh class (e.g. Tattu 57x31x12)
STRAP_W      = 16.0                     # slot length, fits 10-15 mm straps
TOF          = (25.0, 10.7)             # GY-VL53L0XV2 module
MPU          = (21.2, 16.4)             # GY-521

CAM_HOLE_X   = 43.0                     # camera interface: 2x M3 at (43, +/-8)
CAM_HOLE_Y   = 8.0

FOOT_H       = 18.0                     # landing foot height (> battery thickness)


# ------------------------------------------------- motor position from clearance
def rect_dist(px, py, x0, x1, y0, y1):
    dx = max(x0 - px, 0, px - x1)
    dy = max(y0 - py, 0, py - y1)
    return math.hypot(dx, dy)

# Everything that reaches prop height (top view rectangles: x0, x1, y0, y1)
HIGH_STUFF = {
    "deck + FC board + ESP32": (DECK_X[0], DECK_X[1], -DECK_Y, DECK_Y),
    "camera + lens (optional)": (36.0, 57.0, -12.0, 12.0),
    "radio plug at rear":       (-46.0, DECK_X[0], -9.0, 9.0),
}
R_CLEAR = PROP_D / 2 + PROP_MARGIN

def min_clear(a):
    worst = 1e9
    for sx in (1, -1):
        for sy in (1, -1):
            for r in HIGH_STUFF.values():
                worst = min(worst, rect_dist(sx * a, sy * a, *r))
    return worst

A = 40.0
while min_clear(A) < R_CLEAR:
    A += 0.5
MOTORS = [(sx * A, sy * A) for sx in (1, -1) for sy in (1, -1)]
WHEELBASE = 2 * math.sqrt(2) * A


# ------------------------------------------------------------------ BOTTOM PLATE
def frame_bottom():
    arm_len = math.hypot(A, A)
    s = (cq.Sketch()
         .push([(0, 0)]).rect(60, 48)                       # centre body
         .push([(41, 0)]).rect(22, 26)                      # front tab (camera interface)
         .push([(-41, 0)]).rect(22, 26)                     # rear tail (ToF sensor)
         )
    for (mx, my) in MOTORS:
        ang = math.degrees(math.atan2(my, mx))
        s = s.push([(mx / 2, my / 2)]).rect(arm_len, ARM_W, angle=ang)
    s = s.push(MOTORS).circle(PAD_D / 2)
    s = s.clean().reset().vertices().fillet(3.0)
    plate = cq.Workplane("XY").placeSketch(s).extrude(PLATE_T)

    cut = cq.Workplane("XY")
    # ESC stack holes
    plate = plate.cut(cut.pushPoints([(sx * STACK / 2, sy * STACK / 2)
                                      for sx in (1, -1) for sy in (1, -1)])
                      .circle(STACK_HOLE / 2).extrude(PLATE_T))
    # motor mounts: centre hole + 4 radial M2 slots (8.5-9.5 mm circle)
    for (mx, my) in MOTORS:
        plate = plate.cut(cq.Workplane("XY").center(mx, my)
                          .circle(MOTOR_CENTER_HOLE / 2).extrude(PLATE_T))
        for k in range(4):
            t = math.radians(90 * k)
            c = (sum(MOTOR_HOLE_R) / 2)
            slot_len = MOTOR_HOLE_R[1] - MOTOR_HOLE_R[0] + MOTOR_HOLE_W
            plate = plate.cut(
                cq.Workplane("XY")
                .center(mx + c * math.cos(t), my + c * math.sin(t))
                .slot2D(slot_len, MOTOR_HOLE_W, angle=90 * k)
                .extrude(PLATE_T))
    # battery strap slots (both sides of the battery)
    plate = plate.cut(cut.pushPoints([(0, 19.75), (0, -19.75)])
                      .rect(STRAP_W, 2.5).extrude(PLATE_T))
    # battery lead pass-through (rear of ESC)
    plate = plate.cut(cut.center(-26, 0).slot2D(12, 5, angle=90).extrude(PLATE_T))
    # weight / airflow cut-outs in the centre (avoid stack holes)
    plate = plate.cut(cut.pushPoints([(0, 0)]).circle(8).extrude(PLATE_T))
    # camera interface: 2x M3
    plate = plate.cut(cut.pushPoints([(CAM_HOLE_X, CAM_HOLE_Y), (CAM_HOLE_X, -CAM_HOLE_Y)])
                      .circle(STACK_HOLE / 2).extrude(PLATE_T))
    # ToF tail: wire hole + 2 zip-tie slots, shallow locating pocket on underside
    plate = plate.cut(cut.center(-36, 0).slot2D(7, 3.5).extrude(PLATE_T))
    plate = plate.cut(cut.pushPoints([(-41.5, 11.0), (-41.5, -11.0)])
                      .rect(4, 1.8).extrude(PLATE_T))
    plate = plate.cut(cut.center(-44, 0).rect(TOF[1] + 0.6, TOF[0] + 0.6).extrude(0.6))
    # front tab lightening
    plate = plate.cut(cut.center(36, 0).slot2D(9, 5, angle=90).extrude(PLATE_T))
    return plate


# ------------------------------------------------------------------------- DECK
def deck():
    L = DECK_X[1] - DECK_X[0]
    cx = (DECK_X[0] + DECK_X[1]) / 2
    d = (cq.Workplane("XY").center(cx, 0).rect(L, 2 * DECK_Y).extrude(DECK_T)
         .edges("|Z").fillet(3))
    # stack holes
    d = d.cut(cq.Workplane("XY").pushPoints([(sx * STACK / 2, sy * STACK / 2)
                                             for sx in (1, -1) for sy in (1, -1)])
              .circle(STACK_HOLE / 2).extrude(DECK_T))
    # wiring windows (ESC signal cable / BEC wires come up through these)
    d = d.cut(cq.Workplane("XY").pushPoints([(0, 0)]).slot2D(22, 12).extrude(DECK_T))
    d = d.cut(cq.Workplane("XY").pushPoints([(22, 9), (22, -9)]).slot2D(10, 5).extrude(DECK_T))
    # FC board posts with M2 pilot holes (FC board is screwed or foam-taped on)
    fx0, fx1 = FC_X0, FC_X0 + FC_BOARD[0]
    fy = FC_BOARD[1] / 2
    posts = [(fx0 + 3, fy - 3), (fx0 + 3, -fy + 3), (fx1 - 3, fy - 3), (fx1 - 3, -fy + 3)]
    d = d.union(cq.Workplane("XY").workplane(offset=DECK_T).pushPoints(posts)
                .circle(3).extrude(FC_POST_H))
    d = d.cut(cq.Workplane("XY").pushPoints(posts).circle(0.85).extrude(DECK_T + FC_POST_H))
    # radio holder: two grooved posts, radio slides in from the top
    rx = -30.0
    gap = RADIO[1] + 0.4
    post_w, post_d, post_h = 3.0, 5.0, 14.0
    for sy in (1, -1):
        y_c = sy * (gap / 2 + post_w / 2 - 1.2)
        p = (cq.Workplane("XY").workplane(offset=DECK_T)
             .center(rx, y_c).rect(post_d, post_w).extrude(post_h))
        groove = (cq.Workplane("XY").workplane(offset=DECK_T + 4.0)
                  .center(rx, sy * (gap / 2 - 0.6)).rect(RADIO[2] + 0.4, 2.4)
                  .extrude(post_h))
        d = d.union(p).cut(groove)
    return d


# ---------------------------------------------------------------- CAMERA MOUNT
def camera_mount(cam_w):
    base_t = 3.0
    pivot_z = base_t + cam_w * math.sqrt(2) / 2 + 0.6   # camera can tilt 0-45 deg freely
    cheek_t = 2.0
    inner = cam_w + 0.4
    x0, x1 = 36.0, 50.0
    xc = (x0 + x1) / 2
    base = (cq.Workplane("XY").center(xc, 0)
            .rect(x1 - x0, inner + 2 * cheek_t).extrude(base_t)
            .edges("|Z").fillet(2))
    base = base.cut(cq.Workplane("XY")
                    .pushPoints([(CAM_HOLE_X, CAM_HOLE_Y), (CAM_HOLE_X, -CAM_HOLE_Y)])
                    .circle(STACK_HOLE / 2).extrude(base_t))
    # counterbore so M3 heads/nuts sit below the camera sweep
    base = base.cut(cq.Workplane("XY").workplane(offset=base_t - 1.5)
                    .pushPoints([(CAM_HOLE_X, CAM_HOLE_Y), (CAM_HOLE_X, -CAM_HOLE_Y)])
                    .circle(3.2).extrude(1.5))
    for sy in (1, -1):
        y = sy * (inner / 2 + cheek_t / 2)
        cheek = (cq.Workplane("XZ", origin=(0, y + cheek_t / 2, 0))
                 .moveTo(x0, 0).lineTo(x1, 0).lineTo(x1, pivot_z)
                 .threePointArc((xc, pivot_z + 7), (x0, pivot_z)).close()
                 .extrude(cheek_t))
        hole = (cq.Workplane("XZ", origin=(0, y + cheek_t, 0)).center(xc, pivot_z)
                .circle(1.1).extrude(cheek_t * 2))
        base = base.union(cheek).cut(hole)
    return base.translate((0, 0, PLATE_T)), PLATE_T + pivot_z


# ---------------------------------------------------------------- LANDING FOOT
def landing_foot():
    od = 16.0
    f = cq.Workplane("XY").circle(od / 2).extrude(FOOT_H)
    # pocket from below leaves a 2 mm floor at the top (against the arm)
    f = f.cut(cq.Workplane("XY").circle(6.2).extrude(FOOT_H - 2.0))
    f = f.cut(cq.Workplane("XY").circle(MOTOR_CENTER_HOLE / 2).extrude(FOOT_H))
    for k in range(4):
        t = math.radians(90 * k)
        c = sum(MOTOR_HOLE_R) / 2
        f = f.cut(cq.Workplane("XY").center(c * math.cos(t), c * math.sin(t))
                  .slot2D(MOTOR_HOLE_R[1] - MOTOR_HOLE_R[0] + MOTOR_HOLE_W,
                          MOTOR_HOLE_W, angle=90 * k).extrude(FOOT_H))
    f = f.faces("<Z").edges().fillet(0.6)
    return f  # modelled upright: floor on top, print floor-down (flip)


# ------------------------------------------------------------- DUMMY ELECTRONICS
def box(cx, cy, z0, lx, ly, h):
    return cq.Workplane("XY").box(lx, ly, h, centered=(True, True, False)).translate((cx, cy, z0))


def build():
    fb = frame_bottom()
    dk = deck().translate((0, 0, DECK_Z0))
    cam19, pivot19 = camera_mount(19.0)
    cam14, _ = camera_mount(14.0)
    foot = landing_foot()

    fc_z = DECK_Z0 + DECK_T + FC_POST_H
    fc_cx = FC_X0 + FC_BOARD[0] / 2
    dummies = {
        "ESC 4in1 (dummy)":     box(0, 0, ESC_Z0, ESC_L, ESC_W, ESC_H),
        "FC perfboard (dummy)": box(fc_cx, 0, fc_z, FC_BOARD[0], FC_BOARD[1], 1.6),
        "MPU6050 (dummy)":      box(fc_cx, 0, fc_z + 1.6, MPU[0], MPU[1], 1.6),
        "ESP32 DevKit (dummy)": box(fc_cx, 0, fc_z + 1.6 + 8.5, ESP32[0], ESP32[1], 4.8),
        "Radio (dummy)":        box(-30, 0, DECK_Z0 + DECK_T + 4, RADIO[2], RADIO[1], RADIO[0]),
        "Battery 2S 650 (dummy)": box(0, 0, -BATTERY[2], BATTERY[0], BATTERY[1], BATTERY[2]),
        "VL53L0X (dummy)":      box(-44, 0, -1.2, TOF[1], TOF[0], 1.2),
        "Camera 19mm (dummy)":  box(43, 0, pivot19 - 9.5, 19, 19, 19).union(
                                  cq.Workplane("YZ", origin=(52.5, 0, pivot19)).circle(6).extrude(5)),
    }
    for i, (mx, my) in enumerate(MOTORS):
        dummies[f"Motor {i+1} (dummy)"] = (cq.Workplane("XY").circle(MOTOR_D / 2)
                                           .extrude(MOTOR_H).translate((mx, my, PLATE_T)))
        dummies[f"Prop disc {i+1}"] = (cq.Workplane("XY").circle(PROP_D / 2).extrude(0.5)
                                       .translate((mx, my, PLATE_T + MOTOR_H + 2)))
        dummies[f"Foot {i+1}"] = foot.translate((mx, my, -FOOT_H))

    parts = {"frame_bottom": fb, "deck": dk, "camera_mount_19": cam19,
             "camera_mount_14": cam14, "landing_foot": foot}
    return parts, dummies


if __name__ == "__main__":
    parts, dummies = build()

    # printable parts: export in print orientation
    printable = {
        "frame_bottom": parts["frame_bottom"],
        "deck": parts["deck"].translate((0, 0, -DECK_Z0)),
        "camera_mount_19": parts["camera_mount_19"].translate((0, 0, -PLATE_T)),
        "camera_mount_14": parts["camera_mount_14"].translate((0, 0, -PLATE_T)),
        "landing_foot": parts["landing_foot"].rotate((0, 0, 0), (1, 0, 0), 180)
                                             .translate((0, 0, FOOT_H)),
    }
    for name, wp in printable.items():
        cq.exporters.export(wp, os.path.join(OUT, f"{name}.step"))
        cq.exporters.export(wp, os.path.join(OUT, f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)

    colors = {"frame_bottom": (0.15, 0.15, 0.18), "deck": (0.2, 0.35, 0.6),
              "camera_mount_19": (0.8, 0.4, 0.1)}
    asm = cq.Assembly(name="esp32_quad")
    for name in ("frame_bottom", "deck", "camera_mount_19"):
        asm.add(parts[name], name=name, color=cq.Color(*colors[name]))
    for name, wp in dummies.items():
        c = (0.9, 0.9, 0.9) if "Prop" in name else (0.55, 0.55, 0.55)
        if "Battery" in name: c = (0.9, 0.75, 0.1)
        if "ESP32" in name or "FC" in name: c = (0.1, 0.5, 0.25)
        if "Foot" in name: c = (0.3, 0.3, 0.3)
        asm.add(wp, name=name.replace(" ", "_").replace("(", "").replace(")", ""), color=cq.Color(*c))
    asm.save(os.path.join(OUT, "drone_assembly.step"))

    print(f"motor offset A = {A:.1f} mm  -> motor-to-motor diagonal {WHEELBASE:.1f} mm")
    print(f"adjacent motor spacing {2*A:.1f} mm, prop gap {2*A - PROP_D:.1f} mm")
    print(f"min clearance prop tip -> high parts: {min_clear(A) - PROP_D/2:.1f} mm")
    for name, wp in printable.items():
        v = wp.val().Volume() / 1000
        print(f"{name:16s} volume {v:6.2f} cm3  valid={wp.val().isValid()}")
