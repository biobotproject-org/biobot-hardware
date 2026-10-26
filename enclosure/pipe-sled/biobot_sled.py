# biobot_sled.py  -  Autodesk Fusion script
#
# Builds the BioBot pipe-enclosure sled for a 6-inch Schedule 40 PVC tube, plus
# a reference model of the tube so the fit can be checked on screen.
#
# How to run:
#   Utilities tab -> Add-Ins -> Scripts and Add-Ins -> "+" next to My Scripts
#   -> Python, name it biobot_sled -> Create. Click Edit (opens VS Code or
#   Spyder), replace the whole file with this one, save, then select the
#   script and press Run. A new component "BioBot Sled" appears in a new or
#   active design.
#
# Coordinates: Z runs up the tube (0 = outer face of the sensor bulkhead at the
# nose). The tube centreline is X = 0, Y = 0. The plate is a slab at negative X,
# PLATE_DROP below the centreline, and every component stands on its +X face.
#
# Everything marked MEASURE is a placeholder: measure the real part and change
# the number, then rerun. The script is idempotent enough to rerun into a fresh
# design; rerunning in the same design adds a second component.
#
# Printing: the sled is three separate bodies so each prints flat with no
# supports. Right-click each body -> Save As Mesh -> STL.
#   plate          rails facing UP, component face on the bed
#   nose bulkhead  outer (sensor) face on the bed, mounting block pointing up
#   top bulkhead   outer face on the bed, mounting block pointing up
# Each bulkhead has a block standing on its inner face. The plate rests on the
# two blocks and is held by two M3 screws at each end, driven down through the
# plate into the block (self-tapping, or a heat-set insert in a 4.0 mm hole).

import math
import os
import traceback

import adsk.core
import adsk.fusion

# ---------------------------------------------------------------------------
# Parameters (mm)
# ---------------------------------------------------------------------------
PIPE_ID = 154.1          # 6 in Sch 40 PVC inner diameter
PIPE_OD = 168.3
SENSOR_ZONE = 60.0       # tube length below the nose, open to air through mesh
TUBE_LEN = 335.0
TUBE_SHOW = True         # draw the reference tube as a second component

CLEAR = 0.8              # radial clearance, bulkhead edge to pipe wall
PLATE_T = 4.0            # plate thickness
PLATE_W = 136.0          # plate width (chord)
PLATE_DROP = 30.0        # plate mid-plane below the tube centreline
BULK_T = 4.0             # bulkhead thickness (both ends)
RAIL_W = 6.0             # stiffening rails under the plate
RAIL_H = 10.0
RAIL_Y = 44.0            # rail centreline offset from the sled centreline
GAP = 8.0                # spacing between parts along the tube

# Component footprints: W across the plate (Y), L along the tube (Z).
# PCB (fire-detection-device rev from biobot-hardware gerbers): 70.0 x 52.0 mm
# after the panel rails are snapped off. Three plated 3.0 mm mounting holes,
# 2.8 mm in from the edges, at every corner except the Nano's. The four
# 1.65 mm holes are the Nano module's own footprint, not mounts.
# The board lies with its 52 mm side across the sled and the JST sensor
# connectors (P5/P6, the gerber x = 70 edge) toward the nose.
PCB_W, PCB_L = 52.0, 70.0
PCB_HOLES = [(2.8, 49.2), (67.2, 2.8), (67.2, 49.2)]   # gerber (x, y), mm, origin bottom-left
PCB_BOSS_OD = 7.0        # printed standoff outer diameter
PCB_BOSS_H = 6.0         # standoff height: clears the header pins under the board
PCB_PILOT_D = 2.1        # M2.5 self-tapping into PETG; use 3.5 for an M2.5 heat-set insert
BAT_W, BAT_L = 60.0, 107.0
# Victron 75/15, measured: body 100 x 113 x 31 mm, mounting holes 85 mm apart
# across the terminal edge and 105 mm apart along the body. Mounted sideways:
# its 113 mm length runs across the sled and the terminals face the PCB side
# (-Y), so the solar lead from the top plug and the battery and PCB leads all
# meet on one side. See the notes in the reply for why sideways is fine here.
VIC_W, VIC_L, VIC_H = 113.0, 100.0, 31.0
VIC_HOLE_DY, VIC_HOLE_DZ = 105.0, 85.0
VIC_BOSS_OD, VIC_BOSS_H, VIC_PILOT_D = 9.0, 4.0, 2.5          # M3 self-tapping; 4.0 for M3 insert
# Sensors on the outer face of the nose disc, both shifted toward the PCB side
# (-Y) so the Qwiic leads to P5/P6 stay short. X is across the disc from the
# plate; SENSOR_X_OFF is measured from the plate's component face.
SENSOR_X_OFF = 30.0
BME_Y = -48.0                                                 # BME688 centre
BME_PITCH, BME_HOLE_D = 20.32, 3.2                            # SparkFun 1x1 in Qwiic board, 4 holes
BMV_Y = -14.0                                                 # BMV080 centre
BMV_PITCH, BMV_HOLE_D = 26.0, 3.2                             # SparkFun BMV080 enclosure, 30x30, 2 holes
BMV_HOLES_ALONG = 'y'                                         # 'y' across the disc, 'x' toward its centre
SLOT_L, SLOT_W = 25.0, 4.0                                    # battery strap slots
# Two straps at right angles: one across the battery at mid-length, one along
# it from end to end. Between them the pack cannot slide, lift or tip.
STRAP_ACROSS_AT = 0.5
FINGER_D = 26.0          # pull hole in the top bulkhead
# Plate-to-bulkhead joint
BLOCK_D = 12.0           # how far each mounting block reaches along the tube
BLOCK_H = 10.0           # block height below the plate
BLOCK_Y0, BLOCK_Y1 = -60.0, -4.0   # block spans this across the plate (PCB side only,
                                   # so the lengthwise battery strap can pass under the plate)
FIT = 0.2                # gap between plate end and bulkhead face
JOINT_SCREW_Y = (-45.0, -19.0)     # joint screws across the plate
JOINT_CLEAR_D = 3.4      # M3 clearance through the plate
JOINT_PILOT_D = 2.6      # M3 self-tapping pilot in the block (4.0 for a heat-set insert)
PRINT_BED = 256.0        # longest side of your printer's bed

# STL export. After building, each printable body is written as a separate
# STL next to this script (or to EXPORT_DIR if set), ready for the slicer.
EXPORT_STL = True
EXPORT_DIR = ''          # '' = the folder this script lives in
CABLE_D = 10.0           # solar lead pass-through in the top bulkhead

# Derived layout along Z
Z_PARTS = BULK_T + GAP + SLOT_W + 2.0        # first row (PCB + battery), after the lower strap slot
Z_BAT_END = Z_PARTS + BAT_L
Z_SLOT_LO = Z_PARTS - 2.0 - SLOT_W / 2.0      # lengthwise strap slots, just off each battery end
Z_SLOT_HI = Z_BAT_END + 2.0 + SLOT_W / 2.0
Z_VIC = Z_SLOT_HI + SLOT_W / 2.0 + 4.0
Z_VIC_END = Z_VIC + VIC_L
PLATE_L = Z_VIC_END + 6.0 + BULK_T            # total sled length
BULK_R = PIPE_ID / 2.0 - CLEAR
X_PLATE_LO = -(PLATE_DROP + PLATE_T / 2.0)    # plate faces
X_PLATE_HI = -(PLATE_DROP - PLATE_T / 2.0)
Y_PCB = -(PLATE_W / 2.0 - PCB_W / 2.0 - 1.5)  # PCB against the -Y edge
# Battery: two straps wrap across it, down through a slot on each side of it,
# under the plate, and back up. The slots sit just outside its long edges.
Y_BAT = Y_PCB + PCB_W / 2.0 + 4.0 + SLOT_W + 2.0 + BAT_W / 2.0
Y_SLOT_A = Y_BAT - BAT_W / 2.0 - 2.0 - SLOT_W / 2.0   # between PCB and battery
Y_SLOT_B = Y_BAT + BAT_W / 2.0 + 2.0 + SLOT_W / 2.0   # outboard of the battery


def mm(v):
    """Fusion's API works in centimetres."""
    return v / 10.0


def p3(x, y, z):
    return adsk.core.Point3D.create(mm(x), mm(y), mm(z))


def all_profiles(sketch):
    col = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        col.add(sketch.profiles.item(i))
    return col


def extrude(comp, profiles, dist, operation, participants=None, symmetric=False):
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(profiles, operation)
    d = adsk.core.ValueInput.createByReal(mm(dist))
    if symmetric:
        inp.setSymmetricExtent(d, True)
    else:
        inp.setDistanceExtent(False, d)
    if participants:
        inp.participantBodies = participants
    return feats.add(inp)


def extrude_outward_x(comp, profiles, dist, x_face):
    """New bodies extruded `dist` from a sketch lying on the plane x = x_face,
    guaranteed to grow toward +X. Fusion's sign for a sketch normal is not
    reliable, so this tries one way, checks where the body landed, and redoes
    it the other way if it went into the plate."""
    feats = comp.features.extrudeFeatures
    for sign in (1.0, -1.0):
        inp = feats.createInput(profiles, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(sign * mm(dist)))
        feat = feats.add(inp)
        if feat.bodies.count and feat.bodies.item(0).boundingBox.maxPoint.x > mm(x_face) + mm(dist) * 0.5:
            return feat
        feat.deleteMe()
    raise RuntimeError('boss extrude went the wrong way both times')


def join_into(comp, target, feature):
    tools = adsk.core.ObjectCollection.create()
    for i in range(feature.bodies.count):
        tools.add(feature.bodies.item(i))
    cinp = comp.features.combineFeatures.createInput(target, tools)
    cinp.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comp.features.combineFeatures.add(cinp)


def yz_offset_plane(comp, x):
    """Construction plane parallel to YZ at x. Fusion's offset sign for the
    YZ plane is not reliable, so the plane is checked and remade if needed."""
    planes = comp.constructionPlanes
    for sign in (1.0, -1.0):
        inp = planes.createInput()
        inp.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(sign * mm(x)))
        plane = planes.add(inp)
        if abs(plane.geometry.origin.x - mm(x)) < 1e-4:
            return plane
        plane.deleteMe()
    raise RuntimeError('could not place a YZ offset plane at x = %.1f mm' % x)


def plate_body(comp):
    """The plate body, looked up fresh: combines can invalidate old references."""
    b = comp.bRepBodies.itemByName('plate')
    if b is None:
        raise RuntimeError('plate body not found')
    return b


def xy_offset_plane(comp, z):
    """Construction plane parallel to XY at height z (offset is along +Z)."""
    planes = comp.constructionPlanes
    inp = planes.createInput()
    inp.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(z)))
    return planes.add(inp)


def rect_xy(sketch, x1, y1, x2, y2):
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(p3(x1, y1, 0), p3(x2, y2, 0))


def circle_xy(sketch, x, y, d):
    sketch.sketchCurves.sketchCircles.addByCenterRadius(p3(x, y, 0), mm(d / 2.0))


def circle_model(sketch, x, y, z, d):
    """Circle on any sketch, given its centre in model coordinates."""
    c = sketch.modelToSketchSpace(p3(x, y, z))
    sketch.sketchCurves.sketchCircles.addByCenterRadius(c, mm(d / 2.0))


def rect_model(sketch, a, b):
    """Rectangle on any sketch from two model-space corner points (x, y, z)."""
    pa = sketch.modelToSketchSpace(p3(*a))
    pb = sketch.modelToSketchSpace(p3(*b))
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(pa, pb)


def new_component(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


# ---------------------------------------------------------------------------
# Sled
# ---------------------------------------------------------------------------
def build_sled(root):
    comp = new_component(root, 'BioBot Sled')
    new_body = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    cut = adsk.fusion.FeatureOperations.CutFeatureOperation

    # Plate: slab PLATE_T thick in X, PLATE_W wide in Y, running between the
    # two bulkheads (a separate print).
    z0, z1 = BULK_T + FIT, PLATE_L - BULK_T - FIT
    sk = comp.sketches.add(xy_offset_plane(comp, z0))
    sk.name = 'plate'
    rect_xy(sk, X_PLATE_LO, -PLATE_W / 2.0, X_PLATE_HI, PLATE_W / 2.0)
    for y in (-RAIL_Y, RAIL_Y):      # rails under the plate, same sketch, full length
        rect_xy(sk, X_PLATE_LO - RAIL_H, y - RAIL_W / 2.0, X_PLATE_LO, y + RAIL_W / 2.0)
    plate_feat = extrude(comp, all_profiles(sk), z1 - z0, new_body)
    plate = plate_feat.bodies.item(0)
    plate.name = 'plate'              # named before the join so the lookup by name survives it
    extra = adsk.core.ObjectCollection.create()
    for i in range(1, plate_feat.bodies.count):
        extra.add(plate_feat.bodies.item(i))
    if extra.count:
        cinp = comp.features.combineFeatures.createInput(plate, extra)
        cinp.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        comp.features.combineFeatures.add(cinp)

    # ---- standoff bosses on the component face: PCB (three) and Victron (four) ----
    y0, z0 = Y_PCB - PCB_W / 2.0, Z_PARTS
    pcb_at = [(y0 + gy, z0 + (PCB_L - gx)) for gx, gy in PCB_HOLES]
    zc = Z_VIC + VIC_L / 2.0
    vic_at = [(dy, zc + dz) for dy in (-VIC_HOLE_DY / 2.0, VIC_HOLE_DY / 2.0)
              for dz in (-VIC_HOLE_DZ / 2.0, VIC_HOLE_DZ / 2.0)]
    for name, at, od, h, pilot in (('pcb', pcb_at, PCB_BOSS_OD, PCB_BOSS_H, PCB_PILOT_D),
                                   ('victron', vic_at, VIC_BOSS_OD, VIC_BOSS_H, VIC_PILOT_D)):
        sk = comp.sketches.add(yz_offset_plane(comp, X_PLATE_HI))
        sk.name = name + ' bosses'
        for y, z in at:
            circle_model(sk, X_PLATE_HI, y, z, od)
        join_into(comp, plate_body(comp), extrude_outward_x(comp, all_profiles(sk), h, X_PLATE_HI))
        sk = comp.sketches.add(comp.yZConstructionPlane)
        sk.name = name + ' pilot holes'
        for y, z in at:
            circle_model(sk, 0, y, z, pilot)
        extrude(comp, all_profiles(sk), 2 * (PLATE_DROP + PLATE_T + h + 5.0), cut,
                participants=[plate_body(comp)], symmetric=True)

    # Bulkheads: full discs at the nose (z = 0) and the top.
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'nose bulkhead'
    circle_xy(sk, 0, 0, 2 * BULK_R)
    nose = extrude(comp, sk.profiles.item(0), BULK_T, new_body).bodies.item(0)
    nose.name = 'nose bulkhead'
    add_block(comp, nose, BULK_T, +1)

    sk = comp.sketches.add(xy_offset_plane(comp, PLATE_L - BULK_T))
    sk.name = 'top bulkhead'
    circle_xy(sk, 0, 0, 2 * BULK_R)
    top = extrude(comp, sk.profiles.item(0), BULK_T, new_body).bodies.item(0)
    top.name = 'top bulkhead'
    add_block(comp, top, PLATE_L - BULK_T, -1)

    # ---- joint screws: clearance through the plate, pilot into the blocks ----
    discs = [comp.bRepBodies.itemByName('nose bulkhead'), comp.bRepBodies.itemByName('top bulkhead')]
    for d, who in ((JOINT_CLEAR_D, [plate_body(comp)]), (JOINT_PILOT_D, discs)):
        sk = comp.sketches.add(comp.yZConstructionPlane)
        sk.name = 'joint screws %.1f' % d
        for z in (BULK_T + BLOCK_D / 2.0, PLATE_L - BULK_T - BLOCK_D / 2.0):
            for y in JOINT_SCREW_Y:
                circle_model(sk, 0, y, z, d)
        extrude(comp, all_profiles(sk), 2 * (PLATE_DROP + PLATE_T + BLOCK_H + 5.0), cut,
                participants=who, symmetric=True)

    # ---- holes through the plate (cut along X, symmetric about the YZ plane) ----
    sk = comp.sketches.add(comp.yZConstructionPlane)
    sk.name = 'plate holes'
    # Across strap: a slot either side of the battery at mid-length (slots run along Z)
    z = Z_PARTS + STRAP_ACROSS_AT * BAT_L
    for y in (Y_SLOT_A, Y_SLOT_B):
        rect_model(sk, (0, y - SLOT_W / 2.0, z - SLOT_L / 2.0), (0, y + SLOT_W / 2.0, z + SLOT_L / 2.0))
    # Lengthwise strap: a slot just off each end of the battery (slots run along Y)
    for z in (Z_SLOT_LO, Z_SLOT_HI):
        rect_model(sk, (0, Y_BAT - SLOT_L / 2.0, z - SLOT_W / 2.0), (0, Y_BAT + SLOT_L / 2.0, z + SLOT_W / 2.0))
    extrude(comp, all_profiles(sk), 2 * (PLATE_DROP + PLATE_T + RAIL_H + 5.0), cut,
            participants=[plate_body(comp)], symmetric=True)

    # ---- notch the +Y rail where the across strap passes under the plate ----
    if abs(RAIL_Y) > Y_SLOT_A and abs(RAIL_Y) < Y_SLOT_B:
        for z in (Z_PARTS + STRAP_ACROSS_AT * BAT_L,):
            sk = comp.sketches.add(xy_offset_plane(comp, z - SLOT_L / 2.0 - 1.0))
            sk.name = 'rail notch'
            rect_xy(sk, X_PLATE_LO - RAIL_H - 1.0, RAIL_Y - RAIL_W / 2.0 - 1.0, X_PLATE_LO - 0.01, RAIL_Y + RAIL_W / 2.0 + 1.0)
            extrude(comp, sk.profiles.item(0), SLOT_L + 2.0, cut, participants=[plate_body(comp)])

    # ---- nose bulkhead: sensor standoff holes and a cable slot (cut along Z) ----
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'sensor holes'
    x_sensor = X_PLATE_HI + SENSOR_X_OFF
    for dx in (-BME_PITCH / 2.0, BME_PITCH / 2.0):            # BME688: four corner holes
        for dy in (-BME_PITCH / 2.0, BME_PITCH / 2.0):
            circle_xy(sk, x_sensor + dx, BME_Y + dy, BME_HOLE_D)
    for d in (-BMV_PITCH / 2.0, BMV_PITCH / 2.0):             # BMV080: two holes, 26 mm apart
        if BMV_HOLES_ALONG == 'y':
            circle_xy(sk, x_sensor, BMV_Y + d, BMV_HOLE_D)
        else:
            circle_xy(sk, x_sensor + d, BMV_Y, BMV_HOLE_D)
    # Qwiic cable slot, close to the plate on the PCB side
    rect_xy(sk, X_PLATE_HI + 4.0, Y_PCB - 8.0, X_PLATE_HI + 11.0, Y_PCB + 8.0)
    extrude(comp, all_profiles(sk), BULK_T + 1.0, cut, participants=[comp.bRepBodies.itemByName('nose bulkhead')])

    # ---- top bulkhead: finger hole and solar-lead pass-through ----
    sk = comp.sketches.add(xy_offset_plane(comp, PLATE_L - BULK_T))
    sk.name = 'top holes'
    circle_xy(sk, 40.0, 0.0, FINGER_D)
    circle_xy(sk, 15.0, Y_BAT, CABLE_D)     # solar lead comes down over the battery side
    extrude(comp, all_profiles(sk), BULK_T + 1.0, cut, participants=[comp.bRepBodies.itemByName('top bulkhead')])

    return comp


def add_block(comp, disc, z_face, direction):
    """Mounting block standing on a bulkhead's inner face, under the plate.

    z_face is the inner face of the disc; direction +1 grows the block up the
    tube (nose), -1 down the tube (top). The block is joined to the disc, so it
    prints standing up from the disc with no overhang.
    """
    start = z_face if direction > 0 else z_face - BLOCK_D
    sk = comp.sketches.add(xy_offset_plane(comp, start))
    sk.name = 'joint block'
    rect_xy(sk, X_PLATE_LO - BLOCK_H, BLOCK_Y0, X_PLATE_LO - FIT, BLOCK_Y1)
    block = extrude(comp, sk.profiles.item(0), BLOCK_D, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    tools = adsk.core.ObjectCollection.create()
    tools.add(block)
    cinp = comp.features.combineFeatures.createInput(disc, tools)
    cinp.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comp.features.combineFeatures.add(cinp)


# ---------------------------------------------------------------------------
# Reference tube (for fit checks only; not for printing)
# ---------------------------------------------------------------------------
def build_tube(root):
    comp = new_component(root, 'Pipe 6 in Sch 40 (reference)')
    sk = comp.sketches.add(xy_offset_plane(comp, -SENSOR_ZONE))
    sk.name = 'tube section'
    circle_xy(sk, 0, 0, PIPE_OD)
    circle_xy(sk, 0, 0, PIPE_ID)
    # Two concentric circles give two profiles; pick the ring.
    want = math.pi * (mm(PIPE_OD) ** 2 - mm(PIPE_ID) ** 2) / 4.0
    ring = min((sk.profiles.item(i) for i in range(sk.profiles.count)),
               key=lambda pr: abs(pr.areaProperties().area - want))
    body = extrude(comp, ring, TUBE_LEN, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = 'tube'
    body.opacity = 0.3
    return comp


# ---------------------------------------------------------------------------
# STL export: one file per printable body, high refinement, binary STL.
# The slicer then lays each one flat: plate with rails up, discs with the
# block up.
# ---------------------------------------------------------------------------
def export_stls(design, sled_comp):
    out_dir = EXPORT_DIR or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)
    exporter = design.exportManager
    written = []
    for body_name, file_name in (('plate', 'sled_plate.stl'),
                                 ('nose bulkhead', 'sled_nose_disc.stl'),
                                 ('top bulkhead', 'sled_top_disc.stl')):
        body = sled_comp.bRepBodies.itemByName(body_name)
        if body is None:
            continue
        path = os.path.join(out_dir, file_name)
        opts = exporter.createSTLExportOptions(body, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opts.isBinaryFormat = True
        exporter.execute(opts)
        written.append(path)
    return written


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a design first (File -> New Design), then run the script.')
            return
        root = design.rootComponent

        sled = build_sled(root)
        if TUBE_SHOW:
            build_tube(root)
        exported = export_stls(design, sled) if EXPORT_STL else []

        app.activeViewport.fit()
        chord = 2.0 * math.sqrt(BULK_R ** 2 - PLATE_DROP ** 2)
        ui.messageBox(
            'BioBot sled built.\n\n'
            f'Sled: {PLATE_W:.0f} x {PLATE_L:.0f} x {PLATE_T:.0f} mm plate, bulkheads {2 * BULK_R:.1f} mm dia\n'
            f'Plate sits {PLATE_DROP:.0f} mm below centre; chord there is {chord:.0f} mm\n'
            f'Victron headroom at its edges: {math.sqrt(BULK_R ** 2 - (VIC_W / 2) ** 2) + PLATE_DROP:.0f} mm (needs {VIC_H + VIC_BOSS_H:.0f})\n\n'
            f'Print parts: plate {PLATE_W:.0f} x {PLATE_L - 2 * BULK_T - 2 * FIT:.0f} mm, two discs {2 * BULK_R:.0f} mm'
            + ('' if PLATE_L - 2 * BULK_T - 2 * FIT <= PRINT_BED else f'  (PLATE IS LONGER THAN YOUR {PRINT_BED:.0f} mm BED)')
            + '\n\n'
            + ('STL files written:\n  ' + '\n  '.join(exported) + '\n\n' if exported else '')
            + 'Holes marked MEASURE in the script are placeholders. '
            'PCB standoffs take M2.5 screws, Victron standoffs take M3. '
            'Battery: one strap across, one lengthwise.'
        )
    except Exception:
        if ui:
            ui.messageBox('Script failed:\n{}'.format(traceback.format_exc()))
