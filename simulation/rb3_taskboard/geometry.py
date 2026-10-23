"""Procedural USD geometry for the measured nine-module connector taskboard.

All measurements are metres; the table surface is z=0.  The visual models are
approximations of the supplied RGB-D frames, not manufacturer connector CAD.
Moving roots have translate and rotateZ ops and no dynamic rigid bodies, so a
high-level interaction controller can move them without fighting a solver.
"""
from __future__ import annotations

import math
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade


DEFAULT_LAYOUT = [
    ["usb", "locker", "outlet"],
    ["audio", "hdmi", "latch"],
    ["lan", "rotary", "bnc"],
]
INSERTION_TASKS = ("outlet", "audio", "usb", "lan", "hdmi", "bnc")
ROTATION_TASKS = ("rotary", "latch", "locker")
ROOT = "/World/Taskboard"


class _Geometry:
    def __init__(self, stage):
        self.stage = stage
        self.materials = {}
        for name, color, metallic, roughness in (
            ("white", (.84, .85, .83), 0., .64),
            ("white_edge", (.64, .67, .65), 0., .72),
            ("white_side", (.73, .75, .73), 0., .72),
            ("black", (.017, .023, .024), .05, .34),
            ("black_rubber", (.008, .011, .012), 0., .7),
            ("cavity", (.002, .003, .004), 0., .95),
            ("silver", (.56, .59, .60), .85, .24),
            ("dark_silver", (.26, .29, .30), .8, .32),
            ("gold", (.57, .38, .12), .85, .24),
            ("blue", (.016, .14, .32), .05, .35),
            ("blue_stripe", (.12, .32, .48), .1, .42),
            ("table", (.24, .27, .28), .25, .43),
            ("steel", (.32, .36, .37), .65, .37),
            ("wall", (.57, .59, .57), 0., .85),
            ("partition", (.018, .024, .022), 0., .8),
        ):
            mat = UsdShade.Material.Define(stage, f"{ROOT}/Looks/{name}")
            shader = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            self.materials[name] = mat

    def xform(self, path, position=(0, 0, 0), rotate_z=None):
        node = UsdGeom.Xform.Define(self.stage, path)
        node.AddTranslateOp().Set(Gf.Vec3d(*position))
        if rotate_z is not None:
            node.AddRotateZOp().Set(float(rotate_z))
        return node

    def finish(self, node, material, collision=False):
        UsdShade.MaterialBindingAPI.Apply(node.GetPrim()).Bind(self.materials[material])
        if collision:
            UsdPhysics.CollisionAPI.Apply(node.GetPrim())
        return node

    def box(self, path, position, size, material="white", collision=False):
        node = UsdGeom.Cube.Define(self.stage, path)
        node.CreateSizeAttr(1.)
        node.AddTranslateOp().Set(Gf.Vec3d(*position))
        node.AddScaleOp().Set(Gf.Vec3f(*size))
        return self.finish(node, material, collision)

    def cylinder(self, path, position, radius, height, material="silver", axis="Z"):
        node = UsdGeom.Cylinder.Define(self.stage, path)
        node.CreateRadiusAttr(radius)
        node.CreateHeightAttr(height)
        node.CreateAxisAttr(axis)
        node.AddTranslateOp().Set(Gf.Vec3d(*position))
        return self.finish(node, material)

    def mesh(self, path, points, faces, material, collision=False):
        node = UsdGeom.Mesh.Define(self.stage, path)
        node.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        node.CreateFaceVertexCountsAttr([len(f) for f in faces])
        node.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        node.CreateSubdivisionSchemeAttr("none")
        node.CreateDoubleSidedAttr(True)
        # Explicit face normals prevent an importer/renderer from smoothing
        # broad planar faces into the small rounded-edge facets.
        normals = []
        for face in faces:
            normal = Gf.Vec3f(0.)
            for i, j in zip(face, face[1:]+face[:1]):
                p, q = points[i], points[j]
                normal += Gf.Vec3f((p[1]-q[1])*(p[2]+q[2]),
                                  (p[2]-q[2])*(p[0]+q[0]),
                                  (p[0]-q[0])*(p[1]+q[1]))
            if normal.GetLength() > 1e-12:
                normal.Normalize()
            else:
                normal = Gf.Vec3f(0., 0., 1.)
            normals.extend([normal]*len(face))
        node.CreateNormalsAttr(normals)
        node.SetNormalsInterpolation("faceVarying")
        return self.finish(node, material, collision)

    def cable(self, path, points, width=.004):
        node = UsdGeom.BasisCurves.Define(self.stage, path)
        node.CreateTypeAttr("linear")
        node.CreateWrapAttr("nonperiodic")
        node.CreateCurveVertexCountsAttr([len(points)])
        node.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        node.CreateWidthsAttr([width])
        node.SetWidthsInterpolation("constant")
        return self.finish(node, "black_rubber")

    def prism(self, path, polygon, bottom, top, material="black", collision=False):
        n = len(polygon)
        points = [(x, y, z) for z in (bottom, top) for x, y in polygon]
        faces = [list(reversed(range(n))), list(range(n, 2*n))]
        faces.extend([[i, (i+1) % n, (i+1) % n+n, i+n] for i in range(n)])
        return self.mesh(path, points, faces, material, collision)

    def round_box(self, path, position, size, radius=.002, material="white", collision=False):
        x, y, z = position
        sx, sy, sz = size
        r = min(radius, sx/2, sy/2)
        corners = [(sx/2-r, sy/2-r, 0), (-sx/2+r, sy/2-r, 90),
                   (-sx/2+r, -sy/2+r, 180), (sx/2-r, -sy/2+r, 270)]
        poly = []
        for cx, cy, start in corners:
            for k in range(5):
                a = math.radians(start+k*22.5)
                poly.append((x+cx+r*math.cos(a), y+cy+r*math.sin(a)))
        return self.prism(path, poly, z-sz/2, z+sz/2, material, collision)

    def ring(self, path, position, outer, inner, height, material="silver", segments=48):
        x, y, z = position
        points = []
        for level, radius in ((z-height/2, outer), (z+height/2, outer),
                              (z-height/2, inner), (z+height/2, inner)):
            points.extend([(x+radius*math.cos(2*math.pi*i/segments),
                            y+radius*math.sin(2*math.pi*i/segments), level)
                           for i in range(segments)])
        faces = []
        n = segments
        for i in range(n):
            j = (i+1) % n
            faces.extend([[i, j, j+n, i+n], [i+2*n, i+3*n, j+3*n, j+2*n],
                          [i+n, j+n, j+3*n, i+3*n], [i, i+2*n, j+2*n, j]])
        return self.mesh(path, points, faces, material)

    def rect_ring(self, path, position, outer, inner, height, material="silver"):
        x, y, z = position
        ox, oy = outer
        ix, iy = inner
        self.box(f"{path}_xp", (x+(ox+ix)/4, y, z), ((ox-ix)/2, oy, height), material)
        self.box(f"{path}_xn", (x-(ox+ix)/4, y, z), ((ox-ix)/2, oy, height), material)
        self.box(f"{path}_yp", (x, y+(oy+iy)/4, z), (ix, (oy-iy)/2, height), material)
        self.box(f"{path}_yn", (x, y-(oy+iy)/4, z), (ix, (oy-iy)/2, height), material)

    def screw(self, path, position, radius=.002, material="silver"):
        x, y, z = position
        self.cylinder(path, position, radius, .001, material)
        self.box(path+"_slot", (x, y, z+.00052), (radius*1.3, .00045, .00012), "cavity")


def _mount(g, path, top, width=.025, length=.035, material="black"):
    g.round_box(path+"/Flange", (0, 0, top+.002), (length, width, .004), .003, material)
    for sign in (-1, 1):
        g.screw(path+f"/MountScrew_{sign+1}", (sign*(length/2-.0035), 0, top+.0045), .0018)


def _socket(g, path, task, top):
    """Return mouth z in panel-local coordinates."""
    if task == "bnc":
        _mount(g, path, top, .019, .027, "silver")
        g.cylinder(path+"/Nut", (0, 0, top+.0055), .0075, .005, "dark_silver")
        g.ring(path+"/Sleeve", (0, 0, top+.012), .0054, .00415, .014)
        g.ring(path+"/Insulator", (0, 0, top+.0135), .0038, .0012, .008, "white")
        g.ring(path+"/CenterContact", (0, 0, top+.0165), .0013, .00065, .005, "gold", 24)
        for sign in (-1, 1):
            g.cylinder(path+f"/Bayonet_{sign+1}", (0, sign*.0058, top+.012), .0013, .003, "silver", "Y")
        return top+.019
    if task == "outlet":
        g.round_box(path+"/Mount", (0, 0, top+.003), (.064, .062, .006), .009)
        g.cylinder(path+"/Housing", (0, 0, top+.013), .025, .020, "white")
        g.ring(path+"/Cup", (0, 0, top+.032), .026, .019, .018, "white")
        g.cylinder(path+"/Well", (0, 0, top+.0255), .019, .001, "white_edge")
        for sign in (-1, 1):
            g.cylinder(path+f"/PinHole_{sign+1}", (0, sign*.0095, top+.0262), .0025, .0003, "cavity")
            g.box(path+f"/GroundContact_{sign+1}", (sign*.018, 0, top+.033), (.0014, .006, .010), "silver")
            g.screw(path+f"/MountScrew_{sign+1}", (sign*.027, 0, top+.0067), .0022)
        return top+.041
    if task == "audio":
        _mount(g, path, top, .017, .023)
        g.round_box(path+"/Tower", (0, 0, top+.016), (.012, .014, .025), .0016, "black")
        g.cylinder(path+"/Hole", (0, 0, top+.029), .0021, .0008, "cavity")
        g.ring(path+"/JackRim", (0, 0, top+.030), .0035, .0019, .003)
        return top+.0315
    if task == "usb":
        _mount(g, path, top, .018, .031)
        g.round_box(path+"/Tower", (0, 0, top+.019), (.018, .013, .032), .0015, "black")
        mouth = top+.041
        g.rect_ring(path+"/TopRim", (0, 0, mouth-.002), (.018, .013), (.014, .0068), .008, "black")
        g.box(path+"/Well", (0, 0, mouth-.004), (.013, .006, .001), "cavity")
        g.rect_ring(path+"/PortShell", (0, 0, mouth-.001), (.014, .0068), (.0126, .0054), .006)
        g.box(path+"/Tongue", (0, -.0013, mouth-.0015), (.0105, .0016, .004), "black_rubber")
        for i in range(4):
            g.box(path+f"/Contact_{i}", ((i-1.5)*.0024, -.00042, mouth-.001), (.0011, .00015, .003), "gold")
        return mouth+.002
    if task == "lan":
        _mount(g, path, top, .022, .029)
        g.round_box(path+"/Tower", (0, 0, top+.019), (.018, .019, .034), .0017, "black")
        mouth = top+.040
        g.box(path+"/Well", (0, 0, mouth-.004), (.013, .014, .001), "cavity")
        g.rect_ring(path+"/Port", (0, 0, mouth-.0005), (.017, .018), (.012, .013), .006, "black")
        g.box(path+"/ClipNotch", (0, -.0072, mouth+.0001), (.006, .0028, .001), "cavity")
        for i in range(8):
            g.box(path+f"/Contact_{i}", ((i-3.5)*.0011, .004, mouth-.002), (.0005, .004, .0006), "gold")
        return mouth+.0025
    if task == "hdmi":
        _mount(g, path, top, .022, .038)
        g.round_box(path+"/Tower", (0, 0, top+.019), (.025, .018, .031), .002, "black")
        mouth = top+.040
        g.rect_ring(path+"/TopRim", (0, 0, mouth-.002), (.025, .018), (.016, .007), .006, "black")
        # Type-A HDMI outline: a wide edge and a clipped opposite edge.
        outer = [(-.0075, -.003), (.0075, -.003), (.0075, .001), (.0054, .003), (-.0054, .003), (-.0075, .001)]
        inner = [(x*.86, y*.72) for x, y in outer]
        n = len(outer)
        pts = [(x, y, z) for z in (mouth-.004, mouth+.001) for poly in (outer, inner) for x, y in poly]
        faces = []
        for i in range(n):
            j = (i+1) % n
            faces.extend([[i, j, j+2*n, i+2*n], [i+n, i+3*n, j+3*n, j+n],
                          [i+2*n, j+2*n, j+3*n, i+3*n]])
        g.mesh(path+"/PortShell", pts, faces, "silver")
        g.prism(path+"/Well", inner, mouth-.005, mouth-.0045, "cavity")
        g.box(path+"/Tongue", (0, 0, mouth-.002), (.009, .0012, .004), "black")
        for i in range(10):
            g.box(path+f"/Contact_{i}", ((i-4.5)*.0008, -.00067, mouth-.001), (.00035, .00015, .002), "gold")
        return mouth+.001
    raise ValueError(f"Unknown insertion task: {task}")


def _rotation(g, panel_path, moving_path, task, panel_world, top):
    """Create static mount and independent world-space moving root."""
    x, y, z = panel_world
    if task == "rotary":
        g.round_box(panel_path+"/BlueSwitchBase", (0, 0, top+.003), (.031, .031, .006), .0015, "blue")
        g.cylinder(panel_path+"/SwitchBody", (0, 0, top+.011), .013, .014, "black")
        g.ring(panel_path+"/SilverCollar", (0, 0, top+.018), .014, .0095, .007)
        for i in range(18):
            a = 2*math.pi*i/18
            g.box(panel_path+f"/Knurl_{i}", (.0136*math.cos(a), .0136*math.sin(a), top+.018), (.0012, .0012, .005), "dark_silver")
        origin = [x, y, z+top+.022]
        g.xform(moving_path, origin, 0)
        g.cylinder(moving_path+"/Cap", (0, 0, 0), .010, .006, "black")
        g.round_box(moving_path+"/Lever", (.007, 0, .007), (.031, .010, .012), .002, "black")
        g.round_box(moving_path+"/LeverEdge", (.007, -.0048, .009), (.025, .0008, .005), .0003, "silver")
        g.box(moving_path+"/Indicator", (.017, 0, .0132), (.005, .0015, .0006), "white")
        # Tick marks corresponding to the configured OFF/ON detents (45/105 deg).
        for i, angle in enumerate((45, 105)):
            a = math.radians(angle)
            g.box(panel_path+f"/PositionMark_{i}", (.021*math.cos(a), .021*math.sin(a), top+.0004), (.003, .001, .0007), "black")
    elif task == "latch":
        # A two-plate rotating gate latch: fixed catch and rotating sweep handle.
        for i, yy in enumerate((-.018, .018)):
            g.round_box(panel_path+f"/Plate_{i}", (0, yy, top+.0025), (.062, .022, .005), .005, "black")
            for j, xx in enumerate((-.023, .023)):
                g.screw(panel_path+f"/PlateScrew_{i}_{j}", (xx, yy, top+.0057), .0027)
        g.round_box(panel_path+"/FixedCatch", (-.006, .018, top+.010), (.026, .017, .010), .003, "black")
        g.box(panel_path+"/CatchOpening", (-.006, .010, top+.010), (.014, .001, .006), "cavity")
        g.round_box(panel_path+"/MechanicalStop", (-.006, -.036, top+.009), (.016, .009, .012), .002, "black")
        origin = [x-.006, y-.018, z+top+.010]
        g.xform(moving_path, origin, 0)
        g.cylinder(moving_path+"/Pivot", (0, 0, 0), .0095, .010, "black")
        g.round_box(moving_path+"/SweepArm", (0, .015, .005), (.009, .041, .008), .004, "black")
        g.round_box(moving_path+"/Grip", (0, .013, .012), (.012, .027, .012), .005, "black")
        g.cylinder(moving_path+"/PivotPin", (0, 0, .006), .0025, .001, "dark_silver")
    elif task == "locker":
        g.round_box(panel_path+"/MountPlate", (0, 0, top+.002), (.054, .047, .004), .007, "silver")
        g.round_box(panel_path+"/Recess", (0, 0, top+.0042), (.044, .035, .001), .009, "dark_silver")
        for sign in (-1, 1):
            g.screw(panel_path+f"/MountScrew_{sign+1}", (sign*.022, 0, top+.0048), .002)
        g.round_box(panel_path+"/LatchTongue", (.027, 0, top+.001), (.023, .018, .005), .002, "dark_silver")
        origin = [x, y, z+top+.009]
        g.xform(moving_path, origin, 0)
        g.cylinder(moving_path+"/Hub", (0, 0, 0), .007, .009, "dark_silver")
        g.round_box(moving_path+"/HandleLeft", (-.014, 0, .003), (.006, .026, .008), .002, "silver")
        g.round_box(moving_path+"/HandleRight", (.014, 0, .003), (.006, .026, .008), .002, "silver")
        g.round_box(moving_path+"/HandleBridge", (0, .010, .004), (.028, .007, .010), .003, "silver")
        g.round_box(moving_path+"/HandleCenter", (0, -.008, .001), (.025, .008, .006), .002, "dark_silver")
    else:
        raise ValueError(f"Unknown rotation task: {task}")
    return origin


def _plug(g, path, task):
    """Plug root is its grasp centre, with insertion end directed down (-Z).

    Return the distance from grasp centre to insertion tip and how far the tip
    enters the socket mouth in the seated pose.
    """
    if task == "outlet":
        g.round_box(path+"/Grip", (0, 0, 0), (.031, .034, .036), .007, "black")
        g.round_box(path+"/Face", (0, 0, -.020), (.028, .031, .005), .006, "black")
        for sign in (-1, 1):
            g.cylinder(path+f"/Prong_{sign+1}", (0, sign*.0095, -.031), .0023, .018, "silver")
            g.box(path+f"/Ground_{sign+1}", (sign*.014, 0, -.014), (.001, .008, .014), "silver")
        for i in range(4):
            g.round_box(path+f"/GripRib_{i}", (0, 0, -.007+i*.005), (.0318, .0348, .0013), .007, "black_rubber")
        return .040, .018
    if task == "audio":
        g.cylinder(path+"/Grip", (0, 0, 0), .0045, .022, "silver")
        g.cylinder(path+"/CableStub", (0, 0, .014), .0025, .006, "white")
        g.cylinder(path+"/Pin", (0, 0, -.019), .00175, .017, "silver")
        for i in range(2):
            g.ring(path+f"/ContactSeparator_{i}", (0, 0, -.019-i*.005), .0018, .0015, .0008, "black", 24)
        for i in range(6):
            g.ring(path+f"/GripRib_{i}", (0, 0, -.007+i*.0027), .00465, .0043, .0005, "dark_silver", 32)
        return .0275, .014
    if task == "bnc":
        g.cylinder(path+"/Grip", (0, 0, .002), .0065, .022, "silver")
        g.cylinder(path+"/Sleeve", (0, 0, -.012), .0073, .010, "silver")
        g.ring(path+"/Mouth", (0, 0, -.018), .007, .0055, .006, "silver")
        g.cylinder(path+"/CenterPin", (0, 0, -.017), .0008, .008, "gold")
        for i in range(20):
            a = i*math.pi/10
            g.cylinder(path+f"/Knurl_{i}", (.0064*math.cos(a), .0064*math.sin(a), .002), .00045, .018, "dark_silver")
        return .021, .008
    if task == "usb":
        g.round_box(path+"/Grip", (0, 0, 0), (.016, .009, .027), .002, "white")
        g.round_box(path+"/Shoulder", (0, 0, -.011), (.017, .010, .004), .001, "white_edge")
        g.rect_ring(path+"/TipShell", (0, 0, -.019), (.012, .0045), (.0108, .0033), .013, "silver")
        g.box(path+"/Tongue", (0, -.0008, -.019), (.010, .0014, .011), "black")
        g.box(path+"/Stripe", (0, -.0046, .004), (.009, .0005, .005), "blue_stripe")
        return .0255, .010
    if task == "lan":
        g.round_box(path+"/Grip", (0, 0, .002), (.012, .014, .027), .002, "white")
        g.round_box(path+"/PlugBody", (0, 0, -.016), (.011, .012, .016), .0008, "white_edge")
        g.round_box(path+"/LockingClip", (0, -.007, -.010), (.005, .0013, .017), .0004, "white")
        for i in range(8):
            g.box(path+f"/Contact_{i}", ((i-3.5)*.0011, .0059, -.019), (.0006, .00025, .009), "gold")
        for i in range(3):
            g.round_box(path+f"/StrainRib_{i}", (0, 0, .007+i*.003), (.0128, .0148, .001), .002, "white_edge")
        return .024, .014
    if task == "hdmi":
        g.round_box(path+"/Grip", (0, 0, 0), (.021, .011, .031), .003, "black")
        outer = [(-.007, -.0025), (.007, -.0025), (.007, .001), (.005, .0027), (-.005, .0027), (-.007, .001)]
        g.prism(path+"/Tip", outer, -.027, -.013, "silver")
        g.box(path+"/ContactInset", (0, 0, -.0272), (.010, .0024, .0003), "black")
        for i in range(10):
            g.box(path+f"/Contact_{i}", ((i-4.5)*.0008, 0, -.0274), (.00035, .0015, .0002), "gold")
        g.round_box(path+"/GripInset", (0, -.0056, .001), (.013, .0005, .018), .0002, "black_rubber")
        return .027, .011
    raise ValueError(task)


def build_scene(stage, config: dict[str, Any] | None = None) -> dict[str, Any]:
    """Build independent board modules and six snap-ready cable-free plugs.

    ``layout`` is three rows from robot/back to operator/front, +x. Columns
    run from -y to +y. All returned positions are world-space. Socket positions
    are mouth centres; plug_insert_positions are seated grasp-centre positions.
    Panel relocation is performed by rebuilding with another layout, keeping
    each task's socket, rotation mount, and moving part together.
    """
    config = config or {}
    layout = config.get("layout", DEFAULT_LAYOUT)
    tasks = [task for row in layout for task in row]
    if len(layout) != 3 or any(len(row) != 3 for row in layout) or set(tasks) != set(INSERTION_TASKS+ROTATION_TASKS) or len(tasks) != 9:
        raise ValueError("layout must be a 3×3 arrangement containing each of the nine tasks exactly once")
    if stage.GetPrimAtPath(ROOT).IsValid():
        stage.RemovePrim(ROOT)
    UsdGeom.Xform.Define(stage, ROOT)
    g = _Geometry(stage)
    center = config.get("board_center", [.44, 0.])
    panel_size = float(config.get("panel_size", .09))
    panel_height = float(config.get("panel_height", .035))
    gap = float(config.get("panel_gap", .001))
    pitch = panel_size+gap
    base_height = float(config.get("baseplate_height", .008))
    board_width = 3*panel_size+2*gap
    for group in ("Environment", "Panels", "Movables", "Plugs"):
        UsdGeom.Xform.Define(stage, ROOT+"/"+group)
    env = ROOT+"/Environment"
    table_back, table_front, table_width = -.08, .665, .80
    g.round_box(env+"/Table", ((table_back+table_front)/2, 0, -.021),
                (table_front-table_back, table_width, .042), .007, "table", True)
    # Keep the apron below the tabletop's bottom face. Overlapping their two
    # coplanar front faces produces visible z-fighting in RTX at small scales.
    g.box(env+"/FrontApron", (table_front-.004, 0, -.088), (.008, .79, .090), "silver")
    for iy, yy in enumerate((-.32, .32)):
        g.cylinder(env+f"/ApronBolt_{iy}", (table_front+.001, yy, -.060), .004, .002, "dark_silver", "X")
    # The wide black clamp visible across the middle of the real table edge.
    g.round_box(env+"/ClampTop", (table_front-.027, 0, .0035), (.068, .080, .007), .004, "black_rubber")
    profile = []
    for i in range(9):
        angle = math.radians(90-i*90/8)
        profile.append((table_front+.018*math.cos(angle), -.014+.018*math.sin(angle)))
    profile.extend([(table_front+.018, -.078), (table_front+.010, -.078)])
    for i in range(9):
        angle = math.radians(i*90/8)
        profile.append((table_front+.010*math.cos(angle), -.014+.010*math.sin(angle)))
    points = [(x, y, z) for y in (-.040, .040) for x, z in profile]
    count = len(profile)
    faces = [list(range(count)), list(reversed(range(count, 2*count)))]
    faces.extend([[i, i+count, (i+1) % count+count, (i+1) % count] for i in range(count)])
    g.mesh(env+"/ClampBend", points, faces, "black_rubber")
    for iy, yy in enumerate((-.026, .026)):
        g.screw(env+f"/ClampBolt_{iy}", (table_front-.047, yy, .0077), .0031)
    # Small stainless front fixture, estimated from the external-camera image.
    fixture_y = -.245
    g.box(env+"/FrontFixtureBack", (table_front+.009, fixture_y, -.061), (.008, .102, .064), "silver")
    g.box(env+"/FrontFixtureShelf", (table_front+.029, fixture_y, -.091), (.041, .102, .005), "silver")
    for iy, sign in enumerate((-1, 1)):
        g.box(env+f"/FrontFixtureCheek_{iy}", (table_front+.028, fixture_y+sign*.049, -.063), (.038, .005, .054), "silver")
    g.cylinder(env+"/FrontFixtureAxle", (table_front+.030, fixture_y, -.034), .005, .098, "dark_silver", "Y")
    g.box(env+"/FrontFixtureLever", (table_front+.038, fixture_y, -.065), (.010, .018, .039), "dark_silver")
    for ix, x in enumerate((-.04, .61)):
        for iy, y in enumerate((-.34, .34)):
            g.box(env+f"/TableLeg_{ix}_{iy}", (x, y, -.37), (.05, .05, .70), "steel")
    # The source column continues above the external-camera view.  Its top
    # follows the robot's configured base height so robot and support agree.
    pedestal_top = float(config.get("robot_base_m", [0., 0., .30])[2])
    if pedestal_top <= .025:
        raise ValueError("robot_base_m z must leave room for the pedestal plates")
    g.round_box(env+"/RobotBasePlate", (0, 0, .008), (.22, .22, .016), .006, "steel", True)
    g.box(env+"/RobotPedestal", (0, 0, (.016+pedestal_top-.006)/2), (.12, .12, pedestal_top-.022), "steel", True)
    g.round_box(env+"/RobotTopPlate", (0, 0, pedestal_top-.003), (.16, .16, .006), .004, "steel", True)
    for ix, x in enumerate((-.088, .088)):
        for iy, y in enumerate((-.088, .088)):
            g.screw(env+f"/PedestalBolt_{ix}_{iy}", (x, y, .017), .0045, "dark_silver")
    for i in range(5):
        g.box(env+f"/RearTableRail_{i}", (-.068+i*.011, 0, .0007), (.002, .76, .0014), "dark_silver")
    if config.get("background", True):
        g.box(env+"/RearWall", (-.27, 0, .43), (.015, 1.20, 1.1), "wall")
        g.box(env+"/LeftPartition", (.10, -.49, .43), (1.0, .025, 1.1), "partition")
        g.cable(env+"/RearCable", [(-.255, -.18, .80), (-.254, -.18, .57),
                                  (-.245, -.17, .38), (-.19, -.16, .20),
                                  (-.18, -.15, .08), (-.10, -.11, .03), (.01, -.09, .025)])
        g.cable(env+"/SideCable", [(-.13, .26, .60), (-.11, .28, .40),
                                  (-.04, .31, .25), (.08, .36, .14),
                                  (.29, .405, .05), (.48, .42, -.02), (.59, .42, -.27)])
    g.round_box(env+"/BoardBaseplate", (center[0], center[1], base_height/2),
                (board_width+.016, board_width+.016, base_height), .005, "white", True)
    for ix in (-1, 1):
        for iy in (-1, 1):
            g.screw(env+f"/BaseplateBolt_{ix+1}_{iy+1}",
                    (center[0]+ix*(board_width/2+.004), center[1]+iy*(board_width/2+.004), base_height+.0007), .0022)
    handles = {key: {} for key in ("panel_paths", "plug_paths", "socket_positions", "plug_home_positions",
                                   "plug_insert_positions", "rotary_paths", "rotary_origins", "panel_positions")}
    handles["rotation_axis"] = "Z"
    handles["root_path"] = ROOT
    # This slider stays under its panel so relocation and fixture yaw carry
    # the handle housing and bolt together. The controller drives local -X.
    handles["locker_bolt_path"] = ROOT+"/Panels/locker/LatchTongue"
    handles["layout"] = [list(row) for row in layout]
    for row_index, row in enumerate(layout):
        for column_index, task in enumerate(row):
            world = [float(center[0])+(row_index-1)*pitch, float(center[1])+(column_index-1)*pitch, base_height]
            panel = f"{ROOT}/Panels/{task}"
            g.xform(panel, world)
            # Adjacent modules may have zero configured spacing. A 0.3 mm
            # inset cap still exposes their seams while preserving 9×9×3.5 cm.
            cap_height = .001
            g.round_box(panel+"/Board", (0, 0, (panel_height-cap_height)/2),
                        (panel_size, panel_size, panel_height-cap_height), .0012, "white_side", True)
            g.round_box(panel+"/TopSkin", (0, 0, panel_height-cap_height/2),
                        (panel_size-.0006, panel_size-.0006, cap_height), .0012, "white")
            prim = stage.GetPrimAtPath(panel)
            prim.CreateAttribute("taskboard:task", Sdf.ValueTypeNames.String).Set(task)
            prim.CreateAttribute("taskboard:slot", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(row_index, column_index))
            handles["panel_paths"][task] = panel
            handles["panel_positions"][task] = world
            if task in INSERTION_TASKS:
                mouth_z = _socket(g, panel, task, panel_height)
                handles["socket_positions"][task] = [world[0], world[1], world[2]+mouth_z]
            else:
                movable = f"{ROOT}/Movables/{task}"
                origin = _rotation(g, panel, movable, task, world, panel_height)
                handles["rotary_paths"][task] = movable
                handles["rotary_origins"][task] = origin
    rack_center = config.get("rack_center", [float(center[0]), float(center[1])-.20])
    rack = env+"/PlugRack"
    rack_top = .025
    g.round_box(rack, (rack_center[0], rack_center[1], rack_top/2), (.277, .058, rack_top), .0035, "white", True)
    # Narrow raised back edge and shallow channel visible alongside the pegs.
    g.round_box(rack+"BackRail", (rack_center[0], rack_center[1]+.024, rack_top+.005), (.270, .006, .010), .0012, "white")
    g.round_box(rack+"Channel", (rack_center[0], rack_center[1]-.019, rack_top+.0002), (.245, .005, .0004), .002, "white_edge")
    rack_order = config.get("rack_order", ["outlet", "hdmi", "lan", "usb", "audio", "bnc"])
    if len(rack_order) != 6 or set(rack_order) != set(INSERTION_TASKS):
        raise ValueError("rack_order must contain all six insertion tasks exactly once")
    for i, task in enumerate(rack_order):
        path = f"{ROOT}/Plugs/{task}"
        x = float(rack_center[0])+(i-2.5)*.041
        y = float(rack_center[1])
        g.xform(path, (0, 0, 0), 0)
        tip_offset, insertion_depth = _plug(g, path, task)
        home = [x, y, rack_top+tip_offset-.004]
        UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0].Set(Gf.Vec3d(*home))
        stage.GetPrimAtPath(path).CreateAttribute("taskboard:task", Sdf.ValueTypeNames.String).Set(task)
        stage.GetPrimAtPath(path).CreateAttribute("taskboard:tipOffset", Sdf.ValueTypeNames.Double).Set(tip_offset)
        handles["plug_paths"][task] = path
        handles["plug_home_positions"][task] = home
        mouth = handles["socket_positions"][task]
        handles["plug_insert_positions"][task] = [mouth[0], mouth[1], mouth[2]+tip_offset-insertion_depth]
        if task in ("audio", "bnc"):
            g.ring(rack+f"Holder_{task}", (x, y, rack_top+.0008), .008 if task == "bnc" else .004, .006 if task == "bnc" else .002, .0012, "white_edge")
        else:
            g.round_box(rack+f"Holder_{task}", (x, y, rack_top+.0003), (.018, .015, .0006), .002, "white_edge")
    # Fixture orientation is independent of the square module slot. The movable
    # handles live outside their static panels, so rotate their pivots too.
    handles["fixture_yaw_deg"] = {}
    for task, angle in config.get("fixture_yaw_deg", {}).items():
        if task not in handles["panel_paths"]:
            raise ValueError("unknown fixture orientation task: "+task)
        angle = float(angle)
        if not math.isfinite(angle):
            raise ValueError("fixture yaw must be finite")
        handles["fixture_yaw_deg"][task] = angle
        UsdGeom.Xformable(stage.GetPrimAtPath(handles["panel_paths"][task])).AddRotateZOp().Set(angle)
        if task in handles["rotary_paths"]:
            center = handles["panel_positions"][task]
            origin = handles["rotary_origins"][task]
            a = math.radians(angle);dx,dy=origin[0]-center[0],origin[1]-center[1]
            changed=[center[0]+math.cos(a)*dx-math.sin(a)*dy,
                     center[1]+math.sin(a)*dx+math.cos(a)*dy,origin[2]]
            ops=UsdGeom.Xformable(stage.GetPrimAtPath(handles["rotary_paths"][task])).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*changed));ops[1].Set(angle)
            handles["rotary_origins"][task]=changed
    return handles
