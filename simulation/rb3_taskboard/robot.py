"""Manufacturer RB3 meshes with URDF kinematics and an estimated 10 cm gripper.

All transforms returned to callers use column-vector homogeneous convention.
Joint values are radians and Cartesian dimensions are metres. This intentionally
kinematic model is for scene reconstruction and semantic interaction, not torque
control, collision-safe planning, or a calibrated physical gripper model.

Import this module after Isaac's SimulationApp if using it inside Isaac Sim.
It also works in the project's separate OpenUSD inspection environment.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")


def _vector(text, default=(0, 0, 0)):
    return np.asarray(default if text is None else [float(x) for x in text.split()], dtype=float)


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def _origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = _vector(element.get("xyz"))
        r, p, y = _vector(element.get("rpy"))
        transform[:3, :3] = _rotation((0, 0, 1), y) @ _rotation((0, 1, 0), p) @ _rotation((1, 0, 0), r)
    return transform


def _rotation_log(matrix):
    """Stable SO(3) logarithm, including targets approximately 180 degrees away."""
    cosine = np.clip((np.trace(matrix) - 1) / 2, -1, 1)
    angle = float(np.arccos(cosine))
    vee = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if angle < 1e-7:
        return vee / 2
    if math.pi - angle < 1e-5:
        values, vectors = np.linalg.eigh((matrix + np.eye(3)) / 2)
        axis = vectors[:, np.argmax(values)]
        if np.dot(axis, vee) < 0:
            axis = -axis
        return axis * angle
    return vee * (angle / (2 * math.sin(angle)))


def _material(stage, path, color, metallic=0.0, roughness=0.4):
    from pxr import Gf, Sdf, UsdShade
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, color[:3])))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _set_matrix(operation, transform):
    from pxr import Gf
    operation.Set(Gf.Matrix4d(np.asarray(transform, dtype=float).T.tolist()))


def _dae_nodes(root):
    """Yield each geometry instance and its local-to-link matrix."""
    def visit(node, parent):
        transform = parent.copy()
        for child in node:
            tag = child.tag.rsplit("}", 1)[-1]
            if tag == "matrix":
                # COLLADA textual matrices use column-vector equations with
                # the 16 entries printed row by row, as Blender exports them.
                transform = transform @ np.fromstring(child.text, sep=" ").reshape(4, 4)
            elif tag == "translate":
                matrix = np.eye(4)
                matrix[:3, 3] = np.fromstring(child.text, sep=" ")
                transform = transform @ matrix
            elif tag == "rotate":
                values = np.fromstring(child.text, sep=" ")
                matrix = np.eye(4)
                matrix[:3, :3] = _rotation(values[:3], math.radians(values[3]))
                transform = transform @ matrix
            elif tag == "scale":
                transform = transform @ np.diag([*np.fromstring(child.text, sep=" "), 1])
        for instance in node.findall("c:instance_geometry", NS):
            materials = {entry.get("symbol"): entry.get("target").lstrip("#")
                         for entry in instance.findall("c:bind_material/c:technique_common/c:instance_material", NS)}
            yield instance.get("url").lstrip("#"), transform, materials
        for child in node.findall("c:node", NS):
            yield from visit(child, transform)

    for node in root.findall("c:library_visual_scenes/c:visual_scene/c:node", NS):
        yield from visit(node, np.eye(4))


def _load_dae(stage, path, filename):
    """Import the pinned Blender COLLADA meshes, including authored normals."""
    from pxr import Gf, UsdGeom, UsdShade, Vt
    root = ET.parse(filename).getroot()
    unit = root.find("c:asset/c:unit", NS)
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    up_axis = root.findtext("c:asset/c:up_axis", default="Z_UP", namespaces=NS)
    if up_axis != "Z_UP":
        raise ValueError(f"Unexpected up axis in manufacturer asset: {up_axis}")
    effects = {}
    for effect in root.findall("c:library_effects/c:effect", NS):
        color = effect.find(".//c:diffuse/c:color", NS)
        effects[effect.get("id")] = np.fromstring(color.text, sep=" ")[:3] if color is not None else np.array([.65, .65, .65])
    materials = {}
    for index, entry in enumerate(root.findall("c:library_materials/c:material", NS)):
        effect = entry.find("c:instance_effect", NS).get("url").lstrip("#")
        color = effects[effect]
        materials[entry.get("id")] = _material(stage, f"{path}/Materials/M{index}", color, metallic=.18)
    geometry = {entry.get("id"): entry for entry in root.findall("c:library_geometries/c:geometry", NS)}
    mesh_count = triangle_count = 0
    bounds = []
    for instance_index, (identifier, transform, material_map) in enumerate(_dae_nodes(root)):
        mesh = geometry[identifier].find("c:mesh", NS)
        arrays = {}
        for source in mesh.findall("c:source", NS):
            accessor = source.find("c:technique_common/c:accessor", NS)
            stride = int(accessor.get("stride", "1"))
            arrays[source.get("id")] = np.fromstring(source.find("c:float_array", NS).text, sep=" ").reshape(-1, stride)
        vertices = {entry.get("id"): entry.find("c:input[@semantic='POSITION']", NS).get("source").lstrip("#")
                    for entry in mesh.findall("c:vertices", NS)}
        primitives = mesh.findall("c:triangles", NS)
        if mesh.findall("c:polylist", NS) or mesh.findall("c:polygons", NS):
            raise ValueError(f"Nontriangulated manufacturer geometry in {filename}")
        for primitive_index, triangles in enumerate(primitives):
            inputs = triangles.findall("c:input", NS)
            stride = max(int(entry.get("offset", "0")) for entry in inputs) + 1
            indices = np.fromstring(triangles.find("c:p", NS).text, sep=" ", dtype=np.int32).reshape(-1, stride)
            vertex_input = next(entry for entry in inputs if entry.get("semantic") == "VERTEX")
            positions = arrays[vertices[vertex_input.get("source").lstrip("#")]][:, :3]
            positions = (positions @ transform[:3, :3].T + transform[:3, 3]) * scale
            vertex_indices = indices[:, int(vertex_input.get("offset", "0"))]
            count = len(vertex_indices) // 3
            usd_mesh = UsdGeom.Mesh.Define(stage, f"{path}/Part{instance_index}_{primitive_index}")
            usd_mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(positions.astype(np.float32)))
            usd_mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(count, 3, dtype=np.int32)))
            usd_mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(vertex_indices.astype(np.int32)))
            usd_mesh.CreateSubdivisionSchemeAttr("none")
            usd_mesh.CreateDoubleSidedAttr(False)
            minimum, maximum = positions.min(axis=0), positions.max(axis=0)
            usd_mesh.CreateExtentAttr([Gf.Vec3f(*minimum.tolist()), Gf.Vec3f(*maximum.tolist())])
            normal_input = next((entry for entry in inputs if entry.get("semantic") == "NORMAL"), None)
            if normal_input is not None:
                normals = arrays[normal_input.get("source").lstrip("#")][:, :3]
                normals = normals[indices[:, int(normal_input.get("offset", "0"))]] @ np.linalg.inv(transform[:3, :3])
                normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
                usd_mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
                usd_mesh.SetNormalsInterpolation("faceVarying")
            symbol = triangles.get("material")
            if symbol in material_map:
                UsdShade.MaterialBindingAPI.Apply(usd_mesh.GetPrim()).Bind(materials[material_map[symbol]])
            bounds.append([minimum.tolist(), maximum.tolist()])
            mesh_count += 1
            triangle_count += count
    return {"source": str(filename), "meshes": mesh_count, "triangles": triangle_count, "bounds": bounds}


class Robot:
    """Kinematic RB3 arm; semantic grasps are owned by the task controller."""
    joint_names = JOINT_NAMES

    def __init__(self, stage, asset_dir, base_position=(0, 0, 0.12), path="/World/RB3"):
        from pxr import Gf, UsdGeom
        self.stage = stage
        self.path = str(path)
        self.asset_dir = Path(asset_dir).resolve()
        if not (self.asset_dir / "robots/rb3_730es_u.urdf").exists():
            if (self.asset_dir / "rbpodo_description/robots/rb3_730es_u.urdf").exists():
                self.asset_dir /= "rbpodo_description"
            else:
                raise FileNotFoundError(f"Run fetch_robot.py first; URDF missing from {self.asset_dir}")
        self.urdf = ET.parse(self.asset_dir / "robots/rb3_730es_u.urdf").getroot()
        self.base_position = np.asarray(base_position, dtype=float)
        self.base_transform = np.eye(4)
        self.base_transform[:3, 3] = self.base_position
        self.joints = []
        for name in JOINT_NAMES:
            element = self.urdf.find(f"joint[@name='{name}']")
            limit = element.find("limit")
            self.joints.append({"name": name, "origin": _origin(element.find("origin")),
                                "axis": _vector(element.find("axis").get("xyz")),
                                "parent": element.find("parent").get("link"),
                                "child": element.find("child").get("link"),
                                "lower": float(limit.get("lower")), "upper": float(limit.get("upper"))})
        self.lower = np.array([joint["lower"] for joint in self.joints])
        self.upper = np.array([joint["upper"] for joint in self.joints])
        # Manufacturer's fixed tcp_joint ends at the flange face; the estimated
        # gripper is placed after it, avoiding overlap with the wrist mesh.
        self.flange_transform = _origin(self.urdf.find("joint[@name='tcp_joint']/origin"))
        self.finger_length = .10
        self.palm_length = .035
        self.adapter_length = .012
        self.tool_length = self.adapter_length + self.palm_length + self.finger_length
        self.tool_transform = self.flange_transform.copy()
        self.tool_transform[:3, 3] += self.flange_transform[:3, :3] @ np.array([0, 0, self.tool_length])
        # Folded forward pose avoids the straight-arm singularity at startup.
        self.q = np.array([0.0, .4, 1.2, 0.0, 1.4, 0.0])
        root = UsdGeom.Xform.Define(stage, self.path)
        _set_matrix(root.AddTransformOp(), self.base_transform)
        root.GetPrim().SetCustomDataByKey("model", "Rainbow Robotics RB3-730ES-U")
        root.GetPrim().SetCustomDataByKey("control", "kinematic URDF FK; estimated parallel gripper")
        self.link_ops = {}
        self.asset_stats = {}
        for index in range(7):
            name = f"link{index}"
            link_path = f"{self.path}/{name}"
            self.link_ops[name] = UsdGeom.Xform.Define(stage, link_path).AddTransformOp()
            visual = self.urdf.find(f"link[@name='{name}']/visual")
            filename = visual.find("geometry/mesh").get("filename").removeprefix("package://rbpodo_description/")
            visual_path = link_path + "/Visual"
            _set_matrix(UsdGeom.Xform.Define(stage, visual_path).AddTransformOp(), _origin(visual.find("origin")))
            self.asset_stats[name] = _load_dae(stage, visual_path, self.asset_dir / filename)
        self.gripper_path = self.path + "/link6/Gripper"
        _set_matrix(UsdGeom.Xform.Define(stage, self.gripper_path).AddTransformOp(), self.flange_transform)
        self._build_gripper()
        self.tcp_path = self.gripper_path + "/TCP"
        UsdGeom.Xform.Define(stage, self.tcp_path).AddTranslateOp().Set(Gf.Vec3d(0, 0, self.tool_length))
        self.set_joints(self.q)
        self.set_gripper(.065)

    def _build_gripper(self):
        from pxr import Gf, UsdGeom, UsdShade
        metal = _material(self.stage, self.gripper_path + "/Materials/Metal", (.38, .40, .43), metallic=.75, roughness=.28)
        black = _material(self.stage, self.gripper_path + "/Materials/Black", (.025, .03, .035), metallic=.2)
        pad = _material(self.stage, self.gripper_path + "/Materials/Pad", (.075, .075, .07), roughness=.85)

        def box(name, center, size, material):
            cube = UsdGeom.Cube.Define(self.stage, self.gripper_path + "/" + name)
            cube.CreateSizeAttr(1)
            translation = cube.AddTranslateOp()
            translation.Set(Gf.Vec3d(*map(float, center)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
            return translation

        cylinder = UsdGeom.Cylinder.Define(self.stage, self.gripper_path + "/FlangeAdapter")
        cylinder.CreateRadiusAttr(.026)
        cylinder.CreateHeightAttr(self.adapter_length)
        cylinder.CreateAxisAttr("Z")
        cylinder.AddTranslateOp().Set(Gf.Vec3d(0, 0, self.adapter_length / 2))
        UsdShade.MaterialBindingAPI.Apply(cylinder.GetPrim()).Bind(metal)
        box("Palm", (0, 0, self.adapter_length + self.palm_length / 2), (.083, .038, self.palm_length), black)
        self._finger_ops = []
        self._pad_ops = []
        finger_z = self.adapter_length + self.palm_length + self.finger_length / 2
        for side in (-1, 1):
            name = "Left" if side < 0 else "Right"
            self._finger_ops.append((side, box(name + "Finger", (side * .03, 0, finger_z), (.009, .021, self.finger_length), metal)))
            self._pad_ops.append((side, box(name + "Pad", (side * .023, 0, self.tool_length - .015), (.004, .024, .030), pad)))

    def set_gripper(self, opening_m):
        """Set the clear distance between the two inner fingertip pads, [0, .075]."""
        from pxr import Gf
        opening = float(opening_m)
        if not np.isfinite(opening) or not 0 <= opening <= .075:
            raise ValueError("Gripper opening must be between 0 and 0.075 metres")
        self.opening_m = opening
        finger_z = self.adapter_length + self.palm_length + self.finger_length / 2
        for side, operation in self._finger_ops:
            operation.Set(Gf.Vec3d(side * (opening / 2 + .004 + .0045), 0, finger_z))
        for side, operation in self._pad_ops:
            operation.Set(Gf.Vec3d(side * (opening / 2 + .002), 0, self.tool_length - .015))

    def _fk(self, q, world=True):
        transforms = {"link0": self.base_transform.copy() if world else np.eye(4)}
        joint_origins, joint_axes = [], []
        for index, joint in enumerate(self.joints):
            transform = transforms[joint["parent"]] @ joint["origin"]
            joint_origins.append(transform[:3, 3].copy())
            joint_axes.append(transform[:3, :3] @ joint["axis"])
            motion = np.eye(4)
            motion[:3, :3] = _rotation(joint["axis"], q[index])
            transforms[joint["child"]] = transform @ motion
        return transforms, np.asarray(joint_origins), np.asarray(joint_axes)

    def set_joints(self, q6):
        q = np.asarray(q6, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("Expected six finite joint angles in radians")
        if np.any(q < self.lower - 1e-6) or np.any(q > self.upper + 1e-6):
            raise ValueError("Joint angle exceeds manufacturer URDF limits")
        self.q = np.clip(q, self.lower, self.upper)
        transforms, _, _ = self._fk(self.q, world=False)
        for name, transform in transforms.items():
            _set_matrix(self.link_ops[name], transform)

    def get_tcp_pose(self):
        transforms, _, _ = self._fk(self.q)
        return transforms["link6"] @ self.tool_transform

    def solve_ik(self, position, rotation=None, seed=None, max_iterations=220):
        """Return a checked numerical IK solution without changing the scene.

        rotation is a 3x3 world orientation, or None to retain the current TCP
        orientation. Multiple deterministic seeds handle arm branch changes.
        This solves reachability only and does not check robot collisions.
        """
        position = np.asarray(position, dtype=float)
        rotation = self.get_tcp_pose()[:3, :3] if rotation is None else np.asarray(rotation, dtype=float)
        if position.shape != (3,) or rotation.shape != (3, 3) or not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("Target requires a finite 3-vector and 3x3 rotation")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < .999:
            raise ValueError("Target orientation must be a proper rotation matrix")
        initial = self.q.copy() if seed is None else np.asarray(seed, dtype=float)
        if initial.shape != (6,):
            raise ValueError("IK seed must contain six joints")
        azimuth = math.atan2(*(position - self.base_position)[[1, 0]])
        # The URDF bounds each wrist angle to [-3.14, 3.14]. A target across
        # that boundary can be reachable on the opposite wrist branch even
        # when a local solve stalls against +pi/-pi. Keep the arm posture and
        # explicitly try both wrist3 branches before changing elbow branch.
        wrist_negative, wrist_positive = initial.copy(), initial.copy()
        wrist_negative[5], wrist_positive[5] = -2.8, 2.8
        seeds = [initial, wrist_negative, wrist_positive,
                 np.array([azimuth, .6, 1.3, 0, 1.2, 0]),
                 np.array([azimuth, 1.2, -1.3, 0, 1.2, 0]),
                 np.array([azimuth, -.6, -1.3, 0, -1.2, 0]),
                 np.array([azimuth, .6, 1.3, -2.6, -1.2, 0]),
                 np.array([azimuth, .6, 1.3, 2.6, -1.2, 0])]
        best = None
        orientation_weight = .15
        for candidate in seeds:
            q = np.clip(candidate, self.lower + 1e-7, self.upper - 1e-7)
            for iteration in range(max_iterations):
                transforms, origins, axes = self._fk(q)
                pose = transforms["link6"] @ self.tool_transform
                position_error = position - pose[:3, 3]
                rotation_error = _rotation_log(rotation @ pose[:3, :3].T)
                distance, angle = float(np.linalg.norm(position_error)), float(np.linalg.norm(rotation_error))
                score = distance + orientation_weight * angle
                success = distance < .001 and angle < .015
                current = {"success": success, "q": q.copy(), "position_error_m": distance,
                           "rotation_error_rad": angle, "iterations": iteration + 1, "score": score}
                if best is None or score < best["score"]:
                    best = current
                if success:
                    return current
                jacobian = np.vstack([np.cross(axes, pose[:3, 3] - origins).T, orientation_weight * axes.T])
                error = np.r_[position_error, orientation_weight * rotation_error]
                damping = .008 if distance < .05 else .025
                delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping**2 * np.eye(6), error)
                largest = np.max(np.abs(delta))
                if largest > .18:
                    delta *= .18 / largest
                updated = np.clip(q + delta, self.lower + 1e-7, self.upper - 1e-7)
                if np.linalg.norm(updated - q) < 1e-8:
                    break
                q = updated
        return best

    def move_tcp(self, position, rotation=None):
        """Commit a reachable IK solution; leave the arm unchanged on failure."""
        result = self.solve_ik(position, rotation)
        if result["success"]:
            self.set_joints(result["q"])
        return result
