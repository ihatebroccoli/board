"""Geometry-independent, deliberately kinematic manipulation state.

This is a task scene controller, not an MDP or force/contact simulator.
All positions are metres in the scene world, rotations are right-handed.
"""
from copy import deepcopy
import math
import numpy as np

INSERTION_TASKS = ('outlet', 'audio', 'usb', 'lan', 'hdmi', 'bnc')
ROTATION_TASKS = ('rotary', 'latch', 'locker')
TASKS = INSERTION_TASKS + ROTATION_TASKS


def pose(position):
    out = np.eye(4)
    out[:3, 3] = position
    return out


def finite_vector(value, count, label):
    out = np.asarray(value, dtype=float)
    if out.shape != (count,) or not np.isfinite(out).all():
        raise ValueError(f'{label} must have {count} finite values')
    return out


def validate_pose(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4,4) or not np.isfinite(value).all():
        raise ValueError('TCP must be a finite 4x4 pose')
    if not np.allclose(value[3], [0,0,0,1]) or not np.allclose(value[:3,:3].T @ value[:3,:3], np.eye(3), atol=1e-5) or np.linalg.det(value[:3,:3]) < .999:
        raise ValueError('TCP must be a proper rigid transform')
    return value


def validate_layout(layout):
    if len(layout) != 3 or any(len(row) != 3 for row in layout):
        raise ValueError('layout must have exactly three rows and three columns')
    names = [x for row in layout for x in row]
    if len(set(names)) != 9 or set(names) != set(TASKS):
        raise ValueError('layout must contain each of the nine tasks exactly once')
    return deepcopy(layout)


class TaskModel:
    def __init__(self, home_positions, insert_positions, config=None):
        self.config = config or {}
        self.homes = {k: pose(finite_vector(home_positions[k], 3, k)) for k in INSERTION_TASKS}
        self.targets = {k: pose(finite_vector(insert_positions[k], 3, k)) for k in INSERTION_TASKS}
        self.grasp_distance = self.config.get('grasp_distance_m', .028)
        self.insert_distance = self.config.get('insert_distance_m', .040)
        self.insert_angle = self.config.get('insert_angle_deg', 40.)
        self.latch_limits = self.config.get('latch_limits_deg', [-165., 165.])
        self.locker_bolt_travel = float(self.config.get('locker_bolt_travel_m', .016))
        if not math.isfinite(self.locker_bolt_travel) or self.locker_bolt_travel <= 0:
            raise ValueError('locker bolt travel must be finite and positive')
        self.events = []
        self.time = 0.
        self.reset()

    def reset(self):
        self.plugs = {k: v.copy() for k, v in self.homes.items()}
        self.plug_state = {k: 'in_rack' for k in INSERTION_TASKS}
        self.held = None
        self.grip_offset = np.eye(4)
        self.angles = {'rotary': 45., 'latch': 0., 'locker': 0.}
        self.rotary_on = False
        self.velocity = {k: 0. for k in INSERTION_TASKS}
        self.emit('reset')

    def emit(self, event, **data):
        value = {'time_s': float(self.time), 'event': event, **data}
        self.events.append(value)
        return value

    def grasp(self, name, tcp):
        tcp = validate_pose(tcp)
        if name not in self.plugs:
            raise ValueError(f'{name} is not a movable plug')
        if self.held:
            raise ValueError(f'already holding {self.held}')
        if self.plug_state[name] == 'inserted':
            raise ValueError('call unplug before grasping an inserted plug')
        distance = float(np.linalg.norm(tcp[:3, 3] - self.plugs[name][:3, 3]))
        if distance > self.grasp_distance:
            raise ValueError(f'plug out of grasp range: {distance:.3f} m')
        self.grip_offset = np.linalg.inv(tcp) @ self.plugs[name]
        self.held = name
        self.plug_state[name] = 'held'
        return self.emit('grasp', object=name, distance_m=distance)

    def update(self, tcp, dt=0.):
        tcp = validate_pose(tcp)
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        self.time += dt
        if self.held:
            self.plugs[self.held] = tcp @ self.grip_offset
        for name, state in self.plug_state.items():
            if state != 'released':
                continue
            # Simplified gravity to a configurable table support plane.
            self.velocity[name] -= 9.81 * dt
            self.plugs[name][2, 3] += self.velocity[name] * dt
            support = float(self.config.get('released_plug_center_z_m', .035))
            if self.plugs[name][2, 3] <= support:
                self.plugs[name][2, 3] = support
                self.velocity[name] = 0.
                self.plug_state[name] = 'on_table'
                self.emit('land', object=name)

    def release(self):
        if not self.held:
            raise ValueError('no held plug')
        name, self.held = self.held, None
        self.plug_state[name] = 'released'
        self.velocity[name] = 0.
        return self.emit('release', object=name)

    def insert(self, socket):
        if socket not in self.targets:
            raise ValueError('unknown socket')
        if self.held != socket:
            raise ValueError(f'{self.held or "nothing"} cannot mate with {socket}')
        current = self.plugs[socket]
        distance = float(np.linalg.norm(current[:3, 3] - self.targets[socket][:3, 3]))
        # Connector insertion axes must be approximately parallel; snap resolves yaw.
        axis_angle = math.degrees(math.acos(float(np.clip(current[:3, 2] @ self.targets[socket][:3, 2], -1, 1))))
        if distance > self.insert_distance or axis_angle > self.insert_angle:
            raise ValueError(f'not near/aligned with socket: {distance:.3f} m, {axis_angle:.1f} deg')
        self.plugs[socket] = self.targets[socket].copy()
        self.plug_state[socket] = 'inserted'
        self.held = None
        return self.emit('insert', object=socket, distance_m=distance, axis_error_deg=axis_angle)

    def unplug(self, socket, tcp):
        tcp = validate_pose(tcp)
        if self.held:
            raise ValueError('release current plug first')
        if self.plug_state.get(socket) != 'inserted':
            raise ValueError('socket is not occupied')
        distance = float(np.linalg.norm(tcp[:3, 3] - self.plugs[socket][:3, 3]))
        if distance > self.grasp_distance:
            raise ValueError('TCP must be near the inserted plug')
        self.held = socket
        self.grip_offset = np.linalg.inv(tcp) @ self.plugs[socket]
        self.plug_state[socket] = 'held'
        return self.emit('unplug', object=socket)

    def validate_rotation(self, task, angle_deg):
        angle = float(angle_deg)
        if not math.isfinite(angle):
            raise ValueError('rotation angle must be finite')
        limits = {'rotary': (45., 105.), 'locker': (0., 90.), 'latch': self.latch_limits}
        if task not in limits:
            raise ValueError('unknown rotation task')
        lo, hi = limits[task]
        if not lo <= angle <= hi:
            raise ValueError(f'{task} allowed range is [{lo}, {hi}] degrees')
        return angle

    def rotate(self, task, angle_deg):
        angle = self.validate_rotation(task, angle_deg)
        self.angles[task] = angle
        if task == 'rotary':
            before = self.rotary_on
            # Angle-triggered detent substitutes for unmodelled torque threshold.
            threshold = self.config.get('rotary_detent_threshold_deg', 75.)
            hysteresis = self.config.get('rotary_hysteresis_deg', 10.) / 2
            if not before and angle >= threshold + hysteresis:
                self.rotary_on = True
                self.angles[task] = 105.
            elif before and angle <= threshold - hysteresis:
                self.rotary_on = False
                self.angles[task] = 45.
            if before != self.rotary_on:
                self.emit('click', object=task, state='on' if self.rotary_on else 'off')
        return self.emit('rotate', object=task, requested_angle_deg=angle,
                         angle_deg=self.angles[task])

    def translate_panel_target(self, task, offset):
        delta = finite_vector(offset, 3, 'panel offset')
        if task in self.targets:
            self.targets[task][:3, 3] += delta
            if self.plug_state[task] == 'inserted':
                self.plugs[task][:3, 3] += delta

    @property
    def locker_bolt_retraction_m(self):
        """Approximate cam linkage: 0° extended, 90° fully withdrawn."""
        return self.locker_bolt_travel * self.angles['locker'] / 90.

    def snapshot(self):
        return {'time_s': self.time, 'held': self.held, 'plug_state': dict(self.plug_state),
                'plug_poses': {k: v.tolist() for k, v in self.plugs.items()},
                'angles_deg': dict(self.angles), 'rotary_state': 'on' if self.rotary_on else 'off',
                'locker_bolt_retraction_m': self.locker_bolt_retraction_m}
