"""Python task API for the kinematic RB3 modular taskboard scene.

Use after SimulationApp initialization. on_frame receives (label, controller)
and is suitable for camera capture; no RL wrappers are involved.
"""
import math
import numpy as np
from pxr import Gf, UsdGeom
from geometry import build_scene
from robot import Robot
from task_model import TaskModel, validate_layout

DOWN = np.diag([1., -1., -1.])


def matrix_op(stage, path):
    xf=UsdGeom.Xformable(stage.GetPrimAtPath(path))
    original=np.asarray(xf.GetLocalTransformation(),dtype=float).T.copy()
    xf.ClearXformOpOrder()
    op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, 'controller')
    op.Set(Gf.Matrix4d(original.T.tolist()))
    return op, original


def put(op, matrix):
    op.Set(Gf.Matrix4d(np.asarray(matrix,dtype=float).T.tolist()))


def geometry_config(config):
    out=dict(config)
    size=config.get('panel_size_m',[.09,.09,.035])
    out.update(board_center=config.get('board_center_m',[.44,0,0])[:2],
               panel_size=size[0],panel_height=size[2],
               panel_gap=config.get('panel_gap_m',.001),
               rack_center=config.get('rack_center_m',[.44,-.20,0])[:2],
               rack_order=config.get('rack_order_back_to_front',['outlet','hdmi','usb','lan','audio','bnc']))
    return out


class SceneController:
    def __init__(self, stage, config, vendor, fps=12, on_frame=None):
        self.stage=stage
        self.config=config
        self.layout=validate_layout(config['layout'])
        self.fps=fps
        self.on_frame=on_frame
        self.handles=build_scene(stage,geometry_config(config))
        self.robot=Robot(stage,vendor,base_position=config.get('robot_base_m',[0,0,.12]))
        self.model=TaskModel(self.handles['plug_home_positions'],self.handles['plug_insert_positions'],config.get('interaction',{}))
        for name,angle in self.handles.get('fixture_yaw_deg',{}).items():
            if name in self.model.targets:
                a=math.radians(angle)
                self.model.targets[name][:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
        self.plug_ops={k:matrix_op(stage,p)[0] for k,p in self.handles['plug_paths'].items()}
        pairs={k:matrix_op(stage,p) for k,p in self.handles['rotary_paths'].items()}
        self.rotary_ops={k:v[0] for k,v in pairs.items()}
        self.rotary_base={k:v[1] for k,v in pairs.items()}
        pairs={k:matrix_op(stage,p) for k,p in self.handles['panel_paths'].items()}
        self.panel_ops={k:v[0] for k,v in pairs.items()}
        self.panel_poses={k:v[1] for k,v in pairs.items()}
        self.locker_bolt_op,self.locker_bolt_rest=matrix_op(stage,self.handles['locker_bolt_path'])
        self.ik_log=[]
        self.sync()

    def sync(self):
        for name,transform in self.model.plugs.items(): put(self.plug_ops[name],transform)
        for name,angle in self.model.angles.items():
            a=math.radians(angle);r=np.eye(4)
            r[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
            put(self.rotary_ops[name],self.rotary_base[name]@r)
        bolt=self.locker_bolt_rest.copy()
        bolt[0,3]-=self.model.locker_bolt_retraction_m
        put(self.locker_bolt_op,bolt)

    def tick(self, label='', dt=None):
        self.model.update(self.robot.get_tcp_pose(),1/self.fps if dt is None else dt)
        self.sync()
        if self.on_frame: self.on_frame(label,self)

    def hold(self,seconds=.5,label=''):
        for _ in range(max(1,round(seconds*self.fps))): self.tick(label)

    def move_tcp(self,position,rotation=None,duration=.7,label='Move'):
        rotation=DOWN if rotation is None else rotation
        result=self.robot.solve_ik(position,rotation)
        self.ik_log.append({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in result.items()})
        if not result['success']:
            raise RuntimeError(f'IK failed for {position}: error={result["position_error_m"]:.4f} m')
        start=self.robot.q.copy();target=result['q']
        for u in np.linspace(0,1,max(2,round(duration*self.fps))):
            a=u*u*(3-2*u)
            self.robot.set_joints(start+(target-start)*a)
            self.tick(label)
        return result

    def grasp(self,name):
        self.model.update(self.robot.get_tcp_pose())
        result=self.model.grasp(name,self.robot.get_tcp_pose())
        width={'outlet':.030,'usb':.012,'hdmi':.018,'lan':.016,'audio':.006,'bnc':.012}[name]
        self.robot.set_gripper(width)
        self.tick('Grasp '+name.upper())
        return result

    def release(self):
        result=self.model.release();self.robot.set_gripper(.065);self.tick('Release');return result

    def insert(self,name):
        result=self.model.insert(name);self.robot.set_gripper(.065);self.tick('Insert '+name.upper());return result

    def unplug(self,name):
        result=self.model.unplug(name,self.robot.get_tcp_pose());self.robot.set_gripper(.012)
        self.tick('Unplug '+name.upper());return result

    def rotate(self,name,angle_deg,duration=.7):
        angle_deg=self.model.validate_rotation(name,angle_deg)
        if self.model.held: raise ValueError('release the held plug before rotating a handle')
        origin=np.asarray(self.handles['rotary_origins'][name])
        tool=self.robot.get_tcp_pose()
        if np.linalg.norm(tool[:3,3]-origin) > .055:
            raise ValueError('move the TCP within 55 mm of the handle before rotating')
        start=self.model.angles[name]
        for a in np.linspace(start,angle_deg,max(2,round(duration*self.fps))):
            before=self.model.rotary_on
            previous_angle=self.model.angles[name]
            previous_events=len(self.model.events)
            self.model.rotate(name,float(a))
            delta=math.radians(self.model.angles[name]-start)
            rz=np.array([[math.cos(delta),-math.sin(delta),0],[math.sin(delta),math.cos(delta),0],[0,0,1]])
            solution=self.robot.move_tcp(tool[:3,3],rz@tool[:3,:3])
            if not solution['success']:
                self.model.angles[name]=previous_angle
                self.model.rotary_on=before
                del self.model.events[previous_events:]
                raise RuntimeError('rotation pose unreachable')
            self.tick('Rotate '+name.upper())
            if name=='rotary' and before!=self.model.rotary_on: break
        return self.model.angles[name]

    def rearrange(self,layout):
        layout=validate_layout(layout)
        if self.model.held: raise ValueError('release the held object before moving panels')
        # Move each complete module, its independent moving handle, and seated plug.
        locations={name:tuple(self.panel_poses[name][:3,3]) for row in self.layout for name in row}
        slot_positions=[[locations[name] for name in row] for row in self.layout]
        for r,row in enumerate(layout):
            for c,name in enumerate(row):
                position=np.array(slot_positions[r][c]);delta=position-self.panel_poses[name][:3,3]
                self.panel_poses[name][:3,3]=position
                put(self.panel_ops[name],self.panel_poses[name])
                self.handles['panel_positions'][name]=position.tolist()
                if name in self.rotary_base:
                    self.rotary_base[name][:3,3]+=delta
                    self.handles['rotary_origins'][name]=self.rotary_base[name][:3,3].tolist()
                if name in self.model.targets:
                    self.handles['socket_positions'][name]=(np.array(self.handles['socket_positions'][name])+delta).tolist()
                    self.model.translate_panel_target(name,delta)
                    self.handles['plug_insert_positions'][name]=self.model.targets[name][:3,3].tolist()
                self.stage.GetPrimAtPath(self.handles['panel_paths'][name]).GetAttribute('taskboard:slot').Set(Gf.Vec2i(r,c))
        self.layout=layout
        self.model.emit('rearrange',layout=layout)
        self.sync()

    def reset(self):
        self.model.reset();self.robot.set_gripper(.065);self.sync()

    def pick_and_insert(self,name):
        home=self.model.plugs[name][:3,3].copy()
        target=self.model.targets[name][:3,3].copy()
        self.robot.set_gripper(.065)
        self.move_tcp(home+[0,0,.14],label='Approach '+name.upper())
        self.move_tcp(home,duration=.6,label='Pick '+name.upper())
        self.grasp(name)
        self.move_tcp(home+[0,0,.14],label='Lift '+name.upper())
        self.move_tcp(target+[0,0,.14],label='Carry '+name.upper())
        self.move_tcp(target+[0,0,.012],duration=.6,label='Align '+name.upper())
        self.insert(name)
        self.hold(.25,'Seated '+name.upper())
        self.move_tcp(target+[0,0,.16],duration=.5,label='Retract')
