#!/usr/bin/env python3
"""Isaac Sim headless RGB/depth capture and semantic taskboard playback."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import traceback
import wave

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--vendor',type=Path,required=True)
p.add_argument('--config',type=Path,required=True)
p.add_argument('--reference',type=Path,required=True)
p.add_argument('--mode',choices=['preview','demo','commands','replay'],default='demo')
p.add_argument('--commands',type=Path)
p.add_argument('--gpu',type=int,default=3)
p.add_argument('--ffmpeg',required=True)
p.add_argument('--fps',type=int,default=12)
p.add_argument('--width',type=int,default=640)
p.add_argument('--height',type=int,default=480)
p.add_argument('--max-frames',type=int,default=0)
a,kit_args=p.parse_known_args();sys.argv=[sys.argv[0]]+kit_args
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False,
                   'renderer':'RayTracedLighting','anti_aliasing':2,'width':a.width,'height':a.height,'sync_loads':True})
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from pxr import Gf,UsdGeom,UsdLux
import omni.usd
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
import omni.replicator.core as rep
from controller import SceneController,DOWN

class FrameLimit(Exception): pass

def save_json(path,value):
    def default(v):
        if isinstance(v,np.ndarray):return v.tolist()
        if isinstance(v,np.generic):return v.item()
        raise TypeError(type(v).__name__)
    path.write_text(json.dumps(value,indent=2,default=default,allow_nan=False)+'\n')

def look(camera,eye,target,up=(0,0,1)):
    rotation=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(*map(float,up))).GetInverse().ExtractRotationQuat()
    camera.set_world_pose(np.asarray(eye),np.array([rotation.GetReal(),*rotation.GetImaginary()]),camera_axes='usd')

class Capture:
    def __init__(self,world,stage,controller,config):
        self.world,self.stage,self.controller=world,stage,controller
        self.cameras={};self.depth={};self.encoders={};self.logs=[];self.frames=0;self.trace=[]
        self.wrist_config=config['wrist_camera']
        for name in ['head','detail','wrist']:
            camera=Camera('/World/Cameras/'+name,name=name,resolution=(a.width,a.height),frequency=a.fps)
            camera.set_clipping_range(.005,10.)
            spec=config['head_camera'] if name=='head' else config['wrist_camera']
            prim=UsdGeom.Camera(stage.GetPrimAtPath(camera.prim_path))
            prim.CreateHorizontalApertureAttr(float(spec['horizontal_aperture_mm']))
            prim.CreateVerticalApertureAttr(float(spec['horizontal_aperture_mm'])*a.height/a.width)
            prim.CreateFocalLengthAttr(float(spec['focal_length_mm']))
            if name=='head': look(camera,spec['eye_m'],spec['target_m'],spec['up'])
            elif name=='detail':
                detail=config.get('detail_camera',{})
                look(camera,detail.get('eye_m',[.82,-.35,.40]),detail.get('target_m',[.44,-.025,.04]),detail.get('up',[0,0,1]))
                prim.GetFocalLengthAttr().Set(float(detail.get('focal_length_mm',23.)))
            self.cameras[name]=camera
        world.reset()
        for name,camera in self.cameras.items():
            camera.initialize()
            self.depth[name]=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            self.depth[name].attach([camera.get_render_product_path()])
        self.update_wrist()
        for _ in range(64):world.render()
        font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        self.font=ImageFont.truetype(font,16)
        self.small=ImageFont.truetype(font,13)

    def update_wrist(self):
        tool=self.controller.robot.get_tcp_pose()
        local=np.array(self.wrist_config.get('mount_translation_m',[0,.055,.035]),dtype=float)
        local[2]-=self.controller.robot.tool_length
        eye=tool[:3,3]+tool[:3,:3]@local
        forward=tool[:3,:3]@np.array(self.wrist_config.get('forward_axis_in_flange',[0,0,1]))
        up=tool[:3,:3]@np.array(self.wrist_config.get('up_axis_in_flange',[1,0,0]))
        look(self.cameras['wrist'],eye,eye+forward,up)

    def images(self):
        result={}
        for name,camera in self.cameras.items():
            rgba=camera.get_rgba()
            if rgba is None or rgba.shape!=(a.height,a.width,4):
                raise RuntimeError(f'{name} produced an empty/wrong camera frame')
            result[name]=np.asarray(rgba[:,:,:3],dtype=np.uint8)
        return result

    def encoder(self,name,width,height):
        if name not in self.encoders:
            log=(a.output/(name+'_encode.log')).open('w');self.logs.append(log)
            env=os.environ.copy()
            for key in ['LD_LIBRARY_PATH','LD_PRELOAD','PYTHONPATH','PYTHONHOME']:env.pop(key,None)
            proc=subprocess.Popen([a.ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo',
                '-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(a.fps),'-i','-',
                '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20','-pix_fmt','yuv420p',
                '-movflags','+faststart',str(a.output/(name+'_silent.mp4'))],stdin=subprocess.PIPE,stderr=log,env=env)
            self.encoders[name]=proc
        return self.encoders[name]

    def __call__(self,label,controller):
        if a.max_frames and self.frames>=a.max_frames:raise FrameLimit()
        self.update_wrist()
        self.world.render();self.world.render();self.world.render()
        rgb=self.images()
        for name,value in rgb.items(): self.encoder(name,a.width,a.height).stdin.write(value.tobytes())
        canvas=Image.new('RGB',(a.width*2,a.height+64),(22,27,34));canvas.paste(Image.fromarray(rgb['head']),(0,64));canvas.paste(Image.fromarray(rgb['detail']),(a.width,64))
        d=ImageDraw.Draw(canvas)
        d.text((12,6),f'RB3-730 | {label}',font=self.font,fill='white')
        inserted=sum(x=='inserted' for x in controller.model.plug_state.values())
        d.text((12,31),f't={controller.model.time:.1f}s   plugs {inserted}/6   rotary {"ON" if controller.model.rotary_on else "OFF"}   latch {controller.model.angles["latch"]:.0f} deg   locker {controller.model.angles["locker"]:.0f} deg',font=self.small,fill=(192,210,220))
        d.text((a.width-160,48),'Estimated real camera',font=self.small,fill=(175,190,205))
        d.text((a.width+12,48),'Task detail',font=self.small,fill=(175,190,205))
        self.encoder('demo',a.width*2,a.height+64).stdin.write(np.asarray(canvas).tobytes())
        self.trace.append({'frame':self.frames,'label':label,'qpos':controller.robot.q.tolist(),
                           'tcp':controller.robot.get_tcp_pose().tolist(),**controller.model.snapshot()})
        if self.frames==0:
            self.snapshot('initial',rgb);canvas.save(a.output/'preview.jpg')
        self.frames+=1
        if self.frames%48==0:print(json.dumps({'frames':self.frames,'label':label,'state':controller.model.plug_state}),flush=True)

    def snapshot(self,prefix,rgb=None):
        if rgb is None:
            self.update_wrist()
            for _ in range(3):self.world.render()
            rgb=self.images()
        meta={}
        for name,value in rgb.items():
            Image.fromarray(value).save(a.output/(prefix+'_'+name+'.png'))
            depth=self.depth[name].get_data()
            if depth is not None and np.asarray(depth).size:
                depth=np.asarray(depth)
                if depth.shape!=(a.height,a.width): raise RuntimeError('Invalid depth shape')
                np.save(a.output/(prefix+'_'+name+'_depth_m.npy'),depth)
                meta[name]={'shape':list(depth.shape),'finite_pixels':int(np.isfinite(depth).sum()),'units':'metres','quantity':'camera-axis depth (distance_to_image_plane)'}
        if len(meta)!=3: raise RuntimeError('All three depth cameras must produce data')
        save_json(a.output/(prefix+'_depth_metadata.json'),meta)

    def close(self,events):
        for process in self.encoders.values():process.stdin.close()
        for name,process in self.encoders.items():
            if process.wait()!=0:raise RuntimeError(name+' video encoding failed')
        for log in self.logs:log.close()
        if not self.frames:return
        sample_rate=48000;samples=np.zeros(round(self.frames/a.fps*sample_rate),dtype=np.float32)
        rng=np.random.default_rng(7)
        for event in events:
            if event['event']!='click':continue
            begin=round(event['time_s']*sample_rate)
            length=min(round(.035*sample_rate),len(samples)-begin)
            if length>0:
                t=np.arange(length)/sample_rate
                tick=(rng.normal(0,.35,length)+.65*np.sin(2*np.pi*3100*t))*np.exp(-180*t)
                samples[begin:begin+length]+=.65*tick
        audio=a.output/'clicks.wav'
        with wave.open(str(audio),'wb') as w:
            w.setnchannels(1);w.setsampwidth(2);w.setframerate(sample_rate)
            w.writeframes((np.clip(samples,-1,1)*32767).astype('<i2').tobytes())
        env=os.environ.copy()
        for key in ['LD_LIBRARY_PATH','LD_PRELOAD','PYTHONPATH','PYTHONHOME']:env.pop(key,None)
        for name in self.encoders:
            silent=a.output/(name+'_silent.mp4')
            subprocess.run([a.ffmpeg,'-hide_banner','-loglevel','error','-y','-i',str(silent),'-i',str(audio),'-c:v','copy','-c:a','aac','-shortest','-movflags','+faststart',str(a.output/(name+'.mp4'))],env=env,check=True)
            silent.unlink()
        save_json(a.output/'trace.json',self.trace)


def demonstrate(controller,capture):
    controller.hold(1.5,'Reconstructed scene')
    for task in ['usb','lan','hdmi','audio','bnc','outlet']:
        controller.pick_and_insert(task)
    # Move to each handle, grip and rotate within its permitted travel.
    for name,angles in [('rotary',[105,45]),('latch',[135,-135,0]),('locker',[90,0])]:
        origin=np.asarray(controller.handles['rotary_origins'][name])
        controller.move_tcp(origin+[0,0,.15],label='Approach '+name.upper())
        controller.move_tcp(origin+[0,0,.022],label='Grip '+name.upper())
        controller.robot.set_gripper(.014 if name=='rotary' else .025)
        for angle in angles:
            controller.rotate(name,angle,duration=1.0)
            controller.hold(.35,name.upper()+' state')
        controller.robot.set_gripper(.065)
        controller.move_tcp(origin+[0,0,.15],label='Retract')
    controller.move_tcp([.36,.25,.30],label='Park arm')
    controller.hold(1.5,'All six plugs seated; rotation tasks exercised')
    capture.snapshot('completed')
    original=[list(row) for row in controller.layout]
    controller.rearrange([original[2],original[0],original[1]])
    controller.hold(2.0,'Reconfigured board: each complete panel moved')
    capture.snapshot('reconfigured')
    capture.stage.GetRootLayer().Export(str(a.output/'scene_reconfigured.usdc'))
    controller.rearrange(original)


def run_commands(controller,commands):
    for command in commands:
        op=command['op']
        if op=='move_tcp':controller.move_tcp(command['position'],rotation=np.array(command['rotation']) if 'rotation' in command else None,duration=command.get('duration',.7))
        elif op=='grasp':controller.grasp(command['object'])
        elif op=='insert':controller.insert(command['socket'])
        elif op=='unplug':controller.unplug(command['socket'])
        elif op=='release':controller.release()
        elif op=='rotate':controller.rotate(command['object'],command['angle_deg'],command.get('duration',.7))
        elif op=='pick_and_insert':controller.pick_and_insert(command['object'])
        elif op=='rearrange':controller.rearrange(command['layout']);controller.hold(command.get('duration',1),'Reconfigured board')
        elif op=='reset':controller.reset();controller.hold(.5,'Reset')
        elif op=='wait':controller.hold(command.get('seconds',1),'Wait')
        else:raise ValueError('Unknown command: '+op)


def main():
    config=json.loads(a.config.read_text());save_json(a.output/'scene_config.json',config)
    save_json(a.output/'source_hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')})
    world=World(stage_units_in_meters=1,physics_dt=1/60,rendering_dt=1/a.fps,device='cpu')
    stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    controller=SceneController(stage,config,a.vendor,a.fps)
    dome=UsdLux.DomeLight.Define(stage,'/World/Lighting/Fill');dome.CreateIntensityAttr(650.);dome.CreateColorAttr(Gf.Vec3f(.94,.97,1.))
    key=UsdLux.RectLight.Define(stage,'/World/Lighting/Key');key.CreateIntensityAttr(1200.);key.CreateWidthAttr(1.2);key.CreateHeightAttr(.8)
    key.AddTranslateOp().Set(Gf.Vec3d(.3,-.1,1.6))
    # A subdued floor/wall gives the tabletop an actual room backdrop.
    floor=UsdGeom.Cube.Define(stage,'/World/Floor');floor.CreateSizeAttr(1.);floor.AddTranslateOp().Set(Gf.Vec3d(.2,0,-.74));floor.AddScaleOp().Set(Gf.Vec3f(4,4,.03));floor.CreateDisplayColorAttr([Gf.Vec3f(.12,.13,.14)])
    initial=controller.robot.move_tcp([.38,.24,.18],DOWN)
    if not initial['success']:raise RuntimeError('initial park pose unreachable')
    capture=Capture(world,stage,controller,config);controller.on_frame=capture
    stage.GetRootLayer().Export(str(a.output/'scene.usdc'))
    truncated=False
    try:
        if a.mode=='preview':controller.hold(2.,'Scene overview')
        elif a.mode=='demo':demonstrate(controller,capture)
        elif a.mode=='commands':run_commands(controller,json.loads(a.commands.read_text()))
        elif a.mode=='replay':
            data=np.load(a.reference/'robot_state.npz');times=data['time_relative']
            for t in np.arange(0,float(times[-1]),1/a.fps):
                i=int(np.argmin(np.abs(times-t)))
                controller.robot.set_joints(data['qpos'][i]);controller.robot.set_gripper(min(.075,max(0,float(data['gripper_width_m'][i]))))
                controller.tick('Recorded joints; scene and TCP alignment estimated')
    except FrameLimit:truncated=True
    finally:capture.close(controller.model.events)
    capture.snapshot('final')
    stage.GetRootLayer().Export(str(a.output/'scene_final.usdc'))
    reference=a.reference/'0000_head_cam_d455.png'
    initial=a.output/'initial_head.png'
    if reference.exists() and initial.exists():
        canvas=Image.new('RGB',(1280,520),(22,27,34))
        canvas.paste(Image.open(reference).convert('RGB').resize((640,480)),(0,40))
        canvas.paste(Image.open(initial).convert('RGB').resize((640,480)),(640,40))
        draw=ImageDraw.Draw(canvas)
        draw.text((16,12),'Real recording: head camera',font=capture.font,fill='white')
        draw.text((656,12),'Isaac Sim: estimated geometry and camera',font=capture.font,fill='white')
        canvas.save(a.output/'real_vs_sim.png')
    save_json(a.output/'events.json',controller.model.events)
    save_json(a.output/'ik_checks.json',controller.ik_log)
    save_json(a.output/'handles.json',controller.handles)
    if a.mode=='demo' and not truncated:
        assert all(v=='inserted' for v in controller.model.plug_state.values()), 'Demo did not complete every insertion'
        assert not controller.model.rotary_on and controller.model.angles['locker']==0 and controller.model.angles['latch']==0
        assert sum(e['event']=='click' for e in controller.model.events)==2, 'Both switch detents must be exercised'
    save_json(a.output/'result.json',{'completed':True,'mode':a.mode,'truncated':truncated,'frames':capture.frames,'fps':a.fps,'duration_s':capture.frames/a.fps,'layout':controller.layout,'state':controller.model.snapshot(),'camera_calibration':'estimated','physics':'kinematic semantic interactions; no contact or torque simulation','videos':['demo.mp4','head.mp4','detail.mp4','wrist.mp4']})
    print('Scene completed',flush=True)

try:main()
except BaseException:
    (a.output/'error.txt').write_text(traceback.format_exc());traceback.print_exc();app.close();raise
else:app.close()
