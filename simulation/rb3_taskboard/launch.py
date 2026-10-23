#!/usr/bin/env python3
"""Run the taskboard headlessly using the installed Isaac Sim 4.5 runtime."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
PROJECT=HERE.parents[1]
BASE=(PROJECT/'logs/simulation/rb3_taskboard').resolve()
RUNTIME_BASE=(PROJECT/'logs/simulation/wonik_v5').resolve()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gpu',type=int,default=3)
    p.add_argument('--mode',choices=['preview','demo','commands','replay'],default='demo')
    p.add_argument('--config',type=Path,default=HERE/'config/scene.json')
    p.add_argument('--commands',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--fps',type=int,default=12)
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=480)
    p.add_argument('--skip-assets',action='store_true')
    p.add_argument('--max-frames',type=int,default=0,help='0 = complete sequence')
    a=p.parse_args()
    if a.fps<1 or a.width<64 or a.height<64 or a.width%2 or a.height%2:
        p.error('fps must be positive; resolution must be even and >=64')
    if a.mode=='commands' and not a.commands:
        p.error('--commands JSON is required in commands mode')
    BASE.mkdir(parents=True,exist_ok=True)
    if not a.skip_assets:
        subprocess.run([sys.executable,str(HERE/'fetch_robot.py'),'--output',str(BASE/'vendor/rbpodo_description')],check=True)
    runtime=RUNTIME_BASE/'runtime/isaac-sim-4.5.0'
    lock=(RUNTIME_BASE/'runtime.lock').open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: p.error('The shared Isaac runtime is already in use.')
    run=a.output.resolve() if a.output else BASE/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    for key,rel in {'XDG_CACHE_HOME':'cache/xdg','XDG_DATA_HOME':'cache/xdg-data',
                    'XDG_CONFIG_HOME':'cache/xdg-config','TMPDIR':'tmp','CUDA_CACHE_PATH':'cache/cuda',
                    '__GL_SHADER_DISK_CACHE_PATH':'cache/gl'}.items():
        value=RUNTIME_BASE/rel;value.mkdir(parents=True,exist_ok=True);env[key]=str(value)
    env.update(OMNI_KIT_ACCEPT_EULA='YES',PYTHONUNBUFFERED='1',OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1')
    env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
    ffmpeg=subprocess.check_output([str(RUNTIME_BASE/'inspection_env/bin/python'),'-c',
        'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())'],text=True).strip()
    cmd=[str(runtime/'python.sh'),str(HERE/'run_scene.py'),'--output',str(run),
         '--vendor',str(BASE/'vendor'),'--config',str(a.config.resolve()),'--mode',a.mode,
         '--reference',str(BASE/'reference'),'--gpu',str(a.gpu),'--ffmpeg',ffmpeg,
         '--fps',str(a.fps),'--width',str(a.width),'--height',str(a.height),
         '--max-frames',str(a.max_frames),'--portable','--portable-root',str(RUNTIME_BASE/'cache/kit'),
         f'--/log/file={run}/kit.log','--/app/window/enabled=false','--/app/fastShutdown=true',
         '--/privacy/userConsent=false','--/crashreporter/enabled=false']
    if a.commands: cmd+=['--commands',str(a.commands.resolve())]
    (run/'launch.json').write_text(json.dumps({'command':cmd,'mode':a.mode},indent=2))
    print(f'Output: {run}',flush=True)
    print(f'Log: {run}/console.log',flush=True)
    with (run/'console.log').open('w') as log:
        result=subprocess.call(cmd,cwd=runtime,env=env,stdout=log,stderr=subprocess.STDOUT)
    completed=(run/'result.json').exists() and json.loads((run/'result.json').read_text()).get('completed')
    print('Completed' if completed else 'FAILED; inspect console.log',flush=True)
    return 0 if completed and result==0 else result or 1

if __name__=='__main__':sys.exit(main())
