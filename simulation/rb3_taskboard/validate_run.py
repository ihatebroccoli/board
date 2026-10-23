#!/usr/bin/env python3
"""Verify saved semantics, USD, RGB-D arrays and full H.264/AAC video decoding.

Use the separate OpenUSD inspection environment; no Isaac runtime is started.
Positive infinity in depth is permitted only as an explicitly reported no-hit
background value. All finite surface depths must be nonnegative and vary.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import wave

import numpy as np

PLUGS = {'outlet', 'audio', 'usb', 'lan', 'hdmi', 'bnc'}
CAMERAS = {'head', 'detail', 'wrist'}
ROTATIONS = {'rotary', 'latch', 'locker'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    def invalid(value):
        raise ValueError(f'Nonfinite JSON value {value} in {path}')
    return json.loads(path.read_text(), parse_constant=invalid)


def faststart(path):
    atoms = []
    total = path.stat().st_size
    with path.open('rb') as source:
        while source.tell() + 8 <= total:
            start = source.tell()
            size, tag = struct.unpack('>I4s', source.read(8))
            if size == 1:
                size = struct.unpack('>Q', source.read(8))[0]
            elif size == 0:
                size = total - start
            require(size >= 8 and start + size <= total, f'Malformed MP4 atom: {path.name}')
            atoms.append(tag)
            source.seek(start + size)
    return b'moov' in atoms and b'mdat' in atoms and atoms.index(b'moov') < atoms.index(b'mdat')


def video_check(path, ffmpeg, frames, fps, size):
    require(path.is_file() and path.stat().st_size > 1000, f'Missing video {path}')
    env = os.environ.copy()
    for key in ('LD_LIBRARY_PATH', 'LD_PRELOAD', 'PYTHONPATH', 'PYTHONHOME'):
        env.pop(key, None)
    # Mandatory video and audio streams are fully decoded; progress reports
    # actual decoded frames, rather than relying on container frame metadata.
    command = [str(ffmpeg), '-hide_banner', '-nostdin', '-nostats', '-i', str(path),
               '-map', '0:v:0', '-map', '0:a:0', '-threads', '2',
               '-progress', 'pipe:1', '-f', 'null', '-']
    result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=180)
    require(result.returncode == 0, f'{path.name} decoding failed: {result.stderr[-1200:]}')
    metadata = result.stderr.split('Output #', 1)[0]
    video = next((line.strip() for line in metadata.splitlines() if 'Video:' in line), '')
    audio = next((line.strip() for line in metadata.splitlines() if 'Audio:' in line), '')
    require('Video: h264' in video and 'yuv420p' in video, f'Expected H.264/yuv420p: {path.name}')
    require('Audio: aac' in audio, f'Expected AAC audio: {path.name}')
    dimensions = re.search(r', (\d+)x(\d+)(?:[ ,\[])', video)
    require(dimensions and [int(dimensions[1]), int(dimensions[2])] == size, f'Wrong video dimensions: {video}')
    counts = [int(value) for value in re.findall(r'^frame=(\d+)\s*$', result.stdout, re.MULTILINE)]
    require(counts and counts[-1] == frames, f'{path.name}: decoded {counts[-1] if counts else None}, expected {frames}')
    rate = re.search(r'([\d.]+) fps', video)
    require(rate and abs(float(rate[1]) - fps) < .01, f'Wrong video FPS: {path.name}')
    duration = re.search(r'Duration: (\d+):(\d+):([\d.]+)', metadata)
    require(duration is not None, f'No video duration: {path.name}')
    seconds = int(duration[1]) * 3600 + int(duration[2]) * 60 + float(duration[3])
    require(abs(seconds - frames / fps) < max(.05, 1 / fps), f'Wrong video duration: {path.name}')
    require(not any(word in result.stderr.lower() for word in
                    ('error while decoding', 'invalid data found', 'corrupt decoded frame')),
            f'Decode corruption: {path.name}')
    require(faststart(path), f'MP4 lacks fast-start metadata: {path.name}')
    return {'path': path.name, 'bytes': path.stat().st_size, 'decoded_frames': counts[-1],
            'resolution': size, 'fps': float(rate[1]), 'duration_s': seconds,
            'video_stream': video, 'audio_stream': audio, 'faststart': True, 'full_decode': True}


def validate(run, ffmpeg, allow_truncated=False):
    from pxr import Usd, UsdGeom, UsdUtils
    run = Path(run).resolve()
    result = read(run / 'result.json')
    require(result.get('completed') is True, 'Run did not complete')
    require(isinstance(result.get('truncated'), bool), 'Missing explicit truncation status')
    require(allow_truncated or not result['truncated'], 'Run was truncated')
    full_demo = result['mode'] == 'demo' and not result['truncated']
    frames, fps = result['frames'], result['fps']
    require(isinstance(frames, int) and frames > 0 and math.isfinite(fps) and fps > 0, 'Invalid frame count/FPS')
    require(abs(result['duration_s'] - frames / fps) < 1e-6, 'Inconsistent duration')
    events, trace, handles = (read(run / name) for name in ('events.json', 'trace.json', 'handles.json'))
    require(len(trace) == frames and [entry['frame'] for entry in trace] == list(range(frames)),
            'Frame trace length or indices disagree with result')
    qpos = np.asarray([entry['qpos'] for entry in trace], dtype=float)
    tcp = np.asarray([entry['tcp'] for entry in trace], dtype=float)
    require(qpos.shape == (frames, 6) and np.isfinite(qpos).all() and np.max(np.abs(qpos)) <= 3.14001,
            'Invalid robot joint trace')
    require(tcp.shape == (frames, 4, 4) and np.isfinite(tcp).all(), 'Invalid TCP trace')
    for entries in (events, trace):
        times = np.asarray([entry['time_s'] for entry in entries], dtype=float)
        require(np.isfinite(times).all() and np.all(np.diff(times) >= 0), 'Invalid/backwards timestamps')
    insertions = Counter(event.get('object') for event in events if event['event'] == 'insert')
    clicks = [event for event in events if event['event'] == 'click' and event.get('object') == 'rotary']
    state = result['state']
    require(set(state['plug_state']) == PLUGS, 'Incomplete final plug state')
    require(set(state['angles_deg']) == ROTATIONS and all(math.isfinite(v) for v in state['angles_deg'].values()),
            'Invalid final rotation state')
    if full_demo:
        require(insertions == Counter({name: 1 for name in PLUGS}), 'Full demo must insert every connector exactly once')
        require(len(clicks) == 2 and [event['state'] for event in clicks] == ['on', 'off'], 'Expected two rotary on/off clicks')
        require(state['held'] is None and all(v == 'inserted' for v in state['plug_state'].values()), 'Final plugs are not all seated')
        require(state['rotary_state'] == 'off' and state['angles_deg'] == {'rotary': 45., 'latch': 0., 'locker': 0.},
                'Incorrect final rotation states')
        require({event.get('object') for event in events if event['event'] == 'rotate'} == ROTATIONS,
                'Not every rotation task was exercised')
    ik = read(run / 'ik_checks.json')
    require(all(entry['success'] and math.isfinite(entry['position_error_m']) and entry['position_error_m'] < .001
                and math.isfinite(entry['rotation_error_rad']) and entry['rotation_error_rad'] < .015 for entry in ik),
            'Failed or invalid saved IK checks')

    depth_reports = []
    files = sorted(run.glob('*_depth_m.npy'))
    prefixes = {path.name.rsplit('_', 3)[0] for path in files}
    require({'initial', 'final'}.issubset(prefixes), 'Initial/final depth snapshots missing')
    if full_demo:
        require({'completed', 'reconfigured'}.issubset(prefixes), 'Full demo snapshots missing')
    for prefix in sorted(prefixes):
        metadata = read(run / (prefix + '_depth_metadata.json'))
        require(set(metadata) == CAMERAS, f'Incomplete depth cameras: {prefix}')
        for camera in sorted(CAMERAS):
            name = f'{prefix}_{camera}_depth_m.npy'
            data = np.load(run / name, allow_pickle=False)
            require(data.ndim == 2 and list(data.shape) == metadata[camera]['shape'], f'Wrong depth dimensions: {name}')
            require(np.issubdtype(data.dtype, np.floating), f'Metric depth is not floating-point: {name}')
            require(not np.isnan(data).any() and not np.isneginf(data).any(), f'NaN/-inf depth: {name}')
            finite = data[np.isfinite(data)]
            require(finite.size > .1 * data.size, f'Too few finite depth pixels: {name}')
            require(np.all(finite >= 0) and float(np.ptp(finite)) > .001, f'Negative or constant surface depth: {name}')
            require(metadata[camera]['finite_pixels'] == int(finite.size) and metadata[camera]['units'] == 'metres',
                    f'Depth metadata mismatch: {name}')
            require((run / f'{prefix}_{camera}.png').is_file(), f'RGB image missing: {name}')
            depth_reports.append({'path': name, 'shape': list(data.shape), 'dtype': str(data.dtype),
                                  'finite_pixels': int(finite.size), 'no_hit_positive_infinity_pixels': int(np.isposinf(data).sum()),
                                  'minimum_m': float(finite.min()), 'maximum_m': float(finite.max()),
                                  'standard_deviation_m': float(finite.std())})
    initial_meta = read(run / 'initial_depth_metadata.json')
    require(len({tuple(initial_meta[name]['shape']) for name in CAMERAS}) == 1, 'Different camera resolutions')
    height, width = initial_meta['head']['shape']

    usd_reports = []
    scenes = ['scene.usdc', 'scene_final.usdc'] + (['scene_reconfigured.usdc'] if full_demo else [])
    for name in scenes:
        path = run / name
        stage = Usd.Stage.Open(str(path))
        require(stage is not None, f'Cannot open USD: {name}')
        require(UsdGeom.GetStageUpAxis(stage) == 'Z' and abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) < 1e-12,
                f'Wrong USD units/up-axis: {name}')
        paths = ['/World/RB3/link6/Gripper/TCP'] + [f'/World/RB3/link{i}' for i in range(7)]
        for category, count in (('panel_paths', 9), ('plug_paths', 6), ('rotary_paths', 3)):
            require(len(handles[category]) == count, f'Incorrect {category} count')
            paths.extend(handles[category].values())
        paths.extend(f'/World/Cameras/{camera}' for camera in CAMERAS)
        require(all(stage.GetPrimAtPath(p).IsValid() for p in paths), f'Expected robot/task/camera prim missing in {name}')
        meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
        require(len(meshes) >= 49, f'Manufacturer robot meshes missing from {name}')
        triangles = sum(int(np.sum(np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=int) - 2)) for mesh in meshes)
        require(triangles >= 133550, f'Incomplete original robot mesh: {name}')
        require(all(np.isfinite(np.asarray(mesh.GetPointsAttr().Get(), dtype=float)).all() for mesh in meshes),
                f'Nonfinite mesh points: {name}')
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
        require(not unresolved, f'Unresolved USD assets in {name}: {list(unresolved)}')
        usd_reports.append({'path': name, 'mesh_count': len(meshes), 'triangulated_faces': triangles,
                            'checked_prim_paths': len(paths), 'external_assets': [str(asset) for asset in assets],
                            'layer_count': len(layers), 'unresolved': []})
        if name == 'scene_final.usdc':
            for plug, prim_path in handles['plug_paths'].items():
                actual = np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(0)).T
                target = np.asarray(state['plug_poses'][plug], dtype=float)
                require(target.shape == (4, 4) and np.isfinite(target).all() and np.allclose(actual, target, atol=1e-7),
                        f'Final exported pose differs from result: {plug}')
    require(set(result['videos']) == {'demo.mp4', 'head.mp4', 'detail.mp4', 'wrist.mp4'}, 'Missing overview/camera video')
    videos = [video_check(run / name, ffmpeg, frames, fps, [2 * width, height + 64] if name == 'demo.mp4' else [width, height])
              for name in result['videos']]
    with wave.open(str(run / 'clicks.wav'), 'rb') as audio:
        require(audio.getnchannels() == 1 and audio.getsampwidth() == 2, 'Unexpected click audio format')
        sample_rate = audio.getframerate()
        samples = np.frombuffer(audio.readframes(audio.getnframes()), dtype='<i2').astype(float) / 32768
    require(abs(len(samples) / sample_rate - frames / fps) < 1 / fps, 'Audio duration differs from video')
    click_levels = []
    for click in clicks:
        start = round(click['time_s'] * sample_rate)
        segment = samples[start:start + round(.035 * sample_rate)]
        rms = float(np.sqrt(np.mean(segment**2))) if segment.size else 0.
        require(rms > .005, f'No audible click waveform at {click["time_s"]}')
        click_levels.append({'time_s': click['time_s'], 'state': click['state'], 'rms': rms})
    return {'passed': True, 'run': str(run), 'mode': result['mode'], 'full_demo': full_demo,
            'completed': True, 'truncated': result['truncated'], 'frames': frames, 'fps': fps, 'duration_s': frames / fps,
            'event_counts': dict(Counter(event['event'] for event in events)), 'insertions': dict(insertions),
            'rotary_clicks': click_levels, 'final_plug_state': state['plug_state'], 'final_angles_deg': state['angles_deg'],
            'ik_checks': len(ik), 'max_ik_position_error_m': max((entry['position_error_m'] for entry in ik), default=None),
            'trace_frames': len(trace), 'depth': depth_reports, 'usd': usd_reports, 'videos': videos,
            'depth_note': 'Positive infinity is explicitly reported as no-hit background, not finite metric depth.',
            'scope': 'Saved artifact consistency and decoding; no claim of calibrated camera geometry or physical contact accuracy.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--ffmpeg', type=Path)
    parser.add_argument('--output', type=Path, help='Default RUN/validation.json')
    parser.add_argument('--allow-truncated', action='store_true')
    args = parser.parse_args()
    if args.ffmpeg is None:
        import imageio_ffmpeg
        args.ffmpeg = Path(imageio_ffmpeg.get_ffmpeg_exe())
    output = args.output or args.run / 'validation.json'
    try:
        report = validate(args.run, args.ffmpeg, args.allow_truncated)
    except Exception as error:
        report = {'passed': False, 'run': str(args.run.resolve()), 'error_type': type(error).__name__, 'error': str(error)}
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    if report['passed']:
        print(f"PASS: {report['frames']} frames, {len(report['videos'])} fully decoded H.264/AAC videos, "
              f"{len(report['depth'])} depth arrays, {len(report['usd'])} USD scenes. Report: {output}")
    else:
        print(f"FAIL: {report['error_type']}: {report['error']}\nReport: {output}", file=sys.stderr)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
