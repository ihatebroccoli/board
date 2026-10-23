#!/usr/bin/env python3
"""Extract a compact, auditable reference from the recorded RB3 demonstration.

Only NumPy's array reconstruction globals are accepted by the pickle loader.
The recording is loaded into memory (about 6 GB); extracted images are limited
to representative frames. Raw depth is saved unchanged: its unit and camera
calibration were not recorded and must not be treated as measured calibration.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import pickle
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
DEFAULT_SOURCE = Path('/home/marin0625/1_demos_2026-10-23_15-13-18_598539.pkl')
CAMERAS = ('head_cam_d455', 'wrist_cam_d405_b')


class RestrictedNumpyUnpickler(pickle.Unpickler):
    """Reject executable globals except the constructors needed by NumPy arrays.

    This narrows the attack surface of pickle; it is not a resource sandbox for
    arbitrary adversarial input. No imports named by the input are performed.
    """

    def find_class(self, module, name):
        # np.core is the NumPy 1.x spelling; the accepted aliases also read 2.x
        # recordings when the simulation interpreter has NumPy 1.x installed.
        if module in ('numpy.core.multiarray', 'numpy._core.multiarray'):
            if name == '_reconstruct':
                return np.core.multiarray._reconstruct
            if name == 'scalar':
                return np.core.multiarray.scalar
        if module == 'numpy' and name in ('ndarray', 'dtype'):
            return {'ndarray': np.ndarray, 'dtype': np.dtype}[name]
        raise pickle.UnpicklingError('Disallowed pickle global: %s.%s' % (module, name))

    def persistent_load(self, pid):
        raise pickle.UnpicklingError('Persistent pickle references are not allowed')


def load_recording(path):
    with Path(path).open('rb') as stream:
        steps = RestrictedNumpyUnpickler(stream).load()
    if not isinstance(steps, list) or not steps:
        raise ValueError('Expected a nonempty list of demonstration steps')
    required = ('observations', 'actions', 'infos', 'time')
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or any(key not in step for key in required):
            raise ValueError('Invalid or incomplete step %d' % index)
        if not isinstance(step['observations'], dict):
            raise ValueError('Invalid observation at step %d' % index)
    return steps


def sha256_file(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def left_receipt(step):
    return step['infos'].get('command_receipts', {}).get('left', {})


def float_vector(value, size):
    if value is None:
        return np.full(size, np.nan)
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.size != size:
        raise ValueError('Expected %d values, received %d' % (size, array.size))
    return array


def image_array(value, depth=False):
    array = np.asarray(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if depth and array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if depth:
        if array.ndim != 2 or array.dtype != np.uint16:
            raise ValueError('Expected uint16 HxW depth image')
    elif array.ndim != 3 or array.shape[-1] != 3 or array.dtype != np.uint8:
        raise ValueError('Expected uint8 HxWx3 RGB image')
    return array


def observation_schema(observations):
    schema = {}
    for key, value in observations.items():
        array = np.asarray(value)
        schema[key] = {'shape': list(array.shape), 'dtype': str(array.dtype)}
    return schema


def export_trajectories(steps, destination):
    receipts = [left_receipt(step) for step in steps]
    times = np.array([step['time'] for step in steps], dtype=np.float64)
    actions = np.stack([np.asarray(step['actions']).reshape(-1) for step in steps])
    states = np.stack([np.asarray(step['observations']['state']).reshape(-1) for step in steps])
    qpos = np.stack([float_vector(receipt.get('current_qpos'), 6) for receipt in receipts])
    widths = np.array([r.get('current_gripper_telemetry', {}).get('distance_m', np.nan)
                       for r in receipts], dtype=np.float64)
    openings = np.array([r.get('gripper_observation', {}).get('opening_fraction', np.nan)
                         for r in receipts], dtype=np.float64)
    start_poses = np.stack([float_vector(r.get('command_receipt', {}).get('start_pose'), 7)
                            for r in receipts])
    executed = np.stack([float_vector(s['infos'].get('executed_action'), 7) for s in steps])
    robot_time = np.array([r.get('current_robot_timestamp', np.nan) for r in receipts])
    forces = np.stack([np.asarray(s['observations']['left_force_history'])[0, -1]
                       for s in steps])
    np.savez_compressed(destination, time=times, time_relative=times - times[0],
                        qpos=qpos, actions=actions, state=states,
                        executed_action=executed, gripper_width_m=widths,
                        gripper_opening_fraction=openings,
                        command_start_pose_xyz_quaternion=start_poses,
                        robot_timestamp=robot_time, force_history_last=forces)
    return {
        'file': destination.name,
        'sample_count': len(steps),
        'arrays': {
            'time': 'Recorded host Unix timestamp, seconds',
            'time_relative': 'Recorded host timestamp minus first sample, seconds',
            'qpos': 'infos.command_receipts.left.current_qpos; six joint angles in radians',
            'actions': 'Recorded absolute_tcp_xyz_rpy_gripper commands; source robot frame',
            'state': 'Raw 19-component observation; retained without reinterpreting its schema',
            'executed_action': 'infos.executed_action, flattened to seven components',
            'gripper_width_m': 'Measured distance_m from gripper telemetry',
            'gripper_opening_fraction': 'Measured gripper_observation.opening_fraction',
            'command_start_pose_xyz_quaternion': 'Raw command_receipt.start_pose; quaternion convention unverified',
            'robot_timestamp': 'current_robot_timestamp from command receipt, seconds',
            'force_history_last': 'Last raw six-component force-history sample; units/frame unverified',
        },
        'qpos_finite_samples': int(np.isfinite(qpos).all(axis=1).sum()),
        'initial_qpos_rad': qpos[0].tolist(),
        'initial_action': actions[0].tolist(),
        'coordinate_registration': 'Recorded robot coordinates are not calibrated to the reconstructed scene.',
    }


def export_frames(steps, indices, destination):
    records = []
    thumbnails = []
    for index in indices:
        observation = steps[index]['observations']
        frame = {'step': index, 'time_relative_s': float(steps[index]['time'] - steps[0]['time']),
                 'cameras': {}}
        for camera in CAMERAS:
            rgb = image_array(observation[camera])
            depth = image_array(observation[camera + '_depth_image'], depth=True)
            stem = '%04d_%s' % (index, camera)
            Image.fromarray(rgb).save(destination / (stem + '.png'))
            np.save(destination / (stem + '_depth_raw.npy'), depth, allow_pickle=False)
            positive = depth[depth > 0]
            quantiles = np.percentile(positive, [1, 5, 25, 50, 75, 95, 99]).tolist() if positive.size else []
            # Preview normalization is per frame. It has no metric meaning.
            preview = np.zeros(depth.shape, dtype=np.uint8)
            if positive.size:
                lo, hi = np.percentile(positive, [2, 98])
                preview = np.clip((depth.astype(float) - lo) / max(1, hi - lo) * 255, 0, 255).astype(np.uint8)
                preview[depth == 0] = 0
            Image.fromarray(preview).save(destination / (stem + '_depth_preview.png'))
            frame['cameras'][camera] = {
                'rgb': stem + '.png', 'depth_raw': stem + '_depth_raw.npy',
                'depth_preview': stem + '_depth_preview.png',
                'resolution': [int(rgb.shape[1]), int(rgb.shape[0])],
                'depth_nonzero_fraction': float(np.count_nonzero(depth) / depth.size),
                'depth_raw_quantiles_1_5_25_50_75_95_99': quantiles,
            }
            thumb = Image.fromarray(rgb).resize((320, 240))
            label = Image.new('RGB', (320, 266), (23, 25, 28))
            label.paste(thumb, (0, 26))
            ImageDraw.Draw(label).text((8, 7), '%04d | %.1fs | %s' % (index, frame['time_relative_s'], camera), fill='white')
            thumbnails.append(label)
        records.append(frame)
    sheet = Image.new('RGB', (640, 266 * len(indices)), (23, 25, 28))
    for index, thumb in enumerate(thumbnails):
        sheet.paste(thumb, (320 * (index % 2), 266 * (index // 2)))
    sheet.save(destination / 'contact_sheet.jpg', quality=90)
    return records


def refit_head_camera(config):
    """Repeat the illustrative plane fit; requires optional OpenCV and SciPy."""
    import cv2
    from scipy.optimize import minimize_scalar
    fit = config['head_camera']['estimate_provenance']
    world = np.asarray(fit['board_corners_world_m'], dtype=np.float64)
    pixels = np.asarray(fit['board_corners_image_px'], dtype=np.float64)
    top_z = (config['board_center_m'][2] + config.get('baseplate_height', 0.)
             + config['panel_size_m'][2])
    if world.shape != (4, 3) or not np.allclose(world[:, 2], top_z, atol=1e-6):
        raise ValueError('Camera-fit corner Z must match board center + baseplate height '
                         '+ panel height (%.6f m); update the config corner plane.' % top_z)
    if pixels.shape != (4, 2) or not np.isfinite(world).all() or not np.isfinite(pixels).all():
        raise ValueError('Camera fit requires four finite image/world corner correspondences')
    cx, cy = fit['assumed_principal_point_px']

    def solve(focal):
        intrinsic = np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1.]])
        ok, rvec, tvec = cv2.solvePnP(world, pixels, intrinsic, np.zeros(5))
        if not ok:
            raise RuntimeError('Camera plane fit failed')
        projected = cv2.projectPoints(world, rvec, tvec, intrinsic, np.zeros(5))[0].reshape(-1, 2)
        return projected - pixels, cv2.Rodrigues(rvec)[0], tvec.reshape(3)

    solution = minimize_scalar(lambda f: float(np.sum(solve(f)[0] ** 2)),
                               bounds=(300, 800), method='bounded')
    residual, rotation, translation = solve(solution.x)
    eye = -rotation.T @ translation
    if eye[2] <= 0:
        raise ValueError('Plane fit placed camera below the board; check corner ordering')
    result = dict(config['head_camera'])
    result.update(eye_m=eye.tolist(), target_m=(eye + rotation.T[:, 2]).tolist(),
                  up=(-rotation.T[:, 1]).tolist(),
                  intrinsics_px={'fx': float(solution.x), 'fy': float(solution.x), 'cx': cx, 'cy': cy},
                  focal_length_mm=float(solution.x) * result['horizontal_aperture_mm'] / result['resolution'][0])
    result.pop('depth_assumption', None)
    result.pop('assumed_scale_m_per_raw_unit_for_fit', None)
    result['estimate_provenance'] = dict(
        fit, fit_rms_per_coordinate_px=float(np.sqrt(np.mean(residual ** 2))),
        method='Original RGB-only fit: minimize planar solvePnP reprojection error over shared focal length in [300,800] px; four hand-picked board corners; no depth fitting.',
        confidence='Low: the near-exact four-corner fit is sensitive to manually selected corners and does not validate pose, scale, distortion, or depth registration.',
        depth_note='This optional RGB-only refit does not use depth. Raw depth units remain unknown; no calibration is established.')
    return result


def depth_consistency_report(reference, config):
    """Diagnose the scene estimate against raw head depth, without calibrating it.

    Masks are hand-selected for frame 0 of this recording. This intentionally
    separate diagnostic never assumes D405 has the D455's depth scale and never
    writes the live scene config. RGB/depth registration is an unverified premise.
    """
    import cv2
    from scipy.optimize import brentq, least_squares, minimize_scalar

    reference = Path(reference)
    raw = np.load(reference / '0000_head_cam_d455_depth_raw.npy', allow_pickle=False)
    rgb = np.asarray(Image.open(reference / '0000_head_cam_d455.png'), dtype=float)
    if raw.shape != (480, 640) or rgb.shape != (480, 640, 3):
        raise ValueError('Diagnostic masks require the original 640x480 frame 0')
    yy, xx = np.indices(raw.shape)
    camera = config['head_camera']
    provenance = camera['estimate_provenance']
    corners = np.asarray(provenance['board_corners_image_px'], dtype=float)
    world = np.asarray(provenance['board_corners_world_m'], dtype=float)
    cx, cy = provenance['assumed_principal_point_px']
    board_top = float(np.median(world[:, 2]))
    board = np.zeros(raw.shape, np.uint8)
    cv2.fillConvexPoly(board, corners.astype(int), 1)
    board = cv2.erode(board, np.ones((9, 9), np.uint8)).astype(bool)
    exclusions = [(270, 224, 13), (315, 228, 25), (368, 225, 30),
                  (267, 269, 14), (319, 265, 22), (368, 269, 28),
                  (261, 313, 16), (319, 314, 21), (373, 314, 14)]
    for u, v, radius in exclusions:
        board &= (xx - u) ** 2 + (yy - v) ** 2 > radius ** 2
    board &= (rgb.min(axis=2) > 170) & (rgb.max(axis=2) - rgb.min(axis=2) < 45) & (raw > 0)
    table = np.zeros(raw.shape, dtype=bool)
    table_rectangles = [(130, 300, 165, 370), (430, 260, 500, 340),
                        (240, 374, 420, 392), (195, 172, 230, 195)]
    for u0, v0, u1, v1 in table_rectangles:
        table[v0:v1, u0:u1] = True
    table &= raw > 0
    regions = {'board': (board, board_top), 'table': (table, 0.)}

    def quantiles(values):
        return {str(q): float(np.percentile(values, q)) for q in (5, 50, 90, 95, 99)}

    def region_plane(mask):
        observed = raw[mask].astype(float)
        design = np.stack([xx[mask] - cx, yy[mask] - cy, np.ones(mask.sum())], axis=1)
        response = 1000. / observed
        start = np.linalg.lstsq(design, response, rcond=None)[0]
        solution = least_squares(lambda p: design @ p - response, start,
                                 loss='soft_l1', f_scale=.004).x
        fitted = 1000. / (design @ solution)
        relative = (observed - fitted) / fitted
        return {'pixels': int(mask.sum()), 'raw_depth_quantiles': quantiles(observed),
                'inverse_raw_depth_times_1000_coefficients': solution.tolist(),
                'coefficient_equation': '1000/raw = a*(u-cx) + b*(v-cy) + c; 1000 is numerical normalization, not an assumed metric unit',
                'absolute_relative_plane_error_percent': quantiles(100 * np.abs(relative)),
                'outlier_fraction_over_2_percent': float(np.mean(np.abs(relative) > .02))}

    def solve(focal):
        intrinsic = np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1.]])
        ok, rvec, tvec = cv2.solvePnP(world, corners, intrinsic, np.zeros(5))
        if not ok:
            raise ValueError('Planar camera fit failed')
        rotation = cv2.Rodrigues(rvec)[0]
        eye = -rotation.T @ tvec.reshape(3)
        projection = cv2.projectPoints(world, rvec, tvec, intrinsic, np.zeros(5))[0].reshape(-1, 2)
        return rotation, eye, projection

    def candidate(focal):
        rotation, eye, projection = solve(focal)
        result = {'focal_px': float(focal), 'corner_rms_per_coordinate_px': float(np.sqrt(np.mean((projection - corners) ** 2))),
                  'eye_m': eye.tolist(), 'target_m': (eye + rotation.T[:, 2]).tolist(),
                  'up': (-rotation.T[:, 1]).tolist(), 'regions': {}}
        combined_predicted, combined_raw = [], []
        for name, (mask, z) in regions.items():
            rays = np.stack([(xx[mask] - cx) / focal, (yy[mask] - cy) / focal, np.ones(mask.sum())], axis=1)
            predicted = (z - eye[2]) / (rays @ rotation)[:, 2]
            observed = raw[mask].astype(float)
            scale = float(np.median(predicted / observed))
            error = (observed * scale - predicted) / predicted
            result['regions'][name] = {
                'apparent_scale_m_per_raw_unit': scale,
                'absolute_relative_error_after_region_scale_percent': quantiles(100 * np.abs(error)),
                'signed_relative_error_at_unverified_1mm_scale_percent': quantiles(100 * (observed * .001 / predicted - 1)),
                'scaled_error_fraction_over_2_percent': float(np.mean(np.abs(error) > .02)),
                'scaled_error_fraction_over_5_percent': float(np.mean(np.abs(error) > .05)),
            }
            combined_predicted.append(predicted)
            combined_raw.append(observed)
        predicted, observed = np.concatenate(combined_predicted), np.concatenate(combined_raw)
        scale = float(np.median(predicted / observed))
        relative = observed * scale / predicted - 1
        result['combined_apparent_scale_m_per_raw_unit'] = scale
        result['combined_absolute_relative_error_percent'] = quantiles(100 * np.abs(relative))
        result['clipped_depth_shape_objective'] = float(np.mean(np.minimum(np.abs(relative), .02) ** 2))
        return result

    current = candidate(camera['intrinsics_px']['fx'])
    nominal_focal = brentq(lambda f: candidate(f)['regions']['board']['apparent_scale_m_per_raw_unit'] - .001, 350., 420.)
    nominal = candidate(nominal_focal)
    optimum = minimize_scalar(lambda f: candidate(f)['clipped_depth_shape_objective'], bounds=(300, 550), method='bounded')
    free_scale = candidate(optimum.x)
    table_points = {}
    rotation, eye, _ = solve(nominal_focal)
    for name, (u, v) in {'front_left': (64, 407), 'front_right': (575, 403),
                         'back_left_visible_uncertain': (176, 153)}.items():
        ray = rotation.T @ np.array([(u - cx) / nominal_focal, (v - cy) / nominal_focal, 1.])
        optical_z = -eye[2] / ray[2]
        patch = raw[v-3:v+4, u-3:u+4]
        nonzero = patch[patch > 0]
        table_points[name] = {'image_px': [u, v], 'world_on_table_plane_m': (eye + ray * optical_z).tolist(),
                              'predicted_optical_z_m': float(optical_z),
                              'raw_depth_at_pixel': int(raw[v, u]),
                              'raw_depth_7x7_valid_median': float(np.median(nonzero)) if nonzero.size else None}
    temporal = []
    for path in sorted(reference.glob('*_head_cam_d455_depth_raw.npy')):
        step = int(path.name.split('_')[0])
        frame = np.load(path, allow_pickle=False)
        valid = board & (frame > 0)
        difference = (frame[valid].astype(float) - raw[valid]) / raw[valid]
        inliers = np.abs(difference) <= .02
        temporal.append({'step': step, 'valid_baseline_board_pixels': int(valid.sum()),
                         'fraction_changed_over_2_percent_including_occlusion': float(np.mean(~inliers)),
                         'median_absolute_difference_percent_all_valid': float(np.median(np.abs(difference)) * 100),
                         'median_absolute_difference_percent_within_2_percent': float(np.median(np.abs(difference[inliers])) * 100) if inliers.any() else None})
    report = {
        'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'diagnostic_under_unverified_RGB_depth_alignment_and_camera_assumptions',
        'source_manifest': 'manifest.json', 'head_reference_step': 0,
        'reference_rgb_sha256': sha256_file(reference / '0000_head_cam_d455.png'),
        'reference_raw_depth_sha256': sha256_file(reference / '0000_head_cam_d455_depth_raw.npy'),
        'scene_configuration_sha256_sorted_json': hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
        'head_masks': {'board_polygon_px': corners.tolist(), 'board_erosion_kernel_px': [9, 9],
                       'board_fixture_exclusion_circles_u_v_radius_px': exclusions,
                       'board_rgb_filter': 'min(channel)>170 and max(channel)-min(channel)<45',
                       'table_rectangles_u0_v0_u1_v1_px': table_rectangles},
        'raw_depth_plane_fits': {name: region_plane(mask) for name, (mask, _) in regions.items()},
        'current_camera_diagnostic': current,
        'original_RGB_corner_only_candidate': candidate(refit_head_camera(config)['intrinsics_px']['fx']),
        'nominal_1mm_hypothesis_candidate': nominal,
        'free_scale_depth_shape_candidate': free_scale,
        'focal_sensitivity_candidates': [candidate(f) for f in (320., 340., 360., 380., 400., 420., 440., 460., 480.)],
        'nominal_candidate_table_backprojections': table_points,
        'temporal_baseline_board_consistency': temporal,
        'wrist_d405': {'depth_scale_estimated': False, 'reason': 'No registered measured wrist camera pose or independently picked board correspondences are available. D455 scale hypotheses must not be copied to D405.'},
        'interpretation': [
            'Selected board/table depth pixels have a coherent planar structure, so raw depth provides useful relative geometric evidence.',
            'The near-exact four-corner RGB-only camera fit overfits hand-picked image corners relative to the observed depth gradient.',
            'A roughly 387 px focal estimate explains the head depth well if one raw D455 unit is provisionally assumed to be 1 mm; this is an unverified hypothesis, not measured calibration.',
            'The free-scale optimum differs, demonstrating unresolved focal/scale/registration ambiguity. Lens distortion, RGB-depth alignment, manual corner selection, and approximate board/table heights can all affect the result.',
            'Raw depths remain unchanged. No metric point-cloud fusion or automatic replacement of scene.json is performed.',
        ],
    }
    candidate_config = deepcopy(config)
    revised = candidate_config['head_camera']
    for key in ('eye_m', 'target_m', 'up'):
        revised[key] = nominal[key]
    revised['intrinsics_px']['fx'] = revised['intrinsics_px']['fy'] = nominal_focal
    revised['focal_length_mm'] = nominal_focal * revised['horizontal_aperture_mm'] / revised['resolution'][0]
    revised['depth_unit_m'] = None
    revised['assumed_scale_m_per_raw_unit_for_fit'] = .001
    revised['depth_assumption'] = {'assumed_scale_m_per_raw_unit': .001, 'verified': False,
                                  'purpose': 'Camera-estimation hypothesis only; do not convert raw depths as calibrated measurements.',
                                  'RGB_depth_alignment_verified': False}
    revised['estimate_provenance']['method'] = 'Planar solvePnP from manually selected RGB corners with focal chosen so median board predicted optical depth/raw depth equals the unverified nominal 0.001 m/raw D455 hypothesis.'
    revised['estimate_provenance']['fit_rms_per_coordinate_px'] = nominal['corner_rms_per_coordinate_px']
    revised['estimate_provenance']['confidence'] = 'Estimated, not calibrated. Supported by relative raw-depth planarity and gradients, but depth units and RGB-depth registration remain unverified.'
    revised['estimate_provenance']['depth_note'] = 'Raw uint16 preserved. See depth_consistency.json for scale/focal ambiguity, residuals, table-plane consistency, and the independent D405 limitation.'
    candidate_config['candidate_status'] = 'Proposed depth-informed head-camera estimate; not automatically applied to live config.'
    return report, candidate_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, default=PROJECT / 'logs/simulation/rb3_taskboard/reference')
    parser.add_argument('--config', type=Path, default=HERE / 'config/scene.json')
    parser.add_argument('--frame-count', type=int, default=10)
    parser.add_argument('--indices', help='Optional comma-separated sample indices instead of evenly spaced frames')
    parser.add_argument('--refit-camera', action='store_true', help='Recompute the manual-corner camera estimate in the output manifest (OpenCV/SciPy required); never changes scene.json')
    parser.add_argument('--depth-consistency-only', action='store_true', help='Analyze existing extracted head RGB/raw depth; write depth_consistency.json and candidate_config.json only (OpenCV/SciPy required)')
    args = parser.parse_args()
    if args.frame_count < 1:
        parser.error('--frame-count must be positive')
    if args.depth_consistency_only:
        config = json.loads(args.config.read_text())
        report, candidate_config = depth_consistency_report(args.output, config)
        for name, value in [('depth_consistency.json', report), ('candidate_config.json', candidate_config)]:
            target = args.output / name
            target.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
            print('Saved %s' % target, flush=True)
        return 0
    source = args.source.resolve()
    stat = source.stat()
    print('Hashing %s (%.2f GB)...' % (source.name, stat.st_size / 1e9), flush=True)
    digest = sha256_file(source)
    print('Loading recording with restricted NumPy pickle loader...', flush=True)
    steps = load_recording(source)
    indices = sorted(set(int(i) for i in args.indices.split(','))) if args.indices else sorted(set(
        np.linspace(0, len(steps) - 1, min(args.frame_count, len(steps)), dtype=int).tolist()))
    if not indices or indices[0] < 0 or indices[-1] >= len(steps):
        parser.error('Frame indices must be in [0, %d]' % (len(steps) - 1))
    times = np.array([step['time'] for step in steps], dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Recording timestamps must be finite and strictly increasing')
    config_bytes = args.config.read_bytes()
    config = json.loads(config_bytes)
    head_camera = refit_head_camera(config) if args.refit_camera else config['head_camera']
    args.output.mkdir(parents=True, exist_ok=True)
    print('Extracting %d of %d frames per camera...' % (len(indices), len(steps)), flush=True)
    trajectory = export_trajectories(steps, args.output / 'robot_state.npz')
    frames = export_frames(steps, indices, args.output)
    camera_timestamps = [left_receipt(step).get('current_camera_timestamp') for step in steps]
    manifest = {
        'schema_version': 1,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source': {'path': str(source), 'bytes': stat.st_size, 'sha256': digest,
                   'mtime_ns': stat.st_mtime_ns},
        'scene_config': {'path': str(args.config.resolve()),
                         'sha256': hashlib.sha256(config_bytes).hexdigest(),
                         'camera_refitted_for_manifest_only': bool(args.refit_camera),
                         'board_center_m': config['board_center_m'],
                         'panel_size_m': config['panel_size_m'],
                         'baseplate_height_m': config.get('baseplate_height', 0.),
                         'gripper': config['gripper']},
        'sample_count': len(steps), 'duration_seconds': float(times[-1] - times[0]),
        'top_level_keys': list(steps[0]),
        'observation_schema': observation_schema(steps[0]['observations']),
        'action_formats': sorted(set(step['infos'].get('action_format', 'unspecified') for step in steps)),
        'step_interval_s_min_median_max': np.percentile(np.diff(times), [0, 50, 100]).tolist() if len(times) > 1 else [],
        'camera_timestamp_nonnull_samples': sum(value is not None for value in camera_timestamps),
        'force_history_nonzero_samples': sum(bool(np.any(step['observations']['left_force_history'])) for step in steps),
        'trajectory': trajectory, 'frames': frames, 'contact_sheet': 'contact_sheet.jpg',
        'camera_estimates': {'head_camera': head_camera, 'wrist_camera': config['wrist_camera']},
        'limitations': [
            'No measured intrinsic, extrinsic, or depth-scale calibration was supplied.',
            'Head-camera estimates use manually chosen corners of a 27 cm board, centered principal point, square pixels, and zero distortion. Any diagnostic depth-scale hypothesis is unverified; fit residuals do not establish calibration accuracy.',
            'Wrist mount pose and intrinsics are illustrative estimates, not reconstructed calibration.',
            'Depth arrays retain raw uint16 units; metric depth values are not inferred from device model names.',
            'Recorded TCP coordinates are not registered to the scene; qpos replay also depends on unverified robot base/flange conventions.',
            'Connector identities/layout are visual estimates. RGB images are retained in the recorded channel order, interpreted as RGB.',
        ],
    }
    if source.stat().st_size != stat.st_size or source.stat().st_mtime_ns != stat.st_mtime_ns:
        raise RuntimeError('Source changed during extraction; do not use the generated reference')
    target = args.output / 'manifest.json'
    temporary = args.output / 'manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(target)
    print('Saved %s; %.2fs, %d samples, SHA-256 %s' % (target, manifest['duration_seconds'], len(steps), digest), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
