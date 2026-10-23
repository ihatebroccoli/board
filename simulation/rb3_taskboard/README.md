# RB3-730 모듈형 작업 보드

실제 teleoperation 기록을 참고해 만든 Isaac Sim 4.5의 headless 장면입니다. **환경 외형, 패널 교환, 집기·체결·회전, 영상 저장**이 목적이며 observation/action/reward 등의 강화학습 MDP는 정의하지 않습니다.

Git 공유 범위, 패키지 버전, 다른 서버의 실행 경로는 [SHARING.md](SHARING.md)에 정리되어 있습니다. 검증·분석용 의존성 목록은 `requirements-inspect.txt`, `requirements-reference.txt`이며, 실제 사용한 환경은 `environment-record.json`에 기록했습니다.

## 생성된 결과

locker 걸쇠의 확대 영상은 `outputs/rb3_taskboard_20261023/locker_closeup.mp4`, 0°/90° 비교는 `locker_states.png`입니다.

검증한 전체 시연은 `outputs/rb3_taskboard_20261023/demo.mp4`, 실제/시뮬레이션 비교는 같은 폴더의 `real_vs_sim.png`에서 확인할 수 있습니다. `scene.usdc`는 외부 메시 경로 없이 열 수 있는 시작 장면이며, 상호작용은 아래 Python 실행기로 구동합니다.

## 서버에서 실행

저장소 루트(`/home/marin0625/dexhand`)에서 실행합니다. 이 서버의 기존 Isaac Sim 런타임과 영상 인코더를 사용하므로 데스크톱/GUI 세션은 필요하지 않습니다.

```bash
# 장면 확인
python3 simulation/rb3_taskboard/launch.py --gpu 3 --mode preview

# 집기·체결 및 회전 시연
python3 simulation/rb3_taskboard/launch.py --gpu 3 --mode demo

# 기록된 관절 궤적 재생
python3 simulation/rb3_taskboard/launch.py --gpu 3 --mode replay
```

결과는 기본적으로 `logs/simulation/rb3_taskboard/runs/<실행 시각>/`에 저장됩니다. 실행 시 출력되는 경로의 `console.log`에서 진행 상황을 볼 수 있습니다. `--fps 12`, `--width 640 --height 480`, `--max-frames 60`으로 영상 크기·길이를 조절할 수 있습니다. `--output <새 디렉터리>`로 결과 위치를 지정합니다. 기존 디렉터리는 덮어쓰지 않습니다.

| 결과 파일 | 내용 |
|---|---|
| `demo.mp4` | 외부 시점과 상세 시점을 나란히 표시한 영상, 작업 상태 표시 |
| `head.mp4`, `detail.mp4`, `wrist.mp4` | 외부·상세·손목 시점의 개별 영상 |
| `preview.jpg`, `initial_*.png`, `final_*.png` | 미리보기 및 시작·종료 RGB 이미지 |
| `*_depth_m.npy`, `*_depth_metadata.json` | 시작·종료 등의 시뮬레이션 카메라 축 방향 depth와 단위 정보. 원본 raw depth와는 별개 |
| `scene.usdc`, `scene_final.usdc`, `scene_reconfigured.usdc` | 시작·종료·패널 재배치 장면 USD |
| `trace.json`, `events.json`, `ik_checks.json` | 프레임별 관절·TCP·물체 상태, 조작 이벤트, IK 결과 |
| `handles.json`, `scene_config.json`, `result.json` | 소켓·손잡이 좌표, 사용 설정, 완료·중도 제한 여부 |

rotary 상태 전환 시 합성한 짧은 클릭 소리를 MP4에 넣습니다. 실제 장치에서 녹음한 소리는 아닙니다. `--max-frames`로 자른 경우 `result.json`의 `truncated`가 `true`입니다.

첫 실행은 제조사 저장소의 고정 revision에서 RB3-730ES-U URDF/mesh를 받아 `logs/simulation/rb3_taskboard/vendor/`에 보관합니다. 이미 준비된 asset을 사용할 때는 `--skip-assets`를 지정할 수 있습니다. 런타임은 `logs/simulation/wonik_v5/runtime/isaac-sim-4.5.0`, 인코더 환경은 같은 상위 디렉터리의 `inspection_env`를 사용합니다. 공유 런타임의 중복 실행은 잠금으로 차단합니다.

## 패널 및 상호작용 설정

`config/scene.json`을 복사해 수정한 뒤 `--config <설정.json>`으로 실행합니다. 각 패널의 크기는 요청한 **9×9×3.5cm**이며, 아래에 8mm 받침판이 있습니다. `layout`은 로봇 쪽에서 앞쪽으로 세 행, 외부 카메라에서 왼쪽에서 오른쪽으로 세 열입니다.

```json
[
  ["usb", "locker", "outlet"],
  ["audio", "hdmi", "latch"],
  ["lan", "rotary", "bnc"]
]
```

아홉 이름을 한 번씩 유지하면서 위치를 바꿀 수 있습니다. `fixture_yaw_deg`는 보드 안에서 장치가 놓인 방향을 지정하며 HDMI·latch·locker는 외부 영상에 맞춰 90°로 설정했습니다. 체결 시 플러그 방향도 소켓 방향에 맞춰집니다. 좌표는 미터 단위의 오른손 좌표계이며 Z가 위, X가 보드 앞쪽, Y+가 외부 카메라 오른쪽입니다. 테이블 상판은 Z=0입니다. 커넥터 식별과 배치는 영상 기반 추정이며 작은 USB/audio/LAN/HDMI 구멍은 명확하게 판별되지 않아 설정으로 수정할 수 있습니다.

| 대상 | 동작 |
|---|---|
| outlet, audio, usb, lan, hdmi, bnc | 그리퍼가 가까우면 집기, 팔을 따라 이동, 올바른 소켓 가까이에서 정렬·고정, 분리 |
| rotary | 45–105° 범위. 기본 80° 이상에서 ON, 70° 이하에서 OFF로 전환하며 끝 각도로 스냅하고 `click` 이벤트 기록 |
| locker | 0–90° 회전과 걸쇠 슬라이딩을 연동. 0°에서 돌출·잠금, 90°에서 수납·해제, 중간 각도에서는 연속 이동 |
| latch | 막힌 부분을 제외한 회전을 ±165°로 근사. 정확한 차단 각도는 `latch_limits_deg`로 조정 |

locker 걸쇠 이동량은 `interaction.locker_bolt_travel_m`으로 조정하며 기본 16mm입니다. 회전각에 비례해 하우징 안으로 들어가는 근사 링크 구조입니다. 프레임 상태의 `locker_bolt_retraction_m`에서 수납량을 확인할 수 있습니다.

기본 집기 거리는 28mm, 체결 거리는 40mm, 체결 축 오차 허용치는 40°입니다. `interaction`에서 바꿀 수 있습니다. rotary의 실제 임계 토크는 각도 임계점과 히스테리시스로 대신합니다. 회전은 TCP가 손잡이에서 55mm 이내에 있을 때 수행합니다.

Python에서는 Isaac `SimulationApp`을 초기화한 뒤 `controller.SceneController`를 생성해 `move_tcp`, `grasp`, `release`, `insert`, `unplug`, `rotate`, `rearrange`, `reset`을 호출합니다. `move_tcp`의 회전 인수를 생략하면 그리퍼가 아래쪽을 향합니다. 예를 들어 현재 랙 위치와 소켓 위치를 사용하면 배치 변경에도 좌표를 하드코딩하지 않을 수 있습니다.

```python
controller.pick_and_insert("usb")
controller.rearrange([
    ["lan", "hdmi", "bnc"],
    ["audio", "usb", "latch"],
    ["outlet", "rotary", "locker"],
])
controller.reset()
```

`reset()`은 플러그와 회전 상태를 초기화하며 현재 패널 배치는 유지합니다. 자세한 Python 호출은 `controller.py`에 있습니다.

## 배치 조합 시연

`outputs/rb3_layout_combinations_20261023/layout_demo.mp4`에서 기본 A → 조합 B → 조합 C 전환을 확인할 수 있습니다. `layout_comparison.png`는 세 배치와 각 칸의 task 이름을 나란히 보여줍니다. B에서 USB를 체결한 후 보드와 함께 C로 옮기고, 초기화 후 C의 새 위치에서도 USB를 집어 체결합니다.

```bash
python3 simulation/rb3_taskboard/launch.py --gpu 3 --mode commands \
  --commands simulation/rb3_taskboard/config/layout_demo_commands.json
```

재배치는 `rearrange` 명령의 3×3 `layout` 배열로 지정합니다. 보드 단위로 소켓·손잡이·걸쇠·이미 체결된 플러그를 함께 옮기며 다음 체결 목표도 갱신합니다. 랙의 미체결 플러그 위치는 그대로 유지됩니다.

## JSON 명령으로 조작

바로 실행할 예시는 `config/example_commands.json`에 있습니다. 아래 JSON을 별도로 `/tmp/rb3_commands.json`에 저장해 실행할 수도 있습니다. 좌표 예시는 기본 `scene.json`의 USB 플러그·소켓 및 rotary 위치에 맞춰져 있습니다. 배치를 바꾸면 preview 실행 결과의 `handles.json`을 확인하거나 위 Python API에서 좌표를 읽어 사용합니다.

```bash
python3 simulation/rb3_taskboard/launch.py --gpu 3 --mode commands \
  --commands /tmp/rb3_commands.json
```

```json
[
  {"op": "move_tcp", "position": [0.4195, -0.20, 0.1865]},
  {"op": "move_tcp", "position": [0.4195, -0.20, 0.0465]},
  {"op": "grasp", "object": "usb"},
  {"op": "move_tcp", "position": [0.4195, -0.20, 0.1865]},
  {"op": "move_tcp", "position": [0.35, -0.09, 0.2415]},
  {"op": "move_tcp", "position": [0.35, -0.09, 0.1135]},
  {"op": "insert", "socket": "usb"},
  {"op": "move_tcp", "position": [0.35, -0.09, 0.25]},
  {"op": "move_tcp", "position": [0.53, 0.0, 0.215]},
  {"op": "move_tcp", "position": [0.53, 0.0, 0.087]},
  {"op": "rotate", "object": "rotary", "angle_deg": 105, "duration": 1.0},
  {"op": "move_tcp", "position": [0.53, 0.0, 0.25]},
  {"op": "rearrange", "layout": [["lan", "hdmi", "bnc"], ["audio", "usb", "latch"], ["outlet", "rotary", "locker"]]},
  {"op": "wait", "seconds": 1.0},
  {"op": "reset"}
]
```

`{"op":"pick_and_insert","object":"usb"}` 한 명령으로 현재 좌표에 맞춰 접근·집기·운반·체결할 수도 있습니다. 추가 명령은 `release`, `unplug`(`socket` 지정)입니다. `move_tcp`는 선택적으로 3×3 회전 행렬 `rotation`과 이동 시간 `duration`을 받습니다. 거리·대상·각도 조건을 충족하지 않거나 IK에 실패한 명령은 오류로 중단됩니다.

## 원본 기록 다시 추출

원본은 `/home/marin0625/1_demos_2026-10-23_15-13-18_598539.pkl`입니다. 918개 샘플, 약 91.73초이며 외부 D455·손목 D405의 640×480 RGB/depth, 로봇 관절과 TCP 명령·그리퍼 상태를 포함합니다.

```bash
/home/marin0625/miniforge3/envs/metra/bin/python \
  simulation/rb3_taskboard/prepare_reference.py \
  --indices 0,100,200,230,350,460,575,690,800,917
```

`logs/simulation/rb3_taskboard/reference/`에 선택 프레임, raw depth `.npy`, 미리보기, `contact_sheet.jpg`, `robot_state.npz`, `manifest.json`을 생성합니다. 약 5.65GB 원본을 메모리에 한 번 로드하며 추출물은 약 21MB입니다. NumPy 배열 복원에 필요한 pickle global만 허용하고 원본 SHA-256·샘플 수·스키마·설정 출처를 기록합니다.

기본 추출은 `scene.json`의 카메라 추정치를 manifest에 기록합니다. 선택 옵션 `--refit-camera`는 **처음 사용한 RGB 네 모서리만의 추정**을 다시 계산해 manifest에 기록하며, 현재의 depth를 참고한 추정과 다릅니다. 어떤 경우에도 `scene.json`, 그리퍼 및 상호작용 설정은 수정하지 않습니다. 추가 의존성은 NumPy/Pillow이며 재추정에는 OpenCV/SciPy가 필요합니다. 받침판 높이를 바꾸면 카메라 추정용 `board_corners_world_m`의 Z도 패널 상단 높이와 일치하도록 수정해야 합니다.

이미 추출한 depth와 장면 추정치의 일관성은 다음 명령으로 확인합니다. 원본 PKL을 다시 로드하지 않고 `depth_consistency.json`과 검토용 `candidate_config.json`을 reference 디렉터리에 씁니다. 실행 설정은 자동 변경하지 않습니다.

```bash
/home/marin0625/miniforge3/envs/metra/bin/python \
  simulation/rb3_taskboard/prepare_reference.py --depth-consistency-only
```

외부 카메라 추정은 보드의 흰 평면 약 7,772픽셀과 테이블 12,032픽셀의 raw depth 평면·기울기를 참고합니다. D455의 **1 raw unit=1mm라는 미확인 가정** 아래 초점거리 약 386.6px를 선택했으며, 보드 모서리 RGB 오차는 약 1.45px, 보드 depth 상대 오차의 중앙값은 약 0.29%입니다. RGB-depth 정렬과 depth 단위를 확인한 값은 아니므로 `depth_unit_m`는 `null`로 유지합니다. D405에는 이 가정을 적용하지 않습니다.

## 복원 범위

- 팔은 제조사 RB3-730ES-U 관절 구조와 mesh를 사용합니다. 실제 세부 모델·베이스 위치·TCP 오프셋을 보정한 상태는 아닙니다. 그리퍼는 요청한 10cm 길이의 단순 평행 그리퍼이며 최대 벌림은 약 7.5cm입니다.
- 패널 치수는 지정값을 사용하지만 소켓·플러그·래치·랙의 세부 치수, 재질, 테이블, 카메라 장착 위치는 영상 기반 근사입니다.
- 외부 카메라는 27cm 보드의 네 모서리와 raw depth의 평면 구조를 함께 참고하되 중심 주점, 왜곡 없음, D455의 임시 1mm 단위를 가정했습니다. 초점거리·depth 단위·RGB-depth 정렬의 불확실성이 남으며 실제 보정값이 아닙니다. 손목 카메라도 추정 장착값입니다.
- 실측 카메라 보정과 depth 단위가 없어 metric depth 융합이나 정밀 3D 스캔 복원은 수행하지 않았습니다. 원본 depth는 raw uint16로 보존합니다.
- 로봇은 관절과 역기구학을 이용한 kinematic 구동입니다. 힘·토크·접촉·케이블 변형이나 충돌 회피 경로 계획은 구현하지 않습니다. 체결은 가까운 위치에서 명령을 내리면 정렬·고정하는 상태 전환입니다.
- `replay`는 기록된 관절 각도를 시각화합니다. 실제 좌표계 등록과 물체 조작 이벤트 복원이 없으므로 실제 시연을 정확히 재현하는 기능은 아닙니다.

## 검증

```bash
/ssddisk/marin0625/dexhand/simulation/wonik_v5/inspection_env/bin/python \
  -m unittest discover -s simulation/rb3_taskboard -p 'test_*.py'

/ssddisk/marin0625/dexhand/simulation/wonik_v5/inspection_env/bin/python \
  simulation/rb3_taskboard/validate_run.py outputs/rb3_taskboard_20261023
```

논리·장면 통합 테스트 16개는 6종 체결, 잘못된 소켓·거리·각도 거부, 물체 운반, 회전 범위와 detent, 재배치, 초기화, 비정상 pose 거부를 검사합니다. 출력 검증기는 MP4 전체 디코딩·프레임 수·클릭 오디오, depth 배열, USD 외부 의존성과 최종 물체 위치를 확인하고 `validation.json`을 기록합니다.
