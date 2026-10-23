# RB3 Taskboard Git 공유 및 의존성

Git에 올릴 핵심 경로는 **`simulation/rb3_taskboard/` 전체**입니다. 기본 장면은 코드로 생성하며 로봇 메시만 제조사 저장소에서 내려받습니다. `preview`, `demo`, `commands` 모드는 원본 PKL 없이 실행됩니다.

## 1. Git에 포함할 범위

별도 저장소를 만들 때 아래 계층을 유지합니다. 여러 파일이 `__file__`의 상위 디렉터리로 프로젝트 루트를 계산하므로 모듈 파일을 저장소 최상위로 평탄화하면 기본 경로가 달라집니다.

```text
rb3-taskboard/                       # 새 저장소 루트
├── .gitignore                      # gitignore-root.example을 복사
└── simulation/
    └── rb3_taskboard/
        ├── README.md               # 기능·조작 안내
        ├── SHARING.md              # 이 문서
        ├── environment-record.json # 실제 사용한 환경 기록
        ├── requirements-inspect.txt
        ├── requirements-reference.txt
        ├── launch.py              # 현재 서버용 런처
        ├── run_scene.py           # Isaac 실행 및 영상·depth·USD 저장
        ├── controller.py          # 이동·집기·체결·회전·재배치 API
        ├── task_model.py          # 작업 상태, 거리 조건, 회전 범위
        ├── geometry.py            # 테이블·9개 보드·커넥터·걸쇠 모델링
        ├── robot.py               # RB3 메시·FK·IK·근사 그리퍼
        ├── fetch_robot.py         # 고정 revision의 공식 자산 다운로드
        ├── prepare_reference.py   # 선택: 실측 데이터 추출·카메라 추정
        ├── validate_run.py        # 저장 결과 검증
        ├── test_*.py              # 16개 단위·통합 테스트
        └── config/
            ├── scene.json
            ├── example_commands.json
            ├── locker_commands.json
            └── layout_demo_commands.json
```

`__pycache__/`, `*.pyc`는 제외합니다. 현재 프로젝트 최상위의 `requirements.txt`, `dexhand/`, 학습 스크립트와 `simulation/wonik_v5/` 소스는 이 장면의 코드 의존성에 포함되지 않습니다. 다만 현재 `launch.py`는 기존 Wonik 작업에서 설치해 둔 런타임과 영상 인코더 경로를 재사용합니다.

## 2. 현재 서버의 실제 경로

| 항목 | 현재 경로 | 공유 방법 |
|---|---|---|
| 코드·설정·테스트 | `/home/marin0625/dexhand/simulation/rb3_taskboard/` | Git에 포함. 문서 추가 전 소스 약 172 KiB |
| 공식 로봇 자산 | `/ssddisk/marin0625/dexhand/simulation/rb3_taskboard/vendor/rbpodo_description/` | 약 17 MB. `fetch_robot.py`로 다시 다운로드 |
| 선택적 추출 데이터 | `/ssddisk/marin0625/dexhand/simulation/rb3_taskboard/reference/` | 약 21 MB. 필요할 때 별도 배포 |
| 실행 결과 전체 | `/ssddisk/marin0625/dexhand/simulation/rb3_taskboard/runs/` | MP4·USD 등 선별해 Release/공유 저장소에 첨부 |
| Isaac Sim 4.5.0 | `/ssddisk/marin0625/dexhand/simulation/wonik_v5/runtime/isaac-sim-4.5.0/` | 약 15 GiB. 각 실행 환경에 별도 설치 |
| CPU 검증·FFmpeg 환경 | `/ssddisk/marin0625/dexhand/simulation/wonik_v5/inspection_env/` | 가상환경을 복사하지 않고 requirements로 재생성 |
| 원본 촬영 데이터 | `/home/marin0625/1_demos_2026-10-23_15-13-18_598539.pkl` | 5.65 GB. 원본 분석이 필요한 경우 별도 제공 |

프로젝트의 `logs`는 `/ssddisk/marin0625/dexhand`를 가리키는 심볼릭 링크입니다. 다른 서버에서는 저장소 루트에 실제 `logs/` 디렉터리를 만들어도 됩니다.

`outputs/rb3_taskboard_20261023`과 `outputs/rb3_layout_combinations_20261023`도 서버 내부의 절대 경로를 가리키는 링크입니다. Git에 링크를 추가하면 영상 본문은 포함되지 않습니다. 결과 공유 시에는 실제 파일을 복사합니다.

## 3. 용도별 패키지 의존성

### 시뮬레이션·렌더링

**검증한 배포판은 Isaac Sim standalone 4.5.0, 번들 Python 3.10.15입니다.** Isaac Sim의 Python 3.10 계열 요구는 [4.5.0.0 배포 메타데이터](https://pypi.org/project/isaacsim/4.5.0.0/)에서도 확인할 수 있습니다. 아래 실행 예시는 `python.sh`가 있는 standalone 설치를 기준으로 합니다.

| 구성 | 실제 사용 버전 / 모듈 | 설치 위치 |
|---|---|---|
| Isaac Sim | `4.5.0` | 외부 런타임 |
| Python | `3.10.15` | Isaac 번들 |
| NumPy | `1.26.0` | Isaac 번들 |
| Pillow | `11.0.0` | Isaac 번들 |
| 장면·카메라·렌더링 | `isaacsim.core.api`, `isaacsim.sensors.camera`, `omni.usd`, `omni.replicator.core` | Isaac 확장 |
| OpenUSD | `pxr.Gf`, `Usd`, `UsdGeom`, `UsdShade`, `UsdPhysics`, `Sdf` 등 | Isaac/Kit 번들 |
| FFmpeg | 실제 바이너리 `7.0.2`, H.264 `libx264` 및 AAC 인코더 | 별도 실행 파일 |
| 폰트 | `/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf` | OS의 `fonts-dejavu-core` 패키지 |

`SimulationApp`을 생성한 다음 `pxr` 및 `omni`를 import합니다. Isaac 번들 OpenUSD와 아래 CPU 환경의 `usd-core`는 각각의 환경에서 사용합니다. CPU용 `usd-core` wheel을 Isaac Python에 추가할 필요가 없습니다.

이 작업보드 코드의 직접 의존성에는 **ROS 2, Isaac Lab, PyTorch, Gymnasium, Open3D, trimesh, pycollada가 없습니다.** RB3 자산 저장소에 ROS라는 이름이 들어가지만 이 코드는 URDF와 메시 파일만 사용합니다.

실제 실행 서버는 Ubuntu 20.04.6 LTS, RTX 3090 24 GiB, 드라이버 535.183.01입니다. 이는 **동작을 확인한 환경 기록**이며 최소 사양 또는 최신 Isaac의 공식 지원 사양을 의미하지 않습니다. 렌더링에는 RTX 기능을 지원하는 NVIDIA GPU와 호환 드라이버가 필요하며 한 번 실행할 때 GPU 한 개를 사용합니다. 신규 설치는 사용할 Isaac 버전에 맞는 [NVIDIA 설치 안내](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html)를 확인합니다. 최신 문서의 사양을 4.5.0의 사양으로 간주하지 않습니다.

### CPU 테스트·출력 검증·FFmpeg 준비

[`requirements-inspect.txt`](requirements-inspect.txt)는 현재 Python 3.10.14 검증 환경의 최소 패키지를 고정합니다.

```text
numpy==1.26.4
usd-core==24.11
imageio-ffmpeg==0.6.0
```

`unittest`는 Python 표준 라이브러리입니다. 논리·장면 테스트에는 NumPy/OpenUSD가 필요하고, `validate_run.py`는 FFmpeg로 MP4 전체를 디코딩합니다. `imageio-ffmpeg`는 이 프로젝트에서 FFmpeg 실행 파일의 위치를 제공하는 역할입니다.

### 선택적 원본 분석

[`requirements-reference.txt`](requirements-reference.txt)는 원본 추출과 카메라/depth 분석에 사용한 버전을 기록합니다.

```text
numpy==1.24.4
Pillow==10.4.0
scipy==1.10.1
opencv-python==4.7.0.72
```

이 조합의 실제 분석 환경은 Python 3.8.18이었습니다. 기본 추출에는 NumPy와 Pillow만 사용하고, `--refit-camera` 또는 `--depth-consistency-only`에 OpenCV/SciPy가 추가로 필요합니다. `opencv-python`의 서버 import에는 환경에 따라 `libGL` 등 OS 라이브러리가 필요할 수 있습니다. GUI 기능을 쓰지 않으므로 배포 환경에 맞춰 `opencv-python-headless`를 선택할 수 있지만 이 문서의 기록된 환경은 일반 `opencv-python`입니다.

렌더링은 Isaac 번들 Python에서 실행하고, requirements 파일은 별도의 검증·분석 환경에 설치합니다. 환경을 하나로 합친 전체 `pip freeze`보다 이 용도 구분을 유지하는 편이 재현에 도움이 됩니다.

## 4. 다른 서버에서 실행할 때 바꿀 경로

| 코드 | 현재 전제 | 다른 환경에서의 지정 방법 |
|---|---|---|
| `launch.py` | `logs/simulation/wonik_v5/runtime/isaac-sim-4.5.0` | 동일 구조에 런타임을 배치하거나 아래처럼 `run_scene.py`를 직접 실행 |
| `launch.py` | `logs/simulation/wonik_v5/inspection_env/bin/python`에서 FFmpeg 조회 | 직접 실행 시 `--ffmpeg`로 실행 파일 지정 |
| `launch.py` / `run_scene.py` | 기본 GPU `3` | 일반적인 GPU 한 개 서버에서는 `--gpu 0` |
| `run_scene.py` | DejaVu Sans 절대 경로 | `fonts-dejavu-core` 설치 |
| `prepare_reference.py` | 현재 사용자의 원본 PKL 경로 | `--source /path/to/recording.pkl` |
| 테스트 | 루트의 `logs/simulation/rb3_taskboard/vendor` | `fetch_robot.py` 기본 경로로 자산 준비 |

`launch.py`는 Linux의 `fcntl` 잠금을 사용합니다. 현재 검증 범위는 Linux headless 실행이며, 다른 OS와 다른 Isaac 버전은 별도 확인이 필요합니다.

다음은 새 저장소에서 경로를 명시하는 실행 예시입니다. Isaac Sim 4.5.0 standalone 설치는 이미 완료되어 있다고 가정합니다. 폴더 이름과 `RB3_ISAAC_SIM` 값을 실제 경로로 바꿉니다.

```bash
cd /path/to/rb3-taskboard

python3.10 -m venv .venv-tools
.venv-tools/bin/python -m pip install -r simulation/rb3_taskboard/requirements-inspect.txt
.venv-tools/bin/python simulation/rb3_taskboard/fetch_robot.py

# Ubuntu에서 렌더링용 폰트가 없을 때:
# sudo apt-get install fonts-dejavu-core

export RB3_REPO_ROOT="$PWD"
export RB3_ISAAC_SIM="/path/to/isaac-sim-4.5.0"
export RB3_FFMPEG="$(.venv-tools/bin/python -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')"

# run_scene.py 직접 실행 시 출력 디렉터리를 먼저 만듭니다.
# 매 실행마다 새로운 출력 디렉터리 이름을 사용합니다.
mkdir -p logs/simulation/rb3_taskboard/reference
mkdir -p logs/simulation/rb3_taskboard/cache/kit
mkdir logs/simulation/rb3_taskboard/share-preview

"$RB3_ISAAC_SIM/python.sh" "$RB3_REPO_ROOT/simulation/rb3_taskboard/run_scene.py" \
  --output "$RB3_REPO_ROOT/logs/simulation/rb3_taskboard/share-preview" \
  --vendor "$RB3_REPO_ROOT/logs/simulation/rb3_taskboard/vendor" \
  --reference "$RB3_REPO_ROOT/logs/simulation/rb3_taskboard/reference" \
  --config "$RB3_REPO_ROOT/simulation/rb3_taskboard/config/scene.json" \
  --mode preview --gpu 0 --ffmpeg "$RB3_FFMPEG" \
  --portable --portable-root "$RB3_REPO_ROOT/logs/simulation/rb3_taskboard/cache/kit"
```

`--mode demo`로 전체 시연을 실행합니다. 조합 변경은 `--mode commands --commands "$RB3_REPO_ROOT/simulation/rb3_taskboard/config/layout_demo_commands.json"`을 사용합니다. 직접 실행은 현재 런처의 공유 캐시 잠금을 통과하지 않으므로 동일한 cache/output에 여러 프로세스를 동시에 실행하지 않습니다.

위 명령은 현재 서버 전용 경로를 우회하는 예시이며, 새 서버에서의 깨끗한 설치 전체를 검증한 기록은 아닙니다. 실제로 검증한 환경은 `environment-record.json`과 기존 결과의 `validation.json`에 기록되어 있습니다.

테스트와 결과 검증은 CPU 환경에서 실행합니다.

```bash
.venv-tools/bin/python -m unittest discover -s simulation/rb3_taskboard -p 'test_*.py'
.venv-tools/bin/python simulation/rb3_taskboard/validate_run.py \
  logs/simulation/rb3_taskboard/share-preview
```

## 5. 원본 데이터가 필요한 경우

| 기능 | 필요한 추가 파일 |
|---|---|
| `preview`, `demo`, `commands` | 없음. 원본 데이터 없이 장면 생성·상호작용 가능 |
| 실제/시뮬레이션 비교 이미지 | `reference/0000_head_cam_d455.png`가 있으면 생성 |
| `replay` | `reference/robot_state.npz` |
| PKL에서 새 reference 추출 | 원본 PKL + 분석용 Python 환경 |
| depth 일관성 분석 | 추출한 RGB/depth와 해당 `scene.json` |

원본을 공개하지 않고 관절 재생만 공유하려면 `robot_state.npz`를 별도 파일로 배포할 수 있습니다. 카메라 비교까지 보여주려면 선택한 RGB 이미지도 함께 제공합니다. 장면의 카메라·상호작용 설정은 `config/scene.json`에 포함되어 있습니다.

## 6. Git과 별도 결과 배포

새 저장소 루트의 `.gitignore`에는 [`gitignore-root.example`](gitignore-root.example)을 사용할 수 있습니다. 런타임, venv, raw PKL, 캐시, 실행 결과 전체를 제외합니다.

현재 작업 폴더의 Git 루트는 **`/home/marin0625`**이며 기존 `general_skill` 원격 저장소에 연결되어 있습니다. 이 장면을 독립적으로 공유하려면 새 디렉터리에 `simulation/rb3_taskboard/`만 복사하고 별도 저장소로 관리하는 구성이 적합합니다. 현재 홈 디렉터리에서 `git add .`로 모으지 않습니다.

예시 복사 명령은 다음과 같습니다. 실제 `git init`, 원격 등록, commit/push는 원하는 저장소에서 수행합니다.

```bash
mkdir -p /path/to/rb3-taskboard/simulation/rb3_taskboard
rsync -a --exclude='__pycache__' --exclude='*.pyc' \
  /home/marin0625/dexhand/simulation/rb3_taskboard/ \
  /path/to/rb3-taskboard/simulation/rb3_taskboard/
cp /path/to/rb3-taskboard/simulation/rb3_taskboard/gitignore-root.example \
  /path/to/rb3-taskboard/.gitignore
```

결과물은 선별해서 별도 Release/파일 공유에 첨부합니다.

- `demo.mp4`: 전체 시연
- `locker_closeup.mp4`, `locker_states.png`: 걸쇠 회전 연동 확인
- `layout_demo.mp4`, `layout_comparison.png`: 세 가지 보드 조합
- `scene.usdc`: 기본 정적 장면
- 필요 시 `scene_final.usdc`, `scene_reconfigured.usdc`, 사용 설정과 `validation.json`

기존 결과 링크에서 실제 파일을 복사하는 예시입니다.

```bash
mkdir -p /path/to/rb3-taskboard/artifacts
cp -L /home/marin0625/dexhand/outputs/rb3_taskboard_20261023/demo.mp4 \
  /path/to/rb3-taskboard/artifacts/
cp -L /home/marin0625/dexhand/outputs/rb3_taskboard_20261023/scene.usdc \
  /path/to/rb3-taskboard/artifacts/
cp -L /home/marin0625/dexhand/outputs/rb3_layout_combinations_20261023/layout_demo.mp4 \
  /path/to/rb3-taskboard/artifacts/
```

생성한 USD는 외부 메시 참조가 없어서 단독으로 열 수 있습니다. 집기·체결·걸쇠 연동을 실행하려면 Python 코드도 함께 필요합니다.

## 7. 자산 출처와 라이선스 기록

로봇 자산은 [RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2)의 revision `5e8294a985e7ce5e20e70564c2681130afc5f502`, 모델 `rb3_730es_u`를 사용합니다. 다운로드한 `rbpodo_description/LICENSE`는 Apache-2.0입니다.

`fetch_robot.py`가 자산과 함께 `LICENSE`, 각 파일 URL·SHA-256이 있는 `source_manifest.json`을 보관합니다. 로봇 메시나 이를 포함하는 USD를 별도로 공유할 때 출처와 라이선스 파일도 함께 제공합니다. 이 모듈 자체의 코드 라이선스는 아직 선택하지 않았으므로 저장소 소유자가 별도로 지정해야 합니다.

이 정리 작업에서는 Git 원격 생성·변경이나 업로드를 수행하지 않았습니다.
