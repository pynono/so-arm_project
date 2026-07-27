# ArmTwin

SO-ARM101 로봇 팔을 브라우저에서 움직이고, 그 움직임을 3D 모델이 실시간으로 똑같이 따라 그리는 컨트롤러입니다. 팔 옆에 화면 하나 띄워두면 슬라이더를 당기는 대로 실물과 3D 트윈이 같이 움직여요.

파이썬만 있으면 됩니다. Node도, 빌드 도구도, 인터넷도 필요 없어요. Windows · macOS · Linux 다 돌아갑니다.

## 기능

- 관절 6개를 슬라이더나 `−`/`+` 버튼으로 움직이고, 관절마다 따로 잠그거나 풀 수 있어요.
- **Teach** — 모터를 풀고 손으로 팔을 원하는 자세로 옮긴 뒤 이름 붙여 저장. 나중에 그 자세로 다시 보냅니다.
- **Record & Play** — 손으로 움직인 궤적을 녹화했다가 속도 조절해서 재생.
- **디지털 트윈** — 실제 STL 메시로 만든 3D 모델이 로봇을 실시간으로 따라 움직입니다. 마우스로 돌려보고 줌.
- **상태 창** — 관절별 위치·각도·온도·전압·부하·토크를 한눈에.

## 시작하기

**Linux / macOS**

```bash
./scripts/install.sh      # venv 만들고 의존성 설치 (처음 한 번만)
./run.sh                  # http://127.0.0.1:8000
```

**Windows**

```bat
scripts\install.bat
run.bat
```

브라우저에서 `http://127.0.0.1:8000` 열고 **Connect** 누르면 됩니다.

시리얼 포트가 `/dev/ttyACM0`이 아니면 바꿔서 실행하세요. UI 사이드바에서 언제든 바꿀 수도 있어요.

```bash
./run.sh --serial /dev/ttyUSB0 --port 8000
run.bat  --serial COM4        --port 8000
```

포트 이름은 OS마다 다릅니다 — Linux는 `/dev/ttyACM0`(또는 `ttyUSB0`), macOS는 `/dev/tty.usbmodem*`, Windows는 `COM3`·`COM4` 식. Linux에서 매번 `sudo` 없이 쓰려면 한 번만 그룹에 넣어두세요.

```bash
sudo usermod -aG dialout $USER   # 로그아웃 후 다시 로그인
```

## 새 로봇이면 캘리브부터

같은 모델이라도 서보가 박힌 각도가 조금씩 달라서, 새 본체는 한 번 캘리브를 돌려야 슬라이더·트윈·실물이 안 어긋납니다.

```bash
python3 scripts/calibrate.py
```

관절을 손으로 양 끝까지 밀고 Enter 누르는 식이에요. 값은 `data/calibration.json`에 저장되고 서버가 뜰 때 알아서 읽습니다. 자세한 순서는 [docs/CALIBRATION.md](docs/CALIBRATION.md)에 있어요.


## team 이름으로 구분하고 싶다면

팀마다 로봇이 붙은 메인PC 한 대씩. 각 메인PC에서 한 번만 돌리면 됩니다.

```bash
sudo ./scripts/setup_team_pc.sh team1     # team2, team3, ...
```

호스트 이름을 `team1`으로 잡고, avahi를 깔아 학생들이 `http://team1.local:8000`으로 들어올 수 있게 하고, 부팅할 때 컨트롤러가 자동으로 뜨도록 systemd 서비스를 걸어줍니다.

한 팀 학생들이 같은 주소를 함께 씁니다. 기본적으로 누구나 움직일 수 있고, 메인PC 화면의 관리자 패널에서 학생별로 제어권을 회수하거나 다시 돌려줄 수 있어요. 회수된 학생은 상태와 트윈은 계속 보되 움직이지는 못합니다. (관리자 패널은 메인PC 본체에서만 열립니다.)

## 폴더 구조

```
armtwin/
├── app.py                 # 서버 진입점 (API + 웹 UI 서빙)
├── backend/
│   ├── controller.py      # 로봇 상태 관리 + Teach/녹화/재생
│   └── routes.py          # HTTP 라우트 + WebSocket
├── sdk/
│   ├── driver_sdk.py      # Feetech STS3215 시리얼 드라이버 (저수준)
│   ├── soarm.py           # 학생용 Robot 클래스 (HTTP)
│   └── examples/          # 입문 스크립트 + lessons/ 실습
├── frontend/
│   ├── index.html
│   ├── css/ · js/         # app.js · api.js · twin.js · kinematics.js
│   └── vendor/three/      # three.js 번들 (오프라인)
├── model/
│   ├── so101.urdf         # 로봇 정의
│   ├── kinematics.json    # 관절 체인 (백엔드·프론트 공유)
│   └── assets/*.stl       # 링크 메시
├── data/                  # calibration.json + 저장된 포즈/녹화
├── docs/                  # STUDENT_GUIDE · CALIBRATION · PICK_AND_PLACE
├── scripts/               # install · calibrate · check_direction · setup_team_pc
├── run.sh / run.bat
└── requirements.txt
```


## API 훑어보기

프론트와 SDK가 쓰는 `/api` 엔드포인트입니다. 직접 뭔가 만들 때 참고하세요.

| 메서드 | 경로 | 하는 일 |
|---|---|---|
| GET | `/api/status` | 현재 연결·관절 상태 |
| POST | `/api/connect` · `/api/disconnect` | 시리얼 열기 / 닫기 |
| POST | `/api/move` · `/api/move_all` | 관절 하나 / 여러 개 이동 |
| POST | `/api/torque` | 토크 켜기·끄기 (전체 또는 하나) |
| POST | `/api/center` | 전 관절 중앙으로 |
| GET/POST/DELETE | `/api/poses*` | 포즈 저장·이동·삭제 |
| POST | `/api/record/*` · `/api/playback/*` | 녹화 / 재생 |
| WS | `/api/ws` | 상태 실시간 스트림 |

## 3D 엔진

three.js r160(core + OrbitControls + STLLoader)을 [frontend/vendor/three/](frontend/vendor/three/) 버전을 올릴 경우 그 파일 세 개만 갈아끼우면 됩니다.

## 배포

zip

```bash
tar -czf armtwin.tar.gz --exclude=venv --exclude=__pycache__ .
```
압축 풀고 `scripts/install.sh`(또는 `install.bat`) 돌린 다음 `run.sh`

## 막히면

- **연결이 안 돼요 (Linux)** — `groups` 쳐서 `dialout`이 있는지 확인. 없으면 위 설치 단계대로 넣고 다시 로그인.
- **3D 화면이 까매요** — 브라우저 콘솔을 열어보세요. three.js가 로컬에 번들돼 있어서 보통은 파일이 안 읽힌 거예요. Ctrl-Shift-R로 강제 새로고침.
- **슬라이더는 움직이는데 로봇이 안 움직여요** — 그 관절이 *Locked*가 아니라서예요. 관절 오른쪽 토크 버튼을 누르거나 상단 **Lock All**.
- **전원 껐다 켜니 위치가 이상해요** — 서보가 캘리브를 잊은 겁니다. *Teach & Poses*의 *Read Pose*로 UI를 다시 맞추세요.
