# gazebo/ — 칩 + Gazebo 연결 시험 (hardware-in-the-loop)

칩(ESP32-S3)이 로봇 소프트웨어 전체(SLAM, AMCL, 위치 잃음 감지, 경로 계획, 주행 제어)를 돌립니다. Gazebo는 PC에서 로봇 몸체, 라이다, 방을 시뮬레이션합니다. 둘은 USB 직렬 통신(bridge)으로 한 주기씩 맞춰 움직입니다.

## 필요한 것 (PC, Ubuntu 22.04)
- Gazebo Sim 8 (Harmonic)
- 시스템 Python용 모듈: `sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-serial`
- 확인: `/usr/bin/python3 -c "import gz.transport13, gz.msgs10, serial; print('ok')"`
  (pyenv나 conda의 `python3`가 아니라 `/usr/bin/python3`로 실행합니다.)
- 칩 펌웨어 0.11 이상 (`k ... gz` 명령)

## 실행
```
cd firmware
bash test.sh gz                    # 업로드 + 보드 확인 + room 미션 (Gazebo 창 + 브라우저 뷰어)
bash test.sh gz rt nobuild         # 실시간 속도, 업로드 없이
GZ_CMD="k 3 1 0 3" bash test.sh gz nobuild     # 다른 미션 (3 = doors)
```
직접 실행: `/usr/bin/python3 tools/gzbridge.py --chip "k 0 1 0 3" --gui --view`
- `--gui`: Gazebo 창, `--view`: 브라우저 뷰어(http://localhost:8765), `--rt`: 실시간
- `--record f.bin`: 칩이 보낸 것 저장(`tools/viewer.py --play f.bin`로 다시 보기)
- `--csv f.csv`: 주기마다 실제 위치, 오도메트리, 속도 명령, 칩의 추정 위치

## 한 주기 (lock step)
1. 칩: 주기 t의 node들을 실행하고 `/cmd_vel`을 보냅니다. 미션이 요청하면 `/sim/teleport`(납치)와 `/sim/box`(지도에 없는 상자)도 보내고, 마지막에 `/gz/done t`를 보냅니다.
2. PC(`tools/gzbridge.py`): 속도를 Gazebo에 주고(납치, 상자도 반영), 세계를 1/7초(물리 100스텝) 진행한 뒤 새 라이다 스캔을 기다립니다. 그다음 `/ground_truth`, `/odom`, `/gz/scan`, `/gz/contacts`, `/gz/step t+1`을 보냅니다.
3. 칩: `/gz/step t+1`이 올 때까지 기다립니다. 5초 동안 안 오면 실행을 멈춥니다(watchdog). 그다음 `gz_lidar` node가 `/gz/scan`을 X4 바이트로 바꾸고, `x4_driver`가 실제 라이다와 같은 방식으로 해석합니다.

PC가 느려도 결과는 같습니다. 칩 시간 기준으로 한 주기씩 진행하기 때문입니다.

## 파일
- `worlds/*.sdf`: 미션 세계 5개(room, s_corridor, s_corridor_lm, doors, room_tilted)와 로봇.
  - `test_host/gz_worlds.cpp`가 칩의 세계 정의(`src/mission_core.h`)에서 만듭니다. 손으로 고치지 않습니다. 파일이 없으면 `bash test.sh gz`가 만듭니다.
  - 벽: 칩의 5 cm 격자에서 점유 칸을 직사각형으로 합친 것(높이 0.5 m). room_tilted는 기울어진 직사각형 그대로입니다.
  - 로봇: 반지름 0.18 m 원통, 바퀴 2개(간격 0.24 m, 반지름 0.035 m), 캐스터 2개, 접촉 센서.
  - 라이다: 720점, 7 Hz, 0.12~12 m, 잡음 표준편차 5 mm.
  - 다시 만들기: `cd test_host && g++ -O2 -std=c++17 -I../src gz_worlds.cpp -o gzw && ./gzw ../gazebo/worlds`
- `server.log`: `tools/gzbridge.py`가 띄운 Gazebo 서버의 출력.

## 칩 없이 PC만으로 시험
`test_host/test_gz.cpp`는 칩과 같은 코드를 PC에서 돌리고, USB 대신 가상 터미널을 씁니다.
```
cd test_host && g++ -O2 -std=c++17 -pthread -I../src test_gz.cpp -o tgz -lutil && ./tgz     # /dev/pts/N 출력
/usr/bin/python3 tools/gzbridge.py --port /dev/pts/N --chip "k 0 1 0 3" --view               # 다른 터미널에서
```

## 라이다가 안 나올 때
- 라이다는 렌더링(ogre2)이 필요합니다. `gzbridge.py`는 먼저 `--headless-rendering`으로 띄웁니다.
- 스캔이 오지 않으면 그 옵션 없이 다시 띄웁니다.
- 둘 다 안 되면 `gazebo/server.log`를 확인합니다.
