UBTS 열 추종 실행기(row executor) · 현장 실행 안내 (2026-10-22 작성, 10/24–25 방문용; 1·2일차 검토·통합 보정 반영)
================================================================================
[조향 브랜치 사본 2026-11-09] 단일열 작업 546fa8c에서 가져온 문서다. 회전(turn_*, TURN_ASSIST.txt)·재생(replay_*, row_shadow_replay.py)·convert_1003.py·row_gate_report.py는 가져오지 않았다. 바꾼 것: docs/steering/EXECUTOR_IMPORT_KO.md.
10/23 로컬 후보 추가: 수동 SHADOW에서만 보조 회전 관찰 카드(/api/turn_observe)를 사용할 수 있다.
(조향 브랜치 사본에서는 이 요청을 모든 모드에서 거부한다: turn_observe_not_available. 아래 회전 관찰 설명은 이력이다.)
TurnAssist는 INFO_ONLY, 서버에서 observe=True / motor_output=False 고정, Arm·signed 출력·자동 인계 없음.
기존 열 run과 동시에 실행하지 않으며 MOTOR·bench·shadow-auto는 관찰 시작을 거부한다.
관찰 종료는 차량 정지가 아니다(대시보드 STOP·물리 E-stop 사용). 최초 cold kit는 Root가 별도 설치·ARM check했다.
현 local source의 SHADOW 시작 판정/로그 보존 v2 수정은 아직 설치하지 않았다. 초기 cold kit/installed bytes는 그대로다.
SHADOW v2는 종료 컨테이너와 CID별 private 시작 로그를 보존한다. 기존 소유자·정확한 CID를 확인하기 전 삭제하지 않는다.
token 파일은 root0600이므로 일반 cat 실패 시 권한을 완화하지 말고 docker logs URL 또는 권한 있는 읽기를 사용한다.
정확한 패키지 지문·집중 시험·설치 준비는 DAY3 상태 문서(2026-10-23)와 install_kit/MANIFEST.json 참조.

현장 보정은 12절의 한 절차와 한 설정 생성기(tools/field_calib/make_field_config.py)로 한다. 인쇄용 시트:
2026-10-22 현장 자율주행 작업 폴더의 field_sheets/CALIB_SHEET.html(같은 절차).
이 문서는 Jetson에서 실행기를 돌리는 방법이다. 개발 작업 중에는 Jetson에 접속하지 않았고 이 스크립트도 Jetson에서
실행하지 않았다. Jetson 설치·실행은 사용자 결정(WEEKEND_PLAN D1·D2)이 있어야 한다.
1일차 검토(2026-10-22 현장 자율주행 작업 폴더의 review_safety_day1 S1–S10, review_correctness_day1 F1–F9)에서 바뀐 동작은
[S#]/[F#]로, 2일차 검토(review_safety_day2 N1–N9, review_correctness_day2 R1–R9)에서 바뀐 동작은 [N#]/[R#]로 표시했다.

0. 무엇을 하는가
----------------
- MVP-0 (기본, 반드시): SHADOW. /scan, /imu/data_raw, /rover/control/requested를 구독해 열 추종 제안(좌우 duty),
  통로·열 끝·여유·정지 판단을 계산하고 기록만 한다. 실행기 컨테이너에는 ESP 브리지 소켓 디렉터리가 아예 mount되지
  않는다. 실행기는 차량을 제어하지 않는다. [S9] 화면 맨 위 주황색 띠가 이를 알리고, 큰 버튼은 "shadow run 종료
  (차량 정지 아님)"이라고 적혀 있다. 이 버튼은 shadow run만 끝낸다. 차량 정지는 대시보드 STOP과 물리 E-stop으로 한다.
- MVP-1 (조건부): MOTOR. 운용자가 버튼을 누르고 있는 동안만(hold-to-run) 전진하며 좌우 차등으로 통로 중앙을 따른다.
  pivot·후진은 없다. 정지 = min(열 끝 후보, 주행거리 ≥ L−2 m, 이동시간 ≥ 1.15·(L−2)/v_측정).
  마지막 2 m, headland 회전, 다음 열 정렬은 사람이 대시보드 수동 조종으로 한다.
- 현장 보정 run (10절): MOTOR에서 열 밖 넓은 곳에 정해진 duty 표를 hold-to-run으로 낸다. arc(C5: 차등별 yaw 권한),
  rolldown(C3b: 굴러가는 중 최저 duty), stop(C7: 실행기 0/0 정지 거리) [N8]. C0 차체 마스크는 정지 녹화로
  body_mask_static.py가 맞춘다. 녹화(C1–C4·C6·C7 대조)와 이 run을 합쳐 설정을 만드는 순서·규칙은 12절이다.
  모터 출력에는 세 가지가 모두 필요하다: (1) config.json의 motor_output_enabled=true(측정값 필수),
  (2) run_executor.sh motor(= --motor-output 플래그), (3) 운용자 화면에서 run마다 Arm.
  MOTOR 화면의 STOP 버튼은 "STOP (차량 정지)"이고 브리지에 op stop을 보낸다.

1. Jetson 배치 (한 번)
----------------------
  ~/row-executor/
    src/row_executor/      이 저장소 tools/row_executor 전체 복사 (SHA256SUMS 포함)
    SUMS_SHA256            (권장) PC에서 계산한 sha256(src/row_executor/SHA256SUMS) 한 줄
    IMAGE                  한 줄: 설치된 native observe 이미지 ID (sha256:…, 태그 금지). 기존 이미지를 그대로 쓴다.
                           (예: docker inspect --format '{{.Id}}' <native 컨테이너 이름>)
    config/config.json     실행기 설정 (컨테이너에는 읽기 전용)
    config/bench.json      V7(바퀴 띄움) 동안만. V7이 끝나면 지운다: rm ~/row-executor/config/bench.json  [S4]
    logs/                  JSONL 기록과 operator_token.txt (root 소유)
  - 새 이미지 빌드·apt 설치 없음. 소스·설정은 bind mount(:ro), 시작할 때 SHA256SUMS를 호스트(sha256sum -c)와
    컨테이너 안(executor_main.verify_sources)에서 두 번 확인한다. 하나라도 다르면 시작하지 않는다.
  - 대시보드·브리지·드라이버·native observer·펌웨어·compose는 건드리지 않는다. docker compose 명령은 쓰지 않는다.
  - 소스를 고치면 PC에서 python -B tools/row_executor/write_sha256sums.py로 SHA256SUMS를 다시 만들고 함께 복사한다.

2. 명령
-------
  ./run_executor.sh check          이미지 안에서 rclpy·numpy·scipy import, SHA256SUMS, vendored 해시, config 검사만 한다
                                   (--network none, ROS 노드 없음). 처음 한 번과 config를 바꿀 때마다.
  ./run_executor.sh shadow         SHADOW. 운용자 화면에서 Start(열 길이)·누르고 있기로 제안을 계산한다.
  ./run_executor.sh shadow --auto  SHADOW 자동: 누르기를 흉내 내고 run이 끝나면 1 s 뒤 새 run을 시작한다.
                                   사람이 대시보드로 직접 몰 때 제안을 계속 기록하는 용도(MVP-0). 정지 상태에서 출발할 때만
                                   통로를 잡는다(REACQUIRE는 정지를 요구한다).
  ./run_executor.sh motor          MOTOR. /run/ubts-rover(브리지 소켓 디렉터리)를 읽기 전용(:ro)으로 mount한다 [S10].
                                   읽기 전용 bind에서도 Unix 소켓 connect는 된다(Linux 컨테이너에서 실제 대시보드 브리지로
                                   arm·drive·stop과 브리지 재시작 뒤 재연결까지 확인). 브리지 소켓이 없으면 시작하지 않는다.
  ./run_executor.sh bench --wheels-off-ground-confirmed
                                   바퀴를 모두 띄운 벤치 전용(V7, config/bench.json, 5절). MOTOR와 같은 Arm·hold 절차. [S4]
                                   - 플래그가 없으면 거부한다(docker 명령 전에).
                                   - bench.json의 profile이 "bench"가 아니면(예: "field") 거부한다.
                                   - config/config.json이 motor_output_enabled=true(= 현장 설정)이면 거부한다.
                                   - 시작할 때 "바퀴 띄움·E-stop 손에·장애물/움직임을 보지 않음·V7 뒤 bench.json 삭제"를 출력한다.
                                   벤치 패턴은 장애물을 보지 않는다. 과수원에서는 절대 쓰지 않는다.
  ./run_executor.sh status | logs | stop
  보정 run(10절)은 별도 명령이 없다: config에 calib_motor_output_enabled=true를 두고 ./run_executor.sh motor, 화면의 '보정 Start'.
  - 컨테이너: --rm --restart no --network host --ipc host --user 0:0 --security-opt no-new-privileges, 이름 ubts-row-executor.
    ENTRYPOINT는 /bin/bash로 바꾸고 /opt/ros/$ROS_DISTRO/setup.bash를 직접 source한다. 실행기는 PID 1이며 SIGTERM을
    받으면 (MOTOR면) op stop을 보내고 기록을 닫고 끝난다.
  - 시작하면 로그에 운용자 화면 주소가 나온다: http://10.77.0.1:8097/?t=<토큰>. 토큰은 시작마다 바뀐다
    (logs/operator_token.txt에도 저장, root 소유이므로 sudo cat).
  - 설정 주소(bind_host)가 Jetson에 없으면(AP가 꺼져 있음) 실행기는 시작하지 않는다(종료 코드 3). AP가 켜진 뒤 다시 시작한다.
    집 벤치에서는 bind_host를 LAN 주소로 바꾼 별도 config를 쓴다(와일드카드·'0'·이름은 거부).

3. 운용자 화면 (노트북·폰, BTS_Jetson AP에 접속; 한 기기에서만 연다)
--------------------------------------------------------------------
  - 맨 위 띠 [S9]: SHADOW(주황) "실행기는 차량을 제어하지 않는다 … 아래 STOP은 shadow run만 끝낸다",
    MOTOR Arm 전(보라) "실행기는 아직 차량을 몰지 않는다. STOP = 차량 정지", MOTOR ARMED(빨강) "누르고 있는 동안 실행기가
    차량을 몬다". 상태 응답에도 vehicle_control(none/not_armed/armed)과 stop_button(ends_shadow_run_only/vehicle_stop)이 있다.
  - 큰 "누르고 있기" 버튼: 누르는 동안 100 ms마다 신호를 보낸다. 버튼은 pointer capture를 쓴다(터치는 원래 그렇다) [F8].
    그래서 손가락이 버튼 밖으로 미끄러져도 해제되지 않는다. 해제되는 경우: 손을 뗄 때(pointerup), pointercancel(시스템
    제스처·전화 등), 창이 포커스를 잃을 때, 화면이 꺼지거나 탭이 숨겨질 때, 페이지를 떠날 때. 노트북은 Space 키를
    누르고 있어도 된다. 서버는 마지막 신호 뒤 0.5 s면 그 누름을 죽은 것으로 보고 되살리지 않는다(네트워크가 잠깐 끊겨도
    저절로 다시 출발하지 않는다). 다시 누르면 새 누름(새 id)이다. 새 누름은 직전 해제 요청이 끝난 뒤(최대 0.7 s) 시작한다.
  - 두 기기 [S3]: 누름마다 id가 있다. 한 누름이 살아 있는 동안 다른 기기·탭이 누르면 둘 다 해제되고(주행 중이면 일시정지),
    두 id 모두 다시는 쓰이지 않는다. 화면에 "다른 기기/탭의 누름과 겹쳐 둘 다 해제됐다"가 뜬다. 다른 기기의 탭이 첫
    운용자의 누름을 되살리거나 새 출발을 만들지 못한다. 놓은(또는 만료된) 누름의 늦게 도착한 신호도 무시한다.
    다시 가려면 손을 떼고 새로 누른다. 원칙: 실행기 화면은 한 기기에서만 연다.
  - STOP(큰 버튼, Esc 키): MOTOR면 그 자리에서 브리지에 op stop을 보낸다(lease와 무관, 대시보드가 조종 중이어도 정지).
    실행기는 LATCHED가 되고 run이 끝난다. 다시 하려면 "Latch 확인" → Start → Arm. SHADOW에서는 shadow run만 끝난다.
    결과 표시 [S10]: 브리지 확인(초록) / SHADOW 종료(주황) / 실패는 큰 빨간 경고로 남는다(눌러서 닫기):
      HTTP 502 = 브리지가 확인하지 않음, "응답 없음/시간 초과" = 2.5 s 안에 답 없음 → 그 자리에서 대시보드 STOP·물리 E-stop.
      HTTP 403 = 이 화면의 토큰이 무효(실행기 재시작) → 화면이 버튼을 막고 5 s 뒤 안내 화면으로 다시 불러온다.
  - 토큰 무효 [S10]: 상태 조회가 403이면(실행기가 재시작되어 토큰이 바뀜) 같은 경고 후 안내 화면(403 페이지)으로 바뀐다.
    새 주소: run_executor.sh logs의 "operator page:" 줄 또는 sudo cat ~/row-executor/logs/operator_token.txt.
  - 연결 끊김: 상태 조회가 두 번 연속 실패하면 상태 표가 흐려지고 "표시 값은 오래된 것" 경고가 뜬다.
  - Start: 열 길이 L[m] 필수(시작 위치에서 마지막 줄기선까지 줄자로 잰 값). 거리 백스톱 = L−2 m.
    [R4] 열 중간에서 다시 Start할 때(종결 뒤, 통로 상실 반복 상한 뒤, 긴 열을 구간으로 나눌 때)도 L은 "지금 차 위치에서
    마지막 줄기선까지" 그 자리에서 줄자로 다시 잰다(또는 미리 둔 구간 콘까지의 값). 원래 L을 다시 넣거나 "L − 화면
    주행거리"를 넣지 않는다: 원래 L이면 백스톱이 이미 간 거리만큼 열 끝을 넘고, 화면 주행거리는 정합이 적게 셀 때 늦은
    정지로 이어진다. 화면 Start 칸 아래에도 이 안내가 있다.
  - Arm(MOTOR만): run이 시작돼 있고 누르고 있지 않을 때만. 브리지 상태 확인(연결·V2 펌웨어·무장 안 됨·정지·오류 없음)
    후 lease를 잡는다. 대시보드가 lease를 갖고 있으면 control_in_use로 거부된다(대시보드에서 STOP 먼저).
    Arm 전의 누르기는 무시된다. 순서: Start → Arm → 누르고 있기.
  - End run: 정상 종료(래치 없음). 현장 설정: base/reduce/diff(아래 5절), run이 없고 disarmed일 때만 바꿀 수 있다.
  - 상태 표: 상태·이유(일시정지 이유에는 다시 출발하는 법이 붙는다), 종결 이유와 end_note, 제안/전송 duty, 횡오차(+ 왼쪽),
    heading 오차, 통로 폭, 열 끝까지 거리, 전방/좌/우 여유, 입력 나이, 주행거리/한도, 이동시간/한도, 속도·yaw,
    scan 처리 시간·버린 scan 수, hold, bridge 상태, stale/pause 수, 기록 상태(7절), 루프 예외 수, feasibility.

4. 동작 규칙 (코드 그대로)
--------------------------
  - drive는 0/0을 포함해 항상 deadman:true. 연결 하나에 요청 하나. 10 Hz 전송 + 0이 되는 순간 즉시 전송.
    일시정지(누름 해제 등) 중에는 0/0을 계속 보내 lease를 유지한다.
  - 입력 신선도: LiDAR > 0.45 s 또는 IMU > 0.4 s면 0/0, 일시정지. 데이터가 돌아와도, 계속 누르고 있어도 다시 출발하지 않는다.
    입력이 1 s 이상 신선한 상태에서 "새로" 눌러야 재개한다. 한 run에 4번째 stale이면 run이 끝난다.
  - 메인 루프가 0.35 s 멈추면 감시 스레드가 drive(0,0)을 보내고 일시정지시킨다(새 누름으로만 재개, 한 run 4번째면 종료).
  - 종결은 종결로 남는다 [S1/F1]: 열 끝 후보·전방 막힘·옆 막힘·무응답(drive_no_response) 등 종결 결정 뒤 차가 서는 동안
    (제어기 STOP_REQUESTED, 약 0.6–3 s) 누름을 놓거나, 입력이 stale이 되거나, 루프가 멈춰도 run은 그 종결 이유로 끝난다
    (end_note "terminal_stop_pending_at_<사유>"). 다시 눌러도 다시 출발하지 않는다. 새 Start가 필요하다.
  - 통로 상실 [S2/F7]: 제어기가 통로를 잃으면 run은 일시정지(이유 corridor_lost; 재획득 대기 중이면 reacquire_needs_press)하고
    제어기를 버린다. 누르고 있어도
    저절로 다시 출발하지 않는다. 재개 = 입력이 신선하고 추정기가 정지(still)를 확인한 뒤의 "새" 누름. 차가 아직 미끄러지는
    중의 누름은 무시된다(press_ignored_not_standstill). 5 m(셈한 거리) 안에서 이미 3번 통로를 잃은 뒤 또 잃으면 run이 끝난다
    (corridor_lost_repeat_limit).
  - drive_no_response: 첫 전진 명령 뒤 약 3 s 동안 센서가 움직임을 못 보면 종결(바퀴를 띄운 벤치에서는 정상).
    [N9] 재개마다 제어기가 새로 시작해 3 s를 다시 세므로, 3 s 안에 놓았다 누르기를 반복하면 걸린 차를 계속 밀 수 있었다.
    이제 run 전체에서 "출력이 나가는데 움직임이 안 보인 시간"을 누적하고(움직임이 보이면 0으로), 6 s를 넘으면 종결한다
    (drive_no_response_repeat). 걸린 차를 누름 반복으로 빼내려 하지 말고 대시보드 수동 조종·사람이 한다.
  - 기하 guard: 주행 중 |heading 오차| > 25° 또는 차체가 줄기선에 닿는 횡오차면 종결(guard_heading / guard_lateral).
    출발 전 열 입구 정렬은 ±10° 이내로 한다.
  - 거리 백스톱 [F4]: 추정기 주행거리는 정지 중에도 scan 정합 지터로 늘어난다(10/10 재생 약 2.6 cm/s). 그래서 추정기가
    움직임을 보고하는 tick의 증가분만 센다(정지 중 증가분은 still_excluded_m로 따로 기록). run은 센 거리가 L−2 m에 닿으면
    끝나고, 차는 그 뒤 정지 거리만큼 더 가서 선다: 0.45 m/s면 약 L−2+0.28 ≈ L−1.7 m(정지 모델; SIL 중앙값 −1.80 m),
    오도메트리 오차 ±(50 m에 약 2 m). 기록의 backstop.expected_rest_m(Start 위치 기준, v_측정의 정지 모델 값)을 화면
    "주행거리 / 한도" 줄에 "(예상 정지 …)"로 보인다.
  - 정상 종결(열 끝 후보, 백스톱, 제어기 종결 HOLD): 0/0 → 정지 관측 또는 1.5 s → op stop(lease 해제, 수동 조종 가능).
    guard 위반·STOP·브리지 오류·시계 역행·종료: 즉시 op stop + LATCHED.
    [N3] bench가 움직임·전방 막힘·정합 상실로 끝날 때(bench_motion_*, bench_front_*, bench_odometry_*)와 보정 run이
    calib_yaw_rate·calib_overspeed·calib_heading_change로 끝날 때도 즉시 op stop + LATCHED(0/0 감속을 기다리지 않음).
    다시 하려면 "Latch 확인" → Start → Arm.
  - 어떤 브리지 오류든 LATCHED. 자동 재arm은 없다.
    [S7] 단, 브리지가 "lease가 이미 없다"고 답하면(control_not_owned, stop_barrier: 대시보드·관찰자 폰 STOP, 입력 만료 뒤)
    실행기는 op stop을 보내지 않고 LATCHED만 된다. 모터는 이미 그 STOP으로 멈췄고, 그 사이 사람이 대시보드에서 수동
    조종을 Arm했다면 실행기의 stop이 사람의 새 lease를 지우기 때문이다. lease를 가진 상태의 다른 오류(시간 초과·
    request_expired·control_busy_stopped 등)에는 지금처럼 op stop을 보낸다.
  - 루프 안 예외(버그) [S8]:
      MOTOR: 지금과 같다. 기록 후 STOP 경로(즉시 op stop, LATCHED, run 종료)를 타고 화면·기록은 계속한다.
             60 s 안에 6번째 예외면 프로세스가 끝난다(브리지 입력 만료 0.75 s·MCU watchdog 2 s가 모터를 멈춘다).
      SHADOW: 예외로 프로세스가 끝나지 않는다(기록이 산출물). 열린 run을 'loop_exception'으로 끝내고, 예외 수와 마지막
             예외를 화면에 보이며, 처음 10개는 전체 trace, 그 뒤는 10 s마다 요약 한 줄만 기록한다. 40 step 연속 실패하면
             Runner를 새로 만든다(최대 5번).
  - 시스템 시계 step [F5]: scan stamp는 header 간격을 수신 시계에 맞춘다(P4). 0.5 s를 넘는 step은 재설정으로 흡수되지만,
    그보다 작은 역행 step(예: −0.3 s)은 scan 몇 개를 버리고 LiDAR 나이를 0.5–0.6 s까지 올려 stale 일시정지를 한 번 만든다
    (약 2.5 s 뒤 정상). 그래서 시험 중에는 ubts-gnss-time-sync를 멈춰 둔다(D7): sudo systemctl stop ubts-gnss-time-sync.

5. 설정 (config.json; 기본값 = config.field_shadow.example.json)
-------------------------------------------------------------------
  C1 gate(2026-10-22 현장 자율주행 작업 폴더의 gate/C1_GATE_REPORT.txt 5절, 10/10 기록 6개 개방루프 재생)로 바꾼 기본값:
  | 키 | 값 | 근거 |
  | near_confirm_scans / range / tol / tol_step | 2 / 2.0 m / 0.35 m / 0.10 m | P1: 한 scan짜리 근거리 허상이 front_blocked의 대부분. front 72→5, 엄격 조기정지 21 m당→127 m당 1회. 대가: 새 근거리 물체를 0.2 s 늦게 인정 |
  | imu_max_age_s | 0.4 (SIM 0.3) | P2: 실제 IMU 수신 공백 최대 0.365 s, stale_imu 2→0 |
  | guard_heading_deg / guard_lateral_margin_m | 25 / 0.0 | P3: continuous는 큰 오차 정지가 없고, 차가 돌아선 뒤에도 띠가 valid였다(heading 180°, 횡 17.9 m) |
  | scan_stamp_mode | header_offset (창 50, 재설정 0.5 s) | P4: 수신이 몰리면(10/3 간격 0 ms) 속도 폭주·허위 정지. 0.5 s 넘는 시계 step만 재설정으로 흡수(4절 F5) |
  | corridor_every_n_scans | 3 (SIM 2) | P5: 재생 결정 같거나 나음, 처리 꼬리 감소. Jetson 실측(V5)에서 p99가 충분히 낮으면 2로 돌려도 된다 |
  그 밖: mode continuous_side, base 55 / reduce 55 / diff 20, arc 0.15, max_yaw 4.5, duty_cap 75, diff_cap 40, scan_latency 0.08,
  lidar_yaw_offset +179.08°, body_mask 빈 값(10/10 적합 결과; C0 정지 scan으로 재적합), row_end_continuation null.
  MOTOR 설정(motor_output_enabled=true)에는 측정값이 반드시 있어야 로드된다:
    breakaway_duty_pct (C2, 정수), v_measured_m_s (C3, base duty에서, ≤ 0.5), measured_yaw_at_max_diff_deg_s
    (C5, ±max_differential에서), measured_stop_distance_m (C7, 실행기 0/0 정지 거리: 보정 run 종류 stop) [N8],
    inner_rule_enabled=true.
    규칙: 안쪽 바퀴 base−diff ≥ breakaway+10(rolling을 쟀으면 rolling+10, 정지 출발은 base ≥ breakaway+10),
    reduce도 같다, max_yaw_rate ≤ 0.6 × 측정 yaw.
    [조향 GO 규칙, 통합 2026-10-22] 열 추종 MOTOR는 max_differential_pct 10–20(SIM 근거 범위)과 측정 yaw 권한
    ≥ 4.0 °/s(R−L = 2 × max_differential에서)를 실행기가 스스로 요구한다: min_yaw_authority_deg_s(기본 2.0)를 MOTOR에서
    max(설정값, 4.0)으로 올려 검사한다(연속 보정 모드 = 허용된 모든 모드). SHADOW 설정과 보정 run 설정(calib_motor_
    output_enabled)은 그대로다(기본값·digest 불변). 규칙 값은 config.py 끝의 field decision rules 블록 한 곳에 있다(12.3).
    예: C5에서 diff 20(|L−R| 40)으로 6 °/s를 쟀다면 max_yaw_rate_deg_s를 3.6 이하로 낮춰야 한다(4.5면 거부).
    MOTOR의 그 밖 설정 규칙(기하 guard null 금지, 정지 모델·측면 여유·신선도 한계를 기본값보다 느슨하게 못 함, body_mask
    거리 제한, bind_host 검사, 측정한 회전 중 임계값 rolling_breakaway_duty_pct·정지 거리 measured_stop_distance_m)은
    config.py 문서열(Fix A)이 원문이다.
    [N1] MOTOR(열 추종·보정 모두)에서는 아래도 평가된 현장 기본값보다 느슨하게 못 한다(config.py 2일차 블록이 원문):
      근거리 확인 필터(2 scan 이하·2.0 m 이하·0.35 m 이상·+0.10 m/scan 이상; 끄는 것(0)은 허용), range_min_floor ≤ 0.05,
      백스톱 여유 ≥ 2 m·시간 배수 ≤ 1.15, max_speed ≤ 0.55, stall_guard ≤ 0.35 s·반복 상한 3, stale 상한 3,
      bridge_timeout ≤ 0.6, hold_timeout ≤ 0.5, fresh_recover ≥ 1 s, inner_margin ≥ 10, yaw_authority_fraction ≤ 0.6,
      min_yaw_authority ≥ 2, footprint ≥ 1.0 × 0.55 m·LiDAR가 footprint 안, lidar_yaw_offset 179.08 ± 10°, imu_yaw_sign +1,
      breakaway·rolling ≥ 20 %, 제어기 override(corridor_lost_s, drive_grace_s, drive_no_response_s, drive_min_speed_m_s,
      still_confirm_s, max_stops_per_window, reacquire_updates)는 vendored 기본값보다 늦게 멈추는 쪽으로 못 바꾼다.
      SHADOW 설정에는 제한이 없다(제안만 계산).
  현장 설정(화면): base 30–70, reduce 30–70, diff 0–25, 그리고 cap(75/40)·안쪽 바퀴 규칙. MOTOR 설정이면 base와 diff는
    파일 값 이하로만 바꿀 수 있다(측정 속도·yaw 권한은 파일 값에서 잰 것). 올리려면 다시 재고 config.json을 고친 뒤 재시작.
    [R2] diff를 낮추면 측정 yaw 권한도 비례로 낮춰(측정값 × 새 diff / 파일 diff) 다시 검사하고, max_yaw_rate_deg_s를
    0.6 × 그 값 이하로 함께 낮춘다(파일 값보다 올라가지 않음). MOTOR 설정에서는 권한이 4.0 °/s 아래가 되거나 diff가 10 아래가
    되면 거부된다(조향 GO 규칙; 예: 파일 diff 20·측정 6 °/s → diff 14면 4.2 °/s·max_yaw 2.5, diff 13 이하는 거부).
    화면 "범위" 줄에 지금 값이 보인다.
    base를 낮추면 실제 속도가 v_measured보다 느려져 시간 백스톱이 일찍 걸린다(안전한 쪽). 그때는 C3를 그 base로 다시 재거나
    시간 백스톱을 폭주 방지로만 본다.
  bench(바퀴 띄움, V7) 패턴: config/bench.json(profile "bench", bench_ack "WHEELS_OFF_GROUND", bench_pattern
    [[초, 좌, 우], …], 전진만, 단계 ≤ 10 s·합 ≤ 60 s, cap 안)과 ./run_executor.sh bench --wheels-off-ground-confirmed로만
    가능하다(2절의 거부 조건). bench는 현장 측정값이 없어도 Arm할 수 있다(제어기를 쓰지 않음). 같은 hold·신선도·
    OutputGuard·STOP 경로를 지나며 끝나면 lease를 놓는다. 추정기가 움직임을 보면(속도 > 0.1 m/s 또는 0.2 m 넘게 움직임)
    즉시 끝난다(bench_motion_detected: 바퀴가 땅에 닿아 있다는 뜻). 전방 여유가 정지 모델(정지 시 0.35 m)보다 작아도
    끝난다(bench_front_blocked): V7 때 차 앞 약 0.5 m를 비워 둔다 [S4].
    field 설정에 bench_pattern이 있으면 로드가 거부되고, field 실행(shadow/motor)은 bench.json을 읽지 않는다.
    V7이 끝나면 Jetson에서 bench.json을 지운다: rm ~/row-executor/config/bench.json (현장에 가져가지 않는다) [S4].

6. 브리지 고장 복구 (FAULT / mcu_watchdog_rearm_required)
------------------------------------------------------
  1 차량을 안전하게(E-stop 누름 또는 바퀴 띄움).
  2 실제 브리지 컨테이너 이름부터 확인한다 [F8]: docker ps -a --filter name=esp-rover-bridge --format '{{.Names}} {{.Status}}'
    (compose 파일의 이름은 unist-orachard-ros2-esp-rover-bridge이지만 설치된 이름을 그대로 쓴다)
  3 docker restart <2에서 확인한 이름>  (그 컨테이너만. docker compose up/down/restart 절대 금지)
  4 상태가 connected=true, firmware=UBTS_WHEEL_PWM_V2, state=disarmed, last_error=null인지 확인
  5 실행기 화면 "Latch 확인" → Start → Arm (자동 재개 없음). USB 재연결 후 장치가 안 열리면 재생성하지 말고 중단·보고.

7. 기록 (logs/row_executor_<UTC>_<pid>_<n>.jsonl)
-------------------------------------------------
  20 Hz tick 한 줄: 상태·이유·제안/전송 duty·제어기 상태·입력 나이·통로(횡오차·heading·폭·열 끝)·여유·주행거리·속도·yaw·
  정지·진동·scan 수신/버림/처리 ms/header−수신 차이/스탬프 지연·사람 명령(/rover/control/requested)·루프 ms·백스톱.
  이벤트 줄: config(전체 설정·digest·SHA·vendored 커밋), start, request(Start/Arm/params 결과), emergency_stop, bridge_stop,
  bridge_error, watchdog_stall, params, loop_exception(_summary), runner_reset, shutdown.
  디스크가 막혀도 루프는 기다리지 않는다. [S8] 기록 스레드는 쓰기 오류(디스크 가득 ENOSPC, EIO, 폴더 삭제 등)로 죽지 않는다:
  실패한 줄부터 버린 줄로 바로 센다(대기열 4000줄이 찰 때까지 기다리지 않음). 2 s마다 같은 파일을 다시 열어 쓰기를
  재시도하고, 성공하면 이어 쓴다(실패 직후 첫 줄 앞에 빈 줄을 넣어 반쯤 쓰인 줄과 붙지 않게 한다; 읽을 때 빈 줄·깨진
  줄은 건너뛴다). 화면 "기록 (JSONL)" 줄과 /api/status의 log: 정상(ok) / 실패(failing) / 지연(stalled: 5 s 넘게 쓰기가
  끝나지 않음) / 용량 한도(capped) · 버린 줄 N · 마지막 오류. 실패·지연이면 화면에 빨간 경고가 뜬다.
  파일 100 MB마다 교체, 프로세스당 2 GB 넘으면 tick 줄을 멈춘다(이벤트는 계속).

8. 출발 전 점검 목록 (금요일 V6/V7 전과 현장 S0a에서)
---------------------------------------------------
  - AP 방화벽 [F8]: sudo iptables -S INPUT | head -3 (정책), sudo iptables-save | grep -n -- '--dport 8097'.
    INPUT 정책이 DROP/REJECT인데 8097 허용 규칙이 없으면 폰에서 화면이 열리지 않는다. 규칙 추가는 사용자가 정한다
    (예: sudo iptables -I INPUT -i <AP 인터페이스> -p tcp --dport 8097 -j ACCEPT, 재부팅 시 사라짐).
    폰에서 http://10.77.0.1:8097/?t=<토큰> 이 실제로 열리는지, sudo ss -ltnp | grep 8097 이 10.77.0.1:8097만 보이는지 확인.
  - 브리지 컨테이너 이름 [F8]: docker ps --filter name=esp-rover-bridge (6절 복구와 같은 이름을 쓴다).
  - 시계: ubts-gnss-time-sync 정지(4절 F5, D7): systemctl is-active ubts-gnss-time-sync → inactive.
  - 실행기 화면은 한 기기에서만 연다(3절 S3). 대시보드 STOP 화면과 물리 E-stop은 따로 손에 둔다.
  - V7이 끝나면 config/bench.json 삭제(5절).
  - 화면 맨 위 띠가 의도한 모드(SHADOW/MOTOR)인지, 기록 줄이 "정상 · 버린 줄 0"인지 확인한다.
  - [N1] MOTOR config.json은 시작 전에 검토한 예시(config.field_shadow.example.json)와 비교한다(diff). 다른 키는 측정 키
    (motor_output_enabled·calib_motor_output_enabled, breakaway·rolling, v_measured, measured_yaw, max_yaw, base·reduce·diff·cap,
    measured_stop_distance_m, C0 body_mask, 줄자로 잰 footprint·lidar_x)뿐이어야 한다. 느슨한 안전값은 config가 거부한다.

9. 검증 범위와 한계 (2026-10-22)
------------------------------
  - 보정 run(10절)·C0 마스크 도구: PC 단위 시험(운동학 가짜 추정기, 실제 대시보드 RoverControl 뒤의 실제 BridgeClient, 화면 JS는
    node 가짜 DOM)과 10/10 정지 구간 3개(042200·035952·051127)의 실제 scan으로만 확인했다. 실차·Jetson에서 돈 적이 없다.
  - 2일차 수정(N1–N9, R1–R7): 같은 PC·컨테이너 시험과 10/10 shadow 재생으로만 확인했다. stop 종류(C7)의 정지 거리는 추정기
    주행거리(움직임 중에만 셈)로 잰다: 실제 줄자 값과 다를 수 있다(현장에서 1회는 줄자로 대조).
  - PC 단위·프로토콜 시험, ROS Iron 컨테이너(py3.10/numpy 1.21/scipy 1.8)의 실제 rclpy 구독 smoke 시험,
    SIM-in-the-loop 닫힌 루프(기본 45장면 × seed 2, 맞춘 플랜트), 10/10 기록 6개의 shadow 재생은 했다.
  - 1일차 검토 수정(Fix B): 운용자 화면 JavaScript는 node에서 가짜 DOM·타이머·fetch로 시험했다(실제 폰 브라우저는 아님).
    읽기 전용 소켓 mount는 Linux 컨테이너(tmpfs bind)에서 실제 대시보드 브리지로 확인했다(Jetson 아님).
  - Jetson 처리 시간(V5), 실차 닫힌 루프, 바퀴 띄움 벤치(V7), AP 도달 거리, STOP 제동 거리는 측정하지 않았다.
  - 첫 실차 닫힌 루프는 현장이다. hold-to-run, 대시보드 STOP, 물리 E-stop, 관찰자, 콘을 함께 둔다.


10. 현장 보정 run과 C0 차체 마스크 (2026-10-22 추가; review_correctness_day1 F2·F3)
---------------------------------------------------------------------------------
왜: 대시보드 수동 mixer는 (s, 1.5 s) 쌍만 낸다. 그래서 실행기가 실제로 쓰는 차등에서의 yaw 권한(C5)을 잴 수 없다. bench 패턴은
땅 위에서 쓰면 안 된다. 그래서 field 설정·MOTOR 전용 보정 run 세 가지(arc C5, rolldown C3b, 2일차 stop C7)를 넣었다. 열 추종과 같은 hold-to-run·STOP·입력 신선도·
OutputGuard·브리지 경로를 지나고, 제어기 대신 정해진 duty 표를 낸다. 전진만 한다(후진·pivot 없음).

10.1 설정 (config.json; config.py 문서열이 원문)
  - "calib_motor_output_enabled": true (field profile만). 보정 run에만 모터 출력을 허용한다.
    열 추종 run은 motor_output_enabled=true(측정값 전부)일 때만 Arm된다. 보정 전용 설정에서 열 추종 Arm은
    row_motor_output_not_enabled로 거부된다. 화면 띠에 "이 설정은 보정 run만 출력한다"가 붙는다.
  - S6 안전 규칙(기하 guard 필수, 정지 모델·측면 여유·신선도를 기본값보다 느슨하게 못 함)은 보정 설정에도 그대로 검사한다.
  - calib_distance_cap_m (기본 15, 2–20 m), calib_time_cap_s (기본 45, 5–90 s): 한 보정 run의 상한. 화면에서는 낮추기만 된다.
  - 최소 duty [R1, N4] (열 추종의 안쪽 바퀴 규칙(B+10) 대신 보정 run 전용 OutputGuard가 검사; 표에 낮은 칸이 있으면 Start
    거부, 자르지 않음; cap(duty_cap_pct, diff_cap_pct)도 그대로):
      출발(정지에서 양쪽): breakaway_duty_pct(측정 정지 출발 임계 그대로), 없으면 40. base가 이보다 낮으면 거부.
      arc 쌍의 안쪽 바퀴: arc는 base/base로 출발하고 차가 0.10 m/s 이상 굴러간 뒤에야 첫 쌍으로 바꾼다 [N4]. 그래서 쌍의
        안쪽 최소는 rolling_breakaway_duty_pct(측정했으면, B와 함께), 아니면 출발 최소와 같다.
      rolldown: 양쪽 같게, B 아래로도 내려간다. 바닥 floor_pct 20–(base − 단계), 기본 25 [R1].
      어떤 보정 출력도 20 % 아래는 없다.
  - [N2] 보정 설정은 base_duty_pct ≤ 60, duty_cap_pct ≤ 75, max_speed_m_s ≤ 0.5여야 한다(C3가 필요 없으므로 직접 묶는다;
    과속 종결 = max_speed + 0.15 ≤ 0.65 m/s). N1의 MOTOR 규칙도 그대로 검사한다.
  - breakaway_duty_pct를 config에 넣으면 열 추종 규칙(base−max_differential ≥ B+inner_margin)도 파일 단계에서 검사된다.
    맞지 않으면 실행기가 시작하지 않는다. 보정 설정은 make_field_config calib이 결정표로 맞춰 만든다(12.2; 손 계산 대신).
    보정 표는 max_differential과 무관하고, arc 결과(yaw at max diff)는 최종 설정 생성 때 그 max_differential로 환산한다.
  - 현장 설정(base/reduce/diff)은 보정 설정에서도 파일 값 이하로만 바꿀 수 있다. 보정 표는 그 base를 중심으로 만든다.
  - 실행: ./run_executor.sh motor. 순서: 화면 '보정 Start' → Arm → 누르고 있기. SHADOW(./run_executor.sh shadow)에서는
    보정 카드가 보이지 않고 calib_start는 calibration_needs_motor_mode로 거부된다.

10.2 장소·사람·공통 동작
  - 열 밖의 평평한 풀밭(열과 같은 노면). 앞 15 m·좌우 3 m가 비어 있어야 한다. 2–10 m 안에 나무·울타리 같은 구조물이
    있어야 한다(LiDAR 정합 필요). 관찰자는 옆·뒤에서 걷고 물리 E-stop을 손에 쥔다. 대시보드 STOP 화면은 따로 연다.
  - 대시보드에서 'both' 녹화를 켜 둔다(분석기·나중 검증용).
  - 손을 떼거나, 입력이 끊기거나(stale), 루프가 멈추면 run이 끝난다: calib_hold_released / calib_stale_* / calib_stall_guard.
    재개는 없다. 다시 하려면 '보정 Start' → Arm → 누르기. 끝난 단계의 결과는 남는다.
  - 표가 끝나면 스스로 0/0 → 정지 확인 → op stop(lease 해제). 대시보드 수동 조종으로 돌아간다.
  - 종결 이유(모두 0/0 후 정지, 재개 없음):
    | 이유 | 뜻 |
    | calib_complete / rolldown_stalled / rolldown_floor_reached | 정상 끝(측정 완료) |
    | calib_stop_test | stop 종류 정상 끝: 실행기가 스스로 0/0을 내고 정지 거리를 잰다(C7) |
    | rolldown_stalled_at_base | base duty에서도 계속 굴러가지 않음(base가 낮음) |
    | calib_start_stalled / calib_stop_stalled | arc·stop이 base/base로 움직였다가 측정 전에 섰음 |
    | calib_front_blocked | 전방 여유 < 정지 모델(속도별 정지거리 + 0.35 m). 출발 전이면 출발 거부 |
    | calib_side_blocked | 옆 여유 ≤ 0.03 m가 서로 다른 scan 2개(열 추종 side 규칙과 같음) |
    | calib_overspeed | 속도 > max_speed_m_s + 0.15 m/s · 즉시 op stop + LATCHED [N3] |
    | calib_yaw_rate | |yaw| > arc 12 °/s, rolldown·stop 6 °/s (안쪽 바퀴가 서서 pivot처럼 도는 경우) · 즉시 op stop [N3] |
    | calib_heading_change | 방향 변화 > arc 단계 60°·run 90°, rolldown·stop 20° · 즉시 op stop [N3] |
    | calib_no_response | 누른 뒤 3 s 안에 움직임 없음 |
    | calib_odometry_lost / calib_odometry_invalid | LiDAR 정합 1 s 넘게 실패 / 주행거리 이상 |
    | calib_distance_cap / calib_time_cap | 거리·시간 상한 |
    | calib_arc_stalled | arc 중 차가 섬(그 차등 쌍은 이 노면에서 못 씀) |
    | guard_violation:* | 출력 가드 위반(버그): 즉시 op stop + LATCHED |

10.3 C0 차체 마스크 — 정지 scan (body_mask_static.py, Jetson 호스트 python3, 표준 라이브러리만)
  1 차를 빈 땅에 세운다. 1.5 m 안에 사람·물건이 없고, 차에 아무것도 기대거나 매달리지 않게 한다.
    E-stop 함·안테나 등 새 장착물은 그대로 둔다(그것이 마스크 대상이다).
  2 대시보드 'both' 녹화를 30–40 s 한다(같은 녹화를 C1 gyro bias에도 쓴다).
  3 실행(녹화 ID는 대시보드 목록에서):
      R=/home/unist/workspace/unist_orachard_dashboard/runtime/recordings/<녹화 ID>
      python3 -B ~/row-executor/src/row_executor/body_mask_static.py --recording $R --config ~/row-executor/config/config.json \
          --out-fragment ~/c0_fragment.json --out-config ~/config.c0.json --report ~/c0_report.json
    (권한 오류면 sudo. 녹화가 길면 --from 5 --to 35처럼 정지 구간만.)
  4 출력 읽기:
      "마스크 구간": beam 번호, 방위(0° = 진행 방향, + = 왼쪽), 최대 거리. 차의 어느 부품인지 눈으로 맞춰 본다.
      "일관되지 않아 거부": 흔들리는 끈, 움직인 사람, 윤곽에 걸친 물체(object_crosses_limit). 마스크하지 않는다.
      "차체 윤곽 밖의 지속 반환": 차 옆에 무엇이 서 있다. 또는 config의 footprint_length/width_m·lidar_x_in_base_m가 실제보다
        작다(C0에서 줄자로 잰 값으로 고친 뒤 다시).
      거부(종료 코드 3): 차·장면이 움직였거나(not_stationary) scan이 모자람. 다시 녹화한다.
  5 config.c0.json을 확인하고 config.json으로 바꾼다(이전 파일은 보관). 마스크는 기존 body_mask를 "대체"한다.
    빈 마스크도 정상이다(10/10 장착에서는 차체 반사가 없었다).
  [N5] 진행 방향 ±35° 안의 마스크 구간이 있으면 도구는 아무것도 쓰지 않고 종료 코드 4로 끝난다(범퍼에 닿은 다리도 차체처럼
    보이기 때문: 검토 probe에서 그렇게 마스크된 다리를 다음 Start가 보지 못했다). 출력의 "← 진행 방향 ±35°" 구간마다 방위·거리를
    차의 실제 부품과 하나씩 맞춰 본 뒤 --confirm-forward-sector를 붙여 다시 실행한다. 마스크된 방위에서는 측면 hard 여유(0.03 m)
    안의 물체도 가려진다(마스크 여유 0.05 m): 마스크는 실제 부품에만 쓴다.
  규칙: 마스크는 차체 윤곽 + 5 cm 안(진행 방향 ±35°는 앞 모서리 + 5 cm 안)만 가린다(S5, config가 다시 검사).
    정지 scan의 90 % 이상에서 같은 거리(±5 cm)로 보인 beam만 가린다.
  10/10 확인: 042200(오른쪽 뒤 모서리의 물체가 윤곽에 걸침), 051127(윤곽 안 0.25–0.41 m, 86 % scan에만), 035952(8 %)
    모두 마스크 0. 그 반환은 움직일 때 보이지 않았으므로 사람·물건으로 본다(review F3). 출발할 때 차 1.5 m 안에 사람이 없게 한다.

10.4 C3b rolldown — 굴러가는 중 최저 duty (rolling_breakaway_duty_pct)
  - 화면: 종류 rolldown, 단계 2/3/4/5 %, 단계 유지 2–5 s, 거리·시간 한도. 미리보기에 duty 순서가 나온다
    (예: 55 → 52 → 49 → 46 → 43 → 40).
  - 동작: base로 정지에서 출발 → 굴러가기 시작하고 2 s → 단계마다 양쪽 같은 duty로 내린다. 차가 서면
    (정지 또는 0.05 m/s 미만 0.6 s) 끝난다.
  - 결과: 마지막으로 계속 굴러간 duty = rolling 후보. 최소 duty까지 계속 굴러갔으면 "≤ 그 값"(상한, 보수적)이다.
  - [R1] 바닥은 B와 무관하다: 기본 25(화면 미리보기에 순서가 나온다; API floor_pct 20–(base − 단계)). 그래서 config에 C2(B)가
    있어도 B 아래의 rolling 임계를 잰다(예: B 40, 굴러가는 차는 28까지 유지·25에서 섬 → rolling 28). 양쪽 duty가 같아 pivot
    위험이 없고, 서면(rolldown_stalled) 끝, yaw 6 °/s·방향 20°는 그대로다. 출발은 base(≥ B)에서 한다.
    결과 rolling ≤ B여야 config가 받는다(rolling > B면 C2가 낮게 잡힌 것: C2를 다시 잰다). 결정표 O4(diff = min(20, 45 − R))는
    이 측정으로만 가능하다.

10.5 C5 arc — 차등별 yaw 권한 (measured_yaw_at_max_diff_deg_s)
  - 화면: 종류 arc, 묶음, 단계 측정 2–8 s(기본 4 s, 각 단계 앞 1.5 s는 안정 구간이라 측정하지 않음), 거리·시간 한도.
    단계 거리는 0.9 × 거리 한도 / 단계 수다(2 m보다 작으면 거부).
  - 묶음(좌/우 duty, base 55 기준):
    | d10 | 55/45, 45/55 |
    | d20 | 55/35, 35/55 |
    | d10_d20 | 55/45, 45/55, 55/35, 35/55 (기본) |
    | sym10 | 60/50, 50/60 |
    | sym20 | 65/45, 45/65 |
    | sym10_sym20 | 60/50, 50/60, 65/45, 45/65 |
    R > L이면 왼쪽(+yaw)으로 돈다. 최소 duty보다 낮은 칸이 있는 묶음은 화면에 "(불가)"로 표시되고 Start가 거부된다.
    예: B = 40이면 d10_d20(35)은 불가, d10·sym10·sym20은 가능. B 40·rolling 30을 쟀으면 d20(35)도 가능하다(쌍은 굴러간 뒤 시작).
  - [N4] 출발은 base/base(직진)로 하고, 차가 0.10 m/s 이상 굴러가면 첫 쌍으로 바꾼다(정지에서 안쪽 바퀴가 바닥값이면 바깥쪽만
    돌아 pivot처럼 출발할 수 있었다). 화면 단계 표시는 그동안 'start · base/base'다.
  - 실행기가 실제로 쓸 diff로 잰다: 열 추종 diff d이면 sym 묶음(|L−R| = 2d)을 고른다(예: d10 → sym20). 다른 diff에서 잰 값은
    선형으로 환산될 뿐이다(외삽 표시). d가 10보다 크면(R이 낮을 때 최대 20) sym20으로 재고 × 2d/20 환산한다(sym 묶음의 최대가
    |L−R| 20). [통합] 조향 GO 규칙은 방향마다 3구간 이상을 요구하므로 arc run은 3회 한다(run마다 좌·우 1구간).
  - 결과: 단계마다 gyro yaw(°/s), LiDAR yaw(정합이 80 % 이상일 때), 속도, gain = yaw/(R−L)(LiDAR 우선)이 나온다.
    좌·우 각 gain을 최소제곱(원점 통과, 초 가중)으로 구하고, 작은 쪽 × 2 × max_differential이 화면의 yaw 권한이다.
    측정한 최대 |L−R|이 2×max_differential보다 작으면 외삽으로 표시한다. 설정의 measured_yaw_at_max_diff_deg_s는 make_field_config
    final이 정한다(12.4: 녹화 arc와 합친 규칙값·실행기 arc만의 값·가장 최근 arc run 값 중 가장 작은 값).
  - 화면 표와 아래 인쇄 표에 적어 둔다(인터넷·원격 지원 없음).

10.5b C7 stop — 실행기 0/0 정지 거리 (measured_stop_distance_m) [N8]
  - 왜: 열 추종 MOTOR는 이제 C7 값이 있어야 로드된다. 값의 정의는 "실행기의 0/0(MCU 감속) 명령부터 정지까지"다(열 끝·백스톱·
    전방 막힘·누름 해제가 모두 이 0/0을 낸다). 대시보드 STOP(즉시 정지)으로 잰 값은 짧아서 쓰면 안 된다.
  - 화면: 종류 stop, 거리·시간 한도. 동작: base/base로 출발 → 굴러가면 1.5 s 안정 + 3 s 정속(측정 창, 기본 3 s, 2–6 s) →
    실행기가 스스로 0/0을 내고(calib_stop_test) 차가 설 때까지 0/0만 보낸다(최대 4 s, 그 뒤 op stop) → 추정기가 정지를 보면
    정지 거리(0/0 tick부터 정지까지, 움직임 중에만 센 거리), 0/0 때 속도, 정지까지 시간을 화면 결과 줄과 기록(calib_stop)에 남긴다.
  - 3회 한다(같은 노면, 최종 base duty). 정지 지점을 콘·줄자로 한 번은 대조한다(추정기 거리와 차이를 인쇄 표에 적는다).
  - calib_analysis가 같은 날 stop run 3개 이상(calib_stop_test로 끝나고 정지가 관측된 것)의 가장 큰 정지 거리(cm 올림)를
    measured_stop_distance_m로 낸다. config는 그 값이 v_measured에서의 정지 모델 이하이고, 모델 + 0.35 ≥ 측정 + 0.35인지 검사한다.
    [통합] make_field_config final은 모델이 측정을 덮도록 stop_onset_s를 0.05 s 단위로 올린다(최대 1.0 s; 모델이 길수록 전방
    정지를 일찍 시작한다). 1.0 s로도 못 덮으면 G7 NO-GO(base를 낮추거나 C3·C7을 다시 잰다).

10.6 실행기 보정 run 요약 (calib_analysis.py, 표준 라이브러리만) — 설정은 make_field_config(12절)
    python3 -B ~/row-executor/src/row_executor/calib_analysis.py --log-dir ~/row-executor/logs \
        --config $HOME/row-executor/config/config.json [--out-fragment ~/calib_fragment.json]
  - (기록 파일은 root 소유지만 읽기는 된다. 권한 오류면 앞에 sudo.) PC에서는 logs 복사본에 같은 명령.
  - 실행기 JSONL의 보정 run을 모두 표로 보인다. tick으로 다시 계산한 값과 실행 중 값의 대조 결과도 보인다(현장 기록·인쇄 표용).
  - config 조각(참고): measured_yaw_at_max_diff_deg_s(가장 최근 양쪽 유효 arc), rolling_breakaway_duty_pct(가장 최근 rolldown),
    measured_stop_distance_m(stop 3회 이상, 가장 큰 값).
  - [N7] 같은 날만 쓴다: 가장 최근 보정 run보다 12 h 넘게 이전의 run은 빼고 몇 개를 뺐는지 출력한다(--all-days로 포함).
    종류마다 "가장 최근 run"이 무효인데 더 예전 run이 유효하면 그 값에 WARNING을 단다. make_field_config는 그 경우 설정을 쓰지
    않는다(종료 코드 4): 그 run을 다시 하거나, --runs로 고르거나, 확인한 뒤 --allow-older를 붙인다.
  - [통합] --out-config는 거부된다(종료 코드 2): 설정을 쓰는 것은 make_field_config 하나다(보정 설정과 최종 설정, 12절).
    calib_analysis의 읽기 함수(load_runs·field_day·summarize·config_fragment·c5_segments·usable_stop)를 make_field_config가 쓴다.

10.7 인쇄용 보정 표 (현장에서 적기; 전체 절차표는 field_sheets/CALIB_SHEET.html)
  날짜/시각 ____  노면 ____  배터리 V ____  config digest ____  breakaway(최소 duty) ____
  C0: 녹화 ID ____  마스크 구간 수 ____  윤곽 밖 경고(방위) ____  조치 ____
  C3b rolldown: 단계 __%  유지 __s | duty: 55 __ 52 __ 49 __ 46 __ 43 __ 40 __ 37 __ 34 __ 31 __ 28 __ 25 __ (유지 O / 정지 X)
          | 끝 이유 ____ | rolling 후보 ____
  C5 arc(sym20 × 3회): 1회 좌 yaw ____ 우 yaw ____ | 2회 좌 ____ 우 ____ | 3회 좌 ____ 우 ____ °/s | 속도 ____ m/s
          gain 좌 ____ 우 ____ | yaw at 2×diff ____ °/s (외삽 Y/N) | 끝 이유 ____
  C7 stop: 1회 정지 ____ m (0/0 때 ____ m/s, ____ s) | 2회 ____ m | 3회 ____ m | 줄자 대조(1회) ____ m | 끝 이유 ____
  C3 (줄자·스톱워치, base 55, 실제 열 노면, 2 m 도움닫기 + 10 m) 1회 ____ s 2회 ____ s 3회 ____ s → --tape 55=초 × 3


11. 2일차 검토 반영 (2026-10-22; review_safety_day2 N1–N9, review_correctness_day2 R1–R9)
------------------------------------------------------------------------------------
  코드: [N1] MOTOR 설정 완화 차단(5절), [N2] 보정 설정 상한(10.1), [N3] bench·보정 이상 종결은 즉시 op stop(4절),
    [N4] arc는 base/base로 출발(10.5), [N5] C0 전방 구간 확인 플래그(10.3), [N6] bench 정합 상실 종결(아래),
    [N7] 분석기 같은 날·예전 run 거부(10.6), [N8] C7 stop 종류와 열 추종 MOTOR의 C7 필수(10.5b), [N9] 무응답 누적(4절),
    [R1] rolldown 바닥 25(10.4), [R2] 현장 diff를 낮추면 yaw 권한·max_yaw도 함께(5절), [R5] 분석기 tick 대조 오류 수정,
    [R6] 기록: ENDED tick의 end_note, run_end 이벤트(OPERATIONS 10.1).
  [N6] bench(바퀴 띄움) run 중 LiDAR 정합이 1 s 넘게 무효면 끝난다(bench_odometry_lost, 즉시 op stop). 움직임 감지가 정합에
    기대기 때문이다. V7은 주변에 구조물(벽·나무·상자)이 있는 곳에서 한다.
  운용 규칙(문서):
  [R3] 시간 백스톱의 v_measured는 줄자·스톱워치 C3로만 정한다. 화면 "주행거리 / 이동시간"(추정기 오도메트리)으로 v를 낮추지
    않는다: 정합이 거리를 적게 세는 바로 그 상황에서 시간 백스톱(오도메트리와 독립)이 약해진다. 첫 S2 뒤 확인이 필요하면
    정지 지점을 줄자로 재고 스톱워치 시간으로 v_cl을 구한다. v_cl < 1.03 × v / factor면 v를 v_cl / 0.975로 고친다.
  [R4] 열 중간 재Start의 L은 그 자리에서 줄자로 다시 잰다(3절).
  [N8] C7은 실행기의 0/0으로 잰다(10.5b). 대시보드 STOP 값은 쓰지 않는다.
  [N5] C0는 차 1.5 m 안에 아무도 없고 차에 아무것도 닿지 않은 상태에서 한다(10.3).
  arc 보정 중 관찰자는 뒤에서 걷고 좌우 3 m 안에는 아무도 없게 한다. calib_yaw_rate·과속이 뜨면 실행기가 즉시 op stop을
    보내지만, 물리 E-stop을 누를 준비를 한다.
  실행기 화면은 한 기기에서만 열고, 대시보드 STOP과 E-stop은 따로 손에 든다.


12. 통합 현장 보정 절차 (2026-10-22, 통합 브랜치) — 한 절차, 한 설정 생성기
----------------------------------------------------------------------------------------------------
녹화 분석기(tools/field_calib, 대시보드 수동 녹화)와 실행기 보정 run(calib_runs / calib_analysis)을 한 순서로 묶었다.
설정 파일을 쓰는 것은 make_field_config.py 하나다(보정 설정 calib, 최종 설정 final). 규칙 값은 config.py 끝의
field decision rules 블록 한 곳에 있고 field_calib(recommend.py)·make_field_config·실행기 MOTOR 검사가 모두 그것을 쓴다.
모터 출력 스위치(calib_motor_output_enabled, motor_output_enabled)는 어떤 도구도 켜지 않는다: 사람이 켠 뒤 check.

12.1 순서 (토요일; 시트 CALIB_SHEET.html이 같은 순서)
  | # | 단계 | 누가·무엇으로 | 결과 |
  | 1 | S0a V7(바퀴 띄움 벤치)·V6(live shadow) | 실행기 bench / shadow | 출력 경로·부호 확인, bench.json 삭제 |
  | 2 | C0 차체 마스크·줄자 footprint·lidar_x | 대시보드 'both' 녹화 30–40 s → body_mask_static.py (10.3) | config.json(마스크) |
  | 3 | 녹화 C1 정지 30 s(C0 녹화 재사용 가능), C2 출발 사다리 20–50 %(노면별), C3 base 55 × 3회 줄자·스톱워치(필수, 다른 duty는 선택), C4 직진 편향, C6 pivot(정보, headland), C7 키 놓기 3·STOP 1(대조), C5 녹화 arc(선택) | 대시보드 수동 | 녹화 ID |
  | 4 | 녹화 단계 분석 | run_field_calib.sh <ID…> --label … --base 55 --tape 55=초 ×3 | calib_result.json, CALIB_REPORT.txt |
  | 5 | 보정 설정 | run_field_calib.sh config calib --result <4의 폴더>/calib_result.json | calib_config.json(B, v, d = min(20, 45 − B)) |
  | 6 | calib_config.json → config.json, calib_motor_output_enabled=true(사람) → ./run_executor.sh check → motor | 실행기 | |
  | 7 | C3b rolldown 1–2회(10.4) | 실행기 보정 run | R |
  | 8 | 보정 설정 다시(R 반영: d = min(20, 45 − R)) → config.json → check → motor 재시작 | run_field_calib.sh config calib | arc 바닥 = R |
  | 9 | C5 arc sym20 × 3회(10.5), C7 stop × 3회(10.5b, 1회는 줄자 대조) | 실행기 보정 run | arc·stop 기록 |
  | 10 | 최종 설정 | run_field_calib.sh config final --result <4>/calib_result.json | field_config.json, FIELD_CONFIG_REPORT.txt, 끝 코드 |
  | 11 | GO면: field_config.json 확인 → config.json, motor_output_enabled=true(사람) → check → S1 SHADOW → S2 감독 주행 | | |
     NO-GO면 field_config.json은 측정 키를 비운 SHADOW 전용이다(켜도 실행기가 거부): Plan B.
  base: 결정표 base 55(config 기본값 = config.DECISION_BASE_DUTY_PCT). 다른 base(O5)는 사용자 결정이고 --base N으로 준다.
  녹화 단계(4)는 이미 정합한 기록을 캐시에 남긴다: 5·8·10은 녹화를 다시 읽지 않고 수 초에 끝난다(numpy만, scipy 불필요).

12.2 make_field_config.py (tools/field_calib; 래퍼 run_field_calib.sh config calib|final)
  입력: --result(4의 calib_result.json), --config(지금 실행기 config.json; 다른 키는 그대로), 실행기 기록(CALIB_EXECUTOR_LOGS,
  기본 ~/row-executor/logs, 읽기 전용), --base(기본 config의 base_duty_pct). 설치된 실행기 패키지(~/row-executor/src)의 config.py로
  읽고 검증한다(field_calib와 실행기는 같은 커밋에서 설치; 규칙 블록이 없는 예전 실행기면 이유와 함께 멈춘다).
  calib: breakaway_duty_pct(C2), v_measured_m_s(C3), base·reduce = base, max_differential_pct(결정표), rolling_breakaway_duty_pct
    (rolldown이 기록에 있으면), cap은 config 값(필요하면 base + d·2d로 올림). calib_motor_output_enabled=true로 켰을 때의 검사도
    미리 한다(거부 이유를 보이고 끝 코드 3). 보고서에 C5 arc 묶음 제안(sym20, 외삽 배수).
  final: recommend(녹화 측정 + 실행기 run) = 결정표 + 조향 GO 규칙 + C7, GO 표 G0–G7. GO면 field_config.json(검증됨;
    calib_motor_output_enabled는 끔, motor_output_enabled는 config 값 그대로 = 보통 false), 끝 코드 0. NO-GO면 SHADOW 전용 파일과
    끝 코드 3. 실행기 값이 예전 run에서 오면(N7) 아무것도 쓰지 않고 끝 코드 4(--allow-older는 확인 뒤에만). 입력 오류 2.
  출력 폴더(기본 <4의 폴더>/config-<단계>-<시각>): calib_config.json 또는 field_config.json, FIELD_CONFIG_REPORT.txt(한국어: 판정,
    바뀐 키, 근거, 실행기 run 표), field_config_result.json(모든 값과 표본 수).

12.3 결정표와 조향 GO 규칙 (config.py field decision rules 블록 = 한 곳)
  max_differential d = min(20, base − (R + inner_margin), 100 − base)   R = C3b rolling 임계(실행기 rolldown, 같은 날 최신)
                      없으면 R 대신 B_fwd(C2 사다리). base 55·margin 10이면 d = min(20, 45 − R) 또는 min(20, 45 − B).
                      정지 출발은 base ≥ B_fwd + 10이 따로 필요하다(config 검사).
  조향 GO(G3·G3b): 약한 쪽 C5 이득 g ≥ 0.10 °/s/%(R−L 1 %당) 그리고 d ≥ 10 그리고 R−L = 2d에서 yaw ≥ 4.0 °/s.
                   필요한 g = 4.0 / (2d): d 20 → 0.10, 15 → 0.134, 12 → 0.167, 10 → 0.20. d < 10 → NO-GO.
  max_yaw_rate = min(4.5, 0.6 × 권한), 0.1 내림. min_yaw_authority_deg_s = max(config 값, 4.0)(설정에 적는다; 실행기도 MOTOR에서 강제).
  C7: v_measured에서 정지 모델 ≥ 실행기 C7(onset 0.05 s 단위로 올림, ≤ 1.0 s), 모델 + 0.35 ≥ C7 + 0.35.
  실기록 참고(규칙의 입력이 아님): 10/10 B p50 34 % → R 없으면 d 11 → 필요한 g 0.182. 10/3 약한 쪽 0.090, 10/10 arc 약 0.096이면
    R로 d 20을 열어도 0.10에 못 미친다 → C3b·C5 현장값이 정한다(대부분 NO-GO 쪽).
  SIM 근거와 남는 위험: 2026-10-22 현장 자율주행 작업 폴더의 steer_rule/RULE_KO.txt(SIM ≠ 실차, 열 중간 미끄러짐 장면은 GO여도 0.6–1.5 m).

12.4 두 분석기의 역할 (겹치던 값은 이렇게 하나로 정한다)
  | 값 | 출처 | 설정 키 |
  | B_fwd (C2) | 녹화 사다리(field_calib, 노면별, 확정된 노면의 최댓값) | breakaway_duty_pct |
  | v (C3) | base duty의 max(LiDAR 구간 중앙값, 스톱워치 평균); 고정 base는 스톱워치 3회 이상이 있어야 G2 GO(R3) | v_measured_m_s |
  | R (C3b) | 실행기 rolldown(같은 날 최신, 상한값이라 보수적) | rolling_breakaway_duty_pct |
  | d | 결정표(12.3) | max_differential_pct, duty_cap_pct = base + d, diff_cap_pct = 2d |
  | C5 이득·권한 | 실행기 arc + 녹화 arc를 합친 규칙값(방향마다 3구간), 실행기 arc만의 값, 가장 최근 실행기 arc run의 값 중 가장 작은 값. 녹화 arc는 구간 수를 채우고 값을 낮출 수만 있다. 최종 설정은 실행기 arc가 양쪽에 있어야 G3 GO(F2: 대시보드 mixer로는 실행기 차등을 못 냄). 직진 편향(C4)은 둘 다에서 뺀다 | measured_yaw_at_max_diff_deg_s, max_yaw_rate_deg_s, arc_yaw_deg_s_per_pct_diff, min_yaw_authority_deg_s |
  | C7 | 실행기 stop 3회의 최댓값(cm 올림) = 측정값. 녹화 release·STOP(G4)은 대조: 실행기 C7이 GO면 G4는 NO-GO일 때만 막는다 | measured_stop_distance_m, controller_overrides.stop_onset_s |
  | C4 | 녹화(G5, base ±10 %p 8 m 이상 3회) | - |
  | C1 | 녹화(정보) | - |
  | C6 | 녹화 headland pivot(정보). 회전 보조가 꺼져 있으면(기본) 설정에 쓰지 않고 보고서 후보로만 | (pivot_* 후보) |
  GO 표: G0 기록, G1 base 출발 3/3, G2 v ≤ 0.5(+스톱워치), G3 이득(+실행기 arc), G3b 권한·실현 가능성(출력 ON 검사), G4 녹화 정지
  대조, G5 직진, G6 C3b(참고: 없으면 B로), G7 실행기 C7. 참고 행(G6, 그리고 G7 GO일 때의 G4)은 NO-GO일 때만 막는다.

12.5 실행기가 스스로 지키는 것 (도구가 틀려도)
  열 추종 MOTOR: 측정값 전부(B, v, C5, C7), 안쪽 바퀴 규칙, N1 하한들, max_differential 10–20, 권한 ≥ max(min_yaw_authority,
  4.0) °/s, max_yaw ≤ 0.6 × 권한, C7 정지 모델. SHADOW·보정 설정은 이 조향 규칙을 검사하지 않는다(기본값·digest 불변).
  현장 화면에서 diff를 낮추면 권한이 비례로 낮아지고 4.0 아래·diff 10 아래면 거부된다(R2).

12.6 회전 중심(ICR) 규약 하나
  pivot_icr_ahead_<방향>_m / pivot_icr_left_<방향>_m: 실행기 base 좌표(원점 = footprint 중심, x = 진행 방향 +, y = 왼쪽 +),
  <방향> left = 반시계(+yaw, 왼쪽 바퀴 뒤·오른쪽 앞). field_calib는 LiDAR 기준(진행축 앞 +, 왼쪽 +; 뒤집힌 scan 좌표는 이미 풂)으로
  재고 config.icr_base_from_lidar(앞 + lidar_x_in_base_m)로 바꾼다. turn_assist는 이 점을 중심으로 base를 돌린다. 시험:
  tools/field_calib/tests/test_unified_config.py IcrConventionTest(뒤집힌·회전된 LiDAR의 강체 회전 → 같은 값). 회전 보조는
  이번 주말 꺼져 있다(INFO_ONLY).

12.7 시험 (2026-10-22, 이 브랜치)
  tools/field_calib/tests/test_unified_config.py(규칙 한 곳, 결정표, 조향, ICR, make_field_config calib/final/N7/C7/G4/G2),
  tools/row_executor/tests(실행기 MOTOR 조향 규칙: test_config MotorSteeringRule, R2 갱신). 명령은 OPERATIONS.txt 12절.
  한계: 실차·Jetson에서 돈 적이 없다. 실행기 run 기록은 합성 JSONL, 녹화 측정은 합성 표로 시험했다. 조향 규칙은 SIM 근거다.
