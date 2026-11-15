# 조향 펌웨어 V3 구현 메모 (설계 8.3 P3, 결정 R1)

- 작성: 2026-11-09 (P3 펌웨어 작업). 고침: 2026-11-09 fix_p3(검토 결과 RF-01~RF-11 반영, 6절 끝과 7절 끝)
- 근거: `ESP_PROTOCOL_V3_DRAFT_KO.md`(이하 "V3 규격"), `LOCKSTEP_PROTOCOL_KO.md` 11절(lockstep v2), `DECISIONS_KO.md` R1, 설계 2.3.1·8.3 P3.
- 펌웨어 로직은 C++11로 한 번만 썼다(`firmware/ubts_steer_v3/`). 같은 소스를 SIM 네이티브 프로세스에서 Arduino 흉내 층 위에 돌린다. 나중에 실기 ESP8266에 쓴다(P9, 사용자).
- 모든 수치는 자리값이다. 이 문서의 결과는 자리값 기반 SIM 결과다. 현장 검증이 아니다. 실기 쓰기·직렬 포트 열기는 하지 않았다.

## 1. 모듈 지도 (`firmware/ubts_steer_v3/`)

| 파일 | 하는 일 | V3 규격 |
|---|---|---|
| `ubts_steer_v3.ino` | `setup()`, `loop()`만 둔다. 각각 `firmware_setup()`, `firmware_loop()`를 부른다 | 7.1 |
| `fw_main.h/.cpp` | 부팅 순서, `loop()` 12단계, watchdog(3), TTL(5), 결함 동작(6), 상태 줄 주기(10), 핀 쓰기(11). SIM용 읽기 전용 보기 `debug_state()`, `debug_fault_bits()` | 5.4, 6.2, 6.3, 6.4, 7.1, 8.4 |
| `fw_link.h/.cpp` | 줄 받기(64 B/틱), 틀 오류, STOP 특례, CRC 필드, 토큰·정수 문법, 명령 실행, 거절 줄, 나쁜 줄 연속(CRC 결함), MCU 줄 만들기, 512 B 송신 고리와 송신 펌프(12단계) | 2, 4, 5, 6.6 |
| `fw_steer.h/.cpp` | 조향 모드 OFF/ACTIVE/HOLD/JOG/POL, 서보 시계, 정수 서보 법칙(속도 제한, 데드밴드 제동 유지, 방향 전환 휴지, 소프트 한계 바깥 막기), 조향 핀 값, `CAL POL` 절차(통과하면 POLARITY·DRV_STUCK 래치를 지운다) | 6.5, 6.8, 7.4~7.10 |
| `fw_drive.h/.cpp` | V2 램프(255/s, 10 ms 격자, 50 ms 상한), 0 뒤 120 ms 휴지, brake, release, BENCH `D<n>` 요청, "움직이는 중" | 4.12, 7.5, 8.2 |
| `fw_monitor.h/.cpp` | ADC 한 번/틱, 중앙값, 위치, 기록 고리(위치 1024, f·raw 128), 결함 판정, 달리기 계수(DRV_STUCK 관찰 상태 포함), THERMAL 비트 고리(60,000비트, 틱 번호와 따로 감기는 색인), 처리한 틱 수(포화 계수), ARM·`CAL JOG`·`CAL POL`을 막는 활성 결함 조건 | 6.7, 7.2, 7.3 |
| `fw_cal.h/.cpp` | 보정 칸, raw → ‰ 변환, 106 B 레코드 쓰기·읽기 검사, 지움 표시 | 7.3, 9 |
| `fw_params.h/.cpp` | CFG 변수 41개 표(이름, 기본값, 범위, 값 집합, 묶인 검사) | 4.10 |
| `fw_crc.h/.cpp` | CRC-8(0x07), CRC-32/ISO-HDLC. 표 없이 비트 계산(RAM 절약) | 3, 9.3 |
| `fw_text.h/.cpp` | MCU 줄 만들기(대문자 16진, `*HH\n`), 부호 없는·부호 있는 정수 문법, 16진 토큰 | 2.3, 5 |
| `fw_state.h/.cpp` | RAM 상태 한 구조체 `fw`. `setup()`이 처음에 0으로 지운다(리셋 = RAM 잃음) | 6.1 |
| `fw_hw.h/.cpp` | 하드웨어 접근층. Arduino·ESP8266 코어 API를 부르는 유일한 파일 | 8.3, 9.1, 10 |
| `fw_config.h` | 컴파일 상수(`UBTS_STEER_SIM`, `UBTS_STEER_HW`, `UBTS_STEER_BAUD`, `UBTS_STEER_FW_SRC`), 핀 배치 A안 | 8.3, 10.4~10.6 |

지킨 제약:

- 정수 연산만 쓴다(제어 경로에 부동소수 없음). C++11. 예외·RTTI·STL 컨테이너·`new`를 쓰지 않는다. 버퍼는 모두 고정 크기다. `loop()` 안에 기다리는 곳이 없다(직렬 송신은 한 번에 `availableForWrite()` − 1 바이트 이하만. ESP8266 코어 3.1.2의 `Serial.write`는 FIFO가 127 B 이상 차면 기다리기 때문이다. V3 규격 12.1-66). 핀 쓰기(11단계)는 송신(12단계)보다 앞이다.
- 반복은 모두 상한이 있다: 수신 64 B/틱, 중앙값 9개, 잡음 창 100개, THERMAL 다시 세기 60,000비트(CFG `thw` 때만).
- 제어 로직에 `#ifdef UBTS_STEER_SIM`이 없다. 빌드 정의가 바꾸는 것은 배너의 `sim=`, `hw=`, `build=`와 하드웨어 접근층의 대상뿐이다.
- `setup()` 핀 순서: `analogWriteRange(255)`, `analogWriteFreq(1000)` → D8, D5, D3, D0에 `digitalWrite(LOW)`, `pinMode(OUTPUT)`, `analogWrite(0)` → D6, D7 같음 → D2 LOW·OUTPUT → D1 HIGH·OUTPUT. GPIO 쓰기 14번(V3 규격 8.3). 그 뒤 V2처럼 `WiFi.mode(WIFI_OFF)`, `Serial.begin(115200)`, `EEPROM.begin(256)`.
- 한 틱 = 서로 다른 `millis()` 값 하나. 실기에서는 `loop()`가 같은 밀리초 안에 다시 불리면 송신만 옮기고 돌아간다. SIM은 틱마다 `loop()`를 한 번 부르므로 V3 규격 7.1의 틱과 같다.

## 2. SIM 하드웨어 층 (`sim/ubts/steering/native/arduino_shim/`)

같은 펌웨어 소스를 바꾸지 않고 컴파일하게 하는 Arduino 형식 흉내다. V2 네이티브 하네스(`rover_control_test/tests/native/Arduino.h`)의 방식을 넓혔다. V2 파일은 바꾸지 않았다.

| 호출 | SIM에서 |
|---|---|
| `millis()`, `micros()` | lockstep 틱 시계: (n − 리셋 틱 + `millis_start`) mod 2^32. micros = millis × 1000 |
| `analogRead(A0)` | 그 틱의 값: `adc_mode` model이면 가변저항 모형의 앞 틱 끝 표본, script면 대본 값 |
| `analogWrite`, `digitalWrite`, `pinMode` | GPIO 값 기록(구동기 입력, 구동 출력). `analogWrite`·`digitalWrite` 호출 수를 `gpio_writes`로 센다 |
| `Serial` | RX: 256 B 고리, 255 B까지 담는다. 넘치면 가장 옛 바이트를 버린다(ESP8266 코어 3.1.2와 같다. V3 규격 12.1-67). lockstep rx 항목이 그 틱 4단계 전에 들어온다. TX: 쓴 틱이 queued_tick, lockstep 11.7절 규칙으로 done_tick. `availableForWrite()` = 16384 − 아직 안 끝난 바이트. 시험 선택 `set_tx_fifo_model(true)`(C++ 단위 시험만): 코어 3.1.2 송신 규칙(FIFO 128 B, `availableForWrite()` = 128 − 찬 수, 찬 수 127 이상이면 `write`가 기다림). 기다림은 틱 안 CPU 시간으로 센다(`tx_block_events()`, `tx_block_units()`). 그 뒤 핀 쓰기 시각(`pin_write_units()`)에 나타난다 |
| `EEPROM` | 256 B 플래시 흉내. 같은 하위 프로세스 안에서 MCU 리셋을 넘어 남는다. RESET이면 시작 줄 이미지로 돌아간다. `commit()`은 바뀐 바이트가 있을 때만 쓰고 `eeprom_commits`를 올린다. 시험 선택 `set_eeprom_commit_fail(true)`(C++ 단위 시험만): 바뀐 바이트가 있는 `commit()`이 실패한다(플래시와 `eeprom_commits`는 그대로) |
| `ESP.getChipId()` | 시작 줄 `firmware.chip_id` |
| `ESP.random()` | splitmix64 흐름의 아래 32비트(0은 건너뜀). RESET 때 `boot_seed`, MCU 리셋을 넘어 이어진다 |
| `ESP.getResetInfoPtr()->reason` | 마지막 리셋 원인(RESET·전원 켬 0, `mcu_reset` 인자, `port_open_reset` 6) |
| `WiFi.mode()`, `delay()`, `yield()` | 아무것도 하지 않는다 |

`fw_session.cc`가 lockstep 틱마다 사건 → 수신 → `setup()`/`loop()` → 구동기 한 틱 → ADC 표본 → 변화 채널 순서로 돈다(LOCKSTEP 11.5절). 구동기·가변저항 모형은 v1 코드(`SteerModel::fw_tick`)를 그대로 쓴다. 조향 모터 급전이 꺼지면 드라이버 출력 0(`driver_dead`와 같은 처리)이다.

## 3. lockstep firmware 모드 사용

시작 줄은 `protocol` `ubts.steer_lockstep.v2`, `mode` `firmware`, `firmware` 객체다(LOCKSTEP 11.2절). Python 연결은 `actuator_link.FirmwareSteeringBackend`다.

```python
from steering import vehicle_profile as vp
from steering.actuator_link import FirmwareSteeringBackend, host_line
from steering.native import fw_eeprom, lockstep_v2 as v2

profile = vp.load_profile()
startup = v2.firmware_startup_config(profile, seed=0)
image, details = fw_eeprom.calibrated_eeprom(startup, profile)      # LOCKSTEP 11.11: 보정을 마친 차
b = FirmwareSteeringBackend(profile, 0, firmware=dict(boot_seed=1, chip_id=0x5A1B2C, eeprom_hex=image,
                                                      adc_mode='model'))
b.send(host_line('ARM ' + b.boot_id(0)), tick=10)                   # 절대 틱, 그 틱 4단계 전에 RX 버퍼
b.send(host_line('C 1 300 100 100 200'), tick=20, pace='uart')      # 11.7절 도착 규칙
b.event('port_open_reset', tick=900)                                # 사건
b.run_until(1000, vehicle_stopped=False, vehicle_speed_m_s=0.3)     # 10 ms(10틱) FW_STEP씩
lines = b.lines()                     # McuLine(text, queued_tick, done_tick), .kind, .status(), .crc_ok
pins = b.channel('drive_l')           # [(tick, value)] 평가 전용
out = b.last                          # FwStepOutput: base(참값·구동 duty·pot_raw), tx, changes, ...
b.close()
```

- 낮은 수준: `reset(delta_eq_rad=None)`, `fw_step(dt_s, events=[(off, kind, arg)], rx=[(off, byte)], vehicle_speed_m_s=, vehicle_stopped=, supply_scale=)`. 한 프레임 그대로다.
- `step(SteerStepInput)`은 SteeringBackend 계약을 지킨다: `serial_in`은 스텝 첫 틱에 도착, `serial_out`은 그 스텝에 끝난 모든 바이트(부트 쓰레기 포함). 플랜트 결합(P4)용이다.
- 변화 채널, `mcu_state`, `run_flags`, `boot_count`, `mcu_millis`는 평가·적합성 전용이다. 호스트·실행기에는 직렬 바이트만 준다(`read_new_bytes()`).
- 3단계(T1 장치, 2026-11-09): 시작 줄 `firmware` 객체의 `report_servo`(기본 거짓)가 참이면 채널 11~14(`servo_pos`, `servo_tgt`, `servo_cmd`, `steer_mode`)를 더 보낸다(LOCKSTEP 11.6). 세션이 펌웨어 RAM(`ubts_fw::fw.pos/tgt/cmd/mode`)을 틱마다 읽는다. 펌웨어 소스는 그대로라 build id도 `04353B49` 그대로다. 네이티브 세션 소스는 바뀌어 모든 빌드의 소스 SHA-256 앞 12자리가 `b2be888b9a21`에서 `fd0e970b4ccd`가 됐다. 거짓이면 응답 바이트가 전과 같다. T1 장치(`T1_RIG_KO.md`)가 이 채널로 평가 열(`truth_servo_view_rad` 등)을 만든다. 호스트·실행기에는 가지 않는다.
- `native/lockstep_v2.py`: v2 머리·FW_STEP 인코드·디코드, 시작 줄, 틱 누적 규칙(`TickClock`), UART 도착 규칙(`host_arrival_ticks`, `mcu_done_tick`), SIM boot_id(`sim_boot_ids`), JSON merge patch.
- `native/fw_eeprom.py`: 레코드(9.2절)와 256 B 이미지, CFG 표, 프로필 → CFG 변환(4.10절), 보정 이미지 만들기(LOCKSTEP 11.11절). 펌웨어 C++와 따로 쓴 두 번째 구현이라 시험이 서로 맞춰 본다.

## 4. 빌드

네이티브 프로그램 하나에 v1 모드와 firmware 모드가 함께 들어 있다. 빌드 도구 `sim/ubts/steering/native/build.py`가 네이티브 소스, `arduino_shim/`, 펌웨어 `.cpp`를 `-DUBTS_STEER_SIM=1 -DUBTS_STEER_FW_SRC=<build>`로 컴파일한다. 펌웨어 build는 V3 규격 10.5절 규칙(정렬한 `.ino .h .cpp .c`의 경로 + 0x00 + u64 길이 + 바이트의 SHA-256 앞 8자리)이다.

```text
cd sim/ubts/steering/native
python -B build.py --firmware-id                       # 펌웨어 build id (컴파일러 없이)
python -B build.py [--build-dir DIR] [--force]         # 네이티브 프로그램
python -B build.py --unit-test [--build-dir DIR]       # C++ 단위 시험 프로그램(인자 없이 실행, 0 = 통과)
```

- 컴파일러: `UBTS_STEER_CXX` 또는 PATH의 `g++`. 출력 폴더: `UBTS_STEER_BUILD_DIR` 또는 git이 무시하는 `_build/`. 작업 트리 밖 폴더를 쓴다.
- 펌웨어 폴더: `UBTS_STEER_FIRMWARE_DIR`(스케치가 있어야 한다. 없으면 오류) 또는 이 작업 트리의 `firmware/ubts_steer_v3`(저장소 상대 경로).
- `UBTS_STEER_REQUIRE_FIRMWARE=1`: 펌웨어 소스가 없을 때 firmware 모드가 필요한 시험(`test_steering_firmware.py`, `test_steering_host_firmware.py`, `test_steering_conformance.py`의 실기 대본 시험)이 건너뛰지 않고 실패한다. 펌웨어를 추적하는 작업 트리와 통합 실행에서 쓴다(검토 RF-05: `.gitignore` 때문에 소스가 빠진 복사본에서 시험이 조용히 건너뛰었다). 시험 `RequireFirmwareTest`가 두 경우(스케치 없는 `UBTS_STEER_FIRMWARE_DIR`, 기본 폴더 없음)를 본다.
- 기본 펌웨어 폴더가 없으면(예: `sim/ubts`만 복사한 이미지) firmware 모드 없이 빌드한다. `fw_session_absent.cc`가 `fw_session.cc`, `fw_sketch.cc`, `arduino_shim/sim_mcu.cc`를 대신한다. v1 모드와 v2 direct_pwm·test_servo 모드의 응답 바이트는 그대로다. READY는 `fw=none`이고 firmware 모드 시작 줄은 종료 2다. 빌드 결과·manifest의 `firmware_build_id`는 None이다. `FirmwareSteeringBackend`는 이 빌드로 프로세스를 띄우기 전에 `NativeBuildError`를 낸다. 단위 시험 프로그램은 펌웨어 소스가 있어야 한다.
- 플래그: MinGW 32비트 `-std=gnu++11 -O2 -Wall -Wextra -static -Wl,--no-insert-timestamp -msse2 -mfpmath=sse`, Linux `-std=c++11 -O2 -Wall -Wextra`.
- 시험(작업 트리 `sim/ubts`에서): `python -B -m pytest -q -p no:cacheprovider --basetemp=<runs>/pt test_steering_firmware.py`. Python 3.10(WSL, pytest 없음)은 `python3 -B -m unittest test_steering_firmware`.
- 적합성 대본: `python -B -m steering.conformance.run_conformance --executable <네이티브> --out <result.json>`.

실기 ESP8266(컴파일만, P9 전 확인용):

- 이 PC에는 `arduino-cli`, PlatformIO, Arduino IDE 실행 파일이 없다. ESP8266 Arduino 코어 3.1.2와 xtensa-lx106-elf gcc 10.3은 있다(`%LOCALAPPDATA%/UBTS/ESPControl/arduino/data`).
- 그래서 코어 `platform.txt` 규칙(NodeMCU 1.0 기본: 80 MHz, 4M/2M, lwIP v2 낮은 메모리, MMU 32/32, 예외 끔)을 그대로 따른 저장소 밖 스크립트로 컴파일·링크만 했다(`steering-sim-20261108/runs/wf2a/p3_firmware/esp8266_build.py`). 업로드·직렬 포트는 쓰지 않았다.
- 결과: 112개 번역 단위 컴파일, 펌웨어 파일 `-Wall -Wextra` 경고 0, 링크 성공.
- 사용자가 arduino-cli로 빌드할 때(예): `arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --build-property "compiler.cpp.extra_flags=-DUBTS_STEER_FW_SRC=<build>" firmware/ubts_steer_v3`. `UBTS_STEER_FW_SRC`를 주지 않으면 배너가 `build=00000000`이다.

## 5. RAM과 코드 크기 (위 ESP8266 컴파일 결과)

| 항목 | 크기 |
|---|---|
| 펌웨어 RAM 상태 `ubts_fw::fw`(.bss) | 11,080 B. 위치 고리 2,048, f·raw 고리 512, THERMAL 비트 7,500, 송신 고리 512, 줄 버퍼 64, 보정·변수 W/R 약 200, 나머지 상태 약 240. fix_p3 빌드(`04353B49`)는 11,088 B(처리한 틱 수, THERMAL 색인, DRV_STUCK 관찰 상태) |
| 결함 인자 임시(.bss) | 64 B |
| CFG 표 `kParams`(.rodata, ESP8266에서는 RAM) | 492 B, 그 밖 문자열 상수 약 1 KB |
| 펌웨어 코드(플래시) | 약 18 KB(`fw_link` 8 KB가 가장 큼) |
| 전체 이미지 DRAM(코어·Wi-Fi 라이브러리 포함) | 45,088 / 80,192 B (56 %): data 1,504, rodata 2,448, bss 41,136. fix_p3 빌드는 45,104 B(bss 41,152) |
| 전체 이미지 IRAM | 59,267 / 65,536 B (90 %): 캐시 32,768 + 코어 코드 26,499. fix_p3 빌드도 같다 |
| 전체 이미지 플래시 코드 | 255,716 / 1,048,576 B (24 %). fix_p3 빌드는 256,004 B(펌웨어 코드 약 18.3 KB, `fw_link` 8,100 B) |

- `EEPROM.begin(256)`과 `Serial.begin()`의 버퍼는 코어가 `setup()` 안에서 힙에 잡는다. 펌웨어는 `setup()` 뒤에 힙을 쓰지 않는다.
- 스택: 함수마다 지역 버퍼가 작다(가장 큰 것은 줄 만들기 184 B, CFG 시험 82 B, EEPROM 레코드 106 B).

## 6. 규격을 읽은 방법 (구현이 고른 것)

규격이 정하지 않았거나 두 가지로 읽히는 곳이다. 안전한 쪽을 골랐다. P3 통합(2026-11-09)에서 모두 규격 12.1절 결정으로 정했다:

| 아래 번호 | 결정 | 펌웨어 |
|---|---|---|
| 1 | 12.1-63 | 그대로(규격 30번 raw 창). 가변저항 이득과의 관계는 규격 12.2절 P17 |
| 2 | 12.1-49 | 그대로 |
| 3 | 12.1-50 | 그대로 |
| 4 | 12.1-51 | 그대로 |
| 5 | 12.1-52 | 그대로. 대본 `crc_burst_stop_special` |
| 6 | 12.1-53 | 그대로. 대본 `c_nocrc_shows_q` |
| 7 | 12.1-54 | 그대로. 대본 `cal_erase_twice` |
| 8 | 12.1-55 | 그대로 |
| 9 | lockstep 11.5절 | 그대로 |
| 10 | 12.1-56 | 그대로 |
| 11 | 12.1-57 | 그대로 |
| 12 | 12.1-58 | 그대로(규격 8.3절 6단계로 적었다) |

통합에서 펌웨어를 바꾼 곳은 하나다: BENCH의 틀 오류 정지 시각(12.1-48). 예전에는 그 줄의 LF 틱에 brake했다. 지금은 V2처럼 오류를 처음 알아챈 바이트를 읽은 틱에 brake한다(`fw_link.cpp`의 `framing_error_seen()`). `N` 줄과 나쁜 줄 계수는 그대로 LF 틱이다. LF 틱에 다시 brake하지 않으므로 램프 10 ms 격자는 오류 바이트 틱에 묶인다(V2와 같다). 펌웨어 build id가 `1AE138AC`에서 `73575045`로 바뀌었다. 대본 `bench_framing_error_early_brake`가 이 동작을 본다(옛 빌드에서는 실패한다).

fix_p3(2026-11-09)에서 검토 결과 RF-01~RF-10을 고쳤다. 모두 V3 규격 12.1절 결정 66~73으로 적었다. 펌웨어 build id가 `73575045`에서 `04353B49`로 바뀌었다. 표의 "옛 빌드"는 `73575045`다.

| 검토 | 바꾼 것 | 결정 | 고침 전 실패를 본 시험 |
|---|---|---|---|
| RF-01 | `loop()` 11단계를 핀 쓰기, 12단계를 송신 펌프로 바꿨다. 펌프는 한 번에 `availableForWrite()` − 1 바이트까지만 옮긴다(그 값이 1 이하면 돌아간다). 코어 3.1.2는 FIFO에 127 B 이상이면 `write`에서 기다린다. 예전 순서·양에서는 긴 송신 뒤의 핀 쓰기, 곧 STOP의 정지가 늦을 수 있었다 | 66 | `fw_unit_test.cc` `test_tx_fifo_model`(옛 빌드: `write` 기다림이 생기고 STOP 틱의 핀 쓰기가 틱 시작보다 늦다. 5개 검사 실패) |
| RF-02 | THERMAL 조건(창 안 누적 > `tha`)과 가변저항 범위 조건이 참이면 `CAL JOG`, `CAL POL`도 ARM처럼 `N - fault`다(`monitor_arm_blocked()`). THERMAL 래치가 있고 조건이 다시 참이면 매 틱 조향 출력을 끈다 | 68 | 대본 `thermal_blocks_jog_and_pol`(옛 빌드: 래치 뒤 JOG를 받아 100 % 출력이 이어짐), `test_thermal_fault_from_jog` |
| RF-03 | runTrack은 \|cmd − P\| > `trk`이고 위치가 최근 `stt` 틱 동안 cmd 쪽으로 `stm`보다 많이 움직이지 않은 틱만 센다. 따라가는 서보의 뒤처짐은 세지 않는다. 멈춘 조향은 예전처럼 센다(대본 `fault_track`의 TRACK 1272 그대로) | 69 | 대본 `servo_lag_with_progress_no_track`(옛 빌드: 약 1.7 s에 TRACK), `test_large_steps_and_sweeps_do_not_trip_track`(옛 빌드: 정지 중 ±900 계단 세 시드와 주행 중 전폭 스윕 7개 하위 시험 실패) |
| RF-04 | runStuck: BRAKE 틱에서 시작하고 그 위치를 기준으로 둔다. BRAKE 동안 `db`보다 더 움직였으면 다시 미는 동안에도 이어 센다. 미는 쪽으로 기준을 `db`보다 넘게 지나면 끝난다. 기본 변수(db 20, dsm 80)로도 DRV_STUCK이 난다 | 70 | 대본 `fault_drv_stuck_default`, `test_driver_stuck_with_the_default_record`(옛 빌드: pwm 0.6, 1.0에서 DRV_STUCK 없음) |
| RF-06 | (흉내 층) RX 고리를 코어 3.1.2처럼 255 B, 가장 옛 바이트 버림으로 바꿨다. 펌웨어 소스는 그대로다 | 67 | 대본 `rx_overflow_drops_oldest`(옛 흉내 층: 새 바이트를 버려 첫 INFO를 틱 100에 처리) |
| RF-07 | THERMAL 비트 고리 색인을 틱 번호 mod 60,000이 아니라 따로 둔 색인(`thermal_idx`, 60,000에서 감김)으로 바꿨다. 2^32는 60,000의 배수가 아니다. "과거 값이 있는가"는 포화하는 처리 틱 수 `ticks_seen`으로 본다 | 72 | `test_thermal_window_across_tick_wrap`(옛 빌드에 맞춘 판: 2^32 근처에서 시작한 두 경우에 누적이 0으로 돌아오지 않음, 2개 검사 실패) |
| RF-08 | 펌웨어는 그대로다. 저장하지 않은 CFG로 ARM하는 것은 벤치 시험에 쓴다(대본 `cfg_effects`). 호스트가 `cfgdirty=1`이면 arm을 거절한다 | 74 | 호스트 시험 `config_unsaved` |
| RF-09 | POLARITY·DRV_STUCK 래치가 있으면 ARM은 `N - pol`이다. 통과한 `CAL POL`만 두 래치를 지운다. 리셋·전원은 RAM과 함께 지운다 | 71 | 대본 `polarity_latch_needs_pol_check`(옛 빌드: ARM 한 번으로 래치가 풀림) |
| RF-10 | `CAL ERASE`의 `EEPROM.commit()`이 실패하면 R, W, NOCAL 비트를 그대로 두고 `F NOCAL`을 내지 않는다 | 73 | `test_eeprom_commit_failure`(옛 빌드: 실패한 ERASE 뒤 R이 지워짐. 3개 검사 실패) |

- RF-05(`.gitignore`가 펌웨어 소스를 무시)는 저장소 규칙을 고쳤다(7절 끝). RF-11(LOCKSTEP 8행)은 문서를 고쳤다.
- 이 표의 새 시험·대본은 펌웨어를 고친 같은 작업이 썼다. 기대값은 규격 식으로 손 계산했고 옛 빌드에서 실패하는 것을 확인했다. 그래도 R1의 "따로 쓴 대본" 조건은 약하다(적합성 문서 1절).

아래는 구현 때 적은 원래 메모다.

1. POT_NOISE는 raw 창으로 본다(12.1절 30). 그래서 raw가 한 번 크게 바뀌면(가변저항 단선·단락·튐) 그 틱에 POT_NOISE가 먼저 켜지고, 2틱 뒤 f가 바뀌면 그 결함 코드(POT_OPEN 등)가 따로 켜진다. 스크립트 ADC를 10 count 넘게 계단으로 바꿔도 POT_NOISE·POT_JUMP가 켜진다.
2. THERMAL은 "부팅 뒤 창 길이만큼 기록이 없으면 판정하지 않음" 규칙을 쓰지 않는다. 부팅 전 틱은 출력 0으로 센다(그래야 6.7절 예 "t + tha + 1"이 부팅 직후에도 맞는다). fix_p3: 부팅 뒤 기록이 있는지는 처리한 틱 수(포화 계수)로 가른다. 틱 번호는 2^32에서 감기기 때문이다(12.1-72).
3. 결함 창의 틱은 "처리한 틱"이다. SIM에서는 1 ms 틱과 같다. `mcu_stall` 뒤에는 멈춘 틱을 세지 않는다(실기 루프 멈춤도 같다).
4. 64번째 내용 바이트가 제어 문자면 `ctrl`이다(바이트 자체 검사를 길이 검사보다 먼저 한다). CR은 내용이 아니므로 63바이트 뒤 CR LF는 정상 줄이다.
5. `STOP` 특례: `STOP`과 CRC가 맞는 `STOP*HH`는 좋은 줄(나쁜 줄 연속을 0으로). CRC가 틀린 `STOP*xx`는 실행하지만 좋은 줄도 나쁜 줄도 아니다.
6. CRC가 없는 `C` 줄의 거절은 `N <q> nocrc`다(q 토큰이 0..65535면 q를 낸다. 틀·CRC 검사는 통과한 것으로 본다).
7. `CAL ERASE`는 NOCAL 비트가 이미 켜져 있어도 응답으로 `F NOCAL 0 <ms>`를 낸다(4.9절 응답). `wear` 검사는 R이 없어도 한다(4.2절 순서 그대로).
8. 상태 검사에서 `CAL`은 둘째 토큰이 `GET`일 때만 모든 상태, `CFG`는 토큰이 2개(조회)일 때만 모든 상태로 본다.
9. 리셋 뒤 `setup()` 틱에 도착한 바이트는 버린다(LOCKSTEP 11.5절 2단계가 3단계보다 앞).
10. `boot_count`는 `setup()` 수다(11.6절). `setup()` 틱이 `mcu_stall` 안이면 `setup()`만 부른다.
11. `CAL SAVE`의 `EEPROM.commit()`이 실패하면(실기만) R을 바꾸지 않는다. 응답 줄은 그대로 보정 보고 줄이다(실패 사유 줄이 규격에 없다). fix_p3: `CAL ERASE`도 같다(12.1-73).
12. `setup()`에서 V2처럼 Wi-Fi를 끈다(8.3절 순서 목록에는 없다).

## 7. 시험한 것과 하지 않은 것

시험한 것(2026-11-09):

- 빌드: MinGW 6.3(32비트), WSL g++ 11.4, 기반 이미지 g++ 13.3에서 네이티브 프로그램과 단위 시험 모두 `-Wall -Wextra` 경고 0. 같은 플랫폼에서 다른 폴더에 다시 빌드한 SHA-256이 같다.
- C++ 단위 시험 120개 검사 통과(세 컴파일러 모두).
- `test_steering_firmware.py` 50개: Windows 3.13(pytest), 기반 이미지 3.12(pytest 7.4.4), WSL 3.10(unittest) 모두 통과.
- 펌웨어 소스 없는 복사본(기반 이미지 안에 `sim/ubts`만 복사): firmware 모드 없는 빌드 경고 0, `test_steering_firmware.py`·`test_steering_native.py`·`test_steering_backend_api.py` 127 통과, 50 건너뜀(펌웨어 소스 없음, 문서 없음 사유).
- 1단계 시험(`test_steering_native.py`, `test_steering_backend_api.py`) 127개가 그대로 통과(Windows, 기반 이미지). Windows 전체 조향 시험(`test_steering_*.py`) 677 통과, 4 건너뜀.
- 적합성 대본 80개(따로 쓴 대본과 실행기, 스텝 나누기 바꿔 다시 돌림) 80/80 통과.
- ESP8266 코어 3.1.2 컴파일·링크(위 4·5절).

P3 통합 뒤 다시 돌린 것(2026-11-09, `fw=73575045`):

- 빌드: MinGW 6.3, WSL g++ 11.4, 기반 이미지 g++ 13.3에서 네이티브 프로그램과 단위 시험 `-Wall -Wextra` 경고 0. C++ 단위 시험 120개 검사 통과(세 컴파일러).
- ESP8266 코어 3.1.2 컴파일·링크만: 112개 번역 단위, 펌웨어 경고 0, 링크 성공. 크기는 위 5절과 같다(`fw_link` 8,036 B).
- 적합성 대본 85개: 세 환경 모두 85/85, 실행기 코덱과 패키지 코덱(`--transport backend`) 둘 다.
- 호스트↔펌웨어 T1 결합 `test_steering_host_firmware.py` 12개: Windows 3.13, WSL 3.10, 기반 이미지 3.12 통과. A17 P3 항목의 잰 값은 규격 11.4절 끝에 있다.
- `test_steering_firmware.py` 50개: 세 환경 통과.

fix_p3 뒤 다시 돌린 것(2026-11-09, `fw=04353B49`, 모든 빌드의 소스 SHA-256 앞 12자리 `b2be888b9a21`):

- 빌드: MinGW 6.3(32비트), WSL g++ 11.4, 기반 이미지 g++ 13.3에서 네이티브 프로그램과 단위 시험 `-Wall -Wextra` 경고 0. MinGW는 새 폴더에 `--force`로 다시 빌드한 실행 파일이 같은 바이트다.
- C++ 단위 시험 142개 검사 통과(세 컴파일러). 새 검사 22개: 송신 FIFO 모형(12.1-66), EEPROM commit 실패(12.1-73), THERMAL 고리 2^32 감김(12.1-72). 옛 빌드에 맞춘 판으로 돌리면 10개가 실패한다.
- `test_steering_firmware.py` 54개(새 4개, 넓힌 1개): Windows 3.13(pytest), 기반 이미지 3.12(pytest), WSL 3.10(unittest) 통과. 옛 빌드로는 펌웨어 쪽 새·넓힌 시험 3개가 실패한다(하위 시험 9개, 시험 1개). `RequireFirmwareTest` 2개(RF-05)는 고치기 전 `firmware_dir_or_skip()`으로 돌리면 `UBTS_STEER_REQUIRE_FIRMWARE=1`인 하위 시험 2개가 실패한다(건너뛰기로 끝난다).
- `test_steering_host_firmware.py` 13개(새 1개): 세 환경 통과.
- 적합성 대본 90개(새 5개): 세 환경 모두 90/90, 두 전송 모두. 새 대본 5개는 옛 빌드에서 모두 실패한다.
- Windows 전체 조향 시험(`test_steering_*.py`, `UBTS_STEER_REQUIRE_FIRMWARE=1`) 724 통과, 5 건너뜀(외부 자료·gz 환경 사유). 1단계 닫힌 루프 시험(`test_closed_loop_harness_hooks.py`, `test_closed_loop_external_paths.py`, 외부 경로 설정 파일 사용) 33 통과.
- ESP8266 코어 3.1.2 컴파일·링크만: 112개 번역 단위, 펌웨어 경고 0, 링크 성공(크기는 5절). 업로드·직렬 포트는 쓰지 않았다.

3단계 뒤 다시 돌린 것(2026-11-09, T1 장치 작업 p4_rig, `fw=04353B49`, 소스 SHA-256 앞 12자리 `fd0e970b4ccd`):

- `test_steering_firmware.py` 55개(새 `test_report_servo_channels`: 채널이 상태 줄 값과 같고, 끄면 다른 채널과 바이트가 같고, STOP 틱에 tgt = cmd = pos): Windows 3.13(pytest), WSL 3.10(unittest), 기반 이미지 3.12(pytest 7.4.4) 통과.
- 적합성 대본 91개(새 `servo_rate_creep_duty`, RF-03 규칙 고정): 세 환경 모두 91/91, 두 전송 모두.
- T1 장치 `test_steering_t1_rig.py`(호스트 브리지 + V3 메모리 전송 + 이 펌웨어 + `SteeredPlant` + 세계, 한 SIM 시계): Windows 3.13, 기반 이미지 3.12, WSL 3.10 통과(WSL은 W2 1개 건너뜀, R10).
- ESP8266 컴파일은 다시 하지 않았다(펌웨어 소스 그대로).

하지 않은 것:

- 실기 ESP8266 실행, 벤치 시험(P9, 사용자). 실기 가정 B9(송신 FIFO 대기 동작, RX 버퍼 넘침, 리셋 원인 대응, 부팅 지연, Wi-Fi를 끈 상태의 `ESP.random()`, EEPROM commit 정지 시간) 미확인. fix_p3의 송신 FIFO·RX 고리 규칙은 코어 3.1.2 소스를 읽고 옮긴 것이다. 실기 측정이 아니다. lockstep 세션은 여전히 16384 B 단순화로 돈다. FIFO 모형은 C++ 단위 시험에서만 켠다. 하드웨어 RX FIFO(128 B)가 더 담는 몫, "ERASE의 지움은 되고 W 쓰기는 실패" 같은 부분 실패는 흉내 내지 않는다.
- arduino-cli 경로의 빌드(이 PC에 없음). 위 크기는 손으로 옮긴 레시피 결과다.
- G-SIM pty 에뮬레이터(P6a). 호스트 해석기·브리지와의 메모리 전송 결합은 P3 통합에서 했다(`test_steering_host_firmware.py`). 플랜트(`SteeredPlant`) firmware 모드 결합은 3단계 T1 장치에서 했다(`T1_RIG_KO.md`).
- 자리값이 아닌 실제 사양 값. 모든 결과는 자리값 기반 SIM 결과다.

통합 때 확인할 것:

- 해결(fix_p3): 저장소 `.gitignore`의 `firmware/`(원래 PX4 펌웨어 빌드 결과용)가 새 폴더 `firmware/ubts_steer_v3/`도 무시했다. 그 줄 뒤에 `!/firmware/`, `/firmware/*`, `!/firmware/ubts_steer_v3/`를 더했다. 이제 맨 위 `firmware/` 안에서 `ubts_steer_v3/`만 `git status`에 보인다. 다른 깊이의 `firmware/` 폴더와 맨 위 `firmware/`의 다른 항목은 그대로 무시된다(`git check-ignore`로 확인). 커밋은 통합 담당이 한다.
- 해결(3단계 확인): 지금 `docker/Dockerfile.ubts-steer`(HEAD `0c1311e`)는 `COPY firmware /opt/ubts/firmware`로 펌웨어 소스를 복사하고, 펌웨어 build id가 없으면 이미지 빌드가 실패한다. `.dockerignore`에는 `firmware`를 막는 규칙이 없다. 이 문서의 이전 판에 적은 "`sim/ubts`만 복사"는 그보다 앞선 상태다. 3단계 시험은 이미지를 새로 만들지 않고 기반 이미지에 작업 트리를 읽기 전용으로 붙여 돌렸다.

## 8. 조향 법칙 걸이 (2026-11-16)

실기 조향 보드가 ESP32-S3 + 펌웨어 ABS-HOLD-8-REBOUND-LIMITS로 바뀌었다. 이 V3 펌웨어는 이제 SIM의 직렬 규약·안전 껍데기다. 실기 보드의 펄스 유지 법칙을 SIM에서 흉내 내려고 `fw_steer.h`에 비어 있는 함수 포인터 `steer_law_hook`을 더했다. 비어 있으면(기본, 기기에서는 늘) `servo_update()`가 지금까지의 V3 법칙을 그대로 돈다. 네이티브 SIM 세션이 프로필 `steer_fw_mode abs_hold_8`일 때 `native/abs_hold8.cc`를 끼운다. `#if UBTS_STEER_SIM` 분기는 두지 않았다(`test_steering_firmware.py`의 규칙). 소스가 바뀌어 펌웨어 build id가 바뀐다. 자세한 것은 [`ABS_HOLD8_SIM_KO.md`](ABS_HOLD8_SIM_KO.md).
