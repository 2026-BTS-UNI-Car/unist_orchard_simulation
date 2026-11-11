# 조향 네이티브 구동기·가변저항 모델 (P1)

- 작성: 2026-11-09 (P1 네이티브)
- 근거: `DESIGN_v1_KO.md` 2.3.1, 3.3, 3.4, 3.8 C·E표, `DECISIONS_KO.md` R1, `LOCKSTEP_PROTOCOL_KO.md` v1
- 상태: 자리값 기반 SIM 모델이다. 현장 검증이 아니다. MCU 펌웨어 로직은 아직 없다(P3).
- 이 문서는 규약 v1을 구현하면서 정한 세부(규약이 정하지 않았거나 두 가지로 읽히는 곳)를 적는다. 규약과 다르게 읽은 곳은 4.9절에 모았다.
- 고친 곳(2026-11-09, 검토 반영): 서보 최소 PWM 자리값(4.6), 시작 줄 검사 추가(4.2, 4.7), 가변저항 NaN 막기(4.7), 응답 기한(3절), 빌드 잠금·재현·캐시(2절). 규약 문서도 구현에 맞췄다(4.9).

## 1. 파일

| 파일 | 내용 |
|---|---|
| `sim/ubts/steering/native/steer_json.{h,cc}` | 시작 줄용 엄격한 JSON 읽기(중복 키, 비ASCII, NaN, 앞자리 0, 뒤 쓰레기, 깊이 64 초과를 거부) |
| `steer_config.{h,cc}` | 시작 줄 키 집합·형·범위 검사, SI 값으로 변환. 오류는 종료 코드 2 |
| `steer_geometry.{h,cc}` | 링크 표 T(조각 선형, 끝 밖 연장), 바퀴 쪽 등가각, 좌우 바퀴각(Python `geometry.py`와 같은 식) |
| `steer_rng.h` | splitmix64 + xoshiro256**, Box-Muller 정규 난수. 잡음 원천마다 흐름 하나 |
| `actuator.{h,cc}` | 조향 구동기(L0, L1), 백래시, 끝 멈춤, 정지 마찰, 떠돎, 풀림, 구동기 고장 |
| `potentiometer.{h,cc}` | 가변저항·ADC(양자화, 잡음, 직선성, 이득, 급전 처짐, 표류, 고장) |
| `steer_model.{h,cc}` | 틱 시계, 시험용 P 서보, 고장 주입 시각, 응답 칸과 model_flags |
| `steer_wire.h`, `lockstep_main.cc` | 프레임 읽기·쓰기, 순서·길이·값 검사, 종료 코드 |
| `build.py` | 빌드(아래 2절) |
| `sim/ubts/steering/actuator_link.py` | Python 연결 `NativeSteeringBackend` |
| `sim/ubts/test_steering_native.py` | 시험 91개(매개변수 조합 포함) |

C++11만 쓴다. MinGW.org GCC 6.3.0(32비트), g++ 11.4, g++ 13.3에서 경고 없이 빌드된다(`-Wall -Wextra`).

## 2. 빌드

- 컴파일러: 인자 `compiler`, 없으면 환경 변수 `UBTS_STEER_CXX`, 없으면 PATH의 `g++`.
- 출력 폴더: 인자 `build_dir`, 없으면 `UBTS_STEER_BUILD_DIR`, 없으면 `sim/ubts/steering/native/_build/`(같은 폴더 `.gitignore`로 git에서 뺀다). 실행 파일은 커밋하지 않는다.
- 플래그: MinGW 32비트 `-std=gnu++11 -O2 -msse2 -mfpmath=sse -static -Wl,--no-insert-timestamp -Wall -Wextra`, 다른 MinGW `-std=gnu++11 -O2 -static -Wl,--no-insert-timestamp`, Linux x86_64 `-std=c++11 -O2`. `UBTS_STEER_CXXFLAGS`를 뒤에 붙인다. Windows의 `--no-insert-timestamp`는 PE 머리의 빌드 시각을 빼서, 같은 소스·플래그·컴파일러면 실행 파일 바이트가 같게 한다(빌드 방법 버전 2).
- 이름: `ubts_steer_native-<target>-<key>[.exe]`. key는 소스·헤더 바이트, 컴파일러 버전 줄, target, 플래그의 SHA-256 앞 16자리다. 옆에 `<이름>.json`(도구·플래그·소스 SHA·실행 파일 SHA)을 둔다.
- 동시 호출: 출력마다 잠금 파일(`O_EXCL`)을 잡은 하나만 빌드한다. 잠금 파일은 `LOCK_STALE_S`(1020 s: 컴파일 기한 900 s + 시험 실행 기한 60 s보다 길다)가 지나면 낡은 것으로 본다. 기다리는 쪽은 `LOCK_TIMEOUT_S`(1320 s)까지 기다린다. Windows에서 다른 프로세스가 지우는 중인 잠금 파일은 `O_EXCL`에 `PermissionError`를 낸다. 이것은 10 s까지 다시 시도하고, 계속되면 `NativeBuildError`로 낸다. 임시 이름으로 빌드하고 시험 실행(시작 줄 전 EOF → 종료 0)을 한 뒤 원자적 이름 바꾸기로 낸다. 나머지는 끝난 파일을 다시 쓴다.
- 캐시: 한 프로세스 안에서 컴파일러 확인(`--version`, `-dumpmachine`, 컴파일러 파일의 크기·수정 시각별)과 실행 파일 SHA-256(파일 크기·수정 시각별)을 기억한다. 그래서 이미 빌드된 실행 파일을 다시 쓰는 `configure()`는 컴파일러를 다시 돌리지 않는다. 소스 해시는 수정을 놓치지 않게 매번 다시 읽는다. `clear_caches()`가 캐시를 비운다.
- 명령: `python sim/ubts/steering/native/build.py [--compiler 경로] [--build-dir 폴더] [--force] [--json]`. 컴파일러 버전 줄을 출력한다.
- READY의 build id: `ubts-steer-native/1 src=h<소스 SHA 앞 12자리> cxx=gcc-<버전> bits=<32|64>`.

## 3. Python 연결

- `NativeSteeringBackend(profile=None, seed=0, *, mode='direct_pwm', mcu_tick_hz=None, injections=(), executable=None, compiler=None, build_dir=None, exit_timeout_s=5.0)`. 프로필을 주면 바로 시작한다. `executable`에 목록을 주면 명령줄 전체로 쓴다(시험용 가짜 하위 프로세스).
- `configure(profile, seed)`: `backend_api.native_startup_config()`로 시작 줄을 만들어 보내고 READY를 받는다. 돌던 프로세스는 먼저 닫는다.
- `reset(delta_eq_rad)`, `step(SteerStepInput)`: 요청 하나, 응답 하나. 입력 검사는 보내기 전에 Python에서 한다(`ValueError`, 프로세스는 그대로). reset 각이 등가 기계 한계 밖이어도 보내기 전에 `ValueError`다.
- 실패(쓰기 오류, EOF, 잘린 프레임, 받을 수 없는 응답, 응답 기한 초과)는 `NativeBackendError`다. 칸: `phase`, `exit_code`, `exit_name`, `stderr_line`(`steer_native: <이름>: <내용>`), `killed`(부모가 죽였는지), `timed_out`(응답 기한 초과). 실패 뒤 백엔드는 닫힌다.
- 응답 기한: `response_timeout_s`(기본 30 s, `None`이면 끝없이 기다린다. 디버거용). 감시 스레드 하나가 0.25 s마다 지금 하고 있는 요청 쓰기나 응답 읽기의 경과 시간을 보고, 기한을 넘으면 하위 프로세스를 죽인다(읽지 않는 하위 프로세스 때문에 꽉 찬 파이프에 쓰다 멈추는 경우도 포함). 감시를 걸고 푸는 것은 속성 쓰기라서 스텝 비용이 거의 늘지 않는다. READY 전에 멈춘 경우는 `phase` `startup`, RESET·STEP 응답 전에 멈춘 경우는 그 이름이다.
- 수명: `close()`, 가비지 수집(`weakref.finalize`), 인터프리터 종료 모두 stdin을 닫는다. 하위 프로세스는 프레임 경계 EOF로 종료 0을 낸다. `exit_timeout_s` 안에 안 끝나면 죽인다.
- `result_label()`: 프로필 라벨(자리값 기반 SIM 결과, field_validated false) + backend, mode, seed, mcu_tick_hz, build id, 실행 파일 SHA, 주입 목록.

## 4. 모델 세부

### 4.1 상태와 틱

- 상태(등가각 눈금, rad, 왼쪽 +): 모터 쪽 각 θ_m, 모터 쪽 속도 ω_m, 링크 쪽 각 θ_l. 항상 \|θ_m − θ_l\| ≤ b/2.
- 한 틱: 명령 PWM(서보 주기마다 다시 계산) → 드라이버 출력 → 모터 갱신 → 모터 끝 제한 → 링크 갱신 → 링크 끝 멈춤 → ADC 표본 하나.
- 드라이버 출력 applied: driver_dead면 0, driver_stuck_on이면 EN 켬일 때 고착값·EN 끔일 때 0, 아니면 EN 켬일 때 명령·EN 끔일 때 0. 극성 반전은 그 뒤 u = −applied.

### 4.2 L1 모터

- 정상 속도 w = 부호(u) × rate × (\|u\| − min)/(1 − min) (\|u\| > min), 아니면 0. rate·min은 vehicle_stopped에 따라 주행 중·정지 중 값이다. `slow`는 rate에 곱한다.
- 시작 줄 검사: `rate_standstill_rad_s` ≤ `rate_max_rad_s`, `min_pwm_standstill` ≥ `min_pwm`(설계 3.3 L1: 서 있는 차는 조향이 느리고 시작 PWM이 크다). 어기면 종료 2. 프로필 읽기도 같은 검사를 한다(`standstill_order`).
- ω_m은 w를 1차 지연 τ로 따른다. 틱마다 정확한 해를 쓴다: θ_m += w·dt + (ω_m − w)·τ(1 − e^(−dt/τ)), ω_m = w + (ω_m − w)e^(−dt/τ).
- 출력이 0이 되면(EN 끔 포함) ω_m은 같은 τ로 줄어든다. 그래서 모터가 약 w·τ 더 간다(30°/s, 0.08 s면 2.4°).
- 0 < \|u\| ≤ min이면 드라이버 토크는 있지만 모터는 움직이지 않는다(정지 마찰). 이때는 "구동 중"(4.4)으로 본다.
- 끝 제한: θ_m ∈ [θ_l,min − b/2, θ_l,max + b/2]. 닿으면 그 방향 속도를 0으로 한다(멈춤).
- L1b(토크 모델)는 TODO다. 모터 무부하 속도와 부하 모델(Q11)이 없어서 정직하게 만들 수 없다. 시작 줄에서 L1b를 거부한다(종료 2).
- 드라이버 방향 전환 휴지: 설계의 방향 전환 휴지(`servo_reversal_pause_ms`)는 ESP 서보 로직(P3)이다. 플랜트 쪽 드라이버에는 두지 않는다.

### 4.3 끝 멈춤

- 바퀴 쪽 등가각 δ_eq = T(θ_l)(1 + lr·s) + zero_offset. 링크 끝 θ_l,max = δ_eq⁻¹(eq_mech_limit_left), θ_l,min = δ_eq⁻¹(−eq_mech_limit_right)를 시작 때 한 번 계산한다.
- 응답의 δ_eq는 등가 기계 한계로, 좌우 바퀴각은 바퀴별 기계 한계로 다시 자른다. 반올림만 막는 것이다. 시작 줄 검사에서 등가 한계가 바퀴별 한계와 맞는지(한 바퀴가 한계에 닿고 둘 다 안에 있는지, 1e-9 rad) 본다.

### 4.4 링크: 구동 중·잠김·풀림

| 상태 | 조건 | 링크 동작(주행 중만. 서 있으면 링크는 놀이 연산자만 따른다) |
|---|---|---|
| 구동 중 | applied ≠ 0 (L0은 EN 켬) | 모터가 놀이 끝에서 링크를 끈다. 링크는 놀이 안에서 떠돈다(backlash_wander) |
| 잠김 | 출력 0이고 자기 잠금이거나 jam | 모터는 외력으로 안 움직인다. 링크는 놀이 안에서 떠돈다 |
| 풀림 | 출력 0이고 자기 잠금 아님, jam 없음 | 링크가 복원·잡음으로 움직이고 놀이 끝에서 모터를 끈다(역구동). FREE_WHEELING 표시 |

- 떠돎: 거리 기준 OU 과정 q(sd, 상관 거리)의 증분을 링크에 더하고 놀이 안으로 자른다. q는 시작 때 정상 분포에서 뽑고 ds = \|v\|·dt마다 q ← ρq + sd·√(1−ρ²)·N, ρ = e^(−ds/ℓ).
- 풀림: θ_l ← θ_s + (θ_l − θ_s)·e^(−k·ds) + Δn. k = release_scale × release_centering_per_s_per_m_s (1/m), θ_s는 바퀴가 곧은(δ_eq = 0) 링크 각, Δn은 둘째 OU 과정(release_noise)의 증분이다. 복원 0, 잡음 sd σ, 상관 ℓ이면 정상 상태 sd는 σ/√(1 + kℓ)이다(기본값에서 0.3°/√2).
- 두 OU 과정은 상태와 상관없이 주행 거리만큼 늘 나아간다. 그래서 상태가 바뀌어도 난수 흐름이 밀리지 않는다.

### 4.5 L0 (test_servo 전용)

- 이상 서보: 가변저항 쪽 참각이 T⁻¹(목표)를 1차 지연 τ로 따른다. 한 틱 이동은 rate·dt로 자른다. 백래시·끝 멈춤·0점 오차·좌우 차는 플랜트 성질이라 그대로 둔다.
- applied_steer_pwm은 PWM이 아니라 정규화한 노력(한 틱 이동 / (rate·dt))이다.
- 드라이버 고장 주입(driver_dead, driver_stuck_on, polarity_reversed)과 direct_pwm 모드는 시작 줄에서 거부한다.

### 4.6 시험용 P 서보 (test_servo, L1)

- 규약 6절 그대로: m = T(가변저항 쪽 참각, 잡음 없음), e = 목표 − m, \|e\| ≤ 데드밴드면 0, 아니면 부호(e)·min(max, max(min, kp·\|e\|)). 서보 주기(틱 수 = mcu_tick_hz / servo_hz)의 첫 틱에 다시 계산한다.
- 모터는 \|u\| > 시작 PWM일 때만 움직인다(4.2). 데드밴드 바로 밖의 가장 작은 서보 출력은 max(서보 최소 PWM, kp·데드밴드)다. 이 값이 모터 시작 PWM 이하이면 서보는 데드밴드 밖에서 선다: 오차 \|e\| ≤ 시작 PWM / kp인 곳이다.
- 이전 자리값(서보 최소 PWM 25 %)에서는 주행 중(시작 25 %) 약 1.67°, 정지 중(시작 35 %) 약 2.33° 앞에서 섰다. 목표 1~1.5°는 모터가 아예 움직이지 않았다. 이전 문서는 정지 중 경우만 적었다. 자리값을 40 %로 고쳤다(주행·정지 시작 PWM보다 크다). 서보 최소 PWM이 시작 PWM 이하인 프로필은 `warnings()`에 `servo_min_pwm_cannot_start_the_motor`로 남는다. 이 조합도 일부러 시험할 수 있게 읽기는 거부하지 않는다.
- 서 있을 때는 데드밴드 안에서 출력 0으로 멈춘다. 주행 중에는 비자기잠금 풀림 모델(4.4)이 출력 0인 바퀴를 곧은 쪽으로 되돌린다. 시험용 서보에는 데드밴드 안 제동 유지(펌웨어 `servo_dwell_brake_s`, P3)가 없어서 데드밴드 끝에서 다시 민다. 오차는 데드밴드 + 서보 한 주기 동안의 끌림 안에 머문다(기본 프로필, 0.4 m/s에서 시험).

### 4.7 가변저항

- 규약 7절의 식을 그대로 쓴다. round는 0에서 먼 쪽 반올림이다.
- 직선성 항은 전 범위 N count에 사인 한 주기다. 조향 범위(자리값 ±28°, 약 ±106 count)에서는 사인이 거의 직선이라, 굽음보다 국소 이득 변화로 크게 나타난다: 중앙 기울기 × (1 + 2π × 직선성), 2 %면 약 1.126배(잡음 0일 때 5~25°에서 1.107~1.134). 같은 범위에서 이득 오차 5 %(1.05배)보다 크다. 그래서 민감도 스윕에서 `pot_linearity_pct`는 `pot_gain_error`와 섞인 항으로 해석한다. 보정(B2 끝점, B3 각도표)이 이 국소 이득을 흡수한다. 모델 식은 바꾸지 않았다(검토 F6, 규약·프로필 note에 적었다).
- 시작 줄 검사: 이득 g = 2^bits × 비 × (1 + 이득 오차) / 범위가 유한하고 (0, 1e9] count/rad 안이어야 한다(종료 2). 너무 작은 `range_rad`(예: 5e-324)는 g를 무한대로 만들어, 측정각 0에서 r0가 NaN이 되었다. 표본 계산에서 값이 수가 아니면 int 변환 전에 내부 오류(종료 9)로 끝낸다(정의되지 않은 동작을 막는 방어. 시작 줄 검사 뒤에는 정상 입력으로 생기지 않는다).
- 순서: 측정각(+ 고유 표류) → pot_reversed(부호) → pot_slip(+offset) → pot_drift(+rate·(t − at_s)) → 이득·직선성 → 급전 처짐(applied는 이 틱의 조향 출력) → 잡음 → pot_jump(+counts) → 0..N−1로 자르기 → pot_stuck(at_s 뒤 첫 표본을 잡아 유지) → pot_short(N−1) → pot_open(0). open이 가장 앞선다.
- 잡음 난수는 표본마다 늘 하나 뽑는다(고장 상태와 상관없이).
- POT_CLIPPED는 계산값이 0..N−1 밖이라 잘렸을 때만 켠다. open·short로 바꾼 값은 잘림이 아니다.
- RESET도 t = 0 표본 하나를 만든다(조향 출력 0, 급전 배율 1).

### 4.8 고장 주입 시각과 겹침

- at_s의 틱 번호 k_at = ceil(at_s × tick_hz − 1e-9). 구동기 고장은 시작 시각이 at_s 이상인 틱부터, 가변저항 고장은 시각이 at_s 이상인 표본부터 켠다. pot_jump는 시각 < at_s + duration_s인 표본까지다(정수 틱이면 정확히 duration × tick_hz 표본).
- 같은 종류가 여럿이면: slow·pot_noise_x10은 곱, pot_jump·pot_slip·pot_drift는 합, driver_stuck_on은 가장 늦게 켜진 것, 나머지는 하나라도 켜지면 켠다.
- INJECTION_ACTIVE는 응답 시각(ticks_total / tick_hz)에 켜진 주입이 있으면 켠다.

### 4.9 규약과 다르게 읽은 곳

1. 풀림 복원: 처음 규약 식은 −k·\|v\|·θ_l(링크 0점 쪽)이었다. 이 구현은 바퀴가 곧은 링크 각 θ_s 쪽으로 당긴다. 복원 모멘트는 타이어가 바퀴를 곧게 하는 힘이라서다. 0점 오차가 0이면 같다. 규약 7절을 이 구현에 맞춰 고쳤다(2026-11-09).
2. 풀림 잡음: 규약 식의 "OU 잡음"을 OU 과정(sd, 상관 거리)의 증분으로 읽었다.
3. 잠김(자기 잠금, 출력 0): 규약은 "링크는 놀이 안에서만 움직인다"고만 한다. 이 구현은 그 움직임을 backlash_wander로 만든다. 복원은 넣지 않았다.
4. 출력 0일 때 모터 속도: 1차 지연으로 줄어든다(관성 주행). 규약은 따로 정하지 않았다.

## 5. 시험과 합격 검사

- `test_steering_native.py` 91개: 빌드(이름·재사용·동시 호출 3개에서 한 번만 빌드·다른 폴더 빌드와 같은 바이트·실패·명령줄·잠금 PermissionError 다시 시도·재사용 때 컴파일러 확인 한 번), READY·RESET, 응답 기한(READY 전·RESET 응답 전에 멈춘 가짜 하위 프로세스, 읽기를 멈춘 하위 프로세스에 쓰기), 시작 줄 거부(정지 중 값 순서, 가변저항 이득), 기본 프로필 서보가 주행·정지 모두 데드밴드에 듦(목표 6개), 서보 최소 PWM이 시작 PWM과 같을 때 앞에서 섬, 고정 run별 값(0점 오차·보정 잔차)이 두 백엔드에 같음, 직선성 항의 국소 이득, 누적 틱 규칙, 틱 0 스텝, 속도 한계·지연, 최소 PWM·정지 마찰, 백래시, 바퀴별 끝 멈춤(linkage a = 0/0.5/1, axle_pivot, 비대칭 한계·좌우 차·0점 오차·비선형 링크 표), 한계 밖 목표, 풀림·자기 잠금·떠돎·풀림 잡음 통계, 구동기 고장 5종, 서보 법칙·수렴, L0, 가변저항 양자화·위치(모터/바퀴)·잡음 sd·직선성·이득·0점·급전 처짐·고장 8종·잘림 표시, 결정성(같은 seed 같은 바이트, 다른 seed 다름, RESET 뒤 같은 흐름), 종료 코드 0·2·3·4·5·6·7·8, Python 예외, 가짜 하위 프로세스(조각 읽기·도중 종료·잘못된 응답), close·가비지 수집·인터프리터 종료 뒤 남은 프로세스 0.
- 합격 검사 대응(자리값 기반 SIM 결과): A2(속도 한계·τ·데드밴드·백래시·끝 멈춤, 설정값 ±5 % 안), A3의 F 부분(잡음 sd ±10 %, LSB, 가변저항 고장 시작 틱 정확. 결함 코드와 판정 시간은 MCU 몫이라 P3), A8(같은 플랫폼·같은 빌드), A21의 구동기 부분(바퀴별 기계 한계를 넘는 바퀴각 0).
- 종료 코드 9(internal_error)는 정상 입력으로 일으킬 방법이 없어 시험하지 않았다.
