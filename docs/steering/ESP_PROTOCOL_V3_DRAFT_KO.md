# ESP 규약 V3 규격 초안 (조향 펌웨어·적합성 대본·호스트 공통)

- 작성: 2026-11-09 (P3 규격 작업)
- 근거: `DESIGN_v1_KO.md` 2.5, 3.3, 3.4, 3.8.2 D·D2표, 4.1, 4.2, 4.4, 6.5, 7.2.1, 부록 C. `DECISIONS_KO.md` R1~R12. `SIM_ACCEPTANCE_KO.md` A3, A17, A19, A20. 자리값 프로필 `sim/ubts/steering/profiles/front_steer_v0_placeholder.yaml`(D, D2, E, F표). V2 펌웨어 `rover_control_test/firmware/wheel_pwm/wheel_pwm.ino`(SHA-256 `a470445d…`)와 네이티브 하네스, 2026-09-19 제어 설계(`rover_control_test/docs/control-architecture.md`).
- 상태: P3 초안 규격이다. 펌웨어(`firmware/ubts_steer_v3/`), 적합성 대본, 호스트 해석기(`esp_motor_v3.py`)는 이 문서만 보고 같은 바이트를 만들어야 한다. 설계 4.1과 다른 곳은 12절에 결정 제안으로 모았다. 결정 기록(`DECISIONS_KO.md`)에 오르기 전에는 "제안"이지만, P3 구현은 이 문서를 따른다.
- 2026-11-09 통합(P3 통합 작업): 펌웨어, 적합성 대본, 호스트를 서로 맞춰 보고 규격이 두 가지로 읽히던 곳을 정했다(12.1절 48~65). 본문의 해당 줄에는 결정 번호를 붙였다. 호스트↔펌웨어 T1 결과는 11.4절 끝에 있다.
- 2026-11-09 검토 반영(P3 고침 작업): 펌웨어·호스트 검토에서 확인된 결함을 고치며 정한 것을 12.1절 66~77에 적었다. 송신 펌프가 실기 코어에서 기다리지 않게 하고 핀 쓰기를 송신보다 앞에 둔다(66), RX 고리는 옛 바이트를 버린다(67), THERMAL 조건 중 JOG·POL 거절(68), TRACK은 따라가는 서보를 세지 않는다(69), DRV_STUCK 관찰 규칙(70), POLARITY·DRV_STUCK 래치는 CAL POL 통과로만 풀린다(71), 결함 창의 감김(72), ERASE commit 실패(73), 호스트의 `cfgdirty` 거절(74), 호스트 래치는 확인된 ARM에서만 지운다(75), `N - boot`(76), 확인 전 ARM 뒤 STOP 확인(77).
- 모든 수치는 자리값이다. 이 규격으로 얻는 결과는 자리값 기반 SIM 결과다. 현장 검증이 아니다.
- 시험 벡터는 저장소 밖 스크립트로 계산했다: `D:/Develop/UBTS/steering-sim-20261108/runs/wf2a/p3_spec/crc_vectors.py`, 출력 `crc_vectors_out.txt`.
- SIM 실행 규약(lockstep v2, firmware 모드)은 `LOCKSTEP_PROTOCOL_KO.md` 11절에 있다.

과제 항목과 절의 대응: A → 2절, B → 3절, C → 4절, D → 5절, E → 6절, F → 7절, G → 8절, H → 9절, I → 10절, J → 11절, K → 12절, M → 13절, L → `LOCKSTEP_PROTOCOL_KO.md` 11절.

---

## 0. 읽는 법

- "한다", "하지 않는다"는 반드시 지킬 규칙이다. "[자리값]"은 사용자 사양·실측이 오면 바뀌는 값이다. "[가정]"은 실기에서 확인할 동작이다.
- 바이트는 ASCII로 적는다. `\n`은 0x0A(LF), `\r`은 0x0D(CR)다. 예의 줄 끝 LF는 따로 적지 않는다.
- 줄 예의 `*HH`는 그 줄의 CRC-8 두 자리다(3절). 이 문서의 예에 적힌 HH는 모두 실제로 계산한 값이다.
- 세 팀의 몫:
  - 펌웨어: 2~10절 전부. 7.1절의 틱 순서를 그대로 지킨다.
  - 적합성 대본: 기대 바이트는 이 문서에서 손으로 정한다. 형식은 13절이다.
  - 호스트 해석기: 2, 3, 4절의 송신 형식(2.5절 호스트 송신 규칙), 5절의 수신 형식, 6절의 상태 뜻, 11절의 시간 예산.

## 1. 용어와 기호

| 기호 | 뜻 |
|---|---|
| 틱 | MCU 시간 1 ms. SIM에서는 lockstep 틱 하나다(`mcu_tick_hz` 1000). 틱 n에서 `millis()`는 (n − 리셋 틱 + `millis_start`) mod 2^32다 |
| ms | `millis()` 값(u32). 시간 간격은 u32 뺄셈으로 잰다(감김 안전, 2^31 ms 미만 간격) |
| b | 부팅 틱. `setup()`이 도는 틱. 리셋 뒤 `boot_delay` 틱이 지난 틱이다(SIM: lockstep 시작 줄 `boot_delay_ticks`. 실기: 약 70 ms [자리값]) |
| raw | `analogRead(A0)` 값, 0..1023 |
| f | 거른 raw(7.2절 중앙값) |
| pos | 보정 위치, 천분율(‰). 왼쪽 표시 L = +1000, 중앙 C = 0, 오른쪽 표시 R = −1000(7.3절) |
| tgt | 서보 목표(‰). ARMED에서는 마지막으로 받은 `C`의 S |
| cmd | 속도 제한을 건 내부 목표(‰, 소프트 한계 안) |
| u | 조향 서보 출력, 부호 있는 PWM −255..255. 위치 공간 기준(+는 pos를 + 쪽으로 민다) |
| sp | 상태 줄의 실제 조향 핀 값 D6 − D7(OFF·BRAKE면 0) |
| L, R | 구동 적용 duty, 부호 있는 −255..255(+는 전진). 왼쪽·오른쪽 |
| EN | enable 핀. 구동 EN 하나(D1), 조향 EN(D2) |
| W, R레코드 | 보정·변수의 RAM 작업본 W와 EEPROM에 저장된 레코드 R(9절) |
| 자리값 기준 눈금 | 1000 ‰ ≈ 25.0°, 곧 1° ≈ 40 ‰. 기본 변수를 ‰로 바꿀 때만 쓴다[자리값] |

조향 출력 상태(조향 드라이버 핀, 7.7절):

| 이름 | 조향 EN(D2) | D6, D7 | 뜻 |
|---|---|---|---|
| DRIVE | 1 | 한쪽만 \|u\| | 모터에 전압 |
| BRAKE | 1 | 둘 다 0 | 드라이버 두 입력 LOW. BTS7960류면 단락 제동[가정] |
| OFF | 0 | 둘 다 0 | 드라이버 출력이 뜬다. 모터에 토크 없음 |

조향 모드(펌웨어 내부): OFF, ACTIVE(ARMED 서보), HOLD(정지 뒤 유지 서보), JOG(`CAL JOG` 개방 출력), POL(`CAL POL` 절차). ACTIVE와 HOLD에서만 서보(7.6절)가 돈다.

## 2. 줄 틀 (A)

### 2.1 바이트와 줄

- 115200 baud, 8N1, 흐름 제어 없음[자리값: 230400은 벤치 확인 뒤]. 두 방향 모두 ASCII 줄이다.
- 줄 끝은 LF(0x0A) 하나다.
- 줄 내용은 앞 줄 끝(또는 부팅) 뒤부터 LF 앞까지의 바이트다. 단, LF 바로 앞의 CR 하나는 내용에 넣지 않는다.
- CR 규칙(V2와 같다): CR은 LF 바로 앞에 하나만 올 수 있다. CR 뒤에 LF가 아닌 바이트가 오면 그 줄은 틀 오류 `ctrl`이다. `D8 128\r\n`은 정상 줄 `D8 128`이다. `D8 2\r55\n`은 틀 오류다.

### 2.2 줄 버퍼 (MCU 수신)

- 줄 버퍼는 64 B다. 내용은 최대 63 B다. 끝 NUL 자리 1 B를 남긴다. CR과 LF는 63 B에 세지 않는다.
- 64번째 내용 바이트가 오면 그 줄은 과길이다. 그 뒤 바이트는 다음 LF까지 모두 버린다. 다음 LF에서 `N - long`을 한 번 낸다. 버리는 동안의 CR·제어 문자는 따로 세지 않는다.
- 내용에 0x20..0x7E 밖의 바이트(NUL, 제어 문자, 0x7F, 0x80 이상)가 있으면 틀 오류다. 그 바이트부터 다음 LF까지 버리고, 그 LF에서 `N - ctrl`을 한 번 낸다. 앞 2.1절의 CR 규칙 위반도 `ctrl`이다.
- 한 줄에서 `long`과 `ctrl`이 둘 다 생기면 먼저 생긴 것 하나만 낸다. 64번째 내용 바이트 자체가 0x20..0x7E 밖이면 `ctrl`이다(바이트 검사가 길이 검사보다 먼저다. 12.1-51).
- 빈 줄(LF만, 또는 CR LF)은 조용히 무시한다. 응답, 계수, watchdog 갱신이 모두 없다.
- 틀 오류 줄(`long`, `ctrl`)은 명령으로 처리하지 않는다. 6.6절의 나쁜 줄 계수를 1 올린다. BENCH에서는 구동을 즉시 멈춘다(V2와 같다, 8.2절). 다른 상태에서는 출력을 바꾸지 않는다.
- BENCH의 정지 시각: 틀 오류를 처음 알아챈 바이트(64번째 내용 바이트, 제어 문자, CR 뒤의 LF 아닌 바이트)를 읽은 틱에 brake한다. V2가 그 바이트에서 `stopDirections()`를 불렀기 때문이다. `N` 줄과 나쁜 줄 계수는 그 줄의 LF를 읽은 틱이다(12.1-48).
- 부팅 때 파서는 빈 버퍼, 정상 상태로 시작한다. 리셋 전에 호스트가 보내던 반쪽 줄의 나머지는 보통 줄로 해석된다(대개 `cmd`나 `crc`로 거절된다). 부팅 뒤 버리기 상태로 시작하지 않는다.
- 한 틱(`loop()` 한 번)에 수신 버퍼에서 최대 64 B만 읽는다(V2와 같다). 남은 바이트는 다음 틱에 읽는다. 줄이 끝나는 LF를 읽은 그 자리에서 그 줄을 실행한다.
- 수신 버퍼(코어 RX 고리)는 256 B 배열이고 255 B까지 담는다. 넘치면 가장 옛 바이트를 버리고 새 바이트를 넣는다(ESP8266 Arduino 코어 3.1.2 `uart.cpp`는 `UART_DISCARD_NEWEST`를 정의하지 않는다. 12.1-67). 하드웨어 RX FIFO(128 B)가 ISR 전까지 더 담는 몫은 SIM에 없다[가정, 실기 확인]. 1 ms마다 읽으면 115200 baud에서 넘치지 않는다.

### 2.3 토큰과 정수

- 내용 = 몸 [`*` HH]. 몸은 토큰을 공백(0x20) 하나로 나눈 것이다. 앞뒤 공백, 연속 공백은 문법 오류 `syntax`다.
- 첫 토큰이 키워드다. 키워드는 대문자만이다: `INFO PING STOP ARM DISARM C CAL CFG BENCH D0 D1 D2 D3 D4 D5 D6 D7 D8`. 다른 첫 토큰(소문자 `ping`, `D9`, `D10`, 빈 토큰 포함)은 `cmd`다.
- `CAL` 하위 명령(`JOG POL MARK SOFT SAVE ERASE GET`), `MARK`의 인자(`L C R`), `CFG` 키 이름(4.10절)은 표에 적힌 글자 그대로다. 다르면 `syntax`(`CFG`의 모르는 키는 `key`)다.
- 부호 없는 정수: `0` 또는 `[1-9][0-9]{0,8}`. 부호 있는 정수: 부호 없는 정수 또는 `-[1-9][0-9]{0,8}`. `+` 부호, 앞자리 0(`007`), `-0`, 10자리 이상, 소수점은 문법 오류 `syntax`다. 문법은 맞지만 값이 칸의 범위 밖이면 `range`다.
- 16진 토큰(`boot_id` 8자리, CRC 2자리): 0-9, A-F, a-f를 받는다. MCU가 내는 16진은 모두 대문자다. 호스트도 대문자로 보낸다.
- 키워드 뒤 인자 수가 틀리면 `syntax`다.

### 2.4 CRC 필드

- 내용 길이가 3 이상이고 끝에서 셋째 바이트가 `*`이면 CRC 필드가 있다. 그 뒤 두 바이트가 16진 숫자가 아니면 `crc`다. 몸 안의 다른 `*`는 `syntax`다.
- CRC는 `*` 앞의 모든 내용 바이트(몸 전체)로 계산한다(3절). 다르면 `crc`다. `crc` 줄은 나쁜 줄 계수를 1 올린다(6.6절).
- CRC가 꼭 있어야 하는 명령: `ARM`, `DISARM`, `C`, `CAL`, `CFG`, `BENCH`. 없으면 `nocrc`다.
- CRC가 있어도 되고 없어도 되는 명령: `PING`, `INFO`, `D<n>`. 있으면 맞아야 한다.
- `STOP` 특례: 내용이 정확히 `STOP`이거나, 길이 7이고 `STOP*`로 시작하면 CRC 두 자리를 보지 않고 STOP을 실행한다. 정지는 늘 안전하다. 이 줄은 나쁜 줄로 세지 않는다. 그 밖의 `STOP …` 형식(예: `STOP x`)은 일반 규칙을 따른다(`syntax`).

### 2.5 호스트 송신 규칙

- 호스트는 줄마다 정확히 몸 + (필요하면 `*` + CRC 대문자 두 자리) + LF를 보낸다. CR은 보내지 않는다.
- `STOP`, `PING`, `INFO`는 CRC 없이 보낸다(`STOP\n`, `PING\n`, `INFO\n`). `D<n>`도 CRC 없이 보낸다. 나머지는 CRC를 붙인다.
- 정수는 2.3절의 정규형(앞자리 0 없음)으로, `boot_id`는 배너에서 받은 대문자 8자리 그대로 보낸다.
- 그래서 같은 입력이면 호스트 해석기는 늘 같은 바이트를 낸다.

## 3. CRC-8 (B)

| 항목 | 값 |
|---|---|
| 다항식 | 0x07 (x^8 + x^2 + x + 1) |
| 초기값 | 0x00 |
| 입력 반사 | 없음 |
| 출력 반사 | 없음 |
| 끝 XOR | 0x00 |
| 검사값 `123456789` | 0xF4 (CRC-8/SMBUS와 같다) |
| 덮는 바이트 | 줄 첫 바이트부터 `*` 바로 앞까지. `*`, CRC 두 자리, CR, LF는 넣지 않는다 |
| 출력 형식 | 대문자 16진 두 자리. MCU와 호스트 모두 |
| 입력 허용 | 대문자·소문자 모두 |

비트 단위 계산(펌웨어 기준 구현):

```
crc = 0
for each byte x:
    crc ^= x
    repeat 8 times:
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) & 0xFF : (crc << 1) & 0xFF
```

표 방식 첫 16개: `00 07 0E 09 1C 1B 12 15 38 3F 36 31 24 23 2A 2D`.

시험 벡터(계산 출력 그대로):

| 덮는 바이트 | CRC-8 | 완성 줄 |
|---|---|---|
| (빈 문자열) | 00 | — |
| `123456789` | F4 | — |
| `A` | C0 | — |
| `STOP` | 66 | `STOP*66`(CRC 없이 `STOP`도 된다) |
| `PING` | 1F | `PING*1F` |
| `INFO` | 58 | `INFO*58` |
| `DISARM` | 88 | `DISARM*88` |
| `ARM 5F3A19C2` | CC | `ARM 5F3A19C2*CC` |
| `ARM 5f3a19c2` | A7 | `ARM 5f3a19c2*A7`(소문자도 받는다. CRC는 보낸 바이트로 계산한다) |
| `BENCH 5F3A19C2` | 8A | `BENCH 5F3A19C2*8A` |
| `C 1842 250 102 102 -310` | 8E | `C 1842 250 102 102 -310*8E` |
| `C 1 300 0 0 0` | A7 | `C 1 300 0 0 0*A7` |
| `C 2 300 128 128 -250` | E2 | `C 2 300 128 128 -250*E2` |
| `CAL MARK L` | DD | `CAL MARK L*DD` |
| `CAL SAVE` | 0F | `CAL SAVE*0F` |
| `CFG kp 245` | C9 | `CFG kp 245*C9` |
| `S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260` | CB | 설계 예 상태 줄 |
| `N 1842 crc` | 24 | `N 1842 crc*24` |
| `N - long` | 3C | `N - long*3C` |
| `F POT_OPEN 3 123456` | C7 | `F POT_OPEN 3 123456*C7` |
| `W 123456` | CE | `W 123456*CE` |
| `P D8 0 0` | C6 | `P D8 0 0*C6` |

설계 4.1의 예 값은 모두 틀렸다(자리 표시였다). 이 문서의 값으로 바꾼다:

| 설계 예 | 설계 값 | 맞는 값 |
|---|---|---|
| `ARM 5f3a19c2*7d` | 7D | A7 |
| `DISARM*1c` | 1C | 88 |
| `C 1842 250 102 102 -310*4a` | 4A | 8E |
| `S 123456 … 40 260*5C` | 5C | CB |

- CRC는 오류 검출용이다. 인증이 아니다(09-19 설계 6절).
- 9절의 EEPROM 레코드는 CRC-32를 따로 쓴다(9.3절).

## 4. 호스트 → MCU 명령 (C)

### 4.1 요약

| 명령 | 문법(몸) | CRC | DISARMED | ARMED | BENCH | watchdog 갱신 | 성공 응답 |
|---|---|---|---|---|---|---|---|
| `INFO` | `INFO` | 선택 | 받음 | 받음 | 받음 | 아니오 | 배너 + 보정 보고 줄 |
| `PING` | `PING` | 선택 | 받음 | 받음 | 받음 | 예 | 없음 |
| `STOP` | `STOP` | 안 봄(2.4절) | 받음 | 받음 | 받음 | 아니오 | 없음 |
| `ARM` | `ARM <boot>` | 필수 | 받음 | `state` | `state` | 예(시작) | 없음 |
| `DISARM` | `DISARM` | 필수 | 받음(4.7절) | 받음 | 받음 | 아니오 | 없음 |
| `C` | `C <q> <ttl> <L> <R> <S>` | 필수 | `state` | 받음 | `state` | 예 | 없음 |
| `CAL GET` | `CAL GET` | 필수 | 받음 | 받음 | 받음 | 아니오 | 보정 보고 줄 |
| `CAL …`(GET 밖) | 4.9절 | 필수 | 받음 | `state` | `state` | 아니오 | 4.9절 |
| `CFG` 설정 | `CFG <key> <value>` | 필수 | 받음 | `state` | `state` | 아니오 | `A CFG <key> <value>` |
| `CFG` 조회 | `CFG <key>` | 필수 | 받음 | 받음 | 받음 | 아니오 | `A CFG <key> <value>` |
| `BENCH` | `BENCH <boot>` | 필수 | 받음 | `state` | `state` | 예(시작) | 없음 |
| `D<n>` | `D<n> <v>` | 선택 | `state` | `state` | 받음 | 예(4.12절) | `P D<n> <duty> <target>` |

- `<boot>`는 16진 8자리다(대소문자 무관, 수로 비교한다).
- "받음"은 상태 조건을 통과한다는 뜻이다. 다른 검사(4.2절)는 그대로 한다.
- 성공한 `C`, `ARM`, `DISARM`, `BENCH`, `STOP`, `PING`에는 응답 줄이 없다. 적용 결과는 다음 상태 줄(5.3절)에 같은 MCU 시각으로 나온다. 그래서 받음(브리지 송신 기록), 적용(상태 줄), 실제 회전(모름)이 나뉜다.

### 4.2 거절 줄과 우선순위

거절하면 `N <q> <reason>*HH`를 그 줄을 실행한 틱에 낸다(5.5절). `<q>`: 틀·CRC 검사를 통과한 `C` 줄이고 둘째 토큰이 부호 없는 정수 0..65535이면 그 값, 아니면 `-`다. CRC 필드가 없는 줄도 통과한 줄이다. 그래서 `nocrc`로 거절한 `C`도 q를 싣는다(예: `C 5 300 0 0 0` → `N 5 nocrc`. 12.1-53).

| reason | 뜻 |
|---|---|
| `long` | 과길이(2.2절) |
| `ctrl` | 0x20..0x7E 밖 바이트, CR 규칙 위반 |
| `crc` | CRC 필드 형식 틀림 또는 값 불일치 |
| `cmd` | 모르는 키워드 |
| `nocrc` | CRC가 필수인 명령에 CRC 없음 |
| `state` | 지금 상태에서 받지 않는 명령 |
| `syntax` | 토큰 수·공백·정수 문법·하위 명령 이름 오류 |
| `range` | 인자 값이 범위 밖(서로 묶인 `CFG` 값 검사 포함) |
| `boot` | `ARM`·`BENCH`의 boot_id가 이번 부팅 값과 다름 |
| `nocal` | 저장된 유효 보정 레코드 R이 없음 |
| `cal` | 보정 작업본 W가 R과 다름(ARM), 보정 조건 불충분(CAL) |
| `pol` | 보정에 극성 확인 통과 표시가 없음, 또는 POLARITY·DRV_STUCK 래치가 있음(`CAL POL` 통과 전. 12.1-71) |
| `fault` | 지금 켜져 있는 결함 조건이 있음(6.7절 "활성 조건". `ARM`, `CAL JOG`, `CAL POL`. 12.1-68) |
| `old` | `C`의 q가 직전보다 새롭지 않음(4.8절) |
| `busy` | HOLD·JOG·POL이 돌거나 구동 duty가 0이 아님(4.13절) |
| `wear` | 이번 부팅의 EEPROM 쓰기 횟수 한도(8회) 초과 |
| `key` | 모르는 `CFG` 키 |
| `pin` | `D<n>`의 n이 BENCH에서 쓸 수 없는 핀(D2, D4, D6, D7) |

검사 순서(먼저 걸린 것 하나만 낸다):

1. `long`, `ctrl`(틀). 2. `STOP` 특례(2.4절. 해당하면 바로 실행하고 끝). 3. `crc`. 4. `cmd`. 5. `nocrc`. 6. `state`. 7. `syntax`. 8. 명령별 검사(아래 표 순서).

| 명령 | 7 다음 순서 |
|---|---|
| `ARM` | `boot` → `busy`(JOG·POL만) → `nocal` → `cal` → `pol` → `fault` |
| `C` | `range` → `old` |
| `CAL JOG` | `range` → `busy` → `fault` |
| `CAL POL` | `busy` → `cal` → `fault` |
| `CAL MARK` | `busy` → `cal`(지금 f가 보정 창 밖) |
| `CAL SOFT` | `range` → `busy` |
| `CAL SAVE` | `busy` → `cal` → `wear` |
| `CAL ERASE` | `busy` → `wear` |
| `CFG` 설정 | `key` → `range` → `busy` |
| `CFG` 조회 | `key` |
| `BENCH` | `boot` → `busy` |
| `D<n>` | `range` → `pin` |
| `INFO`, `PING`, `DISARM`, `CAL GET` | 없음 |

- 거절된 줄은 상태·출력·watchdog·TTL을 바꾸지 않는다. 예외: 틀 오류는 BENCH에서 구동을 멈춘다(2.2절). `crc`·`long`·`ctrl`은 나쁜 줄 계수를 올린다(6.6절).

### 4.3 `INFO`

- 어느 상태에서나 받는다. 배너 줄(5.2절) 하나와 보정 보고 줄(5.9절) 하나를 이 순서로 낸다.
- watchdog을 갱신하지 않는다. 출력을 바꾸지 않는다.

### 4.4 `PING`

- 어느 상태에서나 받는다. `last_input = now`(링크 watchdog 갱신). 응답이 없다.
- `C`의 TTL을 갱신하지 않는다. 이동을 연장하지 않는다(설계 4.1 원칙 4).
- DISARMED에서도 `last_input`은 바뀐다(상태 줄 `wd`가 0부터 다시 센다). watchdog은 ARMED·BENCH에서만 동작한다.

### 4.5 `STOP`

어느 상태에서나 받는다. 응답이 없다. watchdog을 갱신하지 않는다. 같은 틱에 아래를 한다.

| 지금 상태 | 구동 | 조향 | 다음 상태 |
|---|---|---|---|
| ARMED | 즉시 제동(8.2절 brake) | HOLD(`stoppol`, `hstop` 동안) 뒤 OFF(6.5절) | DISARMED |
| BENCH | 즉시 제동 | OFF 그대로 | DISARMED |
| DISARMED | 즉시 제동(감속 중이던 duty도 0) | JOG·POL이면 OFF. HOLD는 그대로(다시 시작하지도, 늘리지도 않는다) | DISARMED |

- `C` 상태(TTL, TTL 비트)를 지운다. 래치를 풀지 않는다.
- 같은 줄을 여러 번 보내도 된다. 이미 정지면 바뀌는 것이 없다.

### 4.6 `ARM <boot>`

- 조건(4.2절 순서): DISARMED, boot_id 일치, JOG·POL 아님, 저장된 유효 레코드 R 있음, W의 보정 칸이 R과 같음, R의 극성 확인 표시 1이고 POLARITY·DRV_STUCK 래치 없음(12.1-71), 활성 결함 조건 없음(6.7절).
- 받으면 같은 틱에:
  - 상태 ARMED. 조향 모드 ACTIVE. `tgt = cmd = pos`(이 틱의 위치). 서보 내부값은 7.6절의 시작 규칙으로 둔다. 첫 서보 갱신은 이 틱이 서보 틱이면 이 틱 9단계, 아니면 다음 서보 틱이다(7.1절).
  - HOLD 중이었으면 HOLD를 끝내고 ACTIVE로 넘어간다.
  - 구동 목표는 0 그대로다. 감속 중이던 duty는 계속 감속한다. 첫 `C`가 와야 움직인다.
  - TTL 없음(`tl` 0), TTL 비트 0. `last_input = now`.
  - 래치된 결함 비트와 BOOT 비트를 모두 지운다(6.8절. POLARITY·DRV_STUCK이 있으면 ARM은 이미 `pol`로 거절된다). 나쁜 줄 계수를 0으로 한다.
  - `last_q`는 그대로 둔다(4.8절).
- 응답 줄은 없다. 호스트는 다음 상태 줄의 `st` = 2로 확인한다.

### 4.7 `DISARM`

| 지금 상태 | 구동 | 조향 | 다음 상태 |
|---|---|---|---|
| ARMED | release(목표 0, 램프 감속, 8.2절) | HOLD(측정 위치, `hdis` 동안) 뒤 OFF | DISARMED |
| BENCH | release | OFF 그대로 | DISARMED |
| DISARMED | 바꾸지 않음 | JOG·POL이면 OFF. 그 밖에는 바꾸지 않음 | DISARMED |

- DISARMED에서도 거절하지 않는다(정상 종료를 막지 않는다). 응답이 없다. `C` 상태를 지운다.

### 4.8 `C <q> <ttl> <L> <R> <S>`

| 칸 | 문법 | 범위 | 뜻 |
|---|---|---|---|
| q | 부호 없는 정수 | 0..65535 | 명령 번호 |
| ttl | 부호 없는 정수 | 1..`ttlmax` | 유효 시간(ms) |
| L, R | 부호 있는 정수 | −255..255 | 구동 목표 duty. 부호는 방향(+ 전진) |
| S | 부호 있는 정수 | −1000..1000 | 조향 목표(‰, 왼쪽 +) |

q 감김 규칙:

- d = (q − last_q) mod 65536. 1 ≤ d ≤ 32767이면 새 명령이다. d = 0(같음)이나 d ≥ 32768(옛것)이면 `old`로 거절한다.
- `last_q`는 마지막으로 받은(적용한) `C`의 q다. 부팅 때 0이다. ARM·STOP·DISARM은 `last_q`를 바꾸지 않는다. 그래서 이미 받은 q(또는 그보다 옛 q)의 `C`는 재무장 뒤에도 거절된다. 처음 보는 q는 재무장 뒤에 받는다. 직렬선은 바이트 순서를 지키므로 STOP 전에 보낸 `C`는 STOP보다 먼저 도착한다. 호스트는 STOP 뒤에는 새 ARM 확인 전까지 `C`를 보내지 않고, 한 부팅 안에서 q를 다시 쓰지 않는다. STOP 직후 늦게 온 drive 요청은 호스트의 STOP 장벽이 막는다(12.1-59).
- 부팅 뒤 첫 `C`는 q 1..32767이어야 한다. 호스트는 새 boot_id를 볼 때마다 q를 1부터 다시 센다. q는 보낸 `C` 줄마다 1씩 늘린다. 같은 명령을 다시 보낼 때(20 Hz 재전송)도 새 q를 쓴다.
- 예: last_q 65530이면 q 65531..65535, 0..32762를 받는다. last_q 0이면 q 1..32767을 받는다.

받으면 그 틱 t에:

- `last_q = q`. 구동 목표 = (L, R). 조향 `tgt = S`.
- TTL: `c_ms = t`, `c_ttl = ttl`. 이 `C`는 틱 t..t+ttl−1에 유효하다. TTL 비트를 지운다.
- `last_input = t`.
- 응답이 없다. 다음 상태 줄의 `q`, `L`, `R`, `en`, `tgt`가 적용 결과다.

S의 뜻:

- S는 보정 반범위의 천분율이다. +1000은 왼쪽 표시(W의 raw_l), 0은 중앙 표시(raw_c), −1000은 오른쪽 표시(raw_r)다. 사이는 반쪽마다 직선이다(7.3절). 왼쪽이 +다(차체 기준, REP-103).
- MCU는 각도(°, mrad)를 모른다. 호스트가 차량 프로필의 보정표(mrad ↔ ‰)로 바꾼다.
- |S| ≤ 1000이면 받는다. 소프트 한계(`soft_r`..`soft_l`) 밖이면 받되 내부 목표는 한계로 자른다: `cmd`는 clamp(S, soft_r, soft_l)로 간다. 상태 줄 `tgt`는 받은 S 그대로, `cmd`는 자른 값이다(호스트가 포화를 본다).

### 4.9 `CAL …`

`CAL GET` 밖의 하위 명령은 DISARMED에서만 받는다. 모두 CRC가 필수다.

| 하위 명령 | 문법(몸) | 조건 | 동작 | 응답 |
|---|---|---|---|---|
| `JOG` | `CAL JOG <pwm> <ms>` | pwm 부호 있는 −255..255, 0 아님. ms 1..500. busy 아님. 활성 결함 조건 없음(6.7절, 12.1-68) | 조향 모드 JOG. 틱 t..t+ms−1에 조향 EN 1, pwm > 0이면 D6 = \|pwm\|, pwm < 0이면 D7 = \|pwm\|(핀 기준. `mdir`와 소프트 한계를 쓰지 않는다). 틱 t+ms에 OFF | `A JOG <pwm> <ms>` |
| `POL` | `CAL POL` | busy 아님. W의 raw_l, raw_c가 있고 \|raw_l − raw_c\| ≥ 50. 활성 결함 조건 없음(12.1-68) | 7.10절 극성 확인 절차(1000 ms). 통과하면 POLARITY·DRV_STUCK 래치를 지운다(12.1-71) | 끝날 때 `A POL <ok\|fail> <d1> <d2>`와 보정 보고 줄 |
| `MARK` | `CAL MARK L`, `CAL MARK C`, `CAL MARK R` | busy 아님. 이 틱의 f가 [`cwmin`, `cwmax`] 안 | W의 raw_l / raw_c / raw_r = f | 보정 보고 줄 |
| `SOFT` | `CAL SOFT <l> <r>` | l 부호 없는 1..1000. r 부호 있는 −1000..−1. busy 아님 | W의 soft_l = l, soft_r = r | 보정 보고 줄 |
| `SAVE` | `CAL SAVE` | busy 아님. W가 9.4절 검사 통과. 쓰기 한도 안 | R ← W, `cal_id` 증가(9.5절), EEPROM commit | 보정 보고 줄 |
| `ERASE` | `CAL ERASE` | busy 아님. 쓰기 한도 안 | R을 지움 표시로 바꾼다(9.5절). W의 보정 칸을 비운다(표시 없음, soft 0, pol 0, mdir +1). W의 변수는 둔다. NOCAL 비트를 켠다. commit이 실패하면(실기만) R, W, NOCAL 비트를 그대로 둔다(12.1-73) | `F NOCAL 0 <ms>` 다음 보정 보고 줄. commit 실패면 보정 보고 줄만(id는 R의 cal_id 그대로) |
| `GET` | `CAL GET` | 어느 상태나 | 없음 | 보정 보고 줄 |

- JOG·POL은 `STOP`, `DISARM`, 결함 정지(6.7절)로 바로 끝나고 OFF가 된다.
- JOG와 POL 동안에는 조향 결함 감시 가운데 THERMAL과 가변저항 감시만 돈다(6.7절). THERMAL 래치가 이미 있고 그 조건(창 안 누적 > `tha`)이 다시 참이 되면 그 틱에 JOG·POL을 끈다(F 줄은 다시 내지 않는다. 12.1-68).
- 위치 계산(`pos`)은 W를 쓴다. 그래서 `MARK` 직후 상태 줄의 `pos`가 새 표시로 바뀐다(조향 출력이 꺼진 DISARMED에서만 생긴다).

### 4.10 `CFG <key> [<value>]`

- 설정(`CFG <key> <value>`): DISARMED, busy 아님. value는 부호 없는 정수다. 범위는 아래 표다. 묶인 검사는 새 값과 W의 다른 값으로 한다. 받으면 W 값을 바로 바꾼다. 그 뒤 처음 쓰는 곳부터 적용된다. 응답은 `A CFG <key> <value>`다.
- 조회(`CFG <key>`): 어느 상태나. 응답은 `A CFG <key> <W의 값>`이다.
- 바뀐 값은 RAM에만 있다. `CAL SAVE`가 레코드에 저장한다. 리셋하면 R(없으면 기본값)으로 돌아간다.
- `med`를 바꾸면 중앙값 버퍼를 이 틱의 raw로 다시 채운다. `sthz`는 다음 상태 줄 뒤 주기부터 적용된다.
- `svhz` 변경은 이미 잡힌 `next_servo`를 바꾸지 않는다. 그 서보 틱의 `next_servo += P_sv`부터 새 P_sv를 쓴다(7.1절). 예(부팅 틱 0): 500 → 1000을 틱 101에 받으면 102가 서보 틱이고 그 뒤 매 틱이다. 1000 → 500을 틱 101에 받으면 101이 서보 틱이고(4단계가 9단계보다 먼저다) 그 뒤 103, 105, …다(대본 `cfg_svhz_runtime`, `cfg_svhz_runtime_1000_to_500`).
- 기본값은 모두 [자리값]이다. ‰ 단위 값은 자리값 기준 눈금(1° ≈ 40 ‰)으로 바꿨다. SIM이 EEPROM 이미지를 만들 때는 그 run의 보정 표시가 주는 눈금 s(‰/°)를 쓴다(`LOCKSTEP_PROTOCOL_KO.md` 11.11절).

| 순번 | key | 프로필 이름 | 단위 | 기본 | 범위 | 바꾸는 식 |
|---|---|---|---|---|---|---|
| 1 | `kp` | `servo_kp_pct_per_deg` | PWM/‰ × 256 | 245 | 1..4096 | round(kp_pct / 100 × 255 / s × 256) |
| 2 | `db` | `servo_deadband_deg` | ‰ | 20 | 0..200 | round(deg × s) |
| 3 | `minp` | `servo_min_pwm_pct` | PWM | 102 | 0..255, ≤ `maxp` | round(pct × 255 / 100) |
| 4 | `maxp` | `servo_max_pwm_pct` | PWM | 255 | 1..255, ≥ `minp` | 같음 |
| 5 | `dwell` | `servo_dwell_brake_s` | ms | 200 | 0..2000 | s × 1000 |
| 6 | `rev` | `servo_reversal_pause_ms` | ms | 20 | 0..500 | 그대로 |
| 7 | `rmov` | 0.8 × `steer_rate_max_deg_s` | ‰/s | 960 | 1..5000, ≥ `rstd` | round(0.8 × deg/s × s) |
| 8 | `rstd` | 0.8 × `steer_rate_standstill_deg_s` | ‰/s | 480 | 1..5000, ≤ `rmov` | 같음 |
| 9 | `ttlmax` | `c_ttl_max_ms` | ms | 300 | 50..1000 | 그대로 |
| 10 | `wd` | `watchdog_ms` | ms | 500 | 100..5000 | 그대로 |
| 11 | `ttlmode` | `ttl_expiry_drive_mode` | 0 brake, 1 release | 0 | 0..1 | |
| 12 | `wdmode` | `steer_watchdog_mode` | 0 hold, 1 center, 2 free | 0 | 0..2 | |
| 13 | `hloss` | `steer_hold_after_loss_s` | ms | 2000 | 0..10000 | s × 1000 |
| 14 | `hstop` | `steer_hold_after_stop_s` | ms | 2000 | 0..10000 | 같음 |
| 15 | `hdis` | `steer_hold_after_disarm_s` | ms | 2000 | 0..10000 | 같음 |
| 16 | `stoppol` | `stop_policy_steer` | 0 hold_measured, 1 hold_target, 2 center | 0 | 0..2 | |
| 17 | `sthz` | `status_hz` | Hz | 20 | 10, 20, 25, 40, 50 | 그대로(1000을 나누는 값만) |
| 18 | `med` | `pot_filter.median_of` | 표본 | 5 | 1, 3, 5, 7, 9 | 그대로 |
| 19 | `pvmin` | `pot_valid_raw.min_raw` | count | 20 | 0..1023, < `pvmax` | |
| 20 | `pvmax` | `pot_valid_raw.max_raw` | count | 1003 | 0..1023, > `pvmin` | |
| 21 | `cwmin` | `cal_end_raw_window.min_raw` | count | 60 | 0..1023, < `cwmax` | |
| 22 | `cwmax` | `cal_end_raw_window.max_raw` | count | 963 | 0..1023, > `cwmin` | |
| 23 | `jmp` | `pot_jump_counts.counts` | count | 8 | 1..1023 | |
| 24 | `jmpw` | `pot_jump_counts.window_ms` | ms | 2 | 1..100 | |
| 25 | `nsd` | `pot_noise_counts.sd_counts` | count × 10 | 50 | 1..1000 | round(sd × 10) |
| 26 | `nsw` | `pot_noise_counts.window_ms` | ms(= 표본 수) | 50 | 2..100 | |
| 27 | `polp` | `polarity_fault.away_deg` | ‰ | 40 | 1..1000 | round(deg × s) |
| 28 | `polt` | `polarity_fault.duration_s` | ms | 100 | 1..1000 | s × 1000 |
| 29 | `stp` | `stall_detect.pwm_pct` | PWM | 153 | 0..255 | round(pct × 255 / 100) |
| 30 | `stt` | `stall_detect.duration_s` | ms | 500 | 1..1000 | s × 1000 |
| 31 | `stm` | `stall_detect.min_motion_deg` | ‰ | 20 | 0..1000 | round(deg × s) |
| 32 | `trk` | `track_fault.error_deg` | ‰ | 120 | 1..2000 | round(deg × s) |
| 33 | `trt` | `track_fault.duration_s` | ms | 1000 | 1..10000 | s × 1000 |
| 34 | `dsm` | `driver_stuck_fault.motion_deg` | ‰ | 80 | 1..1000 | round(deg × s) |
| 35 | `dst` | `driver_stuck_fault.duration_s` | ms | 300 | 1..1000 | s × 1000 |
| 36 | `thp` | `thermal_proxy.pwm_pct` | PWM | 204 | 0..255 | round(pct × 255 / 100) |
| 37 | `thw` | `thermal_proxy.window_s` | s | 30 | 1..60 | 그대로 |
| 38 | `tha` | `thermal_proxy.accumulated_s` | ms | 5000 | 1..60000, ≤ `thw` × 1000 | s × 1000 |
| 39 | `crcn` | `crc_burst_n` | 줄 | 5 | 1..50 | 그대로 |
| 40 | `movms` | (설계 4.4 "최근 0.3 s") | ms | 300 | 0..2000 | 그대로 |
| 41 | `svhz` | `servo_hz` | Hz | 500 | 500, 1000 | 그대로 |

- round는 0.5에서 위로 올린다(값이 모두 0 이상이다).
- 순번은 9.2절 레코드 안의 저장 순서다. 41개 모두 저장한다.
- 프로필의 `status_latency_s`, `serial_delay_ms`는 MCU 변수가 아니다(전송 모형 값이다). `baud`는 컴파일 상수다(10.6절). `pot_filter.sample_hz`는 1000으로 고정이다(틱마다 표본 하나).

### 4.11 `BENCH <boot>`

- 조건: DISARMED, boot_id 일치, busy 아님(HOLD·JOG·POL 아님, 구동 duty L = R = 0).
- 받으면: 상태 BENCH, 구동 목표 0, 조향 OFF, `last_input = now`. 래치와 BOOT 비트는 지우지 않는다. 래치된 결함이 있어도 들어간다(조향을 쓰지 않는 정비 모드다).
- BENCH에서 받는 명령: `D<n>`, `PING`, `INFO`, `STOP`, `DISARM`, `CAL GET`, `CFG` 조회. 그 밖은 `state`다.
- BENCH를 나가는 길: `STOP`(제동), `DISARM`(release), watchdog(6.2절), 나쁜 줄 연속(6.6절), 새로 켜진 결함(6.7절). 모두 DISARMED로 간다. ARMED로 바로 가지 않는다.

### 4.12 `D<n> <v>` (BENCH 전용, V2 핀 명령)

- v는 부호 없는 정수 0..255다. CRC는 선택이다.

| n | 핀 | 동작(V2 `requestDuty`와 같다) | 응답 |
|---|---|---|---|
| 8 | D8 왼쪽 전진 | v > 0: 왼쪽 목표 = +v. v = 0: 왼쪽 목표가 + 쪽이면 0, 아니면 그대로 | `P D8 <duty> <target>` |
| 5 | D5 왼쪽 후진 | v > 0: 왼쪽 목표 = −v. v = 0: 왼쪽 목표가 − 쪽이면 0, 아니면 그대로 | `P D5 …` |
| 3 | D3 오른쪽 전진 | 오른쪽, + 쪽 | `P D3 …` |
| 0 | D0 오른쪽 후진 | 오른쪽, − 쪽 | `P D0 …` |
| 1 | D1 구동 EN | 바꾸지 않는다(늘 HIGH, V2 "power locked"와 같다) | `P D1 255 255` |
| 2, 4, 6, 7 | 조향 EN, 미사용, 조향 PWM | 거절 | `N - pin` |

- 응답의 duty, target은 그 핀의 크기다: 그 쪽 적용 duty(또는 목표)의 부호가 그 핀 방향이면 절댓값, 아니면 0. 값은 이 명령을 반영한 직후, 이 틱의 램프 전이다.
- 받은 `D<n>`(n = 8, 5, 3, 0, 1)은 `last_input = now`로 watchdog을 갱신한다. 거절된 줄은 갱신하지 않는다.
- 명령 자체는 GPIO를 쓰지 않는다. 핀 쓰기는 램프 단계에서 값이 바뀔 때만 한다(8.4절). 그래서 `D8 0`을 되풀이하면 응답은 매번 나오지만 GPIO 쓰기는 없다(V2 22번 시험).
- BENCH의 구동 램프·휴지·즉시 정지는 ARMED와 같은 8.2절 규칙이다.

### 4.13 busy 조건

| 이름 | 뜻 |
|---|---|
| 조향 busy | 조향 모드가 HOLD, JOG, POL 가운데 하나 |
| 구동 busy | 적용 duty L 또는 R이 0이 아님 |

| 명령 | `busy`로 거절하는 때 |
|---|---|
| `ARM` | 조향 모드 JOG 또는 POL(HOLD는 받는다) |
| `BENCH` | 조향 busy 또는 구동 busy |
| `CAL`(GET 밖) | 조향 busy 또는 구동 busy |
| `CFG` 설정 | 조향 busy |

## 5. MCU → 호스트 줄 (D)

### 5.1 줄 종류

모든 MCU 줄은 `*HH`(CRC-8, 대문자)와 LF로 끝난다. CR을 내지 않는다. 호스트는 CRC가 틀린 줄을 버리고 센다.

| 첫 토큰 | 둘째 토큰 | 이름 | 나오는 때 | 최대 길이(LF 포함) |
|---|---|---|---|---|
| `UBTS_STEER_V3` | `hello` | 배너 | 부팅(`setup()`), `INFO` | 174 B |
| `UBTS_STEER_V3` | `cal` | 보정 보고 | 부팅(배너 바로 뒤), `INFO`, `CAL` 성공, `CAL POL` 끝 | 129 B |
| `S` | — | 상태 | 주기(5.4절) | 85 B |
| `N` | — | 거절 | 거절한 줄마다 | 18 B |
| `F` | — | 결함 | 결함 비트가 새로 켜질 때, TTL 만료, `CAL ERASE` | 35 B |
| `W` | — | watchdog | 링크 watchdog 동작 때 | 16 B |
| `A` | — | 확인 | `CFG`, `CAL JOG`, `CAL POL` 끝 | 26 B |
| `P` | — | 핀 확인 | BENCH에서 받은 `D<n>`마다 | 16 B |

- 펌웨어는 위 밖의 줄(디버그 문구 포함)을 내지 않는다.
- 호스트는 모르는 첫 토큰의 줄을 버리고 센다(앞으로의 확장 대비).

### 5.2 배너

형식(칸 순서 고정, 칸 사이 공백 하나):

```
UBTS_STEER_V3 hello proto=3 build=<B> dev=<D> boot=<H> rst=<reset> hw=<hw> out=drive2,steer1 cal=<id> sim=<s> wd=<wd> ttl=<ttl> st=<st> drop=<n>*HH
```

| 칸 | 값 |
|---|---|
| `proto` | 3 |
| `build` | 펌웨어 소스 식별 대문자 16진 8자리(10.5절) |
| `dev` | 칩 ID 대문자 16진 6자리(`ESP.getChipId()`, 24비트) |
| `boot` | 이번 부팅의 boot_id 대문자 16진 8자리(10.1절) |
| `rst` | 리셋 원인 이름(10.3절) |
| `hw` | 하드웨어 프로필 `front_steer_v0`(컴파일 상수) |
| `out` | `drive2,steer1`(고정) |
| `cal` | 저장 레코드 R의 `cal_id`(10진) 또는 `none` |
| `sim` | SIM 빌드 1, 실기 빌드 0(10.4절) |
| `wd`, `ttl`, `st` | W의 `wd`(ms), `ttlmax`(ms), `sthz`(Hz) |
| `drop` | 부팅 뒤 TX 고리가 차서 버린 줄 수(5.10절), 65535에서 멈춤 |

- 부팅 틱 b의 `setup()`이 배너와 보정 보고 줄을 이 순서로 낸다. `INFO`도 같은 두 줄을 낸다.
- 예(SIM, seed 1의 첫 부팅): `UBTS_STEER_V3 hello proto=3 build=0A1B2C3D dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0*01`. build 값은 예다.

### 5.3 상태 줄

형식: `S <ms> <n> <st> <q> <L> <R> <en> <raw> <pos> <tgt> <cmd> <sp> <fl> <wd> <tl>*HH`

| 순 | 칸 | 뜻 | 범위 | 최대 자리 |
|---|---|---|---|---|
| 1 | ms | 이 줄을 만든 틱의 `millis()` | 0..4294967295, 2^32에서 0으로 감김 | 10 |
| 2 | n | 상태 줄 번호. 부팅 뒤 첫 줄 0. 만든 줄마다 1 증가(버린 줄 포함) | 0..65535, 감김 | 5 |
| 3 | st | 상태: 1 DISARMED, 2 ARMED, 3 BENCH. 0(BOOT)은 줄에 나오지 않는다 | 1..3 | 1 |
| 4 | q | `last_q`(4.8절). 부팅 뒤 `C`를 받기 전에는 0 | 0..65535 | 5 |
| 5 | L | 왼쪽 적용 duty(부호 있음) | −255..255 | 4 |
| 6 | R | 오른쪽 적용 duty | −255..255 | 4 |
| 7 | en | 비트0 구동 EN 핀(D1), 비트1 조향 EN 핀(D2). 10진 한 자리 | 0..3 | 1 |
| 8 | raw | 거른 raw f | 0..1023 | 4 |
| 9 | pos | 위치(‰). W 표시가 불완전하면 0(7.3절) | −9999..9999 | 5 |
| 10 | tgt | 서보 목표. 서보가 돌지 않으면(OFF, JOG, POL) pos와 같다 | −9999..9999 | 5 |
| 11 | cmd | 내부 목표. 서보가 돌지 않으면 pos와 같다 | −9999..9999 | 5 |
| 12 | sp | 이 틱의 실제 조향 핀 값 D6 − D7. ACTIVE·HOLD에서는 u × mdir와 같다. OFF·BRAKE면 0 | −255..255 | 4 |
| 13 | fl | 결함 마스크, 대문자 16진 4자리(앞 0 포함) | 0000..7FFF | 4 |
| 14 | wd | min(now − `last_input`, 9999) ms | 0..9999 | 4 |
| 15 | tl | ARMED이고 `C`가 유효하면 `c_ttl − (now − c_ms)`, 아니면 0 | 0..1000 | 4 |

- 10진 칸은 앞 0·공백 채움 없이 쓴다. 음수는 `-` 하나를 붙인다. fl만 4자리 고정이다.
- 값은 그 틱의 `loop()` 10단계(7.1절) 시점 값이다. 곧 그 틱의 램프와 서보 갱신을 반영한 값이다.
- 최악 길이: `S 4294967295 65535 3 65535 -255 -255 3 1023 -9999 -1000 -1000 -255 FFFF 9999 1000*HH` = 84 B + LF = 85 B. V2 대응표 21번의 96 B 이하를 지킨다.
- 예: `S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*AA`(13.11절 예의 틱 300 줄).

`fl` 비트(lockstep 규약 v1 9절, `backend_api.FAULT_BIT_NAMES`와 같다):

| 비트 | 값 | 이름 | 종류 | 켜는 것 | 끄는 것 |
|---|---|---|---|---|---|
| 0 | 0x0001 | NOCAL | 래치 | 부팅 때 R 없음, `CAL ERASE` | 받은 `ARM` |
| 1 | 0x0002 | POT_OPEN | 래치 | 6.7절 | 받은 `ARM` |
| 2 | 0x0004 | POT_SHORT | 래치 | 6.7절 | 받은 `ARM` |
| 3 | 0x0008 | POT_JUMP | 래치 | 6.7절 | 받은 `ARM` |
| 4 | 0x0010 | POT_NOISE | 래치 | 6.7절 | 받은 `ARM` |
| 5 | 0x0020 | POLARITY | 래치 | 6.7절 | 받은 `ARM` |
| 6 | 0x0040 | DRV_STUCK | 래치 | 6.7절 | 받은 `ARM` |
| 7 | 0x0080 | SOFTLIMIT | 조건(경고) | 매 틱: W 보정이 완전하고 pos > soft_l 또는 pos < soft_r | 조건이 거짓이 될 때 |
| 8 | 0x0100 | STALL | 래치 | 6.7절 | 받은 `ARM` |
| 9 | 0x0200 | TRACK | 래치 | 6.7절 | 받은 `ARM` |
| 10 | 0x0400 | THERMAL | 래치 | 6.7절 | 받은 `ARM` |
| 11 | 0x0800 | TTL | 조건 | ARMED에서 TTL 만료(6.3절) | 다음에 받은 `C`, ARMED를 떠날 때 |
| 12 | 0x1000 | WATCHDOG | 래치 | 6.2절 | 받은 `ARM` |
| 13 | 0x2000 | CRC | 래치 | 6.6절 | 받은 `ARM` |
| 14 | 0x4000 | BOOT | 정보 | 부팅 | 받은 `ARM` |
| 15 | 0x8000 | 예약 | — | 늘 0 | — |

### 5.4 상태 줄 주기와 위상

- 주기 P_st = 1000 / `sthz` ms(`sthz` 20이면 50 ms). 정수다(4.10절 범위).
- 부팅 틱 b에서 `next_status = b + P_st`(ms 값으로)를 둔다. 틱마다 10단계에서 (int32)(now − next_status) ≥ 0이면 줄을 만든다. 그다음 `next_status += P_st`. 그래도 (int32)(now − next_status) ≥ 0이면(루프가 멈췄다 돌아온 경우) `next_status = now + P_st`로 한다. 밀린 줄을 몰아서 내지 않는다.
- 그래서 SIM(틱마다 `loop()` 한 번)에서 상태 줄은 정확히 틱 b + k·P_st(k = 1, 2, …)에 만들어진다. 위상은 부팅 틱 b에 묶여 있고 결정적이다.
- 모든 상태(DISARMED, ARMED, BENCH)에서 낸다.
- `sthz` 변경은 다음 상태 줄을 만든 뒤의 `next_status` 계산부터 쓴다.
- UART가 바쁠 때(TX 고리에 줄 전체가 들어갈 자리가 없을 때): 그 상태 줄은 버린다. `n`은 그래도 1 오르고 `drop`이 1 오른다. 호스트는 `n`의 빈칸으로 안다. 줄을 반쪽만 내지 않는다(5.10절).

### 5.5 거절 줄 `N`

- 형식: `N <q> <reason>*HH`. q와 reason은 4.2절이다.
- 거절한 줄을 실행한 틱(그 줄의 LF를 읽은 틱)에 낸다. 한 줄에 `N` 하나다.
- 예: `N 1842 crc*24`는 형식만의 예다(4.2절에 따르면 `crc` 줄의 q는 `-`다). 실제 `crc` 거절은 `N - crc*D8`다. 옛 q 거절 예: `N 1842 old*4A`.

### 5.6 결함 줄 `F`

- 형식: `F <CODE> <arg> <ms>*HH`. CODE는 아래 표, arg는 부호 있는 10진, ms는 판정 틱의 `millis()`다.
- 래치 비트는 0에서 1로 바뀔 때 한 번 낸다. 이미 켜져 있는 비트는 다시 내지 않는다. 이 규칙은 결함 판정 줄에 쓴다. 받은 `CAL ERASE`의 `F NOCAL 0 <ms>`는 그 명령의 응답이라 NOCAL 비트가 이미 켜져 있어도 낸다(12.1-54).
- 한 틱에 여럿이면 비트 번호 오름차순으로 낸다.
- SOFTLIMIT, BOOT, WATCHDOG에는 `F` 줄이 없다(WATCHDOG은 `W` 줄을 낸다).

| CODE | arg |
|---|---|
| `NOCAL` | 0(`CAL ERASE`). 부팅 때 R이 없을 때는 `F`를 내지 않는다(배너 `cal=none`과 fl로 안다) |
| `POT_OPEN`, `POT_SHORT` | 판정 틱의 f |
| `POT_JUMP` | f[n] − f[n − `jmpw`] |
| `POT_NOISE` | floor(100 × 분산), 분산 = (W·Σx² − (Σx)²) / W², 창 W = `nsw` 표본의 raw |
| `POLARITY` | P[n] − P[n − `polt`](‰) |
| `DRV_STUCK` | P[n] − P[n − `dst`] |
| `STALL` | P[n] − P[n − `stt`] |
| `TRACK` | cmd − P[n] |
| `THERMAL` | 창 안 누적 ms |
| `TTL` | 부팅 뒤 TTL 만료 횟수(u16, 65535 뒤 0으로 감김). 첫 만료가 1 |
| `CRC` | 나쁜 줄 연속 수(= `crcn`) |

- 예: `F POT_OPEN 3 123456*C7`, `F TTL 1 320*6B`.

### 5.7 watchdog 줄 `W`

- 형식: `W <ms>*HH`. ms는 watchdog이 동작한 틱의 `millis()`다. 예: `W 521*78`.
- ARMED·BENCH에서 watchdog이 동작할 때 한 번 낸다(6.2절).

### 5.8 확인 줄 `A`, 핀 확인 줄 `P`

| 줄 | 형식 | 때 |
|---|---|---|
| `A CFG` | `A CFG <key> <value>*HH`(value는 W의 값) | `CFG` 설정·조회 성공 |
| `A JOG` | `A JOG <pwm> <ms>*HH`(받은 값 그대로) | `CAL JOG` 받은 틱 |
| `A POL` | `A POL <ok\|fail> <d1> <d2>*HH`(7.10절의 Δ1, Δ2, raw count, 부호 있음) | `CAL POL` 끝 틱. 바로 뒤에 보정 보고 줄 |
| `P` | `P D<n> <duty> <target>*HH`(4.12절, 0..255) | BENCH에서 받은 `D<n>` |

- 예: `A CFG kp 245*58`, `P D8 0 0*C6`, `P D1 255 255*EB`.

### 5.9 보정 보고 줄

형식(칸 순서 고정):

```
UBTS_STEER_V3 cal id=<id> crc=<C> left=<l> center=<c> right=<r> soft=<sl>,<sr> pol=<p> mdir=<m> caldirty=<x> cfgdirty=<y>*HH
```

| 칸 | 값 |
|---|---|
| `id` | R의 `cal_id`(10진) 또는 `none` |
| `crc` | R의 CRC-32(대문자 8자리, 9.3절). R이 없으면 `00000000` |
| `left`, `center`, `right` | W의 raw_l, raw_c, raw_r(10진). 없으면 `-` |
| `soft` | W의 soft_l, soft_r(10진). 없으면 `0,0` |
| `pol` | W의 극성 확인 표시: `ok` 또는 `no` |
| `mdir` | W의 모터 방향: `1` 또는 `-1` |
| `caldirty` | W의 보정 칸(표시 3개, soft 2개, pol, mdir)이 R과 다르면 1. R이 없으면 보정 칸에 하나라도 값이 있으면 1 |
| `cfgdirty` | W의 변수 41개가 R과 다르면 1. R이 없으면 컴파일 기본값과 다르면 1 |

- 예(9.6절 레코드를 읽은 직후): `UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 caldirty=0 cfgdirty=0*D6`.
- 빈 EEPROM: `UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no mdir=1 caldirty=0 cfgdirty=0*3F`.

### 5.10 송신 순서와 TX 고리

- 펌웨어는 512 B TX 고리를 둔다. 줄은 통째로만 넣는다. 자리가 모자라면 그 줄을 버리고 `drop`을 1 올린다(65535에서 멈춤). 어느 종류의 줄이든 같다.
- 한 틱 안의 줄 순서는 실행 순서다: watchdog `W`(3단계) → 명령 응답(4단계, 받은 바이트 순서) → `F TTL`(5단계) → 결함 `F`(6단계, 비트 순) → `A POL`과 보정 보고(7단계) → 상태 줄(10단계). 부팅 틱에는 `setup()`의 배너와 보정 보고 줄이 맨 앞이다.
- 12단계에서 고리의 바이트를 UART 하드웨어 송신 FIFO로 옮긴다. 한 번 쓰는 양은 `Serial.availableForWrite()` − 1 이하이고, 그 값이 1 이하이면 옮기지 않는다(12.1-66). ESP8266 코어 3.1.2의 `availableForWrite()`는 128 − FIFO에 찬 바이트 수이고 `Serial.write`는 찬 수가 127 이상이면 빈자리를 기다린다(`uart.cpp`의 `uart_tx_free`, `uart_tx_fifo_full`). 그래서 이 규칙이면 `Serial.write`가 기다리는 일이 없다(11.3절). `availableForWrite()`만큼 다 쓰면 마지막 바이트마다 한 바이트 시간(86.8 µs)을 기다려 512 B 고리에서 약 33 ms 동안 `loop()`가 멈췄다(2026-11-09 검토). 실기에서는 `loop()`가 같은 밀리초 안에 여러 번 불리고 그때마다 옮긴다.

## 6. 상태 기계 (E)

### 6.1 상태와 전이

| 상태 | 값(`st`) | 뜻 |
|---|---|---|
| BOOT | 0 | 리셋 순간부터 `setup()` 전까지. 펌웨어가 돌지 않는다. 핀이 떠 있다(구동 EN 풀다운 → coast, 조향 EN 풀다운 → OFF). 줄을 내지 않는다 |
| DISARMED | 1 | 구동 목표 0. `C` 거절. 조향은 OFF 또는 HOLD·JOG·POL |
| ARMED | 2 | `C`로 구동·조향. 조향 모드 ACTIVE |
| BENCH | 3 | 정비 모드. `D<n>`로 구동 핀만. 조향 OFF |

| 전 | 사건 | 뒤 |
|---|---|---|
| (전원·리셋) | 전원 켬, 리셋, 포트 열기 리셋 | BOOT |
| BOOT | 부팅 틱 b의 `setup()` | DISARMED |
| DISARMED | 받은 `ARM` | ARMED |
| DISARMED | 받은 `BENCH` | BENCH |
| ARMED | `STOP`, `DISARM`, watchdog, 나쁜 줄 연속, 새 래치 결함 | DISARMED |
| BENCH | `STOP`, `DISARM`, watchdog, 나쁜 줄 연속, 새 래치 결함 | DISARMED |
| ARMED | TTL 만료 | ARMED(구동만 섬) |
| 아무 상태 | MCU 리셋·전원 끊김 | BOOT |

- ARMED와 BENCH 사이를 바로 오가지 않는다. 늘 DISARMED를 거친다.
- 자동 재무장·자동 재출발은 없다. 움직이려면 운영자 `ARM`과 새 `C`가 있어야 한다.

### 6.2 링크 watchdog

- `last_input`을 갱신하는 것: `PING`(모든 상태), 받은 `C`(ARMED), 받은 `D<n>`(n = 8, 5, 3, 0, 1, BENCH), 받은 `ARM`과 `BENCH`(들어가는 틱). 부팅 틱 b에 `last_input = b`.
- 갱신하지 않는 것: `INFO`, `STOP`, `DISARM`, `CAL`, `CFG`, 거절된 줄, 틀 오류 줄, 빈 줄.
- 동작하는 상태: ARMED, BENCH. 움직임과 상관없이 동작한다(V2는 출력이 있을 때만 동작했다).
- 판정: 틱 n의 3단계(수신 처리 전, 7.1절)에서 (now − `last_input`) > `wd`이면 동작한다. 그래서 그 틱에 도착한 늦은 `PING`이 정지를 막지 못한다(V2 7번 시험과 같다). 마지막 입력이 틱 t이면 동작 틱은 t + `wd` + 1이다.
- 동작(ARMED): 구동 즉시 제동. 조향은 `wdmode`에 따른다: 0 hold면 측정 위치 HOLD를 `hloss` 동안 한 뒤 OFF, 1 center면 목표 0으로 HOLD를 `hloss` 동안 한 뒤 OFF, 2 free면 바로 OFF. 상태 DISARMED. WATCHDOG 래치. `W` 줄. `C` 상태를 지운다.
- 동작(BENCH): 구동 즉시 제동. 조향 OFF 그대로. DISARMED. WATCHDOG 래치. `W` 줄.
- 같은 틱에 그 뒤 들어온 줄은 DISARMED에서 처리된다(`C`는 `state`로 거절된다).

### 6.3 `C` TTL

- 틱 t에 받은 `C`(ttl)는 틱 t..t+ttl−1에 유효하다.
- 판정: 틱 n의 5단계(수신 처리 뒤)에서 ARMED이고 유효한 `C`가 있고 (now − `c_ms`) ≥ `c_ttl`이면 만료다. 같은 틱 4단계에서 새 `C`를 받았으면 `c_ms`가 바뀌어 만료되지 않는다.
- 만료 동작: 구동은 `ttlmode`(0이면 즉시 제동, 1이면 release). 조향은 마지막 목표 tgt를 그대로 유지한다(서보 계속). 상태는 ARMED 그대로. TTL 비트를 켠다. TTL 횟수를 1 올린다. `F TTL <횟수> <ms>`를 낸다. `tl` = 0.
- 다음에 받은 `C`가 TTL 비트를 지우고 다시 움직인다. 구동은 8.2절의 0 뒤 휴지(120 ms)를 지킨다.
- `PING`은 TTL을 늘리지 않는다. 한 `C`의 TTL은 늘어나지 않는다. 늘리려면 새 q의 `C`를 보낸다.
- 시간: 받은 틱 t, ttl이면 구동 0은 틱 t + ttl(A17의 ttl + 1틱 이내).

### 6.4 정지 모드 표

핀 수준의 뜻은 8.3절이다: brake = 방향 핀 4개를 그 틱에 0, 구동 EN(D1) HIGH 유지. release = 목표 0, 적용 duty가 255/s로 줄어든다, D1 HIGH. coast = D1 LOW(펌웨어는 만들지 않는다. 리셋·전원 끊김 때만).

| 사건 | 구동 | 조향 | 상태 | 비트·래치 | 줄 |
|---|---|---|---|---|---|
| `STOP`(ARMED) | brake | HOLD(`stoppol`) `hstop` 동안 → OFF | DISARMED | 없음 | 없음 |
| `STOP`(BENCH, DISARMED) | brake | OFF 유지. JOG·POL이면 OFF. HOLD는 그대로 | DISARMED | 없음 | 없음 |
| `DISARM`(ARMED) | release | HOLD(측정 위치) `hdis` 동안 → OFF | DISARMED | 없음 | 없음 |
| `DISARM`(BENCH) | release | OFF 유지 | DISARMED | 없음 | 없음 |
| `C`의 L = R = 0(키 놓기·실행기 감속) | release | 목표 S 유지 | ARMED | 없음 | 없음 |
| `C` TTL 만료 | `ttlmode`(기본 brake) | 마지막 목표 유지 | ARMED | TTL(조건) | `F TTL` |
| 링크 watchdog | brake | `wdmode`(기본 hold: 측정 위치 `hloss` → OFF) | DISARMED | WATCHDOG 래치 | `W` |
| 나쁜 줄 연속(`crcn`) | brake | ACTIVE였으면 HOLD(측정 위치) `hloss` → OFF. HOLD면 그대로. JOG·POL이면 OFF | DISARMED | CRC 래치 | `F CRC` |
| POT_OPEN, POT_SHORT, POT_JUMP, POT_NOISE, POLARITY | brake | 즉시 OFF(측정값을 믿지 않는다) | DISARMED | 래치(POLARITY는 `CAL POL` 통과 전 ARM `pol`, 12.1-71) | `F` |
| DRV_STUCK | brake | 즉시 OFF(조향 EN 0이 드라이버 고착을 끊는다) | DISARMED | 래치(`CAL POL` 통과 전 ARM `pol`, 12.1-71) | `F` |
| STALL, TRACK | brake | 즉시 OFF | DISARMED | 래치 | `F` |
| THERMAL | release | 즉시 OFF. 래치가 있고 조건이 다시 참이면 JOG·POL을 그 틱에 OFF(12.1-68) | DISARMED | 래치. 조건이 참인 동안 ARM·`CAL JOG`·`CAL POL`은 `fault` | `F`(처음 켜질 때만) |
| SOFTLIMIT | 바꾸지 않음 | 바깥쪽 출력만 막음(7.8절) | 그대로 | SOFTLIMIT(조건) | 없음 |
| 틀 오류 줄(BENCH) | brake(V2 `stopDirections`) | OFF 유지 | BENCH 그대로 | 나쁜 줄 계수 | `N` |
| 틀 오류·CRC 오류 줄(ARMED, DISARMED) | 바꾸지 않음 | 바꾸지 않음 | 그대로 | 나쁜 줄 계수 | `N` |
| MCU 리셋·저전압 리셋·포트 열기 리셋 | coast(핀이 뜸, EN 풀다운) | 서보 없음. 비자기잠금이면 풀린다 | BOOT → DISARMED | 새 boot_id, BOOT | 배너 |
| 전원 차단(물리 비상정지) | coast | 서보 없음(Q15) | — | — | — |

- 래치 결함과 나쁜 줄 연속의 행은 어느 상태에서 생겨도 같은 구동·조향 동작을 한다(DISARMED에서 감속 중인 duty도 brake면 바로 0이 된다).
- 한 틱에 여러 사건이 겹치면 더 강한 쪽을 쓴다: 조향은 OFF > HOLD > 유지, 구동은 brake > release.
- 모터 전원만 끊기고 MCU는 켜져 있으면 펌웨어는 모른다. 서보가 밀어도 위치가 변하지 않으므로 STALL이나 TRACK으로 선다.
- 결함으로 선 뒤 조향 유지 허용 오차 밖으로 움직이라는 명령은 내지 않는다. OFF 뒤 외력으로 바퀴가 움직이는 것은 물리 현상이다(설계 4.1).

### 6.5 HOLD

- 시작: 틱 t0, 목표 정책, 시간 h(`hstop`, `hdis`, `hloss` 가운데 사건에 맞는 것). 조향 모드 HOLD. 서보(7.6절)는 서보 틱마다 돈다.
- 끝: 틱 t0 + h의 7단계에서 OFF. h = 0이면 t0의 7단계에서 바로 OFF(서보가 한 번도 돌지 않는다).
- 목표 정책:
  - 측정 위치(hold_measured, `stoppol` 0, `wdmode` 0, DISARM, 나쁜 줄 연속): `tgt = cmd = P[t0]`.
  - 목표 유지(hold_target, `stoppol` 1): `tgt = cmd`(그 틱의 cmd). cmd는 더 움직이지 않는다.
  - 중앙(center, `stoppol` 2, `wdmode` 1): `tgt = 0`, `cmd = P[t0]`. cmd가 속도 제한(7.5절)으로 0까지 간다.
- HOLD 동안 조향 결함 감시가 모두 돈다. 결함이 켜지면 바로 OFF다.
- HOLD를 일찍 끝내는 것: 받은 `ARM`(ACTIVE로 넘어감), 결함(OFF). `STOP`, `DISARM`, `INFO`, `PING`은 HOLD를 바꾸지 않는다.
- 유지 오차: 서보가 데드밴드에 든 뒤에는 \|cmd − pos\| ≤ 2·`db`로 둔다(7.6절의 다시 밀기 문턱). 기본값 40 ‰ ≈ 1°는 A17의 max(1°, 2·데드밴드)와 같다.

### 6.6 나쁜 줄 계수와 CRC 결함

- 나쁜 줄: 틀 오류(`long`, `ctrl`)와 CRC 오류(`crc`). `STOP` 특례 줄은 세지 않는다.
- 좋은 줄: 틀이 맞고 CRC(있으면)가 맞는 줄. 명령으로 받았는지와 상관없다. 빈 줄은 어느 쪽도 아니다.
- `STOP` 특례 줄: `STOP`과 CRC가 맞는 `STOP*HH`는 좋은 줄이다(`bad_run` = 0). CRC가 틀린 `STOP*xx`는 STOP을 실행하지만 좋은 줄도 나쁜 줄도 아니다(`bad_run` 그대로. 12.1-52).
- `bad_run`: 나쁜 줄마다 1 올린다. 좋은 줄마다 0으로 한다. 받은 `ARM`도 0으로 한다.
- `bad_run`이 `crcn`에 닿으면 그 줄을 처리한 틱(4단계)에 CRC 결함이다. CRC 비트가 이미 켜져 있으면 줄을 다시 내지 않는다. 동작은 6.4절 표의 행이다(어느 상태에서나). 그 뒤 `bad_run = 0`.

### 6.7 결함 판정 표

판정은 틱 n의 6단계(7.1절)에서 한다(CRC는 4단계, WATCHDOG은 3단계, TTL은 5단계). 기호: f[n], raw[n], P[n]은 그 틱 2단계에서 읽은 값이다. u[k]는 틱 k에 쓴 위치 기준 서보 출력(7.6절), 핀값 d[k] = D6[k] − D7[k]이다. 창에 필요한 과거 값이 부팅 뒤에 없으면(처리한 틱 수 < 창) 그 판정을 하지 않는다. 처리한 틱 수는 포화하는 계수로 센다(감기는 틱 번호로 재지 않는다. 12.1-72). THERMAL은 예외다: 부팅 전 틱을 출력 0으로 세므로 부팅 직후에도 판정한다(아래 표의 t + `tha` + 1. 12.1-49).

창의 틱은 처리한 틱이다. 곧 `loop()`가 새 `millis()` 값으로 돈 틱이다. 루프가 멈춘 동안(`mcu_stall`, 실기의 긴 루프 멈춤)의 ms는 창에 들어가지 않는다(12.1-50).

| 코드 | 판정(이 틱에 참이면 켠다) | 도는 때 | 가장 이른 판정 틱(예) |
|---|---|---|---|
| POT_OPEN | f[n] < `pvmin` | 늘(모든 상태, 모든 모드) | raw가 틱 t부터 0, `med` 5면 t + 2 |
| POT_SHORT | f[n] > `pvmax` | 늘 | 같음 |
| POT_JUMP | \|f[n] − f[n − `jmpw`]\| > `jmp`. 이 틱에 OPEN·SHORT 조건이 참이면 보지 않는다 | 늘 | f가 틱 t에 크게 바뀌면 t |
| POT_NOISE | 100·(W·Σx² − (Σx)²) > `nsd`²·W². x는 raw[n − W + 1..n], W = `nsw`(64비트 정수). OPEN·SHORT 조건이 참이면 보지 않는다 | 늘 | 창이 찬 뒤 |
| POLARITY | runPol ≥ `polt`이고 (P[n] − P[n − `polt`])·s < −`polp`. s는 runPol의 부호 | 조향 모드 ACTIVE·HOLD | 출력 시작 틱 t면 t + `polt` |
| DRV_STUCK | runStuck ≥ `dst`이고 \|P[n] − P[n − `dst`]\| > `dsm` | ACTIVE·HOLD | BRAKE로 시작한 관찰이 이어지는 동안 창 안 변위가 `dsm`을 넘는 첫 틱(12.1-70) |
| STALL | runStall ≥ `stt`이고 \|P[n] − P[n − `stt`]\| < `stm` | ACTIVE·HOLD | 큰 출력 시작 틱 t면 t + `stt` |
| TRACK | runTrack ≥ `trt` | ACTIVE·HOLD | runTrack 조건(아래, 12.1-69)이 틱 t부터 참이면 t + `trt` − 1 |
| THERMAL | cnt > `tha`. cnt는 k ∈ [n − 1000·`thw`, n − 1]에서 \|d[k]\| ≥ max(`thp`, 1)인 틱 수 | 늘(JOG·POL 포함). 래치가 있고 조건이 다시 참이면 JOG·POL을 끈다(12.1-68) | 큰 출력이 틱 t부터 계속이면 t + `tha` + 1 |
| CRC | 6.6절 | 늘 | — |
| WATCHDOG | 6.2절 | ARMED, BENCH | 마지막 입력 t면 t + `wd` + 1 |
| TTL | 6.3절 | ARMED | 받은 틱 t면 t + ttl |
| NOCAL | 부팅 때 R이 유효하지 않음, `CAL ERASE` | — | 부팅 틱 b |
| SOFTLIMIT | W 보정이 완전하고 pos > `soft_l` 또는 pos < `soft_r` | 늘(조건 비트) | — |

달리기 계수(틱 k의 11단계, 핀을 정한 뒤 갱신. 65535에서 멈춤):

- runPol: 조향 모드가 ACTIVE·HOLD이고 \|u[k]\| ≥ max(`minp`, 1)이면, 직전 달리기와 부호가 같을 때 +1, 아니면 1로 새로 시작하고 부호를 u[k]의 부호로 둔다. 그 밖에는 0.
- runStall: ACTIVE·HOLD이고 \|u[k]\| ≥ max(`stp`, 1)이면 +1(부호 무관), 아니면 0.
- runStuck(12.1-70): ACTIVE·HOLD이고 조향 EN이 1인 틱을 하나의 관찰로 센다. 관찰은 BRAKE 틱(u[k] = 0)에서 시작하고 그 틱의 위치를 기준 P_s로 둔다. BRAKE 틱은 관찰을 잇는다. 미는 틱(u[k] ≠ 0)에서는:
  - 관찰이 BRAKE 단계이고 \|P − P_s\| ≤ `db`이면(새 목표가 밀게 했다) 관찰을 끝낸다(0). 반대 극성은 POLARITY가 본다.
  - \|P − P_s\| > `db`이면 BRAKE 동안 움직였다는 뜻이다(표류 단계). 그 뒤 (P − P_s)·sign(u) ≤ `db`인 동안(미는 쪽으로 P_s를 `db`보다 넘게 지나지 않음) +1, 넘으면 끝(0).
  - 표류 단계에서 BRAKE로 돌아와 \|P − P_s\| ≤ `db`이면 BRAKE 단계로 돌린다(다음에 미는 틱을 새로 판정).
  - 조향 EN 0이거나 ACTIVE·HOLD가 아니면 0.
  - 이유: 기본 서보는 BRAKE에서 2·`db`를 넘으면 다시 민다(7.6절). 그래서 BRAKE 틱만 세면 한 관찰에 움직임이 약 3·`db`(60 ‰)를 넘지 못해 `dsm` 80 ‰을 볼 수 없었다. 고착 드라이버는 BRAKE에도 움직이고 다시 밀어도 미는 쪽으로 오지 않는다.
- runTrack(12.1-69): 6단계에서 갱신한다. ACTIVE·HOLD이고 \|cmd − P[n]\| > `trk`이고 위치가 최근 `stt` 틱 동안 cmd 쪽으로 `stm`보다 많이 움직이지 않았으면((P[n] − P[n − `stt`])·sign(cmd − P[n]) ≤ `stm`) +1, 아니면 0. cmd는 이 틱 서보 갱신 전 값이다. 처리한 틱 수가 `stt`보다 적으면 움직이지 않은 것으로 본다. 이유: 자리값 P 서보는 속도 제한(정지 중 12°/s, 주행 중 24°/s)을 따라갈 때 약 6° 뒤처진다(`trk` 3°보다 큼). 따라가는 서보의 뒤처짐은 결함이 아니다. 멈추거나 반대로 가는 조향만 센다.
- 위치 기록: P를 틱마다 1024칸 고리에 둔다(창 1000 ms까지). f는 128칸 고리(`jmpw` ≤ 100). THERMAL은 60,000비트 고리(`thw` ≤ 60 s)다. 1024칸·128칸 고리는 2의 거듭제곱이라 틱 번호의 2^32 감김을 넘어 맞다. THERMAL 고리의 색인은 틱 번호와 따로 세고 60,000에서 감긴다(2^32는 60,000의 배수가 아니다. 12.1-72).

켜질 때 동작: 6.4절 표의 그 행. 래치 비트를 켜고 `F` 줄을 낸다. 같은 틱에 여러 코드가 켜지면 모두 켜고 `F`를 비트 순으로 낸다.

ARM, `CAL JOG`, `CAL POL`을 막는 활성 조건(`fault`, 12.1-68): 그 틱에 POT_OPEN 조건(f < `pvmin`), POT_SHORT 조건(f > `pvmax`), THERMAL 조건(cnt > `tha`) 가운데 하나라도 참. 다른 래치는 판정 순간의 사건이라 활성 조건이 없다. `ARM`이 지운다(POLARITY·DRV_STUCK은 12.1-71).

### 6.8 래치 풀기

| 명령·사건 | 지우는 것 |
|---|---|
| 받은 `ARM` | 래치 비트 전부(NOCAL, POT_OPEN, POT_SHORT, POT_JUMP, POT_NOISE, POLARITY, DRV_STUCK, STALL, TRACK, THERMAL, WATCHDOG, CRC)와 BOOT. `bad_run` = 0. POLARITY·DRV_STUCK이 있으면 ARM은 받지 않는다(`pol`, 12.1-71) |
| 통과한 `CAL POL`(`A POL ok`) | POLARITY, DRV_STUCK(12.1-71) |
| 받은 `C` | TTL 비트 |
| ARMED를 떠남 | TTL 비트 |
| `BENCH`, `STOP`, `DISARM`, `INFO`, `PING`, `CAL`(POL 통과 밖), `CFG` | 아무것도 지우지 않는다 |
| MCU 리셋 | 전부(새 부팅에서 BOOT가 켜지고, R이 없으면 NOCAL이 켜진다) |

- `ARM`은 활성 조건이 있으면 거절된다(`nocal`, `cal`, `pol`, `fault`). 그래서 "래치는 운영자 재무장으로만 풀린다"와 "래치된 결함이 있으면 ARM 거절"이 함께 성립한다: 조건이 사라진 래치만 `ARM`이 푼다.
- POLARITY·DRV_STUCK은 하드웨어 결함(배선 반대, 드라이버 고착)이라 재무장만으로 풀지 않는다. 운영자가 `CAL POL`로 드라이버가 두 방향을 따르는 것을 다시 보여야 한다(12.1-71). 리셋·전원 껐다 켬은 RAM과 함께 래치를 지운다. 예전에는 ARM 한 번으로 풀려 반대 방향 조향이 재무장마다 쌓였다(2026-11-09 검토).
- 호스트 쪽 래치(상태 줄 끊김, boot_id 변경, `cal_id` 불일치, STOP 확인 실패 등)는 호스트 규격(설계 4.1·4.2)이다. 호스트도 운영자 재무장으로만 푼다.

## 7. 조향 서보와 틱 순서 (F)

### 7.1 틱과 `loop()` 순서 (규범)

- MCU 틱은 1 ms다. SIM에서는 lockstep 틱 하나마다 `loop()`를 정확히 한 번 부른다. 부팅 틱 b에는 `setup()` 뒤 `loop()`를 한 번 부른다. 실기에서는 `loop()`가 1 ms보다 자주 돌 수 있다. 아래 시간 규칙은 모두 `millis()` 차이로 적었으므로 실기에서도 같은 뜻이다.
- `loop()` 한 번의 순서:

| 단계 | 하는 일 |
|---|---|
| 1 | `now = millis()` |
| 2 | ADC: `analogRead(A0)`를 정확히 한 번 부른다 → raw[n]. 중앙값 → f[n]. 위치 → P[n](7.3절). 기록 고리에 넣는다. 조향 모드가 OFF·JOG·POL이면 `tgt = cmd = P[n]` |
| 3 | 링크 watchdog 판정(6.2절) |
| 4 | 수신: 최대 64 B를 읽어 줄 파서에 넣는다. LF마다 그 줄을 바로 실행한다(4절). 나쁜 줄 계수·CRC 결함(6.6절) |
| 5 | `C` TTL 판정(6.3절) |
| 6 | 결함 판정(6.7절), runTrack 갱신, SOFTLIMIT 비트 갱신. 켜진 결함의 동작 |
| 7 | 조향 모드 시계: HOLD 끝(6.5절), JOG 끝, POL 단계 진행·끝(7.10절) |
| 8 | 구동 램프 두 쪽(8.2절). 움직임 기록(7.5절) |
| 9 | 조향 출력: 모드가 ACTIVE·HOLD이고 서보 틱이면 서보 갱신(7.6절). 모드에 따라 조향 핀 값을 정한다(7.7절) |
| 10 | 상태 줄(5.4절) |
| 11 | 바뀐 핀만 쓴다(8.4절). 달리기 계수와 THERMAL 고리 갱신(6.7절) |
| 12 | TX 고리 → UART FIFO(5.10절). 기다리지 않는다 |

- 4단계에서 실행한 명령(예: `STOP`)의 출력 변화는 같은 틱 11단계에 핀에 쓴다. 그래서 `STOP` 파싱부터 구동 0까지 0틱이다. 핀 쓰기는 그 틱의 직렬 송신(12단계)보다 앞이다(12.1-66).
- 3·5·6단계의 정지 동작도 같은 틱 11단계에 핀에 나간다.
- 서보 틱: 주기 P_sv = 1000 / `svhz` ms(500 Hz면 2, 1000 Hz면 1). 부팅 틱 b에서 `next_servo = b`, `last_servo = b − P_sv`. 9단계에서 (int32)(now − next_servo) ≥ 0이면 서보 틱이다. 그때 dt = min(now − last_servo, 50), `last_servo = now`, `next_servo += P_sv`. 그래도 밀려 있으면 `next_servo = now + P_sv`. 서보 시계(`next_servo`, `last_servo`)는 조향 모드와 상관없이 서보 틱마다 나아간다. 서보 법칙은 모드가 ACTIVE·HOLD일 때만 돈다. SIM에서 서보 틱은 `svhz`가 그대로일 때 b, b + P_sv, b + 2·P_sv, …다(실행 중 바꾸면 4.10절).
- 조향 모드가 OFF·JOG·POL에서 ACTIVE·HOLD로 바뀌면 첫 서보 틱까지 조향 핀은 OFF 그대로다. ACTIVE와 HOLD 사이를 바꿀 때는 핀이 끊기지 않는다.

### 7.2 ADC와 거르기

- `analogRead(A0)`는 0..1023이다. 표본은 틱마다 하나(1 kHz)다. 프로필 `pot_filter.sample_hz` 2000은 이 규격에서 쓰지 않는다.
- 중앙값 거르기: 최근 `med`개(홀수 1..9) raw의 중앙값이 f다. 부팅 뒤 첫 표본(또는 `med`를 바꾼 틱의 표본)으로 버퍼 전체를 채운다. 그래서 첫 틱부터 f가 있다.
- 정수만 쓴다. 정렬은 값 비교만 한다(같은 값이 여럿이어도 결과는 같다).
- SIM의 `analogRead` 값은 lockstep 규약 11.5절이 정한다(가변저항 모형 또는 대본 값).

### 7.3 raw ↔ 천분율

W의 표시 세 개가 모두 있고, 한 방향으로 엄격히 단조이고(raw_l > raw_c > raw_r 또는 raw_l < raw_c < raw_r), 두 반쪽 폭 \|raw_l − raw_c\|, \|raw_c − raw_r\|이 모두 50 이상이면 "W 표시 유효"다. 여기에 soft_l ≥ 1, soft_r ≤ −1이면 "W 보정 완전"이다.

W 표시 유효일 때 f → P(‰):

```
dir  = (raw_l > raw_c) ? +1 : -1
d    = (f - raw_c) * dir                  # 왼쪽 쪽이 +
if d >= 0: span = |raw_l - raw_c|;  P =  (2000*d + span) / (2*span)       # 정수 나눗셈
else:      span = |raw_c - raw_r|;  P = -((2000*(-d) + span) / (2*span))
P = clamp(P, -9999, 9999)
```

- 반올림은 0.5에서 0에서 먼 쪽이다. f = raw_l이면 +1000, raw_c이면 0, raw_r이면 −1000이다. 표시 밖은 같은 기울기로 늘린다.
- W 표시가 유효하지 않으면 P = 0이다(NOCAL 동안 서보를 돌릴 수 없다).
- 예(9.6절 레코드: raw_l 842, raw_c 512, raw_r 181): f 611 → d 99 → (198000 + 330) / 660 = 300. f 400 → d −112 → −((224000 + 331) / 662) = −338.
- 천분율 → raw(호스트 도구용, 참고): pm ≥ 0이면 raw = raw_c + dir·round(pm·\|raw_l − raw_c\| / 1000), pm < 0이면 raw = raw_c − dir·round(\|pm\|·\|raw_c − raw_r\| / 1000).
- 각도(mrad)와 천분율의 관계는 MCU가 모른다. 호스트가 차량 프로필의 보정표(B2·B3에서 잰 각)로 바꾼다.

### 7.4 소프트 한계

- soft_l(1..1000)과 soft_r(−1000..−1)은 W의 보정 칸이다(`CAL SOFT`, 레코드).
- 서보 목표는 goal = clamp(tgt, soft_r, soft_l)이다. 그래서 cmd는 소프트 한계를 넘지 않는다.
- SOFTLIMIT 비트와 바깥쪽 출력 막기(7.8절)는 W 보정이 완전할 때만 쓴다.

### 7.5 속도 제한과 "움직이는 중"

- 움직이는 중(moving): 8단계 뒤 L ≠ 0 또는 R ≠ 0이면 참. 아니면, 부팅 뒤 0이 아닌 duty가 한 번이라도 있었고 (now − `last_drive_nonzero`) < `movms`이면 참. 그 밖에는 거짓. `last_drive_nonzero`는 8단계 뒤 L 또는 R이 0이 아닌 틱의 ms다. 설계 4.4·6.5의 "MCU가 보고한 구동 duty가 최근 0.3 s 안에 0이 아니었음"과 같다.
- 속도 상한: moving이면 `rmov`, 아니면 `rstd`(‰/s). 정지 중 조향 속도 상한(Q7)이다. MCU는 정지 중 조향을 금지하지 않는다. 금지 여부는 호스트·실행기 규칙이다.
- cmd 이동(서보 틱마다, dt ms):

```
if cmd == goal: rrem = 0
else:
    acc  = rrem + rate * dt
    step = acc / 1000;  rrem = acc % 1000
    cmd  = (cmd < goal) ? min(cmd + step, goal) : max(cmd - step, goal)
    if cmd == goal: rrem = 0
```

### 7.6 서보 법칙 (정수)

서보 틱마다(조향 모드 ACTIVE·HOLD), pos = P[n]:

```
goal = clamp(tgt, soft_r, soft_l)
(7.5절로 cmd 이동)
e = cmd - pos
if braked:
    if |e| > 2*db: braked = false; inDb = 0        # 다시 민다
    else: u = 0; goto LIMIT
if |e| <= db:
    u = 0; inDb += dt
    if inDb >= dwell: braked = true                 # 데드밴드 안 제동 유지
else:
    inDb = 0
    mag = (kp * |e| + 128) >> 8                     # kp는 PWM/‰ x 256
    mag = min(max(mag, minp), maxp)
    u = sign(e) * mag
if u != 0 and lastSign != 0 and sign(u) != lastSign and (now - lastNzMs) < rev:
    u = 0                                           # 방향 전환 휴지
LIMIT:
if u > 0 and pos >= soft_l: u = 0                  # 7.8절
if u < 0 and pos <= soft_r: u = 0
if u != 0: lastSign = sign(u); lastNzMs = now
```

- 모든 값은 int32다. kp·\|e\| ≤ 4096·20000 < 2^31이다.
- 서보 틱이 아닌 틱에는 u와 핀을 그대로 둔다.
- 내부값 시작: ACTIVE·HOLD에 들어갈 때 rrem = 0, braked = false, inDb = 0. 직전 틱에 조향 EN이 0이었으면 lastSign = 0도 한다(ACTIVE ↔ HOLD 전환에서는 lastSign, lastNzMs를 둔다).
- 기본값에서: 데드밴드 20 ‰(0.5°) 안이면 BRAKE, 200 ms 머물면 제동 유지, 40 ‰(1°)을 넘으면 다시 민다. 데드밴드 밖 최소 출력 102(40 %), 최대 255.
- 방향 전환 휴지 `rev`(20 ms): 반대 방향 출력은 마지막 0이 아닌 출력 뒤 `rev` ms가 지나야 나간다. 그동안 BRAKE다.

### 7.7 조향 출력 모드별 핀

| 모드 | 조향 EN(D2) | D6 | D7 |
|---|---|---|---|
| OFF | 0 | 0 | 0 |
| ACTIVE·HOLD, u = 0 | 1 | 0 | 0 |
| ACTIVE·HOLD, u ≠ 0, `mdir` = +1 | 1 | u > 0이면 \|u\| | u < 0이면 \|u\| |
| ACTIVE·HOLD, u ≠ 0, `mdir` = −1 | 1 | u < 0이면 \|u\| | u > 0이면 \|u\| |
| JOG(pwm) | 1 | pwm > 0이면 \|pwm\| | pwm < 0이면 \|pwm\| |
| POL 구동 단계 | 1 | 7.10절 | 7.10절 |

- 서보가 돌기 전(ACTIVE·HOLD에 막 들어와 첫 서보 틱 전)은 OFF 행이다(7.1절).
- 상태 줄 `sp` = D6 − D7이다.

### 7.8 소프트 한계 바깥쪽 출력 막기

- 서보 법칙 LIMIT 단계: W 보정이 완전할 때, pos ≥ soft_l이면 + 쪽 u, pos ≤ soft_r이면 − 쪽 u를 0으로 한다. 안쪽 출력은 허용한다.
- JOG에는 쓰지 않는다(바퀴를 띄운 벤치의 짧은 개방 출력, 최대 500 ms).
- SOFTLIMIT 비트는 경고다. 래치하지 않는다. 결함 정지를 하지 않는다.

### 7.9 PWM과 EN

- `analogWriteRange(255)`, `analogWriteFreq(1000)`은 ESP8266 코어에서 모든 PWM 핀에 공통이다. 그래서 조향 PWM도 0..255, 1 kHz다[자리값: 조향 드라이버 사양(Q11) 확인].
- EN 핀(D1, D2)은 `digitalWrite`로 쓴다. PWM 핀은 `analogWrite`로 쓴다.
- 조향 드라이버는 PWM 입력 두 개 + EN 하나(BTS7960류)를 가정한다[자리값: Q11].

### 7.10 `CAL POL` 극성 확인 절차

받은 틱을 t로 한다. 조향 모드 POL. 상수는 모두 [자리값]이다(B0에서 확인).

| 틱 | 출력 |
|---|---|
| t .. t+299 | EN 1, D6 = 153(60 %), D7 = 0 |
| t+300 .. t+499 | OFF |
| t+500 .. t+799 | EN 1, D7 = 153, D6 = 0 |
| t+800 .. t+999 | OFF |
| t+1000 | 7단계에서 판정, 모드 OFF |

- d1 = f[t+500] − f[t], d2 = f[t+1000] − f[t+500].
- 통과: \|d1\| ≥ 4, \|d2\| ≥ 4, d1·d2 < 0. 그러면 W의 mdir = sign(d1)·sign(raw_l − raw_c), pol_ok = 1. 실패면 pol_ok = 0(mdir는 그대로).
- 틱 t+1000에 `A POL ok <d1> <d2>` 또는 `A POL fail <d1> <d2>`, 그다음 보정 보고 줄을 낸다.
- 결과는 W만 바꾼다. `CAL SAVE`로 저장해야 `ARM` 조건(R의 pol_ok)에 들어간다.
- 정지 중 조향은 느리다(자리값 정지 중 15°/s, 시작 PWM 35 %). 벤치에서 바퀴를 띄우면 주행 중 값에 가깝다. 300 ms·60 %에서 바퀴를 띄운 자리값 구동기는 약 3°(약 12 count) 움직인다[추정, 계산].

### 7.11 보정이 없을 때

- 저장된 R이 없으면 NOCAL이다. 부팅 때 W의 표시는 비어 있다. W 표시가 유효해지기 전(`CAL MARK` 세 번 전)에는 P = 0이고 상태 줄 pos, tgt, cmd는 0이다.
- `ARM`은 `nocal`로 거절된다. 조향 모드는 OFF, JOG, POL만 될 수 있다(POL은 L, C 표시가 있어야 한다).
- 구동은 BENCH에서만 쓸 수 있다.

## 8. 구동 (G)

### 8.1 duty의 뜻

- L, R은 부호 있는 −255..255다. +는 전진, −는 후진이다. 크기는 그 방향 핀의 PWM 값(0..255, 1 kHz)이다. 반대 방향 핀은 0이다. 한 쪽의 두 핀이 동시에 0이 아닌 일은 없다(V2 불변식).
- 한 쪽(드라이버 하나)의 앞뒤 두 모터는 같은 duty를 받는다. 좌·우는 `C`의 L, R이다. 호스트가 throttle에서 같은 값을 만든다(설계 4.2).
- PWM은 엔코더가 아니다. 적용 duty는 바퀴 속도의 측정값이 아니다.

### 8.2 램프·휴지·정지 (V2와 같다)

쪽마다 상태: 적용 duty d, 목표 g, 나머지 rem(0..999), `ramp_at`, `zero_at`(ms). 부팅 틱 b에서 d = g = rem = 0, `ramp_at = zero_at = b`.

8단계에서 쪽마다:

```
el = now - ramp_at
if el < 10: return
ramp_at = now
if el > 50: el = 50                      # 루프가 멈춰도 한 번에 50 ms분만
if d == 0:
    if g == 0 or (now - zero_at) < 120: rem = 0; return
    goal = g
elif g != 0 and sign(g) == sign(d): goal = g
else: goal = 0
if d == goal: rem = 0; return
sc  = rem + el * 255
dl  = sc / 1000;  rem = sc % 1000
if |goal| > |d|: d2 = sign(goal) * min(|d| + dl, |goal|)
else:            d2 = sign(d)    * max(|d| - dl, |goal|)
if d != 0 and d2 == 0: zero_at = now
d = d2
if d == goal: rem = 0
```

- 램프는 255/s(0 → 255에 1000 ms)다. 10 ms 격자는 `ramp_at`에 묶인다. 갱신마다 2 또는 3 단위가 바뀐다.
- 휴지: 한 쪽의 duty가 0이 된 뒤(램프든 brake든) 120 ms 동안 그 쪽은 어느 방향으로도 다시 출발하지 않는다. 방향 반전은 0까지 감속 → 120 ms → 반대 방향 가속이다. 부팅 직후 120 ms도 같다(`zero_at = b`).
- 최신 목표가 이긴다: 감속 중에 같은 방향 목표가 다시 오면 그 자리에서 다시 가속한다(V2 18번).
- brake(즉시 정지, V2 `stopDirections`): 쪽마다: d ≠ 0이면 `zero_at = now`. 그리고 d = 0, g = 0, rem = 0, `ramp_at = now`. 같은 틱 11단계에 방향 핀 4개를 0으로 쓴다.
- release: 두 쪽 g = 0. 8단계 램프가 d를 255/s로 줄인다.
- 예: 틱 b = 0에 부팅, 틱 20에 L = 100을 받으면 첫 출력은 틱 120에 2, 130에 5, 140에 7, 150에 10이다. 0 → 100의 끝은 틱 510(100 × 1000 / 255 ≈ 392 ms 뒤, 10 ms 격자)이다.

### 8.3 핀 (배치 A안, 같은 ESP8266, Q5 기본값)

| NodeMCU | GPIO | V3 역할 | V2 역할 | 부팅 특성(일반 자료, 실기 확인) |
|---|---|---|---|---|
| D8 | 15 | 왼쪽 전진 PWM | 같음 | 부팅 때 LOW여야 함(보드 풀다운) |
| D5 | 14 | 왼쪽 후진 PWM | 같음 | — |
| D3 | 0 | 오른쪽 전진 PWM | 같음 | 부팅 모드 핀, 부팅 때 HIGH |
| D0 | 16 | 오른쪽 후진 PWM | 같음 | 부팅 때 HIGH일 수 있음 |
| D1 | 5 | 구동 EN(드라이버 2개의 EN 4선을 묶음, 선마다 풀다운) [자리값: Q5 배선 확인] | EN 4개 가운데 하나 | 부팅 때 조용함 |
| D2 | 4 | 조향 EN | EN | 부팅 때 조용함 |
| D6 | 12 | 조향 PWM A(`mdir` +1에서 + 쪽, 왼쪽) | EN | 부팅 때 조용함 |
| D7 | 13 | 조향 PWM B | EN | 부팅 때 조용함 |
| D4 | 2 | 쓰지 않음(설정하지 않는다) | 미사용 | 부팅 때 HIGH 필요 |
| A0 | ADC | 가변저항 와이퍼(3.3 V 급전, 직렬 저항 + 커패시터) | 미사용 | 약 0..3.2 V 입력 |
| TX, RX | 1, 3 | USB 직렬 | 같음 | — |

`setup()` 핀 순서(V2 `boot_order`의 확장):

1. `analogWriteRange(255)`, `analogWriteFreq(1000)`.
2. D8, D5, D3, D0 순서로 핀마다 `digitalWrite(LOW)`, `pinMode(OUTPUT)`, `analogWrite(0)`.
3. D6, D7 순서로 같은 세 호출.
4. D2: `digitalWrite(LOW)`, `pinMode(OUTPUT)`.
5. D1: `digitalWrite(HIGH)`, `pinMode(OUTPUT)`. 구동 EN은 그 뒤 펌웨어가 다시 내리지 않는다(V2 "power locked HIGH"와 같다).
6. V2처럼 `WiFi.mode(WIFI_OFF)`, `Serial.begin(115200)`, `EEPROM.begin(256)`(12.1-58).

구동 정지 모드의 핀 수준 뜻(`wheel_pwm.ino`의 동작 그대로):

| 모드 | D1 | 방향 핀 | BTS7960에서 일어나는 일[가정, 실차 미확인] |
|---|---|---|---|
| 구동 | HIGH | 한쪽 핀 \|d\|, 반대 핀 0 | PWM 구동 |
| release | HIGH | 활성 핀 duty가 255/s로 줄어 0 | 감속 뒤 brake 상태 |
| brake | HIGH | 네 핀 모두 그 틱에 0 | EN이 켜진 채 두 입력 LOW → 두 출력 GND → 모터 단락 제동 |
| coast | LOW(뜸, 풀다운) | 뜸 | 두 하프브리지 꺼짐 → 관성 주행. 펌웨어는 만들지 않는다(리셋·전원 끊김 때만) |

### 8.4 핀 쓰기

- 펌웨어는 핀 값이 바뀔 때만 `analogWrite`나 `digitalWrite`를 부른다(V2 `writeDuty`). `setup()`의 첫 쓰기는 예외다.
- 그래서 같은 0 명령을 되풀이해도 GPIO 쓰기가 없다(V2 22번). SIM은 쓰기 횟수를 센다(lockstep 규약 11.6절 `gpio_writes`).

## 9. 보정 저장 (H)

### 9.1 영역

- `EEPROM.begin(256)`(ESP8266 코어의 플래시 흉내). 레코드 하나를 오프셋 0에 둔다. 길이 106 B다. 106..255는 쓰지 않는다.
- 부팅 때 읽어 검사한다(9.4절). 유효하면 R = 레코드, W = R의 복사다. 아니면 R 없음(NOCAL), W = 빈 보정 칸 + 컴파일 기본값(4.10절).

### 9.2 레코드 배치 (모든 정수 little-endian)

| 오프셋 | 크기 | 칸 | 값 |
|---|---|---|---|
| 0 | 4 | magic | `USC3`(55 53 43 33) 유효, `USCX`(55 53 43 58) 지움 표시 |
| 4 | 1 | version | 1 |
| 5 | 1 | length | 106 |
| 6 | 2 | cal_id | u16, 1..65535 |
| 8 | 2 | raw_l | u16, 0..1023 |
| 10 | 2 | raw_c | u16 |
| 12 | 2 | raw_r | u16 |
| 14 | 2 | soft_l | i16, 1..1000 |
| 16 | 2 | soft_r | i16, −1000..−1 |
| 18 | 1 | pol_ok | 0 또는 1 |
| 19 | 1 | mdir | 0x01(+1) 또는 0xFF(−1) |
| 20 | 82 | 변수 | u16 41개, 4.10절 표의 순번 순서(`kp`가 20, `db`가 22, …, `movms`가 98, `svhz`가 100) |
| 102 | 4 | crc32 | 오프셋 0..101의 CRC-32(9.3절) |

### 9.3 CRC-32

| 항목 | 값 |
|---|---|
| 이름 | CRC-32/ISO-HDLC(zlib `crc32`와 같다) |
| 다항식 | 0x04C11DB7, 반사형 0xEDB88320 |
| 초기값 | 0xFFFFFFFF |
| 입력·출력 반사 | 예 |
| 끝 XOR | 0xFFFFFFFF |
| 검사값 `123456789` | 0xCBF43926 |
| 저장 | little-endian 4 B. 보고 줄 `crc=`는 값을 대문자 16진 8자리로 쓴다 |

```
crc = 0xFFFFFFFF
for each byte x: crc ^= x; repeat 8: crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1
crc ^= 0xFFFFFFFF
```

### 9.4 검사

읽을 때(부팅) 레코드가 유효하려면 아래를 모두 만족해야 한다. 하나라도 어기면 R 없음(NOCAL)이다:

- magic `USC3`, version 1, length 106, CRC-32 일치.
- cal_id ≥ 1. pol_ok ∈ {0, 1}. mdir ∈ {0x01, 0xFF}.
- 표시: 7.3절의 "W 표시 유효" 조건, 그리고 세 표시가 레코드 자신의 [`cwmin`, `cwmax`] 안.
- soft_l 1..1000, soft_r −1000..−1.
- 변수 41개가 모두 4.10절 범위와 묶인 검사를 통과.

`CAL SAVE` 때 W가 통과해야 할 것: "W 보정 완전"(7.3절), 세 표시가 W의 [`cwmin`, `cwmax`] 안, 변수 검사. pol_ok는 0이어도 저장한다(표시를 먼저 저장할 수 있게). `ARM`은 R의 pol_ok = 1을 요구한다.

### 9.5 쓰기

- 쓰는 때: `CAL SAVE`, `CAL ERASE`뿐이다. 다른 때는 쓰지 않는다.
- 한도: 부팅마다 commit 8회. 넘으면 `wear`로 거절한다[자리값].
- `cal_id`: 앞 값 prev = R이 유효하면 R의 cal_id, 아니면 EEPROM 바이트 0..3이 `USCX`이고 CRC가 맞으면 그 레코드의 cal_id, 아니면 0. 새 값 = prev + 1. 65535 다음은 1이다. 그래서 지운 뒤 다시 저장해도 같은 cal_id를 다시 쓰지 않는다.
- `SAVE`: W로 레코드를 만들어(magic `USC3`, 새 cal_id, CRC-32) 0..105에 쓰고 commit한다. 그 뒤 R = W. caldirty = cfgdirty = 0. 실기에서 commit이 실패하면 R을 바꾸지 않는다. 응답은 그대로 보정 보고 줄이다(실패 사유 줄은 없다). 호스트는 cal_id가 오르지 않은 것으로 안다(12.1-57).
- `ERASE`: R이 유효하면 magic을 `USCX`로 바꾸고 CRC-32를 다시 계산해 commit한다. R이 없으면 쓰지 않는다(commit도 세지 않는다). 두 경우 모두 W의 보정 칸을 비우고 NOCAL 비트를 켠다. 실기에서 commit이 실패하면 R, W의 보정 칸, NOCAL 비트를 바꾸지 않고 `F NOCAL`을 내지 않는다. 응답은 보정 보고 줄뿐이고 `id`는 R의 cal_id 그대로라 호스트는 지워지지 않은 것을 안다(12.1-73, SAVE와 같은 규칙). [가정] 섹터 지우기는 되고 쓰기만 실패하면 플래시가 비어 다음 부팅은 NOCAL이고 지움 표시도 없어 cal_id가 1부터 다시 쓰일 수 있다. 실기 B9에서 commit 실패를 잴 때 함께 본다.
- commit 시간: 실기 ESP8266에서 플래시 섹터 지우기·쓰기가 수십 ms 걸리고 그동안 `loop()`가 멈출 수 있다[가정, B9 확인]. 그래서 `CAL`은 구동 0, 조향 busy 아님일 때만 받는다(4.13절). SIM은 commit을 0틱으로 친다(12.1절 결정).
- 빈 EEPROM(모두 0xFF), CRC가 틀린 레코드, 지움 표시 레코드는 모두 NOCAL이다.

### 9.6 예 레코드

cal_id 7, raw_l 842, raw_c 512, raw_r 181, soft 935/−935, pol_ok 1, mdir +1, 변수는 모두 기본값:

```
55534333016A07004A030002B500A70359FC0101F50014006600FF00C8001400C003E0012C01F40100000000D007D007D0070000140005001400EB033C00C3030800020032003200280064009900F40114007800E80350002C01CC001E00881305002C01F401A4B1AE61
```

- CRC-32 = 0x61AEB1A4(끝 4 B `A4 B1 AE 61`). 256 B 이미지는 이 106 B 뒤에 0xFF 150 B다.
- 같은 레코드의 지움 표시판(`USCX`)의 CRC-32는 0x4F1B5580이다.

## 10. 신원 (I)

### 10.1 boot_id

- 실기: `setup()`에서 `ESP.random()`(32비트 하드웨어 난수)을 한 번 부른다. 0이면 다시 부른다. 부팅마다 다른 값이 나온다고 본다[가정: Wi-Fi를 끈 상태에서도 부팅마다 다름. B9에서 여러 번 리셋해 확인].
- SIM: HAL의 `ESP.random()`은 결정적이다. RESET 때 상태 s = `boot_seed`(시작 줄)로 둔다. 부를 때마다 splitmix64의 다음 출력의 아래 32비트를 돌려준다. 0이면 건너뛰고 다음 출력을 쓴다. 상태는 MCU 리셋을 넘어 이어진다(SIM의 "하드웨어 난수"). 펌웨어는 부팅마다 `ESP.random()`을 boot_id에만, 정확히 한 번 부른다. 그래서 k번째 부팅의 boot_id = 흐름의 k번째 0 아닌 값이다.
- splitmix64: `s += 0x9E3779B97F4A7C15; z = s; z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9; z = (z ^ (z >> 27)) * 0x94D049BB133111EB; z ^ (z >> 31)`(모두 mod 2^64).
- 시험 벡터(SIM boot_id, 첫 세 부팅): seed 0 → `7B1DCDAF`, `A1B965F4`, `8009454F`. seed 1 → `89025CC1`, `658EEC67`, `FB32555E`. seed 42 → `2FEB6E95`, `B266F103`, `130F9F52`.

### 10.2 장치 ID

- `ESP.getChipId()`(24비트)를 대문자 16진 6자리로 낸다. SIM은 시작 줄 `firmware.chip_id`를 쓴다.

### 10.3 리셋 원인

`ESP.getResetInfoPtr()->reason`(정수)으로 정한다. 문자열 `ESP.getResetReason()`을 파싱하지 않는다.

| reason | `rst=` | `ESP.getResetReason()` 문자열(참고) | SIM 사건 |
|---|---|---|---|
| 0 | `power_on` | Power On | RESET, `power_cycle` |
| 1 | `hw_wdt` | Hardware Watchdog | `mcu_reset` 인자 1 |
| 2 | `exception` | Exception | `mcu_reset` 인자 2 |
| 3 | `soft_wdt` | Software Watchdog | `mcu_reset` 인자 3 |
| 4 | `soft_restart` | Software/System restart | `mcu_reset` 인자 4 |
| 5 | `deep_sleep` | Deep-Sleep Wake | `mcu_reset` 인자 5 |
| 6 | `ext_sys` | External System | `port_open_reset`, `mcu_reset` 인자 6 |
| 그 밖 | `unknown` | — | — |

- [가정] RST 핀(버튼)과 USB 포트 열기의 DTR/RTS 자동 리셋은 6, USB 전원 켬은 0으로 나온다. B9에서 확인한다.

### 10.4 SIM 표시와 하드웨어 프로필

- `sim`: 컴파일 매크로 `UBTS_STEER_SIM`이 1인 빌드(SIM 네이티브)만 `sim=1`이다. ESP8266 빌드는 `sim=0`이다. 실행 중에 바꿀 수 없다.
- `hw`: 컴파일 상수 `UBTS_STEER_HW`, 기본 `front_steer_v0`.
- 호스트 규칙(설계 2.5): 실차 래퍼는 `sim=1`이면 결함으로 멈춘다. SIM 래퍼는 `sim=0`이면 결함으로 멈춘다. `hw`가 차량 프로필과 다르면 호스트가 arm을 거절한다.

### 10.5 build 식별

- `build` = 펌웨어 소스 묶음 SHA-256의 앞 8자리(대문자)다. 빌드 도구가 컴파일 매크로 `UBTS_STEER_FW_SRC`로 넣는다.
- 묶음 SHA-256의 입력: `firmware/ubts_steer_v3/` 아래 소스 파일(`.ino`, `.h`, `.cpp`, `.c`)을 상대 경로(`/` 구분) 바이트 순으로 정렬하고, 파일마다 경로 바이트 + 0x00 + 파일 길이(u64 little-endian) + 파일 바이트를 이어 붙인 것.
- 같은 소스면 SIM 빌드와 실기 빌드의 `build`가 같다. SIM HAL·구동기 모형 소스는 넣지 않는다(그것은 lockstep READY의 build id에 들어간다).

### 10.6 baud

- 컴파일 상수 `UBTS_STEER_BAUD`, 기본 115200[자리값: 230400은 B9 뒤]. SIM 시작 줄의 `mcu.baud`가 이 값과 다르면 시작을 거부한다(lockstep 규약 11.2절).

### 10.7 포트 열기 리셋과 연결 절차(호스트)

- NodeMCU는 포트를 열 때 DTR/RTS로 리셋될 수 있다(09-19 설계 6절). 리셋되면 boot 지연 동안 핀이 뜨고(coast), 그 뒤 새 boot_id 배너가 나온다.
- 호스트는 포트를 연 뒤 CRC가 맞는 배너를 받을 때까지 받은 바이트를 모두 버린다. 포트가 열렸다고 준비 완료로 보지 않는다. 일정 시간(자리값 1.5 s, B9) 안에 배너가 없으면 `INFO`를 보내 배너를 받는다.
- 새 boot_id를 보면 호스트는 이전 무장 상태를 버리고(래치 `mcu_reset_detected`), q를 1부터 다시 센다. MCU는 DISARMED로 시작한다. 자동 재무장은 없다.
- MCU 파서는 리셋 직후 반쪽 줄을 보통 줄로 처리한다(2.2절). 리셋 중 들어온 바이트는 사라진다.
- 포트가 열린 채 MCU가 리셋되면 새 배너가 리셋 전의 반쪽 줄이나 LF 없는 부트 쓰레기와 붙어 호스트에서 깨질 수 있다. 그래서 호스트는 상태 줄의 `ms` 역행이나 이어지지 않는 `n`(새 부팅은 0부터)을 보면 `mcu_reset_detected` 래치, 정지, 그리고 배너가 올 때까지 `INFO`를 보낸다(`banner_timeout_s`마다, 무장 중이 아닐 때). 배너가 오기 전의 ARM은 `mcu_reset_detected`로 거절한다(12.1-61). 배너와 첫 상태 줄 사이에 리셋되면 상태 줄로는 알 수 없다(앞 줄이 없다). 그때는 ARM에 오는 `N - boot`이 리셋의 증거이고 호스트는 같은 처리를 한다(12.1-76).

## 11. 시간 예산 (J)

### 11.1 UART

| 항목 | 115200 baud 8N1 | 230400 baud |
|---|---|---|
| 바이트당 비트 | 10 | 10 |
| 초당 바이트 | 11,520 | 23,040 |
| ms당 바이트 | 11.52 | 23.04 |
| 바이트 1개 | 86.8 µs | 43.4 µs |

### 11.2 줄 길이와 대역

| 줄 | 최대(LF 포함) | 보통 주기 | 115200에서 차지 |
|---|---|---|---|
| 호스트 `C` | 32 B(`C 65535 1000 -255 -255 -1000*HH`) | 20 Hz | 640 B/s, 5.6 % |
| 호스트 `PING` | 5 B | 10 Hz | 50 B/s, 0.4 % |
| 호스트 `STOP` | 5 B | 사건 | — |
| 호스트 최대 줄 | 64 B(내용 63 + LF) | — | — |
| MCU 상태 줄 | 85 B | 20 Hz | 1,700 B/s, 14.8 % |
| MCU 배너 + 보정 보고 | 174 + 129 B | 부팅, `INFO` | 303 B = 26.3 ms |
| MCU `N`, `F`, `W`, `A`, `P` | ≤ 35 B | 사건 | — |

### 11.3 송신 FIFO와 수신 버퍼 [가정, 실기 확인]

- ESP8266 UART0의 하드웨어 송신 FIFO는 128 B다. ESP8266 Arduino 코어 3.1.2 소스(`cores/esp8266/uart.cpp`, `HardwareSerial.h`)에서 확인했다: `Serial.availableForWrite()` = `uart_tx_free()` = 128 − FIFO에 찬 바이트 수, `uart_tx_fifo_full()`은 찬 수 ≥ 127, `Serial.write`는 바이트마다 찬 동안 바쁜 대기를 한다. 그래서 `availableForWrite()`만큼 쓰면 마지막 바이트에서 기다린다. 실기 측정은 B9에서 한다.
- 그래서 펌웨어는 `Serial.write`에 `availableForWrite()` − 1보다 많은 바이트를 주지 않고, 그 값이 1 이하이면 쓰지 않는다(12.1-66). 512 B TX 고리에서 틱마다(실기는 `loop()`마다) 옮긴다(5.10절). 서보 루프와 핀 쓰기가 직렬 출력 때문에 멈추지 않는다. 흉내 층의 시험 선택(`SimMcu::set_tx_fifo_model`, LOCKSTEP 11.5절)이 이 코어 규칙으로 기다림이 0인지 본다(`fw_unit_test.cc`의 `test_tx_fifo_model`).
- 상태 줄 85 B는 전선에서 7.4 ms 걸린다. FIFO 128 B는 약 11 ms 분량이다. 틱마다 옮기면 고리에 바이트가 있는 동안 송신이 끊기지 않는다.
- 수신: 코어 RX 고리 256 B(255 B까지 담음). 넘치면 가장 옛 바이트를 버린다(코어 3.1.2 소스. 12.1-67). `loop()`는 틱마다 최대 64 B를 읽는다.

### 11.4 A17 시간(`SIM_ACCEPTANCE_KO.md` 2절)

| 항목 | 합격 기준 | 이 규격에서 |
|---|---|---|
| STOP 요청 → `STOP` 송신 | ≤ 20 ms(브리지 1틱) | 호스트 몫. `STOP\n` 5 B 전송 0.43 ms |
| MCU `STOP` 파싱 → 구동 0 | ≤ 1 서보 틱 | 같은 틱(0틱, 7.1절) |
| STOP 확인(상태 줄) | ≤ 2 상태 주기 | STOP 뒤 첫 상태 줄이 P_st 안에 만들어지고 7.4 ms 안에 전송된다. TX 고리가 비어 있으면 P_st + 8 ms 안. `INFO` 응답 같은 밀린 바이트가 있으면 그만큼 늦다 |
| `C` TTL 만료 → 구동 0 | ≤ ttl + 1틱 | 받은 틱 + ttl(6.3절) |
| 링크 watchdog → 구동 0 | ≤ `watchdog_ms` + 1틱 | 마지막 입력 틱 + `wd` + 1(6.2절) |
| 결함 정지 → 조향 출력 0 | ≤ 1틱 | 판정 틱(0틱, 6.7절) |
| 유지 정지의 조향 유지 오차 | ≤ max(1°, 2·데드밴드) | 다시 밀기 문턱 2·`db`(7.6절) |
| 자동 재출발 | 0 | 래치는 `ARM`으로만 풀리고 움직이려면 새 `C`가 필요하다(6.8절) |

P3 통합에서 잰 값(호스트 코드 + 펌웨어, T1, SIM 시계 하나, 20 ms 브리지 루프, 열린 루프 속도 stub. `sim/ubts/test_steering_host_firmware.py`, 측정 기록은 저장소 밖 `runs/wf2a/p3_integrate/a17_p3.json`). 자리값 기반 SIM 결과다. 플랜트·추정기·실행기를 넣은 A17(F) 판정은 P4다.

| 항목 | 잰 값 |
|---|---|
| STOP 요청 → `STOP` 송신 | 다음 브리지 단계의 첫 줄. 요청이 단계 1 ms 뒤에 오면 19 ms, 최악 20 ms |
| MCU `STOP` 파싱 → 구동 0 | 0틱(20 경우: seed 5개 × 조향 4개) |
| STOP 확인 | 첫 상태 줄이 파싱 뒤 ≤ 9틱, 호스트 확인이 `STOP` 송신 뒤 ≤ 20 ms. 재전송 0 |
| 확인 실패(MCU 800 ms 멈춤) | 재전송 5회(t0 + 100, 160, 200, 260, 300 ms: 50 ms 격자를 20 ms 루프로 올림), 래치 t0 + 400 ms |
| `C` TTL 만료 → 구동 0 | 받은 틱 + ttl(300)틱. ARMED 유지, `F TTL` |
| 링크 watchdog → 구동 0 | `wd` 200: 마지막 입력 + 201틱에 구동 0과 DISARMED, WATCHDOG 비트, 호스트 래치 `mcu_disarmed`(38 ms 뒤). `wd` 500(기본): TTL이 먼저 세우고(+300), DISARMED는 + 501틱 |
| 결함 정지 → 조향 출력 0 | `pot_open` 주입: 판정 틱에 조향 EN 0·PWM 0, 구동 0 |
| 유지 정지의 조향 유지 오차 | MCU 측정 \|tgt − pos\| 최대 0.75°, 참값 바퀴각 변화 최대 0.34°(기준 1°) |
| 자동 재출발 | 0(STOP 뒤 호스트의 `C`·`ARM` 0, 운영자 재무장과 새 drive가 있어야 움직임) |

### 11.5 호스트가 기대는 MCU 시간

- 부팅 뒤 배너: 리셋 틱 + `boot_delay`(실기 [자리값] 약 70 ms, B9) + 배너 전송(약 15 ms).
- 상태 줄: P_st마다(기본 50 ms), 위상은 부팅 틱에 묶임(5.4절). 호스트의 `host_status_gap_ms` 150 ms = 3주기.
- 명령 적용 확인: 받은 `C`는 그 틱에 적용되고, 다음 상태 줄(≤ P_st 뒤)의 `q`에 나온다.
- 구동 출발: 0에서 출발하면 마지막 0 뒤 120 ms 휴지, 그 뒤 255/s 램프(8.2절).

## 12. 결정과 자리값 (K)

### 12.1 설계가 열어 둔 곳에서 정한 것(결정 기록 제안)

1. 줄 내용은 최대 63 B다(CR·LF 제외). 64번째 내용 바이트에서 과길이다.
2. 틀 오류 줄은 그 줄을 끝내는 LF에서 `N - long` 또는 `N - ctrl`을 한 번 낸다.
3. 부팅 때 파서는 정상 상태로 시작한다. 리셋 전 반쪽 줄은 보통 줄로 처리한다.
4. 빈 줄은 조용히 무시한다.
5. 모든 MCU 줄에 `*HH`를 붙인다(배너 포함). MCU가 내는 16진은 대문자다. 입력 16진은 대소문자를 모두 받는다. 호스트는 대문자로 보낸다.
6. `STOP`은 CRC 두 자리를 보지 않는다(`STOP*xx`도 실행). `PING`, `INFO`, `D<n>`은 CRC가 선택이고, 있으면 검사한다.
7. 나쁜 줄 = 틀 오류 + CRC 오류. 좋은 줄이 계수를 0으로 돌린다. `crcn` 연속이면 CRC 결함이다.
8. ARMED·DISARMED에서 나쁜 줄 하나는 출력을 바꾸지 않는다. 잃은 프레임과 같고, TTL과 연속 계수가 지킨다. BENCH의 틀 오류는 V2처럼 구동을 즉시 멈춘다.
9. q 수용 창은 d = 1..32767이다. `last_q`는 부팅 때만 0이 된다. 그래서 STOP 전의 늦은 `C`는 재무장 뒤에도 거절된다. 재전송도 새 q를 쓴다.
10. \|S\| ≤ 1000이면 받는다. 소프트 한계는 cmd만 자른다. 상태 줄 tgt는 받은 S다.
11. 받은 `ARM`이 래치를 모두 푼다. `ARM`은 활성 조건(`nocal`, `cal`, `pol`, 가변저항 범위 밖, THERMAL 누적)이 있으면 거절한다. `BENCH`는 래치를 풀지 않고, 래치가 있어도 들어간다.
12. 정지 모드 표의 "출력 0"은 OFF(조향 EN 0)다. MCU 결함 정지는 모두 그 틱에 OFF다. 예외: 나쁜 줄 연속과 watchdog은 측정 위치를 유지한 뒤 OFF(링크 문제라 위치는 믿을 수 있다).
13. 데드밴드 안 제동 유지(`dwell`)에 히스테리시스를 둔다: 제동 유지 중에는 \|e\| > 2·`db`에서만 다시 민다.
14. MCU 목표 속도 제한 기본값은 0.8 × 프로필 구동기 속도다(설계 6.2의 0.8배 규칙을 빌림). ‰ 기준 눈금은 1° ≈ 40 ‰다.
15. "움직이는 중"은 적용 duty가 지금 0이 아니거나 최근 `movms`(300) ms 안에 0이 아니었던 경우다(설계 4.4).
16. watchdog은 ARMED·BENCH에서 움직임과 상관없이 동작한다. 수신 처리 전에 판정한다(늦은 PING 무효). 조건은 엄격히 `>`다.
17. TTL은 수신 처리 뒤에 판정한다. 만료 틱에 도착한 새 `C`가 이어 받는다.
18. 구동 EN은 D1 하나다(자리값, Q5). 부팅 뒤 늘 HIGH다. 펌웨어는 coast를 만들지 않는다. coast는 리셋·전원 끊김에서만 생긴다.
19. 조향 핀은 D6(`mdir` +1에서 + 쪽), D7, EN D2다. 조향 PWM은 구동과 같은 전역 설정(0..255, 1 kHz)이다.
20. TX 고리 512 B를 두고 줄을 통째로만 넣는다. 자리가 없으면 그 줄을 버린다(`drop`). 상태 줄 n은 버려도 오른다.
21. 상태 줄 위상은 b + k·P_st(k ≥ 1), 서보 틱은 `svhz`가 그대로일 때 b + k·P_sv(k ≥ 0)다(실행 중 바꾸면 4.10절). 서보 시계는 조향 모드와 상관없이 돈다.
22. 성공한 `ARM`, `DISARM`, `BENCH`, `STOP`, `PING`, `C`에는 응답 줄이 없다(상태 줄로 확인). `CFG`, `CAL JOG`, `CAL POL`에는 `A`, `CAL`에는 보정 보고 줄, BENCH의 `D<n>`에는 `P`를 낸다.
23. BENCH의 `D1`은 확인 줄만 낸다. `D2`, `D4`, `D6`, `D7`은 `pin`이다. V2의 램프 중간 핀 보고 줄은 없애고 상태 줄로 대신한다.
24. 보정 작업본 W와 저장본 R을 나눈다. `ARM`은 W의 보정 칸이 R과 같기를 요구한다. 변수만 다른 것(`cfgdirty`)은 MCU의 `ARM`을 막지 않는다(B5 서보 조정용, 벤치 도구). 실차 경로의 호스트는 `cfgdirty=1`이면 arm을 거절한다(74).
25. EEPROM은 256 B, 레코드 106 B, CRC-32/ISO-HDLC다. 지움 표시 `USCX`로 cal_id 재사용을 막는다. 부팅마다 commit 8회까지다.
26. SIM에서 EEPROM commit은 0틱이다(실기 정지 시간은 B9에서 잰다).
27. `CAL POL` 절차와 상수(PWM 153, 구동 300 ms, 정착 200 ms, 최소 4 count)를 정했다.
28. JOG에는 소프트 한계 막기와 `mdir`를 쓰지 않는다(바퀴를 띄운 벤치, 500 ms 이하).
29. 결함 창은 틱 단위로 정확히 잰다(위치 1024칸 고리). POT_OPEN·SHORT 조건이 참인 틱에는 POT_JUMP·NOISE를 보지 않는다.
30. POT_NOISE는 raw로, POT_JUMP는 f로 본다. 분산 비교는 64비트 정수로 한다.
31. THERMAL 창은 비트 고리로 정확히 잰다(최대 60 s). 핀 값 \|D6 − D7\| 기준이라 JOG도 센다.
32. 가변저항 감시는 모든 상태에서 돈다. 움직임 감시(POLARITY, DRV_STUCK, STALL, TRACK)는 ACTIVE·HOLD에서만 돈다.
33. DRV_STUCK은 BRAKE(EN 1, PWM 0)에서 시작한 관찰로 본다(70이 관찰 규칙을 넓힌다). OFF에서는 외력 움직임과 구분할 수 없다.
34. 상태 줄 `sp`는 핀 기준(D6 − D7)이다.
35. pos·tgt·cmd는 ±9999, wd는 9999로 자른다. 그래서 상태 줄 최악이 85 B다.
36. BOOT 상태는 줄에 나오지 않는다. 부팅 뒤 BOOT 비트(0x4000)가 켜지고 받은 `ARM`이 지운다.
37. HOLD·JOG·POL 동안 `CAL`(GET 밖), `CFG` 설정, `BENCH`는 `busy`다. `ARM`은 HOLD 중에도 받는다.
38. `CFG` 키는 41개다(프로필 D·D2표의 MCU 값 + `movms`, `svhz`). 모두 레코드에 저장한다. 설정은 RAM에만, `CAL SAVE`가 저장한다.
39. boot_id: 실기는 `ESP.random()`, SIM은 splitmix64 흐름(RESET에서 `boot_seed`, MCU 리셋을 넘어 이어짐)이다.
40. 리셋 원인은 `rst_info.reason` 숫자로 정한다.
41. 설계 4.1의 CRC 예 값 4개(7d, 1c, 4a, 5C)는 틀렸다. 이 문서의 값(A7, 88, 8E, CB)을 쓴다.
42. firmware 모드 SIM은 `mcu_tick_hz` 1000만 받고 스텝 dt ≤ 0.1 s다(lockstep 규약 11절).
43. `pot_filter.sample_hz` 2000은 쓰지 않는다(틱마다 표본 하나).
44. 부팅 때 R이 없으면 `F NOCAL`을 내지 않는다(배너 `cal=none`). `CAL ERASE` 때 `F NOCAL 0`을 낸다.
45. SOFTLIMIT, BOOT에는 `F` 줄이 없다. WATCHDOG은 `F` 대신 `W`를 낸다.
46. `DISARM`은 DISARMED에서도 받는다(JOG·POL만 끝낸다).
47. 코어 RX 버퍼는 256 B, 넘치면 새 바이트를 버린다고 본다. → 67이 바꾼다(코어 소스: 255 B, 옛 바이트를 버림).

P3 통합(2026-11-09)에서 정한 것. 규격 문장이 두 가지로 읽히던 곳은 안전한 쪽을 골랐다. 펌웨어·대본·호스트를 이에 맞췄다.

48. BENCH의 틀 오류는 오류를 처음 알아챈 바이트를 읽은 틱에 구동을 멈춘다(V2 `stopDirections`와 같다). `N` 줄과 나쁜 줄 계수는 LF 틱이다. 정지가 LF를 기다리지 않는다. 펌웨어를 고쳤고 대본 `bench_framing_error_early_brake`를 더했다(옛 빌드에서는 실패한다).
49. THERMAL 창은 부팅 전 틱을 출력 0으로 센다. 그래서 부팅 직후에도 판정한다(6.7절 "n − b < 창이면 판정하지 않음"의 예외).
50. 결함 창의 틱은 처리한 틱이다(`loop()`가 새 `millis()` 값으로 돈 틱). 루프가 멈춘 동안의 시간은 창에 넣지 않는다.
51. 64번째 내용 바이트가 0x20..0x7E 밖이면 `ctrl`이다(바이트 검사가 길이 검사보다 먼저). CR은 내용이 아니므로 내용 63 B + CR LF는 정상 줄이다.
52. `STOP` 특례 줄: `STOP`과 CRC가 맞는 `STOP*HH`는 좋은 줄이다(`bad_run` = 0). CRC가 틀린 `STOP*xx`는 STOP을 실행하지만 좋은 줄도 나쁜 줄도 아니다(`bad_run` 그대로). 틀린 CRC가 연속 계수를 지우지 않는 쪽이 안전하다. 대본 `crc_burst_stop_special`.
53. CRC 필드가 없는 줄도 4.2절의 "틀·CRC 검사를 통과한 줄"이다. 그래서 `nocrc`로 거절한 `C`도 둘째 토큰이 0..65535이면 `N <q> nocrc`다. 대본 `c_nocrc_shows_q`.
54. 받은 `CAL ERASE`는 늘 `F NOCAL 0 <ms>`와 보정 보고 줄로 답한다. 명령의 응답이라 NOCAL 비트가 이미 켜져 있어도 낸다. 5.6절의 "한 번"은 결함 판정 줄에만 쓴다. `wear` 검사는 R이 없어도 한다(4.2절 순서). R이 없으면 쓰지 않고 commit도 세지 않는다. 대본 `cal_erase_twice`.
55. 상태 검사(4.2절 6단계)에서 `CAL`은 둘째 토큰이 `GET`일 때만, `CFG`는 토큰이 2개(조회)일 때만 모든 상태에서 받는다. 그 밖의 `CAL`·`CFG` 줄은 DISARMED가 아니면 문법 검사 전에 `state`다.
56. `boot_count`와 boot_id 차례는 `setup()`을 부른 수로 센다(lockstep 11.5절). 부팅 지연 안에 다시 리셋되면 두 리셋에 부팅은 하나다.
57. 실기에서 `CAL SAVE`의 commit이 실패하면 R을 바꾸지 않는다. 응답은 보정 보고 줄이다(실패 줄은 없다). 호스트는 cal_id가 오르지 않은 것으로 안다.
58. `setup()`은 8.3절 핀 순서 뒤에 V2처럼 Wi-Fi를 끄고 UART와 EEPROM을 연다.
59. 늦은 `C`(4.8절): 이미 받은 q 이하(창 밖)만 재무장 뒤에도 거절된다. 처음 보는 q는 재무장 뒤에 받는다. 직렬선은 순서를 지키므로 STOP 전에 보낸 `C`는 STOP보다 먼저 도착한다. 호스트는 STOP 뒤에는 새 ARM 확인 전까지 `C`를 보내지 않고, 한 부팅 안에서 q를 다시 쓰지 않는다(재전송도 새 q). STOP 직후 늦은 drive 요청은 호스트의 STOP 장벽(`stop_barrier`)이 막는다.
60. 호스트는 lease가 있거나 ARM 확인을 기다리는 동안 `INFO`와 `CAL GET`을 보내지 않는다. 그 답(303 B, 129 B)이 TX 고리에 쌓여 STOP 확인 상태 줄을 늦출 수 있다(11.4절). 배너 직후의 보정 줄은 보통 바로 따라오므로 `CAL GET`은 배너 뒤 0.1 s 안에 보정 줄이 없을 때만 보낸다.
61. 포트가 열린 채 MCU가 리셋되면 새 배너가 리셋 전의 반쪽 줄이나 LF 없는 부트 쓰레기와 붙어 호스트에서 깨질 수 있다(T1 통합 시험에서 실제로 생겼다). 호스트는 상태 줄 `ms` 역행이나 이어지지 않는 `n`을 보면 `mcu_reset_detected` 래치, 정지, 그리고 배너가 올 때까지 `INFO`를 보낸다(`banner_timeout_s`마다, 무장 중이 아닐 때). 배너 전에는 ARM을 `mcu_reset_detected`로 거절한다. 예전 호스트는 옛 boot_id로 ARM을 보내 `N - boot`를 받았고, 링크를 다시 열기 전에는 재무장할 수 없었다.
62. (77이 예외를 더한다) 호스트의 STOP 재전송은 첫 STOP 시각 t0에 묶인 고정 격자다. k번째(k = 0..4) 재전송은 t0 + 100 + 50·k ms, 래치는 마지막 예정 재전송 뒤 100 ms(t0 + 400 ms)다. 20 ms 브리지 루프는 각 시각을 다음 단계로 올리지만 격자를 늘리지 않는다(예전 구현은 앞 재전송 시각에서 다시 재서 60 ms 간격, 래치 t0 + 440 ms였다). STOP 확인은 st 1, L = R = 0인 상태 줄이다. STOP을 쓴 단계에서 읽은 줄도 확인으로 쓴다. 그 줄은 STOP보다 먼저 MCU를 떠났지만, DISARMED·구동 0은 STOP이 만드는 상태이고 호스트가 확인 전에는 ARM을 보내지 않으므로 그 상태는 바뀌지 않는다.
63. POT_NOISE는 raw 창의 분산이다(30번). 가변저항이 끊기거나 붙으면(raw 계단이 약 36 count 넘음) POT_NOISE가 그 틱에 켜지고 POT_OPEN·POT_SHORT는 중앙값(`med` 5)이 따라가는 2틱 뒤에 켜진다. 둘 다 정지한다. 코드 순서는 진단용이다. 조향 램프도 분산을 키운다: 창 안 변화가 Δ = rate·g·`nsw`/1000(g는 count/°)이면 분산은 약 Δ²/12다. 자리값(g 3.87 count/°, 30°/s, 50 ms)에서 Δ ≈ 5.8, 분산 ≈ 2.8로 문턱 25 아래다. g가 큰 가변저항(예: 13 count/°)이면 30°/s에서 문턱에 닿는다. 가변저항 사양(Q2, B7)이 오면 `nsd`를 다시 정한다(12.2절 P17).
64. 보정 묶음: 호스트의 steer_mrad ↔ ‰ 표는 호스트 프로필 JSON(`ubts.steer_host_profile.v1`)의 `calibration` 절이다(`cal_id`, `mark_left_mrad`, `mark_right_mrad`, `permille_per_deg`, `expected_soft_permille`). SIM은 lockstep 11.11절 표시(등가 기계 한계 − 1°)를 쓰고, EEPROM 이미지(`fw_eeprom.calibrated_eeprom`)와 같은 cal_id·soft·눈금이다(`test_steering_host_firmware.py`가 맞춰 본다). 현장에서는 B2·B3 측정 표시로 바꾼다.
65. firmware 모드는 `mcu_tick_hz` 1000만 받는다(42번). 그래서 P8 스윕은 firmware 모드에서 `sim_mcu_tick_hz` 2000과 `pot_filter.sample_hz` 2000을 쓰지 않는다.

P3 검토 반영(2026-11-09, 고침 작업)에서 정한 것. 검토가 확인한 결함마다 하나다. 결정 기록에 오르기 전에는 제안이다.

66. 송신 펌프(7.1절 12단계)는 한 번 쓰는 양을 `availableForWrite()` − 1 이하로 하고, 그 값이 1 이하이면 돌아간다. ESP8266 코어 3.1.2는 `availableForWrite()` = 128 − 찬 수이고 찬 수 ≥ 127에서 `Serial.write`가 기다린다. 핀 쓰기(11단계)를 송신(12단계)보다 앞에 둔다. 예전 순서·양에서는 부팅 배너 뒤 약 13 ms, ARMED의 INFO + STOP 틱에 약 17 ms, 짧은 쓰레기 줄 29개 + STOP에 약 21 ms 동안 `loop()`가 막혀 STOP의 핀 쓰기도 그만큼 늦었다(검토의 코어 규칙 모형 결과, 실기 측정 아님). SIM lockstep의 송신 바이트 시각은 바뀌지 않는다(16384 − 남은 수라 한 틱에 다 옮긴다).
67. 코어 RX 고리는 256 B 배열이고 255 B까지 담는다. 넘치면 가장 옛 바이트를 버린다(코어 3.1.2 `uart.cpp`: `UART_DISCARD_NEWEST` 정의 없음, "discard oldest data"). 47을 바꾼다. 흉내 층도 그렇게 한다. 하드웨어 RX FIFO가 더 담는 몫은 흉내 내지 않는다[가정]. 대본 `rx_overflow_drops_oldest`.
68. THERMAL 조건(창 안 누적 > `tha`)과 가변저항 범위 조건은 ARM처럼 `CAL JOG`, `CAL POL`도 `fault`로 거절한다(검사 순서는 4.2절 표의 끝). THERMAL 래치가 이미 있고 그 조건이 다시 참이 되면(누적이 줄어 받은 JOG가 다시 쌓은 경우) 매 틱 조향 출력을 끈다. F 줄은 처음 켜질 때만 낸다. 예전에는 래치 뒤의 JOG를 받아 100 % 출력이 이어졌다. 대본 `thermal_blocks_jog_and_pol`.
69. runTrack은 \|cmd − P\| > `trk`이고 위치가 최근 `stt` 틱 동안 cmd 쪽으로 `stm`보다 많이 움직이지 않은 틱만 센다(6.7절). 따라가는 서보의 뒤처짐(자리값 P 서보로 속도 제한에서 약 6°)은 TRACK이 아니다. 멈춘 조향(지금 대본 `fault_track`의 TRACK 1272 그대로), 느리게 움직이는 조향(`stm`/`stt`보다 느림, 기본 약 1°/s), 반대로 가는 조향은 센다. 예전에는 결함 없는 정지 중 12.5° 넘는 계단과 주행 중 전폭 스윕이 TRACK으로 DISARM되었다(A19 오검출). 대본 `servo_lag_with_progress_no_track`. 남은 것: 더 나쁜 조건(정지한 차의 duty가 데드밴드 안이라 MCU는 주행 중 속도 제한을 쓰는데 구동기는 정지 중 속도만 내는 경우)의 큰 스윕은 TRACK이 아니라 THERMAL 프록시가 오래 이어진 100 % 출력을 세어 끊을 수 있다(자리값 관찰, 7.5 s 안에 전폭 스윕 세 번에서 THERMAL). P4에서 오검출률을 잰다(A19 보고).
70. DRV_STUCK 관찰(runStuck, 6.7절): BRAKE 틱에서 시작해 BRAKE 동안 `db`보다 더 움직였으면 다시 미는 동안에도 이어 센다. 미는 쪽으로 기준을 `db`보다 넘게 지나면 끝난다. 새 목표가 밀게 한 경우(기준에서 `db` 안)는 곧바로 끝나 POLARITY와 겹치지 않는다. 기본 변수(db 20, dsm 80)에서 예전 규칙은 다시 미는 순간 계수가 0이 되어 DRV_STUCK을 낼 수 없었다(A19). 자리값 모형의 고착 드라이버(pwm 0.6, 1.0, 주행 중 0.45)는 이제 DRV_STUCK이다. 정지 중 0.6 음의 방향은 POLARITY가 먼저다. 느린 고착(정지 중 0.45 이하)은 TRACK이 늦게 본다. 대본 `fault_drv_stuck_default`.
71. POLARITY·DRV_STUCK 래치가 있으면 ARM은 `pol`이다. 통과한 `CAL POL`이 두 래치를 지운다(극성이 바뀌어 mdir가 R과 다르면 `CAL SAVE` 전에는 `cal`이다). 리셋·전원 껐다 켬은 RAM과 함께 지운다. 다른 래치는 지금처럼 ARM이 푼다. 예전(11번)에는 ARM 한 번으로 풀려 배선이 반대인 차가 재무장마다 반대로 조향했다. 대본 `polarity_latch_needs_pol_check`. 호스트는 상태 줄 `fl`에 두 비트가 있으면 arm을 `polarity_check_missing`으로 미리 거절한다.
72. 결함 창의 "과거 값이 있는가" 검사는 처리한 틱 수를 포화하는 계수로 센다(틱 번호는 2^32에서 감긴다). THERMAL 비트 고리의 색인은 틱 번호와 따로 60,000에서 감긴다. 예전에는 49.7일 연속 가동 뒤 감길 때 THERMAL 계수에 영구 오프셋이 남아 ARM이 막힐 수 있었다(C++ 단위 시험 `test_thermal_window_across_tick_wrap`).
73. `CAL ERASE`의 commit이 실패하면(실기만) R, W, NOCAL 비트를 그대로 두고 `F NOCAL`을 내지 않는다. 응답은 보정 보고 줄뿐이다(57의 SAVE 규칙과 같다. C++ 단위 시험 `test_eeprom_commit_failure`).
74. 호스트(실차 경로와 SIM 래퍼 모두)는 보정 줄 `cfgdirty=1`이면 arm을 `config_unsaved`로 거절한다. MCU는 지금처럼 받는다(24, 벤치). 브리지는 `CFG`를 보내지 않고 포트를 혼자 쓰므로 세션 시작의 보정 줄이 그 세션 RAM 값의 상태다. 예전에는 저장하지 않은 `CFG`로 watchdog·가변저항 문턱을 바꾼 채 arm이 되었다.
75. 호스트 래치는 운영자 재무장의 ARM이 MCU에서 확인될 때(첫 `st` = 2 상태 줄) 지운다. ARM의 `N` 거절, 확인 없음(`arm.confirm_ms`), 쓰기 실패는 래치를 그대로 두고 `arm_rejected`·`arm_unconfirmed`(쓰기 실패는 `hardware_fault`)를 더한다. 그래서 state가 `fault`로 남는다. 예전에는 ARM을 보내기 전에 지워, 거절된 재무장 뒤 MCU 결함이 남은 채 state가 `disarmed`가 되었다.
76. ARM에 온 `N - boot`는 MCU 리셋이다. 호스트는 `mcu_reset_detected` 래치, 정지, 그리고 배너가 올 때까지 `INFO`를 보낸다(61과 같다). 배너와 첫 상태 줄 사이에 리셋되어 새 배너를 잃으면 상태 줄 검사로는 알 수 없기 때문이다(앞 상태 줄이 없다). 예전에는 재무장마다 옛 boot_id로 `N - boot`를 받았다.
77. 62의 예외: ARM 확인을 기다리는 동안 STOP을 쓰면, 그 STOP은 마지막으로 읽은 상태 줄 번호 n + 2 이상인 줄(두 상태 주기 뒤에 만든 줄)만 확인한다. ARM이 아직 MCU에 닿기 전에 만든 줄도 DISARMED·구동 0이라 STOP 확인처럼 보이기 때문이다. 새 부팅의 줄(ms 역행, n 재시작)은 바로 확인으로 쓴다. 확인은 보통 2주기(100 ms) 안이고, 늦으면 격자대로 한 번 더 보낸다.

### 12.2 사양·실측에 달린 값(자리값)

| 번호 | 값 | 바꿀 근거 |
|---|---|---|
| P1 | 구동 EN을 D1 하나로 묶음, EN 선마다 풀다운 | Q5 배선 확인 |
| P2 | 조향 드라이버 = PWM 두 입력 + EN, 1 kHz 허용 | Q11 부품 사양 |
| P3 | D6이 왼쪽(+)으로 미는지(`mdir`) | B0 극성 확인 |
| P4 | ‰ 눈금(1° ≈ 40 ‰)과 그것으로 바꾼 `db`, `polp`, `stm`, `trk`, `dsm`, `kp` | B2·B3 표시 각도 |
| P5 | `kp` 15 %/°, `minp` 40 %, `maxp` 100 %, `dwell` 200 ms, `rev` 20 ms, `svhz` 500 | B4, B5 |
| P6 | `rmov` 960 ‰/s, `rstd` 480 ‰/s | B5, F4, F5, Q2, Q11 |
| P7 | `ttlmax` 300, `wd` 500, `hloss`·`hstop`·`hdis` 2000 ms, `wdmode` hold, `stoppol` hold_measured | B8, B9, Q6 |
| P8 | `pvmin` 20, `pvmax` 1003, `cwmin` 60, `cwmax` 963 | A0 분압기·단선 동작(B8) |
| P9 | `jmp` 8 / 2 ms, `nsd` 5.0 / 50 ms | B7 |
| P10 | POLARITY 1° / 0.1 s, STALL 60 % / 0.5 s / 0.5°, TRACK 3° / 1.0 s, DRV_STUCK 2° / 0.3 s, THERMAL 80 % / 30 s / 5 s, `crcn` 5 | B8, B9, Q11, Q13 |
| P11 | `CAL POL` 상수, 표시 반쪽 폭 최소 50 count | B0, Q2(가변저항 범위·기어비) |
| P12 | 실기 boot 지연 약 70 ms, 리셋 원인 대응, boot_id 난수 품질 | B9 |
| P13 | UART 송신 FIFO 대기 동작(코어 3.1.2 소스: 128 − 찬 수, 찬 수 127에서 대기. 12.1-66), RX 고리 255 B·옛 바이트 버림(12.1-67), baud 115200 | B9(실기 측정) |
| P14 | EEPROM commit 시간, 쓰기 한도 8회 | B9, 부품 |
| P15 | `movms` 300 ms | 설계 4.4 값. F3 뒤 확인 |
| P16 | 하드웨어 프로필 이름 `front_steer_v0` | 사양 확정 때 |
| P17 | `nsd`와 가변저항 이득 g(count/°)·최대 조향 속도의 관계: 램프 분산 (rate·g·`nsw`/1000)²/12이 `nsd`²보다 넉넉히 작아야 한다(12.1-63) | Q2, B7 |
| P18 | 호스트 `CAL GET` 유예 0.1 s, STOP 재전송 격자 50 ms·5회, 래치 t0 + 400 ms(12.1-60, 62) | B9 |
| P19 | TRACK 진행 판정에 STALL 값(`stm`, `stt`)을 쓰는 것, DRV_STUCK 관찰 규칙(12.1-69, 70). 실측 조향 속도·서보 이득(B4, B5)이 오면 `trk`·`kp`·속도 제한을 함께 다시 정한다 | B4, B5, B8 |

- 참고: 기본 `kp`·`minp`와 `rmov`·`rstd`에서 긴 조향 이동은 서보 뒤처짐이 TRACK 문턱 3°를 넘는다(2026-11-09 검토에서 정지 중 12.5° 계단, 주행 중 전폭 스윕이 TRACK으로 섰다). 12.1-69로 따라가는 서보는 세지 않는다. P4에서 오검출률을 잰다(A19 보고 항목).

## 13. 적합성 대본 형식 `ubts.steer_conformance.v1` (M)

### 13.1 파일

- 위치: `sim/ubts/steering/conformance/scripts/<name>.json`(적합성 담당이 만든다). UTF-8, BOM 없음, LF.
- 대본 하나 = JSON 객체 하나. 모르는 키는 실행기가 거부한다.
- 기대값은 이 규격에서 손으로 정한다. 다른 구현의 출력을 정답으로 쓰지 않는다(R1).

### 13.2 맨 위 키

| 키 | 형 | 필수 | 뜻 |
|---|---|---|---|
| `schema` | 문자열 | 예 | `ubts.steer_conformance.v1` |
| `name` | 문자열 | 예 | `[a-z0-9_]{1,64}`. 파일 이름과 같다 |
| `spec` | 문자열 배열 | 예 | 이 대본이 검사하는 규격 절 번호(예: `["6.3", "8.2"]`) |
| `v2_map` | 문자열 | 아니오 | 대응하는 V2 시나리오 이름(14절) |
| `description` | 문자열 | 아니오 | 설명 |
| `setup` | 객체 | 예 | 13.3절 |
| `plant` | 객체 | 아니오 | 13.4절 |
| `inputs` | 배열 | 예(빈 배열 가능) | 13.5절 |
| `events` | 배열 | 아니오 | 13.6절 |
| `expect` | 배열 | 예 | 13.7절 |
| `end` | 객체 | 예 | 13.8절 |

### 13.3 `setup`

| 키 | 형 | 기본 | 뜻 |
|---|---|---|---|
| `mcu_tick_hz` | 정수 | 1000 | 1000만 된다 |
| `boot_seed` | 정수 0..2^64−1 | 필수 | SIM `ESP.random()` 흐름(10.1절) |
| `chip_id` | 16진 6자리 문자열 | 필수 | 배너 `dev` |
| `boot_delay_ticks` | 정수 0..1000 | 0 | 리셋 뒤 `setup()`까지 틱 수 |
| `millis_start` | 정수 0..4294967295 | 0 | 리셋 순간의 `millis()`(감김 시험용) |
| `eeprom` | `"blank"` 또는 객체 | `"blank"` | `{"record": R}` 또는 `{"hex": "<512자>"}`. R = `{"cal_id", "raw_l", "raw_c", "raw_r", "soft_l", "soft_r", "pol_ok", "mdir", "params": {키: 값}, "magic": "USC3"\|"USCX"}`. `params`에 없는 키는 기본값. 실행기가 9.2절 배치로 256 B 이미지를 만든다(뒤는 0xFF) |
| `adc` | 객체 | `{"mode": "script", "raw": 512}` | `{"mode": "script", "raw": n}`(ADC가 대본 값을 돌려준다) 또는 `{"mode": "model"}`(가변저항 모형) |
| `report_adc` | 불 | false | 참이면 lockstep 변화 목록에 `adc_read`를 싣는다 |
| `initial_delta_eq_deg` | 수 | 0 | 구동기 모형의 처음 바퀴 쪽 등가각 |
| `profile` | 문자열 | `front_steer_v0_placeholder` | 구동기·가변저항 모형 값의 출처 |
| `seed` | 정수 | 0 | lockstep 시작 줄 seed(잡음 흐름) |
| `startup_overrides` | 객체 | `{}` | lockstep 시작 줄 객체에 JSON merge patch(RFC 7386)로 덧씌운다. 예: `{"pot": {"noise_sd_counts": 0.0}}` |
| `injections` | 배열 | `[]` | `{"kind", "at_tick", "params"}`. lockstep v1 주입(at_s = at_tick / 1000) |
| `motor_power` | 불 | true | 조향 모터 급전 처음 상태 |
| `boot_garbage_hex` | 문자열 | `""` | 리셋 때 SIM이 내는 부트 롬 쓰레기 바이트(16진) |

### 13.4 `plant`

| 키 | 형 | 기본 | 뜻 |
|---|---|---|---|
| `speed_m_s` | `[[tick, 값], …]` | `[[0, 0.0]]` | 차속(조각 상수, 그 틱부터) |
| `stopped` | `[[tick, 불], …]` | 차속 = 0이면 참 | 정지 여부(명시하면 이 값) |
| `supply_scale` | `[[tick, 값], …]` | `[[0, 1.0]]` | 가변저항 급전 배율 |

적합성 실행은 개방 루프다. MCU의 구동 duty가 차속을 바꾸지 않는다. 차속은 이 표를 따른다.

### 13.5 `inputs`

- 항목: `{"tick": t, "text": "…"}` 또는 `{"tick": t, "hex": "…"}`, 선택 키 `"pace": "instant"`(기본) 또는 `"uart"`. tick 오름차순(같으면 적힌 순서).
- `text`는 ASCII JSON 문자열이다(`\n`, `\r`, `\u0000` 이스케이프 가능). 0x80 이상은 `hex`로 쓴다.
- 매크로(`text`만): `{crc}`은 바로 앞이 `*`이어야 한다. 그 줄 첫 바이트(앞 LF 다음 또는 문자열 처음)부터 `*` 앞까지의 CRC-8 대문자 두 자리로 바꾼다. `{boot}`은 틱 t에 살아 있는 부팅의 boot_id(대문자 8자리)로 바꾼다. T1(네이티브)에서는 실행기가 10.1절 규칙과 그때까지의 부팅 수로 계산한다. 실기에서는 그때까지 받은 마지막 배너에서 읽는다.
- `instant`: 모든 바이트가 틱 t의 `loop()` 4단계 전에 RX 버퍼에 들어 있다(고리는 255 B까지 담고 넘치면 가장 옛 바이트를 버린다, 12.1-67). `uart`: lockstep 규약 11.7절의 RX 시각 규칙(지연 0)으로 틱 t부터 바이트마다 도착 틱을 정한다. 앞 `uart` 입력이 아직 전송 중이면 그 뒤에 이어 붙인다.

### 13.6 `events`

| `kind` | 추가 키 | 뜻(lockstep 규약 11.4절) |
|---|---|---|
| `mcu_reset` | `reason` 1..6 | 리셋(RAM 잃음, EEPROM 유지) |
| `power_cycle` | `off_ticks` ≥ 1 | 전원 끔 뒤 켬(EEPROM 유지, reason 0) |
| `port_open_reset` | 없음 | 포트 열기 리셋(reason 6) |
| `motor_power` | `on` 불 | 조향 모터 급전 켬·끔 |
| `mcu_stall` | `ticks` ≥ 1 | 그 틱 수만큼 `loop()`를 부르지 않음 |
| `adc_set` | `raw` 0..1023 | 그 틱부터 ADC 대본 값 |
| `adc_model` | 없음 | 그 틱부터 가변저항 모형 값 |

항목은 `{"tick": t, "kind": …, …}`다. 같은 틱의 사건은 적힌 순서로, 그 틱의 입력 바이트보다 먼저 적용한다.

### 13.7 `expect`

모든 기대에는 선택 키 `id`(보고용), `spec`(절 번호), `tol`(13.9절)이 있다.

| `type` | 키 | 합격 조건 |
|---|---|---|
| `line` | `text`, `tick` 또는 `window`, `at`(`"queued"` 기본 / `"done"`) | 패턴과 맞고 그 틱(±tol) 또는 창 [t0, t1](양끝 포함)에 있는 MCU 줄이 정확히 하나 |
| `status` | `tick`, `fields` | 그 틱에 만든 상태 줄이 있고 각 칸이 맞음 |
| `no_line` | `window`, `kinds`(배열) 또는 `text` | 창 안에 그 종류(또는 패턴) 줄이 없음 |
| `count` | `window`, `kinds` 또는 `text`, `min`, `max` | 창 안 개수가 [min, max] |
| `pins` | `tick`, 핀 칸(아래) | 그 틱 끝(11단계 뒤)의 값이 맞음 |
| `pins_window` | `window`, 핀 칸 | 창 안 모든 틱에서 맞음 |
| `change` | `channel`, `tick`, `to` | 그 채널이 정확히 그 틱(±tol)에 `to`로 바뀜 |
| `first_change` | `channel`, `after`, `tick`, `to` | `after` 틱 이후 그 채널의 첫 변화가 그 틱(±tol)이고 값이 `to` |

- 줄 패턴: 문자 그대로 비교한다. `{crc}`(바로 앞 `*`)는 실제 줄의 두 자리가 그 줄의 CRC-8과 같은지 본다. `{boot}`은 그 줄 틱에 살아 있는 boot_id다. `{any}`는 공백과 `*`가 아닌 문자 하나 이상이다. 실행기는 기대와 상관없이 모든 MCU 줄의 CRC를 검사하고 틀리면 실패로 친다.
- 줄 종류 이름(`kinds`): `BANNER`, `CAL`, `S`, `N`, `F`, `W`, `A`, `P`, `OTHER`.
- `status`의 `fields` 키: `ms n st q L R en raw pos tgt cmd sp fl wd tl`(5.3절). 값 비교자: 정수(같음), `{"min": a, "max": b}`, `"any"`. `fl`은 16진 4자리 문자열(같음) 또는 `{"mask": "0800", "value": "0800"}`.
- 핀 칸: `drive_l`, `drive_r`, `drive_en`, `steer_pwm`, `steer_en`, `mcu_state`, `fault_bits`, `gpio_writes`, `eeprom_commits`(lockstep 규약 11.6절 채널). 비교자는 위와 같고 `"pos"`, `"neg"`, `"zero"`, `"nonzero"`(부호)도 된다. `mcu_state`는 0..3 또는 255(꺼짐·리셋 중). `fault_bits`는 `fl`과 같은 비교자다.
- `channel`: lockstep 채널 이름(`drive_l`, `drive_r`, `drive_en`, `steer_pwm`, `steer_en`, `mcu_state`, `fault_bits`, `gpio_writes`, `eeprom_commits`, `adc_read`).

### 13.8 `end`

| 키 | 기본 | 뜻 |
|---|---|---|
| `tick` | 필수 | 대본 끝 틱. 이 틱까지 입력·사건·기대를 본다 |
| `drain_ticks` | 300 | 끝 틱 뒤 입력 없이 더 돌려 끝 틱까지 만든(queued) 줄의 바이트를 다 받는다 |
| `unexpected` | `"fail"` | 어느 기대(`line`, `status`, `count`)에도 맞지 않은 줄이 있으면 실패. `"ignore"`면 보지 않음 |
| `ignore_kinds` | `["S"]` | `unexpected` 검사에서 빼는 종류 |
| `checks` | `[]` | 끝에서 볼 기대(13.7절 형식, 창 생략 시 대본 전체) |

### 13.9 틱 허용 오차

- 기대 틱은 기본으로 정확하다(`tol` 0). 줄의 틱은 `at`에 따라 만든 틱(queued, 기본) 또는 LF가 다 나간 틱(done)이다.
- `tol` 1은 결함 판정 시각 기대(`F` 줄, `W` 줄, `fault_bits` 변화)에만 쓸 수 있다(A3·A19의 ±1틱). 다른 기대에 `tol` > 0을 쓰거나 `tol` > 1을 쓰면 실행기가 대본을 거부한다.
- T1 네이티브 실행은 결정적이다. 같은 대본·같은 빌드면 줄 바이트, 틱, 채널 변화가 모두 같아야 한다. 스텝 나누기(예: 10틱씩, 1틱씩)를 바꿔도 결과가 같아야 한다(실행기가 시험한다).
- T2(pty, 벽시계)와 T3(실기 벤치) 실행기는 대본 밖 설정으로 전체 허용 오차를 더할 수 있다. 그 값은 결과에 따로 적는다. T1은 그 설정 없이 통과해야 한다.

### 13.10 실행기 규칙(요지)

- 실행기는 lockstep 규약 v2 firmware 모드로 하위 프로세스를 띄운다. 기본 스텝은 10틱이다. `plant` 표가 바뀌는 틱에서 스텝을 나눈다.
- 입력 바이트는 스텝의 RX 항목(틱 오프셋)으로, 사건은 사건 항목으로 넣는다. 응답의 TX 항목으로 줄을 만든다: 줄의 queued 틱 = 그 줄 첫 바이트의 `queued_tick`, done 틱 = LF의 `done_tick`.
- 결과 파일(저장소 밖): `ubts.steer_conformance_result.v1` JSON. 대본 이름·SHA-256, 실행 파일 SHA, build id, 기대마다 합격·실패와 실제 값, 맞지 않은 줄 목록.

### 13.11 예: `c_ttl_expiry_brake`

손으로 정한 기대(9.6절 레코드, ADC 512 고정, 부팅 틱 0):

- 틱 0: 배너, 보정 보고 줄. 상태 줄은 50, 100, …(n 0, 1, …).
- 틱 10: `ARM` → ARMED. 틱 10은 서보 틱(500 Hz, 짝수)이라 조향 EN이 그 틱에 1이 된다(e = 0이므로 BRAKE).
- 틱 20: `C 1 300 100 100 0` → 목표 100. 휴지 때문에 출력은 틱 120에 2로 시작한다(0 → 2 → 5 → 7 → 10 …, 8.2절).
- 틱 300: L = R = 48(갱신 19번 × 2550 / 1000). 틱 310: 51.
- 틱 320: TTL 만료(20 + 300) → brake, L = R = 0, TTL 비트, `F TTL 1 320`.
- 틱 350: 상태 줄 L 0, fl 0800, tl 0. watchdog은 틱 521이므로 대본(끝 400) 안에서는 없다.

```json
{
  "schema": "ubts.steer_conformance.v1",
  "name": "c_ttl_expiry_brake",
  "spec": ["4.8", "6.3", "8.2", "5.3"],
  "v2_map": "watchdog_active",
  "setup": {"boot_seed": 1, "chip_id": "5A1B2C",
            "eeprom": {"record": {"cal_id": 7, "raw_l": 842, "raw_c": 512, "raw_r": 181,
                                   "soft_l": 935, "soft_r": -935, "pol_ok": 1, "mdir": 1, "params": {}}},
            "adc": {"mode": "script", "raw": 512}},
  "inputs": [{"tick": 10, "text": "ARM {boot}*{crc}\n"},
             {"tick": 20, "text": "C 1 300 100 100 0*{crc}\n"}],
  "expect": [
    {"type": "line", "tick": 0, "text": "UBTS_STEER_V3 hello proto=3 build={any} dev=5A1B2C boot={boot} rst=power_on hw=front_steer_v0 out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0*{crc}"},
    {"type": "line", "tick": 0, "text": "UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 caldirty=0 cfgdirty=0*{crc}"},
    {"type": "change", "channel": "steer_en", "tick": 10, "to": 1},
    {"type": "change", "channel": "mcu_state", "tick": 10, "to": 2},
    {"type": "status", "tick": 50, "fields": {"ms": 50, "n": 0, "st": 2, "q": 1, "L": 0, "R": 0, "en": 3,
                                              "raw": 512, "pos": 0, "tgt": 0, "cmd": 0, "sp": 0, "fl": "0000",
                                              "wd": 30, "tl": 270}},
    {"type": "first_change", "channel": "drive_l", "after": 20, "tick": 120, "to": 2},
    {"type": "status", "tick": 150, "fields": {"L": 10, "R": 10, "wd": 130, "tl": 170}},
    {"type": "status", "tick": 300, "fields": {"ms": 300, "n": 5, "st": 2, "q": 1, "L": 48, "R": 48, "en": 3,
                                               "raw": 512, "pos": 0, "tgt": 0, "cmd": 0, "sp": 0, "fl": "0000",
                                               "wd": 280, "tl": 20}},
    {"type": "pins", "tick": 319, "drive_l": 51, "drive_r": 51, "drive_en": 1, "steer_en": 1, "steer_pwm": "zero"},
    {"type": "pins", "tick": 320, "drive_l": 0, "drive_r": 0, "drive_en": 1, "mcu_state": 2},
    {"type": "line", "tick": 320, "text": "F TTL 1 320*{crc}"},
    {"type": "change", "channel": "fault_bits", "tick": 320, "to": {"mask": "0800", "value": "0800"}},
    {"type": "status", "tick": 350, "fields": {"st": 2, "L": 0, "R": 0, "fl": "0800", "wd": 330, "tl": 0}},
    {"type": "no_line", "window": [0, 400], "kinds": ["N", "W", "A", "P"]}
  ],
  "end": {"tick": 400}
}
```

- 이 대본의 상태 줄 틱 300 완성 줄은 `S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*AA`, TTL 줄은 `F TTL 1 320*6B`다(5절 예와 같다).

## 14. V2 네이티브 시험 22개 → V3 대응 (설계 부록 C를 구체화)

| 번호 | V2 시나리오 | V3 대본에서 볼 것 | 상태 |
|---|---|---|---|
| 1 | `boot_order` | `setup()` 쓰기 순서(8.3절): PWM 범위·주파수 → 방향 핀 4개 LOW·0 → D6·D7 → D2 LOW → D1 HIGH. DISARMED 동안 조향 EN 0 | DISARMED |
| 2 | `mapping_and_full_range` | BENCH `D8/D5/D3/D0` 값 1, 3, 64, 128, 191, 252, 255가 목표·적용 값 그대로(PING 200 ms마다). ARMED `C`의 L·R −255..255, S −1000..1000 수용, 범위 밖 `range` | BENCH, ARMED |
| 3 | `reversal_interlock` | `C`로 L 부호 반전: 0까지 감속 → 120 ms → 반대 가속. 반대 핀 겹침 없음 | ARMED |
| 4 | `pending_stop` | 램프 중 `STOP`: 그 틱 duty 0, 목표 0, DISARMED. 뒤 `D` 명령은 `state` | BENCH |
| 5 | `watchdog_active` | 둘로 나눈다: (가) `C` TTL 만료 → brake, ARMED 유지, `F TTL`. (나) `C`·PING 끊김 → t + `wd` + 1에 brake, DISARMED, WATCHDOG, `W` | ARMED |
| 6 | `watchdog_pending` | 램프 상승 중 TTL·watchdog 만료 | ARMED |
| 7 | `late_ping_no_resurrection` | 만료 틱에 도착한 `PING`이 watchdog을 막지 못함(3단계 판정). 그 뒤 PING이 와도 DISARMED·래치 유지 | ARMED |
| 8 | `timely_ping` | **바뀜**: PING은 watchdog만 갱신. TTL은 PING으로 늘지 않음 | ARMED |
| 9 | `locked_power_and_d4` | BENCH `D1 0`, `D1 128` → `P D1 255 255`, GPIO 쓰기 없음. `D2`, `D6`, `D7`, `D4` → `N - pin` | BENCH |
| 10 | `strict_parser` | BENCH에서 D8 128 출력 중: `D8 -1`, `D8 1.5`, `D8 +1`, `D8  1`, `D8 1 `, `D8 1junk`, `D8`, `D8 `, `PING junk` → `syntax`. `D8 256`, `D8 999999999` → `range`. `D8 999999999999999999999` → `syntax`. `D9 255`, `D10 255`, `GPIO15 255`, `d8 255`, ` PING` → `cmd`. 모두 출력·watchdog 그대로 | BENCH |
| 11 | `nul_rejected` | `PING\0junk` → `N - ctrl`, 갱신 없음 | 아무 상태 |
| 12 | `crlf_and_embedded_cr` | `D8 128\r` 받음. `D8 2\r55` → `N - ctrl` + 구동 즉시 정지(BENCH). 그 뒤 `D8 64\r` 회복 | BENCH |
| 13 | `overflow_and_recovery` | 64 B 이상 내용(80 × `X`) → `N - long` + 즉시 정지(BENCH). 다음 줄 회복. 64번째 바이트와 LF가 다른 틱이면 정지는 64번째 바이트 틱(12.1-48, `bench_framing_error_early_brake`) | BENCH |
| 14 | `bounded_serial_work` | 4096 × `X`(LF 없음): 틱마다 64 B 이하만 읽음. watchdog은 제때(t + `wd` + 1). 서보 틱이 밀리지 않음 | ARMED, BENCH |
| 15 | `info_does_not_renew_watchdog` | `INFO`, `STOP`이 watchdog을 갱신하지 않음 | ARMED |
| 16 | `millis_rollover` | `millis_start` = 2^32 − 300: 램프·휴지·TTL·watchdog·서보 dt·상태 줄 위상이 감김을 넘어 같음 | ARMED |
| 17 | `gradual_acceleration_and_soft_release` | 0 → 255에 1000 ms(500 ms에 126..128), L = 0의 `C`는 release(500 ms에 127..129) | ARMED |
| 18 | `latest_request_wins` | **바뀜**: q가 새로울 때만 적용, 같거나 옛 q는 `N <q> old`. 감속 중 같은 방향 새 목표는 그 자리에서 다시 가속 | ARMED |
| 19 | `stall_catchup_capped` | `mcu_stall` 1000틱 뒤 첫 램프 갱신이 50 ms분(≤ 13), 다음 갱신도 밀리지 않음. 서보 dt ≤ 50 | ARMED |
| 20 | `emergency_stop_immediate` | `STOP` → 그 틱 구동 0(brake) + 조향 HOLD(측정 위치) → `hstop` 뒤 OFF | ARMED |
| 21 | `bounded_actual_reports` | **바뀜**: 상태 줄 20 Hz(틱 b + 50k), 줄 ≤ 85 B, ARMED에서 램프 중간 핀 보고 줄 없음 | ARMED |
| 22 | `repeated_zero_ack_without_gpio_write` | BENCH `D8 0` 세 번 → `P D8 0 0` 세 줄, `gpio_writes` 변화 0 | BENCH |

V3 전용으로 최소한 둘 대본(A19, A20): 결함 코드마다 하나(NOCAL, POT_OPEN, POT_SHORT, POT_JUMP, POT_NOISE, POLARITY, DRV_STUCK, SOFTLIMIT, STALL, TRACK, THERMAL, TTL, WATCHDOG, CRC, BOOT), `port_open_reset`(배너 대기, 새 boot_id, 옛 boot_id `ARM` → `boot`), `ARM` 거절 사유마다 하나(`boot`, `nocal`, `cal`, `pol`, `fault`), `CAL` 절차(MARK·SOFT·SAVE·ERASE·POL·JOG, `wear`), `CFG` 범위·묶인 검사, HOLD 정책 3가지, 09-19 필수 장애 시험 가운데 MCU 쪽(반쪽 프레임, CRC·범위 오류, MCU 리셋, STOP 직후 늦게 온 `C`).
