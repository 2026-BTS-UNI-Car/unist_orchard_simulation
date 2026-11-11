# 조향 MCU 적합성 대본과 실행기 (P3)

- 작성: 2026-11-09 (P3 적합성 작업)
- 근거: `ESP_PROTOCOL_V3_DRAFT_KO.md`(이하 "규격") 13절과 14절, `LOCKSTEP_PROTOCOL_KO.md` 11절(lockstep v2 firmware 모드), 설계 8.3 P3, 부록 C, `SIM_ACCEPTANCE_KO.md` A19(MCU 부분)·A20, `DECISIONS_KO.md` R1.
- 위치: 대본 `sim/ubts/steering/conformance/scripts/*.json`(90개), 실행기 `sim/ubts/steering/conformance/run_conformance.py`, 시험 `sim/ubts/test_steering_conformance.py`.
- 모든 수치는 자리값이다. 대본 통과는 자리값 기반 SIM 결과다. 현장 검증이 아니다. 실기에서는 아무것도 돌리지 않았다.
- 2026-11-09 P3 통합: 9절의 열린 문제를 규격 12.1절 결정(48~65)으로 정했고, 그 결정을 고정하는 대본 5개를 더했다(`bench_framing_error_early_brake`, `cfg_svhz_runtime`, `cal_erase_twice`, `crc_burst_stop_special`, `c_nocrc_shows_q`). 같은 대본을 패키지 코덱(`FirmwareSteeringBackend`)으로도 돌리는 두 번째 전송을 더했다(3절 `--transport backend`). 결과는 7절.
- 2026-11-09 fix_p3: 검토 결과(RF-01~RF-11)를 규격 12.1절 결정(66~73)으로 고치고, 그 결정을 고정하는 대본 5개를 더했다(`fault_drv_stuck_default`, `polarity_latch_needs_pol_check`, `rx_overflow_drops_oldest`, `servo_lag_with_progress_no_track`, `thermal_blocks_jog_and_pol`). 옛 대본 85개는 바꾸지 않았다. 결과는 7절 끝.

## 1. 무엇을 하나

- 대본 하나는 호스트가 보내는 바이트와 그 틱, MCU 사건(리셋, 전원, ADC 값), 그리고 기대(MCU 줄, 상태 줄 칸, 핀 값, 채널 변화)를 적은 JSON이다.
- 실행기는 네이티브 lockstep 프로그램을 firmware 모드로 띄워 대본을 틱 단위로 돌리고 결과를 기대와 비교한다(T1, 결정적).
- 기대값은 모두 규격에서 손으로 정했다. 펌웨어 소스(`firmware/ubts_steer_v3/`)와 네이티브 HAL·shim 소스는 읽지 않았다(R1). 계산은 저장소 밖 보조 스크립트로 규격의 식 하나씩만 옮겨 했다: CRC-8(3절), CRC-32와 레코드(9절), boot_id(10.1절), 구동 램프(8.2절 의사 코드), 서보 속도 제한과 법칙(7.5·7.6절), 중앙값과 위치(7.2·7.3절), 잡음 분산(6.7절). 펌웨어 전체를 흉내 내는 모형은 만들지 않았다(R1의 대안 경로 금지).
- 예외(fix_p3): 새 대본 5개는 펌웨어를 고친 같은 작업이 썼다. 기대값은 규격의 식(6.7절, 7.5·7.6절, 12.1-67~71)으로 손 계산했고, 5개 모두 고치기 전 빌드(`73575045`)에서 실패하는 것을 확인했다. 그러나 펌웨어 소스를 읽은 뒤에 썼으므로 이 5개는 "규격만 보고 따로 쓴 대본"이 아니다. 다른 작업이 규격만 보고 다시 확인해야 한다.

## 2. 대본 형식 요약 (`ubts.steer_conformance.v1`)

규격 13절이 정본이다. 요점과 실행기가 정한 해석만 적는다.

| 부분 | 요점 |
|---|---|
| 파일 | UTF-8, BOM 없음, LF. 파일 이름 = `name`(`[a-z0-9_]{1,64}`). 모르는 키, 중복 키는 거부 |
| 맨 위 | `schema`, `name`, `spec`(절 번호 배열), `v2_map`(선택), `description`, `setup`, `plant`(선택), `inputs`, `events`(선택), `expect`, `end` |
| `setup` | `boot_seed`, `chip_id`(대문자 6자리) 필수. `boot_delay_ticks`, `millis_start`, `eeprom`(`"blank"` / `{"record": R}` / `{"hex": 512자}`), `adc`(`script` 값 또는 `model`), `report_adc`, `initial_delta_eq_deg`, `profile`, `seed`, `startup_overrides`(RFC 7386), `injections`, `motor_power`, `boot_garbage_hex` |
| `inputs` | `{"tick", "text" 또는 "hex", "pace"}`. 틱 순서. `text`는 ASCII만. 매크로 `{boot}`(그 틱에 도는 부팅의 boot_id), `{crc}`(바로 앞이 `*`, 그 줄 처음부터 `*` 앞까지의 CRC-8). `instant`는 그 틱 4단계 전에 모두 RX 버퍼에, `uart`는 lockstep 11.7절 규칙(지연 0)으로 바이트마다 도착 틱 |
| `events` | `mcu_reset`(reason), `power_cycle`(off_ticks), `port_open_reset`, `motor_power`(on), `mcu_stall`(ticks), `adc_set`(raw), `adc_model`. 같은 틱의 입력보다 먼저 |
| `expect` | `line`, `status`, `no_line`, `count`, `pins`, `pins_window`, `change`, `first_change`. 선택 키 `id`, `spec`, `tol` |
| `end` | `tick`(필수), `drain_ticks`(300), `unexpected`(`fail`), `ignore_kinds`(`["S"]`), `checks` |
| `tol` | 0 또는 1. 1은 `F `·`W ` 줄과 `fault_bits` 변화에만(A3·A19의 ±1틱). 그 밖에 쓰면 대본 거부 |

실행기가 정한 해석(규격이 열어 둔 곳, 결정 제안으로 올린다):

1. 줄 틱: queued 틱 = 그 줄 첫 바이트의 `queued_tick`, done 틱 = LF의 `done_tick`. `line`은 `at`으로 고른다(기본 queued). `status`, `no_line`, `count`는 queued 틱으로 본다.
2. `line`은 창 안에 패턴이 맞는 줄이 정확히 하나여야 한다. 같은 틱에 같은 줄이 둘이면 `count`로 쓴다.
3. 줄을 "맞춘" 것으로 치는 기대는 `line`, `status`, `count`(끝 `checks` 포함)다. `no_line`은 치지 않는다. `end.unexpected`가 `fail`이면 끝 틱까지 queued된 줄 가운데 맞춰지지 않고 `ignore_kinds`에 없는 줄이 하나라도 있으면 실패다.
4. `change ... to V`는 그 틱에 값이 바뀌었고, 앞 틱 값은 V에 맞지 않고 그 틱 값은 V에 맞는다는 뜻이다(비교자가 거짓에서 참으로). `"any"`는 그 틱에 바뀌기만 하면 된다. 틱 0의 앞 값은 RESET 값(모두 0, `mcu_state` 255)이다.
5. `first_change`의 `after`는 그 틱을 포함한다("이후"). 대본은 `after` 틱 자체에 변화가 없게 써서 어느 해석이든 같게 했다.
6. 채널 값은 그 틱 처리 뒤 값이다. 변화 목록으로 틱마다 되살린다. `gpio_writes`, `eeprom_commits`는 부호 없는 값으로 읽는다.
7. 부트 롬 쓰레기 바이트(flags bit0)는 줄에 넣지 않고 수만 센다. 리셋·전원 끔이 보내던 줄을 자르면 그 앞부분은 "조각"으로 따로 적고 줄로 세지 않는다.
8. 모든 MCU 줄을 기대와 상관없이 검사한다: 0x20..0x7E만, `*HH` 대문자와 맞는 CRC-8, 알려진 첫 토큰(5.1절), 5.1절 최대 길이, 5절 칸 형식(상태 줄 15칸과 범위, `N`, `F`, `W`, `A`, `P` 형식). 어기면 실패다.
9. 끝 틱까지 queued된 줄이 drain 뒤에도 LF를 다 보내지 못했으면 실패다.
10. `{boot}`(입력과 기대 모두)는 T1에서 boot_seed와 대본의 리셋·전원 사건으로 계산한다(setup() 틱마다 흐름의 다음 0 아닌 값). 부팅 지연·전원 끔 중에는 도는 부팅이 없어 입력에 `{boot}`를 쓰면 대본 오류다.
11. 스텝은 기본 10틱이고 `plant` 표가 바뀌는 틱에서 나눈다. 한 스텝의 사건이 64개, 수신 바이트가 4096개를 넘으면 스텝을 줄인다.
12. 결정성과 스텝 독립(13.9절): 같은 프로세스를 RESET하고 7틱 스텝으로 다시 돌려 MCU 바이트(두 틱 포함), 채널 변화, 공통 스텝 끝의 기본 칸(참값 포함)을 비교한다. 다르면 새 프로세스로 7틱을 한 번 더 돌려 원인(RESET, 스텝 나누기, 비결정)을 적는다.

## 3. 실행 방법

`sim/ubts`에서(빌드 출력은 `UBTS_STEER_BUILD_DIR`로 작업 트리 밖에 둔다):

```
python -B -m steering.conformance.run_conformance --list
python -B -m steering.conformance.run_conformance --validate-only
python -B -m steering.conformance.run_conformance --out <저장소 밖>/result.json
python -B -m steering.conformance.run_conformance 'fault_*' cal_flow_blank_to_saved --verbose
python -B -m steering.conformance.run_conformance --executable <네이티브 실행 파일> --check-step-ticks 1
python -B -m steering.conformance.run_conformance --transport backend --out <저장소 밖>/result_backend.json
```

- 선택: 이름 또는 glob. 없으면 전부.
- `--step-ticks N`(1..100, 기본 10), `--check-step-ticks N`(0이면 스텝 독립 검사를 건너뛴다, 기본 7), `--expect-build HEX`(시작 줄 `firmware.expect_build`), `--timeout S`(응답 하나 기한, 기본 30 s), `--compiler`, `--build-dir`.
- `--transport native`(기본): 실행기 자신의 lockstep v2 코덱(`NativeFirmwareTransport`). `--transport backend`: 같은 실행 파일을 `actuator_link.FirmwareSteeringBackend`(패키지 코덱 `native/lockstep_v2.py`)로 돈다(`BackendFirmwareTransport`). backend는 대본 setup에서 시작 줄을 스스로 만들고, 그것이 `startup_config(script)`와 다르면 돌지 않는다. 시험 `test_the_package_codec_gives_the_same_trace`가 대본마다 두 전송의 trace(MCU 바이트와 두 틱, 채널 변화, 스텝 끝 기본 칸)가 같은지 본다.
- 종료 코드: 0 모두 통과, 1 실패 있음, 2 대본 형식 오류·인자 오류, 3 firmware 모드를 쓸 수 없음(컴파일러 없음, 빌드 실패, 시작 줄 거부).
- 시험: `python -B -m pytest -q -p no:cacheprovider --basetemp=<저장소 밖> test_steering_conformance.py`, Python 3.10은 `python3 -B -m unittest test_steering_conformance`. firmware 모드가 없으면 실기 대본 시험만 이유와 함께 건너뛴다.

## 4. 결과 파일 (`ubts.steer_conformance_result.v1`)

`--out`은 JSON 하나를 쓴다: 실행 파일 경로와 SHA-256, READY build id, 대본마다 이름·경로·SHA-256·규격 절·V2 이름, 상태(`pass`/`fail`/`error`), 기대마다 합격과 설명·실제 값, 끝 검사, 형식 실패 줄, 맞지 않은 줄, 끝나지 않은 줄, 조각, 부트 쓰레기 바이트 수, 스텝 독립 검사 결과, 돈 틱·스텝 수, `rx_lost_total`, 부팅 수, 걸린 시간, 그리고 표지 "placeholder-based SIM result (not field validation)". 상태 줄 밖의 MCU 줄은 모두 틱과 함께 싣는다.

## 5. 대본과 범위

결함 코드 칸은 대본이 `F`/`W` 줄 또는 `fl`·`fault_bits` 기대로 확인하는 코드다. 거절 사유는 대본이 기대하는 `N` 줄 사유다. 부록 C 칸은 `v2_map`이다.

| 대본 | 규격 절 | 결함 코드 | 거절 사유 | 부록 C |
|---|---|---|---|---|
| `adc_model_center` | 7.2, 13.3, 13.6 | BOOT | - | - |
| `arm_reject_reasons` | 4.6, 4.2, 4.9, 4.13, 6.1 | - | boot, busy, cal, crc, nocrc, state, syntax | - |
| `bench_entry_rules` | 4.11, 4.13, 4.2, 6.1 | BOOT | boot, busy, nocrc, state, syntax | - |
| `bench_framing_error_early_brake` (통합) | 2.2, 6.4, 8.2, 12.1-48 | - | ctrl, long | overflow_and_recovery |
| `bench_watchdog` | 6.2, 4.11, 4.12, 5.7 | WATCHDOG, BOOT | - | - |
| `boot_order` | 8.3, 8.4, 5.4, 6.1, 10 | BOOT | - | boot_order |
| `bounded_actual_reports` | 5.3, 5.4, 5.1 | - | - | bounded_actual_reports |
| `bounded_serial_work` | 2.2, 7.1, 6.2, 13.5 | TTL, WATCHDOG | long | bounded_serial_work |
| `c_field_ranges` | 4.8, 2.3, 4.2 | - | nocrc, old, range, syntax | mapping_and_full_range |
| `c_nocrc_shows_q` (통합) | 4.2, 4.8, 12.1-53 | - | nocrc | - |
| `c_q_wrap_and_old` | 4.8 | - | old | - |
| `c_ttl_expiry_brake` | 4.8, 6.3, 8.2, 5.3 | TTL | - | watchdog_active |
| `cal_erase_tombstone` | 4.9, 9.5, 5.6, 5.9, 6.8, 4.6 | NOCAL, BOOT | cal, nocal, pol | - |
| `cal_erase_twice` (통합) | 4.9, 9.5, 5.6, 12.1-54 | NOCAL | nocal | - |
| `cal_flow_blank_to_saved` | 4.9, 7.3, 7.10, 9.4, 9.5, 4.6, 4.10, 6.8 | BOOT | nocal, pol | - |
| `cal_jog` | 4.9, 7.7, 7.8, 4.13, 4.2 | - | busy, range, state, syntax | - |
| `cal_pol_fail_and_reverse` | 7.10, 4.9, 7.7, 9.5 | - | - | - |
| `cal_reject_reasons` | 4.9, 4.2, 4.13, 9.4 | - | busy, cal, range, state, syntax | - |
| `cal_wear_limit` | 9.5, 4.9, 4.2 | - | wear | - |
| `cfg_effects` | 4.10, 5.4, 6.2, 7.2, 6.3 | TTL, WATCHDOG | range | - |
| `cfg_query_all_keys` | 4.10, 5.8 | - | state | - |
| `cfg_set_ranges` | 4.10, 4.2 | - | key, range, syntax | - |
| `cfg_set_states` | 4.10, 4.2, 4.13, 4.11 | - | busy, key, nocrc, range, state | - |
| `cfg_svhz_runtime` (통합) | 4.10, 7.1, 7.5, 7.6, 5.9 | - | - | - |
| `crc_burst_armed` | 6.6, 6.4, 6.5, 5.6 | CRC | crc | - |
| `crc_burst_disarmed_bench` | 6.6, 6.4, 4.11 | CRC, BOOT | crc | - |
| `crc_burst_stop_special` (통합) | 2.4, 6.6, 12.1-52 | CRC | crc | - |
| `crc_field_rules` | 2.4, 3, 4.2, 4.3, 4.5 | - | cmd, crc, nocrc, syntax | - |
| `crlf_and_embedded_cr` | 2.1, 2.2, 6.4, 8.2 | BOOT | ctrl | crlf_and_embedded_cr |
| `disarm_armed` | 4.7, 6.4, 6.5, 8.2 | - | state | - |
| `disarm_bench` | 4.7, 6.4, 8.2, 4.12 | - | state | - |
| `emergency_stop_immediate` | 4.5, 6.4, 6.5, 7.1, 7.6 | - | state | emergency_stop_immediate |
| `fault_boot_bit` | 5.3, 6.8, 4.11, 6.4, 10.3, 8.4 | BOOT | - | - |
| `fault_drv_stuck` | 6.7, 6.4, 7.6 | DRV_STUCK | - | - |
| `fault_drv_stuck_default` (fix_p3) | 6.7, 6.4, 7.6, 4.6, 12.1-70 | DRV_STUCK | pol | - |
| `fault_nocal_boot` | 9.1, 9.4, 5.3, 5.6, 4.6, 4.11, 7.11 | BOOT | nocal | - |
| `fault_polarity` | 6.7, 6.4, 7.6 | POLARITY | - | - |
| `fault_pot_jump` | 6.7, 6.4, 6.8, 7.2 | POT_JUMP | - | - |
| `fault_pot_jump_and_noise_same_tick` | 6.7, 5.6, 5.10, 7.2 | POT_JUMP, POT_NOISE | - | - |
| `fault_pot_noise` | 6.7, 6.4, 7.2 | POT_NOISE | - | - |
| `fault_pot_open` | 6.7, 6.4, 6.8, 4.6, 7.2, 5.6 | POT_OPEN, SOFTLIMIT | fault | - |
| `fault_pot_short` | 6.7, 6.4, 6.8, 4.6, 5.6 | POT_SHORT, SOFTLIMIT | fault | - |
| `fault_softlimit` | 7.4, 7.8, 5.3, 6.7 | SOFTLIMIT, BOOT | - | - |
| `fault_stall` | 6.7, 6.4, 7.5, 7.6 | STALL, TTL | - | - |
| `fault_thermal_jog_default` | 6.7, 4.9, 6.4, 4.6 | THERMAL, BOOT | fault | - |
| `fault_thermal_release` | 6.7, 6.4, 7.6, 8.2 | THERMAL | - | - |
| `fault_track` | 6.7, 6.4, 7.5 | TRACK, TTL | - | - |
| `frame_ctrl_and_cr` | 2.1, 2.2, 2.4, 6.6 | - | ctrl, long | - |
| `frame_line_length_limits` | 2.2, 4.2, 6.6 | - | cmd, long | - |
| `gradual_acceleration_and_soft_release` | 8.2, 4.8, 6.4 | - | - | gradual_acceleration_and_soft_release |
| `half_frame_rejected` | 2.2, 4.2, 6.2 | - | nocrc | - |
| `hold_ended_by_fault` | 6.5, 6.7, 6.4 | POT_JUMP | - | - |
| `hold_policy_center` | 6.5, 4.5, 7.5, 4.10 | - | - | - |
| `hold_policy_target` | 6.5, 4.5, 7.6, 4.10 | - | - | - |
| `info_does_not_renew_watchdog` | 4.3, 4.5, 6.2, 5.3 | TTL, WATCHDOG | - | info_does_not_renew_watchdog |
| `late_c_after_stop` | 4.5, 4.8, 6.1, 6.8 | - | old, state | - |
| `late_ping_no_resurrection` | 6.2, 4.4, 4.8 | TTL, WATCHDOG | state | late_ping_no_resurrection |
| `latest_request_wins` | 4.8, 8.2 | - | old | latest_request_wins |
| `locked_power_and_d4` | 4.12, 8.3, 8.4 | - | pin, range, syntax | locked_power_and_d4 |
| `mapping_and_full_range` | 4.11, 4.12, 8.2, 5.8 | - | - | mapping_and_full_range |
| `mcu_reset_mid_row` | 6.4, 6.1, 10.1, 10.3, 10.7, 4.8, 7.1, 8.2 | - | boot, state | - |
| `millis_rollover` | 1, 5.3, 5.4, 6.2, 6.3, 7.1, 7.5, 8.2 | TTL, WATCHDOG | - | millis_rollover |
| `nul_rejected` | 2.2, 6.2, 6.4 | TTL, WATCHDOG | ctrl | nul_rejected |
| `overflow_and_recovery` | 2.2, 6.4, 8.2 | - | long | overflow_and_recovery |
| `pending_stop` | 4.5, 4.12, 8.2, 6.4 | - | state | pending_stop |
| `polarity_latch_needs_pol_check` (fix_p3) | 6.7, 6.8, 4.6, 7.10, 12.1-71 | POLARITY | pol | - |
| `port_open_reset` | 10.7, 10.3, 10.1, 2.2, 5.2, 6.1 | BOOT | boot, crc | - |
| `power_cycle` | 6.1, 6.4, 9.1, 10.3, 10.1 | - | - | - |
| `repeated_zero_ack_without_gpio_write` | 4.12, 8.4, 11.6 | - | - | repeated_zero_ack_without_gpio_write |
| `reset_reasons` | 10.1, 10.2, 10.3, 5.2 | - | boot | - |
| `reversal_interlock` | 8.2, 4.8 | - | - | reversal_interlock |
| `rx_overflow_drops_oldest` (fix_p3) | 2.2, 11.3, 12.1-67 | - | - | - |
| `servo_deadband_dwell` | 7.6, 6.5 | - | - | - |
| `servo_lag_with_progress_no_track` (fix_p3) | 6.7, 7.5, 7.6, 12.1-69 | - | - | - |
| `servo_rate_and_law` | 7.4, 7.5, 7.6, 4.8 | - | - | - |
| `servo_rate_creep_duty` (3단계) | 7.5, 8.2, 7.1 | - | - | - |
| `servo_reversal_pause` | 7.6, 7.7 | TTL | - | - |
| `stall_catchup_capped` | 8.2, 7.1, 5.4, 7.5 | TTL, WATCHDOG | - | stall_catchup_capped |
| `status_cadence_fields` | 5.3, 5.4, 7.1, 4.8, 6.2 | BOOT | - | - |
| `status_tx_drop` | 5.10, 5.4, 5.2 | - | - | - |
| `stop_bench_disarmed` | 4.5, 6.4, 6.5, 4.9 | - | - | - |
| `strict_parser` | 2.3, 2.4, 4.2, 4.12, 6.2 | WATCHDOG, BOOT | cmd, crc, range, syntax | strict_parser |
| `svhz_1000` | 7.1, 4.10, 7.5 | - | - | - |
| `thermal_blocks_jog_and_pol` (fix_p3) | 6.7, 4.9, 4.2, 6.4, 12.1-68 | THERMAL | fault | - |
| `timely_ping` | 4.4, 6.2, 6.3, 8.2 | TTL, WATCHDOG | - | timely_ping |
| `ttl_expiry_release` | 6.3, 6.4, 8.2, 4.10 | TTL, WATCHDOG | - | - |
| `ttl_new_c_at_expiry_tick` | 6.3, 7.1, 5.6 | TTL | - | - |
| `watchdog_active` | 6.2, 6.4, 6.5, 5.7, 4.10 | WATCHDOG | - | watchdog_active |
| `watchdog_pending` | 6.2, 6.3, 8.2 | TTL, WATCHDOG | - | watchdog_pending |
| `watchdog_wdmode_center` | 6.2, 6.5, 7.5, 4.10 | WATCHDOG | - | - |
| `watchdog_wdmode_free` | 6.2, 6.4 | WATCHDOG | - | - |

과제 항목과 대본:

| 항목 | 대본 |
|---|---|
| 명령마다 받는 길 | INFO(`info_does_not_renew_watchdog`, `crc_field_rules`, `bench_entry_rules`), PING(`timely_ping`, `half_frame_rejected`), STOP(`emergency_stop_immediate`, `stop_bench_disarmed`), ARM, DISARM(`disarm_armed`, `disarm_bench`), C(`c_field_ranges` 외), CAL JOG/POL/MARK/SOFT/SAVE/ERASE/GET(`cal_*`), CFG 설정·조회(`cfg_*`), BENCH, D8/D5/D3/D0/D1(`mapping_and_full_range`, `locked_power_and_d4`) |
| 거절 사유 18개 | 위 표 거절 사유 칸. 시험이 18개가 모두 어딘가에 있는지 본다 |
| CRC 좋음·나쁨, 연속 | `crc_field_rules`, `crc_burst_armed`, `crc_burst_disarmed_bench`, `crc_burst_stop_special`(STOP 특례와 연속 계수) |
| 줄 길이 64 B 앞뒤, 제어 문자, CR, 빈 줄, 반쪽 줄 | `frame_line_length_limits`, `frame_ctrl_and_cr`, `crlf_and_embedded_cr`, `overflow_and_recovery`, `half_frame_rejected`, `port_open_reset`, `bench_framing_error_early_brake`(오류 바이트와 LF가 다른 틱) |
| RX 고리 넘침(255 B, 가장 옛 바이트 버림) | `rx_overflow_drops_oldest` |
| q 감김(65535 -> 0), ARM 뒤 첫 C | `c_q_wrap_and_old`, `late_c_after_stop`, `mcu_reset_mid_row`(새 부팅 q 1) |
| ttl, L, R, S 범위 | `c_field_ranges`, `cfg_effects`(ttlmax), `c_nocrc_shows_q`(CRC 없는 C의 q) |
| ARM 조건 | `arm_reject_reasons`(boot, busy, cal), `fault_nocal_boot`, `cal_erase_tombstone`(nocal, pol), `cal_flow_blank_to_saved`(pol), `fault_pot_open`·`fault_pot_short`·`fault_thermal_jog_default`·`thermal_blocks_jog_and_pol`(fault), `fault_pot_jump`(활성 조건 없는 래치는 ARM이 푼다), `polarity_latch_needs_pol_check`·`fault_drv_stuck_default`(POLARITY·DRV_STUCK 래치는 ARM이 풀지 않고 `pol`, 통과한 `CAL POL`이 푼다) |
| STOP(상태마다, 되풀이) | `emergency_stop_immediate`, `stop_bench_disarmed`, `pending_stop`, `crc_field_rules`(STOP 특례) |
| TTL, PING이 TTL을 늘리지 않음 | `c_ttl_expiry_brake`, `ttl_expiry_release`, `ttl_new_c_at_expiry_tick`, `timely_ping`, `watchdog_pending` |
| watchdog(ARMED, BENCH, 래치, INFO) | `watchdog_active`, `watchdog_wdmode_center`, `watchdog_wdmode_free`, `bench_watchdog`, `late_ping_no_resurrection`, `info_does_not_renew_watchdog`, `strict_parser` |
| 상태 줄 주기, n, q, tl, wd | `status_cadence_fields`, `bounded_actual_reports`, `status_tx_drop`, `stall_catchup_capped`, `cfg_effects` |
| 결함 코드 15개와 판정 틱 | `fault_*`(기본 변수의 DRV_STUCK은 `fault_drv_stuck_default`), `crc_burst_*`, `c_ttl_expiry_brake`, `watchdog_active`, `hold_ended_by_fault`, `thermal_blocks_jog_and_pol`(래치 뒤 다시 참이 된 THERMAL 조건의 출력 끄기) |
| 정지 모드 표 행(핀 수준) | STOP(ARMED) `emergency_stop_immediate`, STOP(BENCH·DISARMED) `stop_bench_disarmed`, DISARM `disarm_armed`·`disarm_bench`, L = R = 0 `gradual_acceleration_and_soft_release`, TTL `c_ttl_expiry_brake`·`ttl_expiry_release`, watchdog `watchdog_*`, 나쁜 줄 연속 `crc_burst_*`, 가변저항·움직임 결함 `fault_*`, THERMAL(release) `fault_thermal_release`, SOFTLIMIT `fault_softlimit`, 틀 오류(BENCH) `crlf_and_embedded_cr`·`overflow_and_recovery`, 틀·CRC 오류(ARMED·DISARMED) `nul_rejected`·`frame_ctrl_and_cr`, MCU 리셋 `mcu_reset_mid_row`, 전원 차단 `power_cycle` |
| 서보 속도 제한, 정지 중 속도, HOLD 시간, 바깥쪽 막기 | `servo_rate_and_law`, `servo_rate_creep_duty`(데드밴드 안 duty도 움직이는 중, movms 꼬리), `millis_rollover`, `hold_policy_target`, `hold_policy_center`, `emergency_stop_immediate`, `disarm_armed`, `servo_deadband_dwell`, `servo_reversal_pause`, `fault_softlimit`, `svhz_1000`, `servo_lag_with_progress_no_track`(따라가는 서보의 뒤처짐은 TRACK이 아니다) |
| CAL 흐름과 EEPROM 지속, cal_id 증가 | `cal_flow_blank_to_saved`, `cal_erase_tombstone`, `cal_wear_limit`, `cal_pol_fail_and_reverse`, `cal_jog`, `cal_reject_reasons`, `cal_erase_twice`(R 없는 ERASE), `thermal_blocks_jog_and_pol`(THERMAL 조건 중 JOG·POL 거절), `polarity_latch_needs_pol_check`(POL 통과가 극성 래치를 지움). commit 실패는 C++ 단위 시험 `test_eeprom_commit_failure`(대본으로는 만들 수 없다) |
| CFG 키 | `cfg_query_all_keys`, `cfg_set_ranges`, `cfg_set_states`, `cfg_effects`, `cfg_svhz_runtime`(실행 중 svhz) |
| BENCH(램프, 휴지, watchdog), D 거절(ARMED·DISARMED), ARMED <-> BENCH 직행 없음 | `mapping_and_full_range`, `pending_stop`, `bench_watchdog`, `bench_entry_rules`, `arm_reject_reasons`, `disarm_bench` |
| 포트 열기 리셋, 주행 중 MCU 리셋(coast) | `port_open_reset`, `mcu_reset_mid_row`, `reset_reasons`, `power_cycle` |
| 09-19 필수 장애 시험(MCU 쪽) | 반쪽 프레임 `half_frame_rejected`·`port_open_reset`, CRC·범위 오류 `crc_field_rules`·`c_field_ranges`, STOP 직후 늦은 C `late_c_after_stop`·`emergency_stop_immediate`, MCU 리셋 `mcu_reset_mid_row` |

A19 정상 대본 오검출: 결함을 노리지 않는 대본에서 `F` 줄은 없다(맞지 않은 줄 검사와 `no_line`). STALL·`steering_ineffective` 오검출률(보고 항목)은 구동기 모형이 닫힌 루프로 도는 P4에서 잰다. 여기 대본은 ADC를 대본 값으로 고정해 서보 출력이 결정적이다. 그래서 큰 출력을 오래 내는 대본은 STALL·TRACK이 끼지 않게 짧게 하거나 그 판정값을 레코드 변수로 바꿨다(설명 칸에 적었다).

fix_p3: 결함 없는 큰 계단과 쓸기의 TRACK 오검출(검토 RF-03, A19)을 규격 12.1-69로 고쳤다. 대본 `servo_lag_with_progress_no_track`(고치기 전 규칙은 약 1.7 s에 TRACK)과 `test_steering_firmware.py`의 `test_large_steps_and_sweeps_do_not_trip_track`(가변저항·구동기 모형, 정지 중 ±900‰ 계단 세 시드, 주행 중 전폭 스윕)이 본다. 남은 것(자리값 관찰): 정지한 차에서 duty가 데드밴드 안이라 MCU는 주행 중 속도 제한을 쓰고 구동기는 정지 중 속도만 내는 경우, 연속 전폭 스윕 세 번(7.5 s 안)은 TRACK이 아니라 THERMAL로 끊겼다(6,235 ms). 오검출률은 P4에서 잰다. 기본 변수로는 DRV_STUCK이 날 수 없던 문제(검토 RF-04)는 12.1-70으로 고쳤다(대본 `fault_drv_stuck_default`).

3단계(2026-11-09, T1 장치)에서 RF-03 남은 경로를 쟀다(`T1_RIG_KO.md` 10절). 정지, 전폭 스윕 3번을 7.5 s에, seed 10개: duty 4 %(구동 데드밴드 안)는 THERMAL 10/10(중앙값 7.18 s), 출력 80 % 이상 틱 66 %, 가장 큰 |cmd − pos| 546 ‰. duty 0은 THERMAL 0/10, 36 %, 252 ‰. 실제로 움직이는 40 %도 THERMAL 9/10(7.50 s). 펌웨어는 규격 7.5대로다(차속을 모르므로 0 아닌 적용 duty는 움직이는 중). 그래서 펌웨어를 바꾸지 않고 규격 결정을 제안했다(실행기는 데드밴드 안 duty를 내지 않는다 등). 대본 `servo_rate_creep_duty`가 지금 규칙을 고정한다: duty 10/255도 rmov, release 뒤 마지막 0 아닌 duty부터 `movms` 300 ms 동안 rmov, 그 뒤 rstd. 기대값은 규격 7.1·7.5·8.2 산술로 손 계산했고(저장소 밖 계산 스크립트 `runs/wf3a/p4_rig/scripts/spec_model_creep_duty.py`, 펌웨어 소스를 쓰지 않음), 같은 계산이 기존 `servo_rate_and_law`의 값(틱 50·100·150·300·400)도 그대로 낸다. 처음 실행에서 통과했다.

## 6. V2 22개 대응(설계 부록 C, 규격 14절)

V2 시나리오마다 같은 이름의 대본이 있다(`v2_map` 같음). 5번 `watchdog_active`는 둘로 나눴다: TTL 만료는 `c_ttl_expiry_brake`(13.11절 예), 링크 watchdog은 `watchdog_active`. 2번은 BENCH 쪽 `mapping_and_full_range`와 ARMED 쪽 `c_field_ranges`다.

lockstep v2로 볼 수 없어 대본에서 빠진 부분(적용하지 않는 이유):

| V2 | 빠진 부분 | 이유 |
|---|---|---|
| 1 `boot_order` | `setup()` 안의 쓰기 순서(PWM 설정 -> 방향 핀 -> D6·D7 -> D2 -> D1) | 채널은 틱 끝 값과 쓰기 수만 준다. 대본은 리셋부터 setup()까지 핀이 뜨는 것, setup() 뒤 값, 쓰기 수 14를 본다 |
| 3 `reversal_interlock` | 한 쪽 두 핀(D8, D5)이 동시에 0이 아님 | 채널 `drive_l`은 D8 - D5 하나다. 대본은 0을 지나 120 ms 동안 0인 것을 본다 |
| 14 `bounded_serial_work` | 직렬 처리 때문에 서보 틱이 밀리지 않음 | SIM은 틱마다 loop()를 부르므로 밀릴 수 없다. 대본은 64 B/틱 읽기(N 틱)와 cmd가 쓰레기 바이트와 상관없이 같음을 본다. 실기 루프 시간은 B9에서 잰다 |
| 21 `bounded_actual_reports` | 실제 직렬 대역(1.7 kB/s)의 여유 | 규격 11.2절 계산이다. 대본은 줄 수와 5.1절 길이를 본다 |

## 7. 이번에 실제로 돌린 것

모두 자리값 기반 SIM 결과다. 펌웨어는 같은 시각에 다른 작업이 쓰고 있었다. 아래 build id는 이 작업 끝 무렵 작업 트리에서 빌드한 값이다. 통합 단계에서 다시 돌린다.

| 환경 | 결과 |
|---|---|
| Windows, Python 3.13, MinGW 6.3 32비트 (`fw=1AE138AC`, `cxx=gcc-6.3.0 bits=32`) | 대본 80/80 통과(첫 실행 77/78: `cfg_effects`의 상태 줄 틱 두 개를 내가 잘못 셌다. 주기를 바꾼 뒤 위상은 270 + 50k다. 대본을 고쳤다). pytest 35 통과(실기 대본 80개를 두 번씩 돌리는 시험 포함 약 8 s), CLI 약 8 s |
| WSL Ubuntu-22.04, Python 3.10.12, g++ 11.4 | unittest 35 OK(실기 대본 시험 포함, 건너뜀 없음) |
| 기반 이미지 `6209168a3507`, Python 3.12.3, g++ 13.3 (`bits=64`) | pytest 35 통과, CLI 80/80 |
| 스텝 독립 검사(RESET 뒤 7틱 스텝) | 80개 모두 같음 |
| 실행기 변이 검사(저장소 밖) | 실제 trace에 기대를 하나씩 틀리게 바꾼(틱 ±1, 값 +1, 글자, 개수) 변이 4112개가 모두 실패로 잡힘 |

P3 통합(2026-11-09, 펌웨어 `fw=73575045`, 대본 85개):

| 환경 | 결과 |
|---|---|
| 통합 전 빌드(`fw=1AE138AC`)에 대본 80개 | 80/80 통과. 맞지 않는 곳은 없었다. 열린 문제(9절)만 정했다 |
| 새 대본 5개를 옛 빌드(`1AE138AC`)에 | `bench_framing_error_early_brake`만 실패(정지가 LF 틱 420·660이라 410·650의 기대와 다름). 나머지 4개는 이미 그렇게 동작했다 |
| Windows, Python 3.13, MinGW 6.3 32비트 | CLI 85/85(`--transport native`), 85/85(`--transport backend`). pytest 36 통과(두 코덱 trace 비교 포함) |
| WSL Ubuntu-22.04, Python 3.10.12, g++ 11.4 | unittest 36 OK. CLI 85/85 두 전송 모두 |
| 기반 이미지 `6209168a3507`, Python 3.12.3, g++ 13.3 | pytest 36 통과(조향 시험 묶음 233 안), CLI 85/85 두 전송 모두 |

처음 쓴 `cal_erase_twice`는 틱 400의 `F NOCAL`을 `ms 400`으로 적어 실패했다. MCU 리셋(틱 300) 뒤 `millis()`는 0부터 다시 세므로(1절) 맞는 값은 `ms 100`이다. 내 계산 실수라 대본을 고쳤다.

fix_p3(2026-11-09, 펌웨어 `fw=04353B49`, 대본 90개):

| 환경 | 결과 |
|---|---|
| 새 대본 5개를 고치기 전 빌드(`73575045`)에 | 5개 모두 실패. `fault_drv_stuck_default`: 틱 636에 `F DRV_STUCK`이 없고 계속 ARMED. `polarity_latch_needs_pol_check`: 틱 400의 ARM을 받아 POLARITY 래치가 풀림. `rx_overflow_drops_oldest`: 새 바이트를 버려 첫 INFO를 틱 100에 처리. `servo_lag_with_progress_no_track`: 따라가는 중에 TRACK으로 DISARMED. `thermal_blocks_jog_and_pol`: THERMAL 조건 중 틱 450의 JOG를 받아 100 % 출력이 이어짐 |
| 옛 대본 85개를 고친 빌드에 | 85/85 통과. `fault_track`은 TRACK 1272 그대로다. 처음 시도한 TRACK 규칙("cmd가 목표에 가는 동안 세지 않음")은 이 대본에서 실패해 버리고 지금 규칙(12.1-69)으로 바꿨다(실패 결과는 저장소 밖에 둔다) |
| Windows, Python 3.13, MinGW 6.3 32비트 | CLI 90/90(`--transport native`), 90/90(`--transport backend`). pytest는 조향 시험 묶음 안에서 통과 |
| WSL Ubuntu-22.04, Python 3.10.12, g++ 11.4 | unittest 통과(펌웨어·적합성·호스트 결합·호스트 프로필 묶음 111개). CLI 90/90 두 전송 모두 |
| 기반 이미지 `6209168a3507`, Python 3.12.3, g++ 13.3 | pytest 통과(조향 시험 묶음 238개 안), CLI 90/90 두 전송 모두 |

3단계(2026-11-09, T1 장치 작업 p4_rig, 펌웨어 `fw=04353B49` 그대로, 대본 91개):

| 환경 | 결과 |
|---|---|
| Windows, Python 3.13, MinGW 6.3 32비트 | CLI 91/91(`--transport native`), 91/91(`--transport backend`). 새 대본 `servo_rate_creep_duty` 통과 |
| WSL Ubuntu-22.04, Python 3.10.12, g++ 11.4 | CLI 91/91 두 전송 모두. unittest 통과 |
| 기반 이미지 `6209168a3507`, Python 3.12.3, g++ 13.3 | CLI 91/91 두 전송 모두. pytest(7.4.4) 통과 |

- 네이티브에 평가 전용 시작 줄 키 `report_servo`와 채널 11~14를 더했다(LOCKSTEP 11.6). 적합성 실행기는 이 키를 늘 거짓으로 보낸다. 거짓이면 응답 바이트가 더하기 전과 같다(시험 `test_report_servo_channels`).

## 8. 나중에 실기 벤치에서(사용자, 지금은 하지 않는다)

같은 JSON 대본을 실기 보드에 쓸 수 있게 형식을 정했다(T3, 설계 8.3 P9, A20). 지금은 하드웨어에서 아무것도 돌리지 않았고 T3 실행기도 없다. 준비할 것:

- 직렬 연결: 115200 8N1. 포트를 연 뒤 CRC가 맞는 배너가 올 때까지 받은 바이트를 버린다(규격 10.7절). `{boot}`는 마지막 배너에서 읽는다. 대본 틱 0은 그 배너의 부팅 틱으로 잡는다.
- 볼 수 있는 것은 직렬 줄뿐이다. `pins`, `pins_window`, `change`, `first_change`는 논리 분석기로 D1, D2, D5..D8을 잴 때만 볼 수 있다. 그 장비가 없으면 그 기대는 "실기에서 보지 않음"으로 결과에 적는다.
- 사건: `mcu_reset`은 RST 버튼 또는 DTR, `port_open_reset`은 포트 다시 열기, `power_cycle`은 전원 스위치, `adc_set`은 A0에 DAC나 분압기가 있어야 한다. 그 장비가 없는 대본은 건너뛴다.
- EEPROM: `setup.eeprom`의 레코드는 `CAL` 명령으로 만들거나 플래시 도구로 쓴다. 쓰기 한도(부팅마다 8회)와 플래시 수명을 생각해 반복 횟수를 줄인다.
- 허용 오차: 실기는 벽시계다. 대본 밖 설정으로 전체 틱 허용 오차와 raw·pos 칸 허용 오차를 더하고 그 값을 결과에 따로 적는다(규격 13.9절). T1은 그 설정 없이 통과해야 한다.
- 안전: 구동 바퀴를 띄우거나 구동 모터 전원을 뗀다. 조향 모터 급전은 CAL JOG·POL 대본에서만 켜고, 손으로 끌 수 있는 전원 차단을 둔다. 이 대본들은 자리값 기준이라 실기 값(가변저항 범위, 조향 속도)이 오면 바뀔 수 있다.

## 9. 알려진 한계와 열린 문제

P3 통합(2026-11-09)에서 1~9를 정했다. 원래 문장은 남기고 처리 결과를 붙였다.

1. `C` 줄이 CRC 없이 와서 `nocrc`로 거절될 때 `N`의 q 칸이 q인지 `-`인지 규격 문장("틀·CRC 검사를 통과한")만으로 갈리지 않는다. 대본은 이 경우를 둘째 토큰이 정수가 아닌 줄로만 시험한다. → 규격 12.1-53: q를 싣는다. 대본 `c_nocrc_shows_q`.
2. BENCH에서 과길이·제어 문자 줄이 여러 틱에 걸치면 구동을 멈추는 틱이 오류를 처음 본 틱인지 LF 틱인지 규격이 정하지 않는다. 대본은 오류 바이트와 LF가 같은 틱에 읽히게 썼다. → 규격 12.1-48: 오류를 처음 알아챈 틱(V2와 같음, 더 이른 정지). 펌웨어를 고쳤다(예전에는 LF 틱). 대본 `bench_framing_error_early_brake`(옛 빌드 `1AE138AC`에서는 실패, 새 빌드 `73575045`에서 통과).
3. NOCAL이 이미 켜진 채 `CAL ERASE`를 다시 하면 `F NOCAL`을 다시 내는지(5.6절 "한 번"과 4.9절 응답 표가 갈린다). 대본은 이 경우를 쓰지 않는다. → 규격 12.1-54: 명령 응답이라 늘 낸다. 대본 `cal_erase_twice`.
4. `STOP*xx` 특례 줄이 나쁜 줄 계수를 0으로 돌리는 "좋은 줄"인지 정하지 않았다. 대본은 이것에 기대지 않는다. → 규격 12.1-52: `STOP`과 CRC가 맞는 `STOP*HH`는 좋은 줄, CRC가 틀린 `STOP*xx`는 어느 쪽도 아니다. 대본 `crc_burst_stop_special`.
5. 부팅 지연 안에 다시 리셋되면 `boot_count`(11.5절은 리셋마다 1, 11.6절은 setup() 수)와 boot_id 차례가 갈린다. 대본은 이 경우를 쓰지 않는다. 실행기는 setup() 수로 센다. → 지금 lockstep 11.5절은 setup() 수로 적혀 있다. 규격 12.1-56.
6. `status_tx_drop`은 SIM HAL의 `availableForWrite()`가 한 틱에 고리 바이트를 모두 받는다고 본다(11.5절 "FIFO 한도를 흉내 내지 않는다"). 지금 빌드와 맞았다. → 그대로(SIM 단순화, 실기 확인은 B9). fix_p3: 펌웨어는 한 번에 `availableForWrite()` − 1 바이트까지만 옮긴다(12.1-66). lockstep에서는 16384 − 남은 수라 바이트 시각이 그대로다. 코어 3.1.2 송신 FIFO 규칙은 C++ 단위 시험의 흉내 선택으로만 본다(대본으로는 볼 수 없다).
7. STOP 전에 보낸 `C`가 재무장 뒤에 도착하면 q가 last_q보다 크므로 받는다. 4.8절의 "재무장 뒤에도 거절된다"는 그 q가 이미 받은 q 이하일 때만 맞다. 결정 제안으로 올린다. → 규격 4.8절 문장을 고쳤다(12.1-59): 직렬선은 순서를 지키고, 호스트는 STOP 뒤 새 ARM 확인 전까지 `C`를 보내지 않는다.
8. 실행 중 `svhz` 바꾸기의 적용 시점은 시험하지 않았다(레코드 값으로만 시험). → 대본 `cfg_svhz_runtime`: 바꾼 뒤 첫 서보 틱에서 새 주기로 다음 서보 틱을 잡는다(4.10절, 7.1절).
9. 실행기의 lockstep v2 코덱은 규격만 보고 따로 썼다. `NativeSteeringBackend`에 firmware 모드가 생기면 같은 전송 인터페이스(`FirmwareTransport`)로 감싸 두 코덱을 함께 쓴다. → `BackendFirmwareTransport`(`--transport backend`)를 더했다. 85개 대본 모두 두 전송에서 trace가 같다.
10. ADC 모형으로 서보를 닫는 대본은 `adc_model_center` 하나(정지, 급전 배율)뿐이다. 닫힌 루프 결과는 P4 F-SIM에서 본다. → 그대로. 호스트까지 붙인 T1 결합(ADC 모형, 구동기 모형, 열린 루프 속도 stub)은 `test_steering_host_firmware.py`에 있다. 플랜트를 닫는 것은 P4다.
11. (fix_p3) RX 넘침 규칙: 예전 흉내 층은 256 B를 담고 새 바이트를 버렸다. 코어 3.1.2 `uart.cpp`는 255 B까지 담고 가장 옛 바이트를 버린다(규격 12.1-67이 47을 바꿨다). 하드웨어 RX FIFO(128 B)가 ISR 전까지 더 담는 몫은 흉내 내지 않는다[가정]. 실기 확인은 B9다.
12. (fix_p3) 새 대본 5개의 독립성(1절 예외). 다른 작업이 규격만 보고 기대값을 다시 계산해야 한다. → 확인했다(3단계). 3단계 안전 검토가 규격만 보고 다섯 대본의 기대값을 다시 계산했고 모두 일치했다. 그 검토에서 규격 13.5의 instant 넘침 문장이 12.1-67과 달라 고쳤다(RS-08).
13. (통합 단계) 1~8번 질문에 답하며 통합 단계가 쓴 대본 5개(`c_nocrc_shows_q`, `bench_framing_error_early_brake`, `cal_erase_twice`, `crc_burst_stop_special`, `cfg_svhz_runtime`)도 펌웨어를 바꾼 단계가 썼다. 커밋 0c1311e 기준으로 규격만 보고 기대값을 따로 다시 계산했다(2026-11-09). 기대 74개가 모두 계산과 같고 critical·major 지적은 없다. minor 3건은 규격 8.2절 brake 문장의 두 가지 읽기, bench 대본이 LF 틱과 CR 경우를 다 고정하지 못함, 규격이 실행 중 svhz 변경의 서보 위상을 정하지 않음이다. 펌웨어는 고칠 것이 없다. 규격 문장과 대본을 4단계에서 고치고 A20을 다시 돈다(`DECISIONS_KO.md` R17, R20).
