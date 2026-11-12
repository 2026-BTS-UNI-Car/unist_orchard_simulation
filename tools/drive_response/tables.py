"""Tables (CSV + Markdown) and the machine-readable result file. Every value cell carries its grade:
측정 (>= 5 segments / events), 참고 (1-4), 보간 (between measured duties), 외삽 (outside the measured duty range),
모델 (computed from a model whose inputs are listed). No GNSS position appears anywhere."""
import os

import numpy as np

import common
import speed as sp
import startstop as ss

F = common.fmt
OLD_CAR = '조향 장치 장착 전 차(좌우 차동 구동)의 2026-10-03·10-10 수동 주행 값'


def _v(c, nd=3):
    return None if c is None or c.get('v') is None else round(c['v'], nd)


def _cellstr(c):
    if c is None or not c.get('n'):
        return ''
    ref = '' if c.get('ref') in (None, 'LiDAR') else ' %s' % c['ref']
    return '%.3f [%s%s, n %d]' % (c['v'], c['grade'], ref, c['n'])


def _q(vals, nd=2):
    v = common.finite(vals)
    if not len(v):
        return ''
    return '/'.join(('%.' + str(nd) + 'f') % np.percentile(v, p) for p in (10, 50, 90))


def _low_start_note(starts):
    low = [e for e in starts if e['cls'] == 'straight_fwd' and e['duty'] < 88]
    high = sorted({e['duty'] for e in starts if e['cls'] == 'straight_fwd' and e['duty'] >= 88})
    parts = ['정지 출발 요청 duty는 %s %%다' % ', '.join(str(d) for d in high)]
    for e in low:
        parts.append('그 밖에 %s/%s %s(%s)의 %d %% 출발 한 건: 곡선 구간 스캔의 %.0f %%만 정합 품질 통과%s, 2 s 거리 %s m. '
                     '측정 칸에는 쓰지 않고 표 4에 참고로 남김' % (
                         e['day'][:2], e['day'][2:].lstrip('0'), e['short'], e['ground_label'], e['duty'],
                         100 * e.get('good_frac', 0.), '' if e.get('onset_s') is not None else ', 움직임 시각 못 잡음',
                         F(e.get('d_2'), 2) or '—'))
    parts.append('30~60 %로 정지에서 출발한 다른 자료는 없다')
    return '. '.join(parts) + '.'


class Writer:
    def __init__(self, out_dir):
        self.dir = os.path.join(out_dir, 'tables')
        self.md = []

    def table(self, name, title, header, rows, note=None, md_rows=None):
        common.write_csv(os.path.join(self.dir, name + '.csv'), header, rows)
        text = '## %s\n\n' % title
        if note:
            text += note + '\n\n'
        text += common.md_table(header, md_rows if md_rows is not None else rows) + '\n'
        common.write_text(os.path.join(self.dir, name + '.md'), '# %s\n\n%s\n\n%s' % (title, OLD_CAR, text.split('\n', 2)[2]))
        self.md.append(text)

    def finish(self):
        common.write_text(os.path.join(self.dir, 'ALL_TABLES.md'),
                          '# 구동 응답 표 모음\n\n**%s.** 조향 차량의 값이 아니다.\n\n' % OLD_CAR + '\n'.join(self.md))


def write_all(R, out_dir):
    w = Writer(out_dir)
    G = R['grounds']
    # ---- T0 reading check --------------------------------------------------------------------------
    rows = []
    for r in R['read_check']:
        c = r['checks'].get('compact_motor_equals_motor_jsonl')
        rows.append([r['rid'], r['motor_layout'], r['rows'], r['expected_rows'], F(r['span_s'], 1),
                     F(r['expected_span_s'], 1), '같음' if r['same'] else '다름',
                     '' if c is None else ('같음' if all(c.values()) else '다름 %s' % c),
                     'O' if r['lidar'] else '-', r['gnss_rows'], 'O' if r['trajectory'] else '-',
                     ';'.join(r['warnings'])])
    w.table('T0_read_check', '표 0. 읽기 확인(PW-02): 모터 줄 수와 길이를 11/11 목록과 대조',
            ['기록', '모터 읽기(field_calib.records)', '모터 줄', '목록의 모터 줄', '모터 시간 폭 s', '목록의 폭 s', '목록과',
             '10/3 압축본 모터 = motor.jsonl', 'LiDAR 움직임', 'GNSS 줄', 'Cartographer 궤적', '경고'], rows,
            note='목록 = steering-sim-20261108/runs/replan_20261111/pwm_data/probe_pwm_inventory.out.txt. '
                 '10/3은 원래 motor.jsonl(전체 추출본)을 읽고, 같은 기록의 압축 추출본 모터 줄과 줄 수·수신 시각·요청·보고값을 대조했다.')
    # ---- T1 recordings -----------------------------------------------------------------------------
    inrow = {x['short']: x for x in R['inrow']}
    rows = []
    for r in R['recordings']:
        secs = sorted(((d, s) for d, s in r['settled_straight_s'].items() if s >= .9), key=lambda x: -x[1])
        sec_txt = ', '.join('%d: %.1f' % (d, s) for d, s in secs) or '없음'
        ir = inrow.get(r['short'], {})
        ir_txt = ''
        if ir.get('moving_inrow_frac') is not None:
            ir_txt = '%.0f %% (직진 구간 %.0f %%)' % (100 * ir['moving_inrow_frac'], 100 * (ir.get('segment_inrow_frac') or 0))
        elif r['day'] == '1010' and r['analysed']:
            ir_txt = '궤적 없음'
        rows.append(['10/%s' % r['day'][2:].lstrip('0') if r['day'] == '1003' else '10/10', r['short'],
                     (r['kst_start'][11:16] + '~' + r['kst_end'][:5]) if r['kst_start'] else '',
                     F(r['duration_s'], 1), G.get(r['ground'], r['ground']), r['evidence'], r['motor_rows'], sec_txt,
                     r['segments_used'], ', '.join(r['refs']), ir_txt, '; '.join(r['files'])])
    w.table('T1_recordings', '표 1. PWM이 기록된 실제 주행 14개(PW-01)',
            ['날짜', '기록', '실제 시각(KST)', '길이 s', '지면', '지면 근거', '모터 줄', '정착된 같은-duty 직진 duty: 초',
             '표에 쓴 직진 구간 수', '참조', '10/10 열 안 비율(움직인 시간)', '파일(설정 키 기준)'], rows,
            note='실제 시각은 GNSS 수신기 UTC(첫·끝 값)를 KST로 바꾼 것이다(072238·073551은 호스트 시계가 약 7,880 s, 10/10은 약 '
                 '10,595 s 늦어 파일 이름과 다르다). 정착 시간은 MCU 보고 duty가 요청과 2 %p 안으로 같은 2 Hz 상태가 다음 '
                 '상태까지(최대 1.5 s) 이어진 시간의 합이다(11/11 목록과 같은 셈법, 허용 폭만 1 → 2 %p). 표에 쓴 구간은 2 s 넘게 정착, '
                 '첫 1 s 제외, |yaw| < 2 °/s를 만족한 구간이다.')
    # ---- T1b in-row ----------------------------------------------------------------------------------
    rows = []
    for x in R['inrow']:
        if 'nodes' not in x:
            rows.append([x['short'], '', '', '', '', '', '', '', x.get('note', '')])
            continue
        rows.append([x['short'], x['nodes'], x['corridors_used'], '%.1f' % (100 * x['node_frac']),
                     '%.1f (%.0f m)' % (100 * x['path_frac'], x['path_m']),
                     '%.1f' % (100 * x['moving_inrow_frac']), F(x['moving_s'], 0),
                     '%.1f (열 안 %.0f s / 밖 %.0f s)' % (100 * (x['segment_inrow_frac'] or 0), x['segment_s']['inrow'],
                                                       x['segment_s']['out']),
                     '노드 수 일치' if x['catalogue_check_ok'] else '불일치'])
    w.table('T1b_inrow', '표 1b. 10/10 기록의 열 안 비율(Cartographer 궤적 × 회랑 카탈로그)',
            ['기록', '궤적 노드', '쓴 회랑 수', '노드 기준 %', '경로 길이 기준 %', '움직인 시간 기준 %', '움직인 시간 s',
             '직진 구간 시간 기준 %', '카탈로그 대조'], rows,
            note='열 안 = 카탈로그 회랑 중심선에서 |d| < 행 간격/2 − 0.35 m이고 두 줄이 다 있는 구간 안(카탈로그 빌더의 규칙 그대로). '
                 '회랑 카탈로그에는 길이 25 m 넘는 주 방향 행 쌍만 있어 짧은 행·다른 방향 행의 행간은 "그 밖"으로 들어간다. '
                 '움직인 시간 = LiDAR 속도 > 0.1 m/s인 스캔. 궤적은 SLAM 재구성이라 정답이 아니다. 카탈로그 대조 = 회랑마다 이 코드로 센 '
                 '노드 수가 카탈로그의 nodes_inside와 같은지.')
    # ---- T2 main (pooled) -----------------------------------------------------------------------------
    rows = []
    for r in R['speed_main']['rows']:
        if r['grade'] in ('보간', '외삽'):
            rows.append([r['duty'], r['grade'], '', F(r['v']), '', 0, '', '', '', '', '', '', r.get('basis', '')])
            continue
        li, gn, ca = r['lidar'], r['gnss'], r['carto']
        ratio = (gn['v'] / li['v']) if (gn['n'] and li['n']) else None
        rows.append([r['duty'], r['grade'], r['ref'], F(r['v']), '%s–%s' % (F(r['p10']), F(r['p90'])), r['n'],
                     F(r['seconds'], 1), '%s (%d)' % (F(li['v']), li['n']) if li['n'] else '',
                     '%s (%d)' % (F(gn['v']), gn['n']) if gn['n'] else '',
                     '%s (%d)' % (F(ca['v']), ca['n']) if ca['n'] else '', F(ratio), ' '.join(r['records']), ''])
    fit = R['speed_main']['fit']
    w.table('T2_speed_all', '표 2. duty → 정상 직진 속도 기준표(모든 기록·지면을 합침, PW-03)',
            ['duty %', '등급', '기준 참조', '속도 m/s', '구간 p10–p90', '구간 수', '창 합계 s', 'LiDAR (n)', 'GNSS (n)',
             'Cartographer (n)', 'GNSS/LiDAR', '기록', '보간·외삽 근거'], rows,
            note='값 = 구간 중앙값들의 중앙값. 기준 참조는 LiDAR 정합이고, 그 duty에 LiDAR 값이 있는 구간이 하나도 없으면 GNSS를 쓰고 '
                 '그렇게 적었다. 등급: 측정 = 구간 5개 이상, 참고 = 1~4개, 보간 = 측정된 duty 사이 직선, 외삽 = 측정 범위 밖(측정 duty '
                 '직선 맞춤 v = %.4f·duty %+.3f, 점 %d개). 합친 표라 지면과 날이 섞였다: 90 %%는 10/10 과수원 기록 하나(035952)가 '
                 '대부분이라 88 %%(10/3 도로)보다 느리게 나온다. 지면별 값은 표 3을 본다.' % (fit['b'], fit['a'], fit['points']))
    # ---- T3 by group -----------------------------------------------------------------------------------
    groups = [g for g in sp.GROUPS if R['speed_by_group'].get(g)]
    duties = sorted({d for g in groups for d in R['speed_by_group'][g]})
    missing = R['missing_ranges']
    rows = []
    for d in duties:
        rows.append([d] + [_cellstr(R['speed_by_group'][g].get(d)) for g in groups])
    for lo, hi in missing:
        rows.append([('%d~%d' % (lo, hi)) if hi > lo else '%d' % lo] + ['외삽(자료 없음)'] * len(groups))
    rows.sort(key=lambda x: float(str(x[0]).split('~')[0]))
    w.table('T3_speed_by_ground', '표 3. 지면 묶음별 duty → 정상 직진 속도(PW-03, 주 표)',
            ['duty %'] + groups, rows,
            note='칸 = 속도 m/s [등급(참조가 LiDAR가 아니면 참조 이름), 구간 수]. "외삽(자료 없음)" 줄은 어느 기록에도 정착된 직진 '
                 '구간이 없는 duty 범위다(그림 1·2의 회색 띠). 10/3 과수원은 열 안/밖을 나눌 궤적이 없다. 10/10 "그 밖"에는 열 끝 공간, '
                 '콘크리트 농로, 카탈로그에 없는 행간이 섞여 있다.')
    rows = []
    for d in duties:
        row = [d]
        for g in groups:
            c = R['speed_by_group'][g].get(d)
            row += ([F(c['v']), c['grade'], c['ref'], c['n'], F(c['p10']), F(c['p90']), F(c['seconds'], 1),
                     F(c['lidar']['v']), c['lidar']['n'], F(c['gnss']['v']), c['gnss']['n'], F(c['carto']['v']),
                     c['carto']['n']] if c and c['n'] else [''] * 13)
        rows.append(row)
    hdr = ['duty %']
    for g in groups:
        hdr += ['%s %s' % (g, k) for k in ('속도', '등급', '참조', 'n', 'p10', 'p90', '초', 'LiDAR', 'LiDAR n', 'GNSS',
                                           'GNSS n', 'Carto', 'Carto n')]
    common.write_csv(os.path.join(w.dir, 'T3_speed_by_ground_full.csv'), hdr, rows)
    # by day
    rows = []
    for d in sorted({d for day in R['speed_by_day'] for d in R['speed_by_day'][day]}):
        rows.append([d] + [_cellstr(R['speed_by_day'][day].get(d)) for day in ('1003', '1010')])
    w.table('T3c_speed_by_day', '표 3c. 날짜별 duty → 정상 직진 속도', ['duty %', '10/3(도로 5 + 과수원 2)', '10/10(과수원 6)'],
            rows)
    # by record
    shorts = list(R['speed_by_record'])
    rd = sorted({d for s in shorts for d in R['speed_by_record'][s]})
    rows = []
    for d in rd:
        rows.append([d] + [('%.3f (%d%s)' % (c['v'], c['n'], '' if c['ref'] == 'LiDAR' else ' ' + c['ref'])) if c and c['n']
                           else '' for c in (R['speed_by_record'][s].get(d) for s in shorts)])
    w.table('T3b_speed_by_record', '표 3b. 기록별 duty → 정상 직진 속도(구간 수)', ['duty %'] + shorts, rows,
            note='칸 = 구간 중앙값들의 중앙값 m/s (구간 수, LiDAR가 아니면 참조 이름).')
    # ---- T8 reference reproduction -------------------------------------------------------------------
    ref = R['reference']
    s10 = R['sample_level']['1010']
    st10 = [e for e in R['starts'] if e['day'] == '1010' and e['cls'] == 'straight_fwd' and e['precise']]
    on10 = [e for e in st10 if e.get('onset_s') is not None]
    clean10 = sorted([e for e in R['stops'] if e['day'] == '1010' and e['clean']], key=lambda e: e['dist_m'])
    seg10 = R['speed_by_day']['1010']
    rows = []

    def add(item, expected, got, tol, method):
        diff = None if (got is None or expected is None or isinstance(expected, list)) else got - expected
        ok = None
        if isinstance(expected, list):
            ok = len(expected) == len(got) and all(abs(a - b) <= tol for a, b in zip(expected, got))
            got_txt = ', '.join(F(x, 3) for x in got)
            exp_txt = ', '.join(F(x, 3) for x in expected)
        else:
            ok = diff is not None and abs(diff) <= tol
            got_txt, exp_txt = F(got, 3), F(expected, 3)
        rows.append([item, exp_txt, got_txt, F(diff, 3), '±%g' % tol, '재현' if ok else '다름', method])
    add('10/10 주행축(°)', ref['axis_1010'], R['axis']['1010']['axis_deg'], .05, 'lidar_response.py 2절 규칙')
    add('10/10 100 % 속도 m/s(0.4 s 표본 중앙값)', ref['speed_1010_100'], s10[100]['p10_p50_p90'][1], ref['speed_tol'],
        'lidar_response.py settled_response, 표본 %d개' % s10[100]['samples'])
    add('10/10 90 % 속도 m/s(0.4 s 표본 중앙값)', ref['speed_1010_90'], s10[90]['p10_p50_p90'][1], ref['speed_tol'],
        '같은 규칙, 표본 %d개' % s10[90]['samples'])
    add('10/10 100 % 속도 m/s(구간 단위, 이번 규칙)', ref['speed_1010_100'], _v(seg10.get(100), 4), ref['speed_tol'],
        '구간 %d개의 중앙값들의 중앙값' % seg10[100]['n'])
    add('10/10 90 % 속도 m/s(구간 단위, 이번 규칙)', ref['speed_1010_90'], _v(seg10.get(90), 4), ref['speed_tol'],
        '구간 %d개' % seg10[90]['n'])
    add('10/10 정지 출발 사건(움직임 잡힘)', ref['start_events_1010'], len(on10), 0, 'start_onset_1010.py 규칙(전체 %d건)' % len(st10))
    add('출발 때 duty p50 %', ref['start_duty_p50'], float(np.median([e['duty_at_onset'] for e in on10])), .5, '')
    add('램프 시작 → 움직임 p50 s', ref['start_onset_p50'], float(np.median([e['onset_s'] for e in on10])), .005, '')
    add('10/10 깨끗한 놓기 정지 건수', ref['stop_clean_1010'], len(clean10), 0, 'from_release.py 규칙')
    add('깨끗한 놓기 정지 거리 m', ref['stop_clean_dist'], [e['dist_m'] for e in clean10], .0015, '')
    add('깨끗한 놓기 정지 시간 s', ref['stop_clean_time'], [e['time_s'] for e in clean10], .005, '')
    for duty in (100, 95, 90, 88, 70, 58, 55, 45, 40):
        v = R['sample_level']['1003'].get(duty)
        if v:
            rows.append(['10/3 %d %% 0.4 s 표본 중앙값(참고: 11/11 1차 계산)' % duty, '', F(v['p10_p50_p90'][1]), '', '',
                         '참고', '표본 %d개' % v['samples']])
    w.table('T8_reference_check', '표 8. 기존 값 재현(PW-03·PW-04 완료 기준)',
            ['항목', '기대값', '이번 값', '차이', '허용', '판정', '방법'], rows,
            note='0.899·0.715는 0.4 s 속도 표본(±2 스캔)을 기록 구분 없이 모은 중앙값이다. 이번 주 표는 같은 표본을 구간(2 s 넘게 '
                 '정착, 첫 1 s 제외, 직진)으로 먼저 묶고 구간 중앙값들의 중앙값을 낸다. 두 값의 차이는 방법 차이이며 아래 "구간 단위" 줄에 '
                 '그대로 적었다.')
    # ---- T8b reference comparison ---------------------------------------------------------------------
    rows = []
    for g in groups + ['전체']:
        segs = [s for s in R['main_segments'] if (g == '전체' or s.get('group') == g)]
        for d in sorted({s['duty'] for s in segs}):
            ds = [s for s in segs if s['duty'] == d]
            both_g = [(s['lidar_v'], s['gnss_v']) for s in ds if s.get('lidar_v') and s.get('gnss_v')]
            both_c = [(s['lidar_v'], s['carto_v']) for s in ds if s.get('lidar_v') and s.get('carto_v')]
            if not both_g and not both_c:
                continue
            rows.append([g, d, len(both_g), F(float(np.median([b / a for a, b in both_g])) if both_g else None),
                         F(float(np.median([b - a for a, b in both_g])) if both_g else None),
                         len(both_c), F(float(np.median([b / a for a, b in both_c])) if both_c else None)])
    w.table('T8b_reference_compare', '표 8b. 참조끼리 비교: 같은 구간의 GNSS/LiDAR, Cartographer/LiDAR',
            ['지면 묶음', 'duty %', 'LiDAR·GNSS 둘 다 있는 구간', 'GNSS/LiDAR 중앙값', 'GNSS−LiDAR 중앙값 m/s',
             'LiDAR·Cartographer 둘 다 있는 구간', 'Cartographer/LiDAR 중앙값'], rows,
            note='어느 참조도 정답이 아니다(LiDAR 정합 거리를 줄자로 대조한 적이 없음). 비율의 퍼짐이 절대 속도 눈금의 불확도다.')
    # ---- T10 missing duty ranges -------------------------------------------------------------------
    rows = [[lo, hi, '외삽(표 2·3) / 회색 띠(그림)'] for lo, hi in missing]
    w.table('T10_missing_duty', '표 10. 정착된 직진 구간이 하나도 없는 duty 범위', ['부터 %', '까지 %', '표시'], rows,
            note='표에 쓴 구간(2 s 넘게 정착, 첫 1 s 제외, |yaw| < 2 °/s) 기준. 후진은 −100 %% 구간 %d개뿐이다.' % len(R['reverse_segments']))
    # ---- T4 starts ------------------------------------------------------------------------------------
    rows = []
    for e in sorted(R['starts'], key=lambda e: (e['day'], e['short'], e['t0'])):
        if e['cls'] != 'straight_fwd':
            continue
        rows.append([e['day'], e['short'], e['kst'], e['ground_label'], e['duty'], '예' if e['precise'] else '아니오',
                     F(e.get('onset_s'), 2), F(e.get('duty_at_onset'), 0), F(e.get('t90_s'), 2), F(e.get('steady_mps')),
                     F(e['last_status_after_t0_s'], 1), F(e.get('d_1'), 2), F(e.get('d_2'), 2), F(e.get('d_5'), 2),
                     F(e.get('d_10'), 2)])
    w.table('T4_start_events', '표 4. 정지에서 직진 출발 사건(PW-04)',
            ['날', '기록', 'KST', '지면', '요청 duty %', '램프 시작 정밀', '램프 시작→움직임 s', '움직일 때 duty %',
             '정상 속도 90 %까지 s', '정상 속도 m/s', '요청 유지 s', '1 s 거리 m', '2 s 거리 m', '5 s 거리 m', '10 s 거리 m'], rows,
            note='start_onset_1010.py 규칙(측정). 움직임 = LiDAR 이동 속도 > 0.05 m/s가 스캔 3개 연속(±2 스캔 중심 창이라 최대 '
                 '약 0.2 s 이르게 잡힘). 램프 시작은 MCU 보고 duty로 역산(100 %/s). 거리 = 램프 시작부터 주행축 방향 LiDAR 이동, '
                 '요청이 그 시간 넘게 유지된 사건만. 움직임이 비어 있는 사건은 LiDAR 정합이 품질 기준을 못 넘은 경우다.')
    rows = []
    for day in ('1003', '1010', '합'):
        evs = [e for e in R['starts'] if e['cls'] == 'straight_fwd' and e['precise'] and (day == '합' or e['day'] == day)]
        on = [e for e in evs if e.get('onset_s') is not None]
        rows.append([day, len(evs), len(on), _q([e['onset_s'] for e in on]), _q([e['duty_at_onset'] for e in on], 0),
                     _q([e.get('t90_s') for e in on]), ', '.join(sorted({str(e['duty']) for e in evs})),
                     '측정' if len(on) >= 5 else '참고'])
    w.table('T4s_start_summary', '표 4s. 출발 요약', ['날', '사건', '움직임 잡힘', '램프 시작→움직임 s p10/50/90',
                                                     '움직일 때 duty % p10/50/90', '정상 속도 90 %까지 s p10/50/90',
                                                     '요청 duty', '등급'], rows,
            note=_low_start_note(R['starts']))
    # ---- T5 stops -------------------------------------------------------------------------------------
    rows = []
    for e in sorted(R['stops'], key=lambda e: (e['day'], e['short'], e['kst'] or '')):
        rows.append([e['day'], e['short'], e['kst'], e['ground_label'], e['prev_duty'], e['kind'], '예' if e['clean'] else '',
                     '; '.join(e['tags']), e['basis'], F(e.get('speed_at_release')), F(e.get('speed_1s_before')),
                     F(e.get('dist_m')), F(e.get('time_s'), 2), F(e.get('model_dist_m')),
                     F(e.get('dist_from_first_zero_status_m')), F(e.get('time_from_first_zero_status_s'), 2)])
    w.table('T5_stop_events', '표 5. 직진 중 요청 0이 된 사건(PW-04)',
            ['날', '기록', 'KST', '지면', '직전 duty %', '종류', '깨끗함', '태그', '감속 시작 근거', '놓을 때 속도 m/s',
             '놓기 전 1 s 중앙값 m/s', '놓기부터 거리 m', '놓기부터 시간 s', '모델 거리 m', '첫 "요청 0" 상태부터 거리 m',
             '그때부터 시간 s'], rows,
            note='from_release.py 규칙(측정). 깨끗함 = 무장 상태 유지, 다음 요청도 0, 직전 요청이 약 1 s 넘게 유지. 놓기 = MCU가 '
                 'duty를 100 %/s로 내림(감속 시작을 보고 duty로 역산). STOP/해제 = 상태가 disarmed로 바뀜(duty 즉시 0, 감속 시작 '
                 '시각을 알 수 없음). 모델 거리 = v·0.25 s + v²/(2·1.3 m/s²)(SIM 플랜트의 놓기 정지 모델, 10/10 깨끗한 4건에 맞춘 것). '
                 '놓을 때 속도와 놓기 전 1 s 중앙값이 0.15 m/s 넘게 다르면 LiDAR 자세가 그 근처에서 흔들린 것이다.')
    rows = []
    for day in ('1003', '1010', '합'):
        cl = [e for e in R['stops'] if e['clean'] and (day == '합' or e['day'] == day)]
        ok = [e for e in cl if e.get('speed_consistent') is True]
        rows.append([day, len([e for e in R['stops'] if day == '합' or e['day'] == day]), len(cl), len(ok),
                     _q([e['speed_at_release'] for e in ok]), _q([e['dist_m'] for e in ok], 3), _q([e['time_s'] for e in ok]),
                     _q([e['dist_m'] / e['model_dist_m'] for e in ok if e['model_dist_m']]),
                     len([e for e in R['stops'] if e['kind'] == 'STOP/해제' and (day == '합' or e['day'] == day)])])
    w.table('T5s_stop_summary', '표 5s. 정지 요약', ['날', '요청 0 사건', '깨끗한 놓기', '그중 속도 일관', '놓을 때 속도 p10/50/90',
                                                   '거리 m p10/50/90', '시간 s p10/50/90', '거리/모델 p10/50/90', 'STOP/해제 사건'],
            rows, note='속도 일관 = 놓을 때 속도(5 스캔 변위)와 놓기 전 1 s 중앙값의 차이가 0.15 m/s 안(확인할 표본이 없으면 일관으로 '
                       '세지 않음). 요약 통계는 속도 일관 사건만 쓴다. 이 사건들은 모두 0.75~0.98 m/s에서 시작했다. 0.2~0.6 m/s에서의 '
                       '정지와 STOP(즉시 0) 제동 거리는 잰 자료가 없다(이 칸은 모델뿐).')
    # ---- T6 distance -----------------------------------------------------------------------------------
    D = R['distance']
    rows, md_rows = [], []
    for r in D['rows']:
        row = [r['duty'], F(r['v_ss']), r['v_grade'] + ('' if r['v_ref'] in (None, 'LiDAR') else ' ' + r['v_ref'])]
        md = list(row)
        for T in ss.HOLD_TIMES:
            c = r['d_%g' % T]
            row += [F(c['measured'], 2), c['n'], F(c['model'], 2)]
            if c['measured'] is not None:
                g = '측정' if c['n'] >= 5 else '참고'
                md.append('%.2f %s(n %d, %.2f–%.2f) · 모델 %s' % (c['measured'], g, c['n'], c['measured_range'][0],
                                                              c['measured_range'][1], F(c['model'], 2) or '—'))
            else:
                md.append('%s 모델' % F(c['model'], 2) if c['model'] is not None else '— (출발 문턱 아래)')
        c = r['stop']
        row += [F(c['measured'], 2), c['n'], F(c['model'], 2)]
        if c['measured'] is not None:
            md.append('%.2f %s(n %d, %.2f–%.2f) · 모델 %.2f' % (c['measured'], '측정' if c['n'] >= 5 else '참고', c['n'],
                                                            c['measured_range'][0], c['measured_range'][1], c['model']))
        else:
            md.append('%.2f 모델' % c['model'])
        rows.append(row)
        md_rows.append(md)
    hdr = ['duty %', '정상 속도 m/s', '속도 등급']
    for T in ss.HOLD_TIMES:
        hdr += ['%g s 거리 측정 m' % T, '%g s 측정 n' % T, '%g s 거리 모델 m' % T]
    hdr += ['놓기 정지 거리 측정 m', '놓기 정지 n', '놓기 정지 거리 모델 m']
    md_hdr = ['duty %', '정상 속도 m/s', '속도 등급'] + ['%g s 뒤 거리 m' % T for T in ss.HOLD_TIMES] + ['놓기 정지 거리 m']
    note6 = ('측정 칸 = 표 4의 정지 출발 사건(같은 요청 duty, 그 시간 넘게 유지, 정합 품질 통과 스캔 50 %% 이상) 중앙값과 범위, '
             '놓기 정지는 표 5의 깨끗하고 속도가 일관된 놓기 정지 중 그 duty에서 놓은 것. 모델 칸 = 정지에서 요청 duty까지 100 %%/s '
             '램프(펌웨어 255/s), duty가 출발 문턱 %.0f %%(10/10 출발 %d건 중앙값)를 넘으면 움직이기 시작, 속도는 1차 지연 0.25 s'
             '(가정)로 표 2의 정상 속도를 따라감. 놓기 정지 모델 = v·0.25 + v²/(2·1.3). 출발 문턱 아래 duty(30 %%)는 정지에서 출발 '
             '못 할 수 있어 비웠다. 표 2를 그대로 쓰므로 90 %% 칸(구간 대부분이 10/10 과수원 035952)이 88 %%보다 느려 80~90 %%에서 모델 '
             '거리가 늘지 않는다. 모델 칸은 측정이 아니다.' % (D['breakaway_duty'], D['breakaway_n']))
    w.table('T6_distance', '표 6. 정지에서 duty를 T초 유지했을 때 이동거리와 놓기 정지 거리(PW-04)', hdr, rows, note=note6,
            md_rows=None)
    # replace the md of T6 with the compact view
    common.write_text(os.path.join(w.dir, 'T6_distance.md'), '# 표 6. 정지에서 duty를 T초 유지했을 때 이동거리(PW-04)\n\n%s\n\n%s\n\n%s\n'
                      % (OLD_CAR, note6, common.md_table(md_hdr, md_rows)))
    w.md[-1] = '## 표 6. 정지에서 duty를 T초 유지했을 때 이동거리와 놓기 정지 거리(PW-04)\n\n' + w.md[-1].split('\n\n', 2)[1] + \
        '\n\n' + common.md_table(md_hdr, md_rows) + '\n'
    # ---- T7 session -------------------------------------------------------------------------------------
    S = R['session']
    head = ('**배터리 효과로 단정하지 않는다.** 까닭: (1) 배터리 전압·전류가 어디에도 기록되지 않았다. (2) 기록마다 지면·경사가 다르고 '
            '같은 지면 묶음 안에서도 구간마다 노면이 다르다. (3) 거의 100 % duty뿐이다. (4) 충전 상태를 모르고 깊게 쓴 날이 없다.')
    rows = []
    for r in S['records']:
        gtxt = '; '.join('%s %.3f (%d)' % (g, v['v'], v['n']) for g, v in (r.get('by_group') or {}).items())
        rows.append([r['day'], r['short'], r['kst_first'], r['kst_last'], G.get(r['ground'], r['ground']),
                     '%s (%d)' % (F(r['LiDAR']['v']), r['LiDAR']['n']), '%s (%d)' % (F(r['GNSS']['v']), r['GNSS']['n']), gtxt])
    w.table('T7_session_records', '표 7. 100 % 직진 속도를 실제 시각 순으로(기록별, PW-05)',
            ['날', '기록', '첫 구간 KST', '끝 구간 KST', '지면', 'LiDAR 중앙값 m/s (구간)', 'GNSS 중앙값 m/s (구간)',
             '지면 묶음별 LiDAR (구간)'], rows, note=head)
    rows = []
    for g in S['groups']:
        for name in ('LiDAR', 'GNSS'):
            a, b = g.get(name + '_segments'), g.get(name + '_records')
            pts = g.get(name + '_record_points') or []
            rows.append([g['day'], g['group'], name, a and a['n'],
                         a and '%+.3f ± %.3f' % (a['slope_per_h'], a['se_per_h']),
                         a and '%+.1f' % (100 * a['rel_slope_per_h']), len(pts),
                         b and '%+.3f ± %.3f' % (b['slope_per_h'], b['se_per_h']),
                         ', '.join('%s %.3f' % (p[2], p[1]) for p in pts)])
    w.table('T7b_session_slopes', '표 7b. 같은 날·같은 지면 묶음 안의 시간 기울기(100 % 직진)',
            ['날', '지면 묶음', '참조', '구간 수', '기울기 m/s/h ± 표준오차(구간)', '기울기 %/h(구간)', '기록 수',
             '기울기 m/s/h ± 표준오차(기록 중앙값)', '기록 중앙값'], rows,
            note=head + ' 구간 단위 표준오차는 같은 기록 안 구간들이 독립이 아니라 실제보다 작다. 기록 단위는 점이 2~5개뿐이다.')
    rows = []
    for x in S['first_last']:
        rows.append([x['day'], x['scope'], x['ref'], '%s (%s)' % (x['first'], x['first_kst']),
                     '%s (%s)' % (x['last'], x['last_kst']), F(x['v_first']), F(x['v_last']), F(x['ratio'])])
    w.table('T7c_first_last', '표 7c. 날의 첫 기록 대 마지막 기록(100 % 직진 속도)',
            ['날', '범위', '참조', '첫 기록(KST)', '마지막 기록(KST)', '첫 m/s', '마지막 m/s', '마지막/첫'], rows, note=head)
    # ---- T9 segments ------------------------------------------------------------------------------------
    rows = []
    for s in R['segments']:
        if not s.get('used'):
            continue
        rows.append([s['day'], s['short'], s.get('kst'), s['duty'], s.get('group'), F(s['window_s'], 2), s['n_status'],
                     F(s.get('lidar_v')), s.get('lidar_n'), F(s.get('lidar_cover'), 2), F(s.get('lidar_yaw_dps'), 2),
                     F(s.get('imu_yaw_dps'), 2), '예' if s.get('straight_ok', True) else '아니오(|yaw| ≥ 2 °/s)',
                     F(s.get('gnss_v')), s.get('gnss_n'), F(s.get('carto_v')), F(s.get('inrow_frac'), 2)])
    common.write_csv(os.path.join(w.dir, 'T9_segments.csv'),
                     ['날', '기록', 'KST(창 가운데)', 'duty %', '지면 묶음', '창 s', '상태 수', 'LiDAR m/s', 'LiDAR 표본',
                      'LiDAR 비율', 'LiDAR yaw °/s', 'IMU yaw °/s', '직진', 'GNSS m/s', 'GNSS 기원 수', 'Cartographer m/s',
                      '열 안 비율'], rows)
    w.finish()
    return w


def write_results(R, out_dir):
    keep = {k: v for k, v in R.items() if k not in ('recs', 'by_rid', 'segments', 'main_segments', 'config')}
    keep['segments'] = [{k: v for k, v in s.items()} for s in R['segments']]
    common.write_json(os.path.join(out_dir, 'results.json'), keep)
    common.write_json(os.path.join(out_dir, 'INPUTS.json'), dict(
        note='SHA-256 of every input file read (paths relative to data_root of the config). No positions are stored.',
        files=dict(sorted(R['inputs'].items()))))
