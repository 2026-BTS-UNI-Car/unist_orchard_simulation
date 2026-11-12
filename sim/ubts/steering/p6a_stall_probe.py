"""Stall probe of the steering G-SIM stack (WF-4 g_gaps; docs/steering/P6A_HEADLESS_KO.md section 16).

The steer container (ESP emulator, bridge wrapper) showed pauses of about 335-340 ms: at the stack start both
processes stopped together, once the bridge wrapper alone stopped for 334.8 ms while driving (phase 3, decision R19-9).
This probe runs inside one container (docker exec, same PID namespace and cgroup) and records what the processes were
doing while they made no progress, without touching them:

  every --period-ms (default 10 ms)
    per target process (cmdline match: the emulator, the bridge wrapper, the native lockstep program, the supervisor;
    --match adds more): utime + stime (clock ticks) and the context switch counts of /proc/<pid>/status. A process
    whose counters stay the same for longer than --stall-ms (default 100 ms) is "stalled" (the bridge wakes every
    20 ms, the emulator every /clock of 8.3 ms). While a process has not moved for 40 ms or more, every thread's state
    and wchan (/proc/<pid>/task/<tid>/stat, wchan) are sampled.
    the probe's own wake-up gap (monotonic) and the CLOCK_REALTIME - CLOCK_MONOTONIC offset (a jump of the wall
    clock, as seen after some phase 3 stalls)
  every 100 ms
    the container cgroup v2 files: cpu.stat (usage, nr_throttled, throttled_usec), memory.current, memory.events,
    selected memory.stat fields, cpu / memory / io .pressure (PSI totals), and the VM-wide /proc/pressure/*,
    /proc/meminfo Dirty / Writeback, /proc/loadavg

Output (JSONL, --out): a header line, one summary line per second (CPU % per process, cgroup deltas, PSI deltas,
largest gaps), and one 'stall' line per stall with the samples of the stall window (thread states, wchan, cgroup and
PSI deltas over the window). Stdlib only (Python 3.10+, Linux /proc). CLI:
  python3 -B -m steering.p6a_stall_probe --seconds 600 --out /runtime/steer/stall_probe.jsonl
The probe itself costs a few % of one CPU at 10 ms. Placeholder-based SIM diagnostics, not field validation.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import signal
import sys
import time

SCHEMA = 'ubts.steer_stall_probe.v1'
DEFAULT_MATCH = (('emulator', 'steering.gz_esp_emulator'), ('bridge', 'scripts.steer_rover_bridge'),
                 ('native', 'ubts_steer_native'), ('supervisor', 'steering.supervisor_steer'),
                 ('clip_node', 'scan_unobserved_clip'), ('sensor_contract', 'steering.sensor_contract_steer'),
                 ('cpu_lidar', 'cpu_lidar.py'), ('gz', 'gz sim'), ('ros_gz_bridge', 'parameter_bridge'))
CGROUP = '/sys/fs/cgroup'
MEMORY_STAT_KEYS = ('anon', 'file', 'file_dirty', 'file_writeback', 'shmem', 'pgfault', 'pgmajfault', 'pgscan',
                    'pgsteal', 'workingset_refault_file', 'pgactivate', 'pgdeactivate')
CGROUP_PERIOD_S = .1
DEEP_AFTER_S = .04                      # per-thread sampling once a process has not moved for this long
CLK_TCK = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100


def read(path):
    try:
        with open(path, 'rb') as handle:
            return handle.read().decode('utf-8', 'replace')
    except OSError:
        return None


def proc_cmdline(pid):
    text = read('/proc/%d/cmdline' % pid)
    return None if text is None else text.replace('\0', ' ').strip()


def find_targets(matches):
    """{pid: role} of the processes whose command line holds a match string (the probe itself excluded)."""
    out = {}
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit() or int(name) == me:
            continue
        cmd = proc_cmdline(int(name))
        if not cmd:
            continue
        for role, needle in matches:
            if needle in cmd:
                out[int(name)] = role
                break
    return out


def stat_fields(text):
    """(state, utime, stime, num_threads) of a /proc/.../stat line (the comm may hold spaces and parentheses)."""
    if not text:
        return None
    rest = text[text.rindex(')') + 2:].split()
    return rest[0], int(rest[11]), int(rest[12]), int(rest[17])


def ctxt(text):
    vol = nonvol = 0
    for line in (text or '').splitlines():
        if line.startswith('voluntary_ctxt_switches:'):
            vol = int(line.split()[1])
        elif line.startswith('nonvoluntary_ctxt_switches:'):
            nonvol = int(line.split()[1])
    return vol, nonvol


def rss_kb(text):
    for line in (text or '').splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1])
    return None


def process_sample(pid):
    stat = stat_fields(read('/proc/%d/stat' % pid))
    if stat is None:
        return None
    status = read('/proc/%d/status' % pid)
    vol, nonvol = ctxt(status)
    return dict(state=stat[0], cpu=stat[1] + stat[2], threads=stat[3], vol=vol, nonvol=nonvol, rss_kb=rss_kb(status),
                wchan=(read('/proc/%d/wchan' % pid) or '').strip() or None)


def thread_states(pid):
    """[(tid, comm, state, wchan)] of every thread of pid."""
    out = []
    try:
        tids = os.listdir('/proc/%d/task' % pid)
    except OSError:
        return out
    for tid in sorted(tids, key=int):
        text = read('/proc/%d/task/%s/stat' % (pid, tid))
        if not text:
            continue
        comm = text[text.index('(') + 1:text.rindex(')')]
        state = text[text.rindex(')') + 2:].split()[0]
        wchan = (read('/proc/%d/task/%s/wchan' % (pid, tid)) or '').strip() or None
        out.append((int(tid), comm, state, wchan))
    return out


def kv_file(text):
    out = {}
    for line in (text or '').splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                out[parts[0]] = int(parts[1])
            except ValueError:
                pass
    return out


def pressure(text):
    """{'some': total_us, 'full': total_us} of a PSI file."""
    out = {}
    for line in (text or '').splitlines():
        parts = line.split()
        if not parts:
            continue
        for item in parts[1:]:
            if item.startswith('total='):
                out[parts[0]] = int(item[6:])
    return out


def meminfo(text):
    out = {}
    for line in (text or '').splitlines():
        key, _, rest = line.partition(':')
        if key in ('Dirty', 'Writeback', 'MemAvailable', 'Cached'):
            out[key] = int(rest.split()[0])
    return out


def cgroup_sample():
    stat = kv_file(read(CGROUP + '/cpu.stat'))
    memory = kv_file(read(CGROUP + '/memory.stat'))
    out = dict(mono=time.monotonic(),
               cpu=dict(usage_usec=stat.get('usage_usec'), nr_throttled=stat.get('nr_throttled'),
                        throttled_usec=stat.get('throttled_usec'), nr_periods=stat.get('nr_periods')),
               memory_current=None, memory_max=(read(CGROUP + '/memory.max') or '').strip() or None,
               memory_events=kv_file(read(CGROUP + '/memory.events')),
               memory_stat={k: memory.get(k) for k in MEMORY_STAT_KEYS if k in memory},
               psi=dict(cpu=pressure(read(CGROUP + '/cpu.pressure')), memory=pressure(read(CGROUP + '/memory.pressure')),
                        io=pressure(read(CGROUP + '/io.pressure'))),
               vm_psi=dict(cpu=pressure(read('/proc/pressure/cpu')), memory=pressure(read('/proc/pressure/memory')),
                           io=pressure(read('/proc/pressure/io'))),
               vm_meminfo=meminfo(read('/proc/meminfo')),
               loadavg=(read('/proc/loadavg') or '').split()[:3])
    current = (read(CGROUP + '/memory.current') or '').strip()
    out['memory_current'] = int(current) if current.isdigit() else None
    return out


def delta_dict(a, b):
    out = {}
    for key, value in (b or {}).items():
        if isinstance(value, dict):
            out[key] = delta_dict((a or {}).get(key), value)
        elif isinstance(value, int) and isinstance((a or {}).get(key), int):
            out[key] = value - a[key]
    return out


class Probe:
    def __init__(self, out, matches, period_s=.01, stall_s=.1, keep_s=2., refresh_s=2.):
        self.out = open(out, 'w', encoding='utf-8', newline='\n')
        self.matches = matches
        self.period_s = period_s
        self.stall_s = stall_s
        self.refresh_s = refresh_s
        self.ring = collections.deque(maxlen=max(10, int(keep_s / period_s)))
        self.targets = {}
        self.last_move = {}          # pid -> (mono of the last progress, progress key)
        self.open_stalls = {}        # pid -> dict
        self.cg = None
        self.cg_ring = collections.deque(maxlen=60)
        self.second = None
        self.stalls = 0

    def write(self, entry):
        self.out.write(json.dumps(entry, separators=(',', ':'), default=str) + '\n')

    def refresh_targets(self, now):
        found = find_targets(self.matches)
        for pid in list(self.targets):
            if pid not in found:
                self.write(dict(kind='target_gone', pid=pid, role=self.targets[pid], mono=round(now, 4)))
                self.targets.pop(pid)
                self.last_move.pop(pid, None)
        for pid, role in found.items():
            if pid not in self.targets:
                self.targets[pid] = role
                self.write(dict(kind='target', pid=pid, role=role, mono=round(now, 4), cmd=proc_cmdline(pid)[:200]))

    def run(self, seconds, stop):
        start = time.monotonic()
        self.write(dict(kind='header', schema=SCHEMA, wall=time.time(), mono=start, period_s=self.period_s,
                        stall_s=self.stall_s, clk_tck=CLK_TCK, pid=os.getpid(), cgroup=read('/proc/self/cgroup'),
                        result_kind='자리값 기반 SIM 결과', field_validated=False))
        self.refresh_targets(start)
        next_t = start
        last_wake = start
        last_refresh = start
        last_cg = 0.
        offset0 = time.time() - time.monotonic()
        summary = self.new_summary(start)
        while not stop() and time.monotonic() - start < seconds:
            next_t += self.period_s
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
            now = time.monotonic()
            gap = now - last_wake
            last_wake = now
            offset = time.time() - now
            if abs(offset - offset0) > .02:
                self.write(dict(kind='realtime_jump', mono=round(now, 4), jump_s=round(offset - offset0, 4),
                                probe_gap_s=round(gap, 4)))
            offset0 = offset
            if now - last_refresh >= self.refresh_s:
                last_refresh = now
                self.refresh_targets(now)
            sample = dict(mono=round(now, 4), gap_ms=round(gap * 1000., 2), procs={})
            for pid, role in list(self.targets.items()):
                p = process_sample(pid)
                if p is None:
                    continue
                key = (p['cpu'], p['vol'], p['nonvol'])
                moved_at, last_key = self.last_move.get(pid, (now, None))
                if key != last_key:
                    if pid in self.open_stalls:
                        self.close_stall(pid, now)
                    self.last_move[pid] = (now, key)
                    moved_at = now
                idle = now - moved_at
                if idle >= DEEP_AFTER_S:
                    p['threads_detail'] = thread_states(pid)
                if idle >= self.stall_s and pid not in self.open_stalls:
                    self.open_stall(pid, role, moved_at, now)
                if pid in self.open_stalls:
                    self.open_stalls[pid]['samples'].append(dict(mono=round(now, 4), state=p['state'],
                                                                 wchan=p['wchan'],
                                                                 threads=p.get('threads_detail')))
                p['idle_ms'] = round(idle * 1000., 1)
                sample['procs'][role + ':%d' % pid] = p
                name = '%s:%d' % (role, pid)
                summary['last'][name] = p['cpu']
                summary['first'].setdefault(name, p['cpu'])
            self.ring.append(sample)
            summary['max_gap_ms'] = max(summary['max_gap_ms'], sample['gap_ms'])
            if now - last_cg >= CGROUP_PERIOD_S:
                last_cg = now
                cg = cgroup_sample()
                self.cg_ring.append(cg)
                if summary['cg0'] is None:
                    summary['cg0'] = cg
                summary['cg1'] = cg
            if now - summary['t0'] >= 1.:
                self.flush_summary(summary, now)
                summary = self.new_summary(now)
        for pid in list(self.open_stalls):
            self.close_stall(pid, time.monotonic(), open_at_end=True)
        self.flush_summary(summary, time.monotonic())
        self.write(dict(kind='end', mono=time.monotonic(), stalls=self.stalls))
        self.out.close()

    def new_summary(self, now):
        return dict(t0=now, first={}, last={}, max_gap_ms=0., cg0=None, cg1=None)

    def flush_summary(self, summary, now):
        span = max(now - summary['t0'], 1e-6)
        cpu = {role: round((summary['last'][role] - summary['first'][role]) / CLK_TCK / span * 100., 1)
               for role in summary['last']}
        entry = dict(kind='second', mono=round(now, 3), span_s=round(span, 3), cpu_pct=cpu,
                     probe_max_gap_ms=summary['max_gap_ms'])
        if summary['cg0'] is not None and summary['cg1'] is not None:
            entry['cgroup_delta'] = delta_dict(summary['cg0'], summary['cg1'])
            entry['memory_current'] = summary['cg1']['memory_current']
            entry['memory_stat'] = summary['cg1']['memory_stat']
            entry['vm_meminfo'] = summary['cg1']['vm_meminfo']
        self.write(entry)
        self.out.flush()

    def open_stall(self, pid, role, moved_at, now):
        before = [s for s in self.ring if s['mono'] >= moved_at - .05]
        self.open_stalls[pid] = dict(pid=pid, role=role, moved_at=round(moved_at, 4), opened=round(now, 4),
                                     cg_before=self.cg_ring[-1] if self.cg_ring else None,
                                     samples=[dict(mono=s['mono'], gap_ms=s['gap_ms'],
                                                   state=(s['procs'].get('%s:%d' % (role, pid)) or {}).get('state'),
                                                   wchan=(s['procs'].get('%s:%d' % (role, pid)) or {}).get('wchan'))
                                              for s in before])

    def close_stall(self, pid, now, open_at_end=False):
        stall = self.open_stalls.pop(pid)
        stall['resumed'] = round(now, 4)
        stall['duration_ms'] = round((now - stall['moved_at']) * 1000., 1)
        stall['open_at_end'] = open_at_end
        cg_after = cgroup_sample()
        stall['cgroup_delta'] = delta_dict(stall.pop('cg_before'), cg_after) if stall.get('cg_before') else None
        others = {}
        for other, (moved_at, _) in self.last_move.items():
            if other != pid:
                others['%s:%d' % (self.targets.get(other, '?'), other)] = round((now - moved_at) * 1000., 1)
        stall['others_idle_ms_at_resume'] = others
        stall['probe_gaps_ms'] = [s['gap_ms'] for s in self.ring if s['mono'] >= stall['moved_at']]
        states = collections.Counter(s.get('state') for s in stall['samples'])
        stall['main_states'] = dict(states)
        wchans = collections.Counter(s.get('wchan') for s in stall['samples'])
        stall['main_wchan'] = dict(wchans.most_common(5))
        thread_states_seen = collections.Counter()
        for s in stall['samples']:
            for tid, comm, state, wchan in s.get('threads') or ():
                thread_states_seen['%s|%s|%s' % (comm, state, wchan)] += 1
        stall['thread_states'] = dict(thread_states_seen.most_common(12))
        stall['kind'] = 'stall'
        self.stalls += 1
        self.write(stall)
        self.out.flush()


def build_parser():
    p = argparse.ArgumentParser(prog='p6a_stall_probe', description=__doc__.splitlines()[0])
    p.add_argument('--seconds', type=float, default=600.)
    p.add_argument('--out', required=True)
    p.add_argument('--period-ms', type=float, default=10.)
    p.add_argument('--stall-ms', type=float, default=100.)
    p.add_argument('--match', action='append', default=[], help='ROLE=SUBSTRING of the command line (more targets)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    matches = list(DEFAULT_MATCH)
    for item in args.match:
        role, _, needle = item.partition('=')
        matches.append((role, needle))
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.append(1))
    probe = Probe(args.out, matches, period_s=args.period_ms / 1000., stall_s=args.stall_ms / 1000.)
    probe.run(args.seconds, lambda: bool(stopped))
    return 0


if __name__ == '__main__':
    sys.exit(main())
