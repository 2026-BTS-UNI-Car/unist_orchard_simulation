from pathlib import Path
import threading
import time

class MappingOperations:
    def __init__(self,live,archives):
        self.live=live;self.archives=archives;self.lock=threading.Lock()
        self.last_resource=(0.,0.);self.resources={}

    def status(self,client=None):
        now=time.monotonic()
        with self.lock:
            if now-self.last_resource[0]>=2:
                try:
                    import resource
                    r=resource.getrusage(resource.RUSAGE_SELF);c=resource.getrusage(resource.RUSAGE_CHILDREN)
                    import os
                    ticks=os.sysconf('SC_CLK_TCK');children=[];pending=[]
                    for task in Path('/proc/self/task').glob('*/children'):
                        pending.extend(task.read_text().split())
                    seen=set()
                    while pending:
                        pid=pending.pop()
                        if pid in seen:continue
                        seen.add(pid)
                        try:
                            # Read process times only, never command arguments.
                            fields=Path('/proc',pid,'stat').read_text().rsplit(')',1)[1].split()
                            children.append((int(fields[11])+int(fields[12]))/ticks)
                            for task in Path('/proc',pid,'task').glob('*/children'):
                                pending.extend(task.read_text().split())
                        except (OSError,ValueError,IndexError):pass
                    cpu=r.ru_utime+r.ru_stime+c.ru_utime+c.ru_stime+sum(children)
                    elapsed=now-self.last_resource[0]
                    percent=None if not self.last_resource[0] else max(0.,(cpu-self.last_resource[1])/elapsed*100)
                    rss=None
                    for row in Path('/proc/self/status').read_text().splitlines():
                        if row.startswith('VmRSS:'):rss=int(row.split()[1])*1024
                    self.resources=dict(cpu_percent_one_core=percent,rss_bytes=rss,scope='dashboard_and_mapper_children',
                        running_child_cpu_included=True,running_child_count=len(children))
                    self.last_resource=(now,cpu)
                except (ImportError,OSError,ValueError):self.resources=dict(scope='unavailable')
            resources=dict(self.resources)
        return dict(ok=True,live_first=True,max_recorded_workers=1,resources=resources,
            jobs=[self.live.task_status(),*self.archives.jobs(client)],autonomy_output_enabled=False)

    def stop(self,job_id,expected_epoch=None):
        if job_id=='live':
            result=self.live.action('stop',expected_epoch=expected_epoch)
            if result.get('ok') is False:return result
            return dict(ok=True,state='stopped',partial_map_preserved=True)
        if isinstance(job_id,str) and job_id.startswith('archive:'):
            return self.archives.stop_job(job_id[8:],expected_epoch)
        raise ValueError('unknown_mapping_job')
