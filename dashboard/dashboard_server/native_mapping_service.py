"""One isolated SIM map owner. HTTP/media have a separate Python interpreter."""
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import threading

from .raw_sim_mapping import RawSimulationMapping
from .sim_mapping_operations import MappingOperations

SCHEMA='ubts.native_mapping_rpc.v1'


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(5.)
        try:
            raw=self.rfile.readline(65537)
            if len(raw)>65536 or not raw.endswith(b'\n'):raise ValueError('invalid_request_size')
            request=json.loads(raw)
            if request.get('schema')!=SCHEMA:raise ValueError('invalid_schema')
            op=request.get('op');args=request.get('args',[]);kwargs=request.get('kwargs',{})
            if not isinstance(args,list) or not isinstance(kwargs,dict):raise ValueError('invalid_arguments')
            mapper=self.server.mapper
            methods=dict(snapshot=mapper.snapshot,action=mapper.action,task_status=mapper.task_status,
                navigation=mapper.navigation.snapshot,begin_archive=mapper.begin_archive,
                finish_archive=mapper.finish_archive,archive_snapshot=mapper.archive_snapshot,
                regenerate=mapper.regenerate,jobs=mapper.jobs,stop_job=mapper.stop_job,
                operations=self.server.operations.status,stop_operation=self.server.operations.stop)
            if op=='ping':value=dict(ok=True,schema=SCHEMA)
            elif op in methods:value=methods[op](*args,**kwargs)
            else:raise ValueError('unknown_operation')
            body=json.dumps(dict(ok=True,result=value),allow_nan=False,separators=(',',':')).encode()+b'\n'
            self.wfile.write(body)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            try:self.wfile.write(json.dumps(dict(ok=False,error=type(exc).__name__)).encode()+b'\n')
            except OSError:pass


class Server(socketserver.ThreadingUnixStreamServer):
    request_queue_size=32
    daemon_threads=True
    def __init__(self,path,mapper):
        self.mapper=mapper;self.operations=MappingOperations(mapper,mapper)
        super().__init__(str(path),Handler)


def main():
    if os.environ.get('DASHBOARD_MODE')!='simulation':raise SystemExit('simulation_only')
    state=Path(os.environ.get('SIM_MAPPING_STATE_DIR','/run/ubts-sim/raw-mapping'))
    state.mkdir(parents=True,exist_ok=True)
    owner=(state/'owner.lock').open('a')
    fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
    path=Path(os.environ.get('SIM_MAPPING_SERVICE_SOCKET','/run/ubts-sim/native-mapper.sock'))
    if path.exists():
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as probe:
            probe.settimeout(.5)
            try:probe.connect(str(path))
            except (FileNotFoundError,ConnectionRefusedError):path.unlink(missing_ok=True)
            else:raise RuntimeError('mapper_socket_already_owned')
    mapper=RawSimulationMapping(source_socket=os.environ.get('SIM_RAW_SENSOR_SOCKET','/run/ubts-sim/sensor-source.sock'),
        state_dir=state,archive_dir=os.environ.get('RECORDING_ARCHIVE_DIR','/runtime/recordings'))
    server=Server(path,mapper);os.chmod(path,0o660)
    stopping=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopping.set())
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:stopping.wait()
    finally:
        server.shutdown();server.server_close();mapper.action('stop');mapper.close()
        path.unlink(missing_ok=True);owner.close();thread.join(3)


if __name__=='__main__':main()
