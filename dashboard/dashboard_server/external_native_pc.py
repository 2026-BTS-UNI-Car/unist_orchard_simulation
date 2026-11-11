"""Explicit external mapping display; never constructs a mapper or controller."""
import json,os,re,stat
from pathlib import Path
from urllib.parse import urlencode
DESCRIPTOR=Path('/runtime/NATIVE_PC_VIEW.private.json')
ORIGIN='http://127.0.0.1:18891'
DASHBOARD_ORIGIN='http://127.0.0.1:18080'
def require(ok,reason):
 if not ok:raise ValueError(reason)
def parse_descriptor(raw):
 require(type(raw)is bytes and 0<len(raw)<=4096,'bounded_external_native_view_descriptor')
 def unique(items):
  result={}
  for key,value in items:require(key not in result,'duplicate_external_native_view_field');result[key]=value
  return result
 value=json.loads(raw,object_pairs_hook=unique,parse_constant=lambda _:require(False,'finite_external_native_view_JSON'))
 require(type(value)is dict and set(value)=={'schema','session_id','origin','client','read_token','read_only'}and value['schema']=='ubts.sim.external_native_pc_view.v1','exact_external_native_view_schema')
 require(type(value['session_id'])is str and re.fullmatch('[0-9a-f]{32}',value['session_id'])and value['origin']==ORIGIN and type(value['client'])is int and value['client']==2 and type(value['read_token'])is str and re.fullmatch('[0-9a-f]{64}',value['read_token'])and value['read_only']is True,'fixed_session_client2_loopback_read_only_view')
 return value
def read_descriptor():
 before=DESCRIPTOR.lstat();require(stat.S_ISREG(before.st_mode)and 0<before.st_size<=4096,'regular_nonlink_external_view_descriptor')
 identity=lambda st:(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)
 flags=os.O_RDONLY|getattr(os,'O_NOFOLLOW',0)|getattr(os,'O_CLOEXEC',0)|getattr(os,'O_BINARY',0)
 with os.fdopen(os.open(DESCRIPTOR,flags),'rb')as stream:
  opened=os.fstat(stream.fileno());require(stat.S_ISREG(opened.st_mode)and identity(before)==identity(opened),'same_external_view_descriptor_open')
  raw=stream.read(4097);ended=os.fstat(stream.fileno())
 after=DESCRIPTOR.lstat();require(stat.S_ISREG(after.st_mode)and identity(before)==identity(opened)==identity(ended)==identity(after)and len(raw)==before.st_size and before.st_ctime_ns==after.st_ctime_ns and opened.st_ctime_ns==ended.st_ctime_ns,'same_external_view_descriptor_lifetime')
 for first,last in((before,after),(opened,ended)):
  if hasattr(first,'st_birthtime_ns'):require(first.st_birthtime_ns==last.st_birthtime_ns,'same_API_external_view_birthtime')
 return parse_descriptor(raw)
def admit_request(method,host,origin,site):
 require(method=='GET'and host=='127.0.0.1:18080','external_view_literal_loopback_GET_only')
 require(origin==DASHBOARD_ORIGIN or(origin is None and site in('same-origin','same-site')),'external_view_same_site_request_only')
def view_link(value):
 # Construction follows strict descriptor validation; there is no arbitrary URL.
 value=parse_descriptor((json.dumps(value,separators=(',',':'))+'\n').encode())
 return dict(ok=True,profile='external_native_pc',session_id=value['session_id'],iframe_url=ORIGIN+'/?'+urlencode({'client':'2','token':value['read_token']}),configured=True,external_managed=True,read_only=True,map_health_observed=False,native_completion_or_sensor_freshness_proven=False)
def managed(*_,**__):
 return dict(ok=False,state='externally_managed',error='external_native_pc_manages_mapping',mapping_profile='external_native_pc',active=None,native_ACK=False,runtime_authority=False)
class ExternalNativePC:
 """No threads, UDS, ROS, SDK, store, history or native owner is created."""
 snapshot=action=begin_archive=finish_archive=archive_snapshot=regenerate=task_status=status=stop=managed
 def view(self):return view_link(read_descriptor())
