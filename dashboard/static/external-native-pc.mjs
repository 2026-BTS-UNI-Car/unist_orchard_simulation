const ORIGIN='http://127.0.0.1:18891';
export function admittedView(value){
  if(!value||Object.getPrototypeOf(value)!==Object.prototype||value.ok!==true||value.profile!=='external_native_pc'||value.configured!==true||value.external_managed!==true||value.read_only!==true||value.map_health_observed!==false||value.native_completion_or_sensor_freshness_proven!==false||!(/^[0-9a-f]{32}$/).test(value.session_id)||typeof value.iframe_url!=='string')return null;
  const match=(/^http:\/\/127\.0\.0\.1:18891\/\?client=2&token=([0-9a-f]{64})$/).exec(value.iframe_url);
  return match?{session_id:value.session_id,url:ORIGIN+'/?client=2&token='+match[1]}:null;
}
export function attachExternalNativePC({active}){
  const panel=document.getElementById('simLiveMapPanel');let pending=false,configured=false;
  for(const id of ['simNativeOwnerPanel','simNativeMapPanel','simMapHistoryPanel','simArchiveMapPanel','simNavigationPanel','simMappingJobsPanel']){const node=document.getElementById(id);if(node)node.hidden=true;}
  for(const node of panel.querySelectorAll('.sim-map-actions,.map-view-toolbar,.sim-map-stage,.sim-map-legend,.sim-map-note'))node.hidden=true;
  for(const id of ['simMapStart','simMapStop','simMapReset','simArchiveMapRegenerate']){const node=document.getElementById(id);if(node)node.disabled=true;}
  const driveMode=document.getElementById('simDriveMode');if(driveMode){driveMode.value='manual';driveMode.disabled=true;}
  document.getElementById('simMapState').textContent='주행 지도';
  const status=document.getElementById('simMapStatus');status.textContent='지도 보기를 준비하고 있습니다.';
  const host=document.createElement('div');host.id='externalNativePCView';panel.append(host);
  const link=document.createElement('a');link.textContent='지도 크게 열기';link.target='_blank';link.rel='noopener noreferrer';link.hidden=true;host.append(link);
  const frame=document.createElement('iframe');frame.title='주행 지도 · 읽기 전용';frame.referrerPolicy='no-referrer';frame.loading='lazy';frame.style.cssText='width:100%;height:500px;max-height:70vh;border:0';frame.hidden=true;host.append(frame);
  async function routeChanged(){
    if(!active()||pending||configured)return;pending=true;
    try{
      const response=await fetch('/api/sim/external-native-pc-view',{cache:'no-store',signal:AbortSignal.timeout(3000)});
      const raw=await response.text();if(raw.length>4096)throw new Error('descriptor_response_bound');
      const value=response.ok?admittedView(JSON.parse(raw)):null;if(!value)throw new Error('fixed_client2_view_unavailable');
      link.href=value.url;link.hidden=false;frame.src=value.url;frame.hidden=false;configured=true;
      status.textContent='지도 보기를 준비했습니다. 이 화면은 조회 전용입니다.';
    }catch{status.textContent='지도를 아직 열 수 없습니다. 센서 화면은 계속 사용할 수 있습니다.';}
    finally{pending=false;}
  }
  return {routeChanged};
}
