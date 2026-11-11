// Optional display controller; no producer start/stop, fetch or Worker at import.
import {NativeProbabilityView} from './native-probability/view.mjs';

// Keep transport details in diagnostics, never in the product status label.
export function displayStatus(value) {
  if(value.startsWith('Retained complete'))return '관찰 종료 · 마지막 완료 지도 표시';
  if(value.startsWith('Inactive'))return '표시 비활성 · 완료 지도 대기';
  if(value.includes('stopped')||value.includes('client_stopped'))return '표시 멈춤 · 마지막 완료 지도 유지';
  if(value.includes('loading_complete_snapshot'))return '새 완료 지도 준비 중 · 마지막 화면 유지';
  if(value.startsWith('Latest complete'))return '완료 지도 표시 · 관찰 전용';
  if(value.includes('no_complete')||value.includes('awaiting_complete'))return '첫 완료 지도 준비 중';
  return '연결 대기 · 마지막 완료 지도 유지';
}

export function attachNativeView({document:doc=globalThis.document,
  active=()=>globalThis.location.pathname.replace(/\/$/,'')==='/rover-control'&&!doc.hidden,
  viewFactory=options=>new NativeProbabilityView(options)}={}) {
  if(doc.documentElement.dataset.nativeViewEnabled!=='true')return null;
  const element=id=>doc.getElementById(id),panel=element('simNativeMapPanel');
  if(!panel||!element('simNativeMapCanvas'))throw Error('native_view_panel_missing');
  const listeners=[];let desired=false,destroyed=false;
  const listen=(target,name,fn)=>{if(target){target.addEventListener(name,fn);listeners.push([target,name,fn]);}};
  const shown=()=>!destroyed&&active()&&panel.open!==false;
  const status={set textContent(value){element('simNativeMapStatus').textContent=displayStatus(String(value));}};
  const view=viewFactory({pollMs:100,canvas:element('simNativeMapCanvas'),statusElement:status,
    onView:value=>{
      const count=value.submaps.reduce((n,s)=>n+s.known_cells,0);
      element('simNativeMapSummary').textContent=`관측 셀 ${count.toLocaleString()}개 · 기존 지도와 별도로 표시 · 주행 제어에 사용하지 않음`;
    },onInspect:cell=>{
      element('simNativeMapCell').textContent=cell?`선택 셀 확률 ${cell.probability.toFixed(5)} · 지도 좌표 ${cell.map_center.slice(0,2).map(v=>v.toFixed(3)).join(', ')} m`:'선택한 관측 셀 없음';
    }});
  function buttons(){element('simNativeMapStart').disabled=desired;element('simNativeMapStop').disabled=!desired;}
  function routeChanged(){if(destroyed)return;if(desired&&shown())view.start();else view.stop();buttons();}
  function start(){if(destroyed)return;desired=true;routeChanged();}
  function stop(){desired=false;view.stop();buttons();}
  panel.hidden=false;
  listen(element('simNativeMapStart'),'click',start);
  listen(element('simNativeMapStop'),'click',stop);
  listen(element('simNativeMapFit'),'click',()=>view.fit());
  listen(element('simNativeMapResume'),'click',()=>view.setAutoFit(true));
  listen(element('simNativeMapPalette'),'change',event=>view.setPalette(event.target.value));
  listen(panel,'toggle',routeChanged);
  listen(doc,'visibilitychange',routeChanged);
  buttons();
  return {start,stop,routeChanged,state:()=>({desired,producerControlConnected:false,...view.state()}),
    destroy(){if(destroyed)return;desired=false;destroyed=true;view.destroy();for(const [target,name,fn]of listeners)target.removeEventListener(name,fn);listeners.length=0;}};
}
