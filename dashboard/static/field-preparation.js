/* Field evidence and explicit OBSERVE_ONLY controls. No motor requests. */
(() => {
  const labels = {
    profile:'관측용 시험 설정', native_sources:'휠 엔코더 없는 센서 입력',
    body_dimensions:'현재 차체 치수 실측', sensor_extrinsics:'LiDAR·IMU 장착 위치와 방향',
    planar_envelope:'기울기·진동에서 센서 유효 범위', body_velocity_feedback:'실제 속도 피드백·PWM 반응',
    physical_emergency_stop:'모터 전원 비상 차단', braking:'시험 속도에서 정지 거리·시간',
    october_source_review_complete:'10월 3일 전체 센서 자료 검토',
    september_orchard_review_complete:'9월 10일 과수원 자료 검토',
    current_vehicle_localization_validated:'현재 차량 지도·위치 정확도',
    live_latency_under_load_validated:'부하 중 실시간 센서·지도 지연',
    obstacle_and_unknown_space_stop_validated:'장애물·미관측 영역 정지',
    row_end_and_turn_space_validated:'열 끝·회전 공간 판단',
    manual_takeover_verified:'수동 인계·통신 상실 정지', work_boundary_defined:'과수원 작업 경계',
    live_mapping_fresh:'현재 지도 수신·정합 상태'
  };
  function render(container, report) {
    if (!container) return;
    if (typeof window!=='undefined') mountObservation(container);
    const valid=report?.schema==='ubts.field_preflight.v1' && Array.isArray(report.checks);
    const checks=valid ? report.checks.slice(0,32).filter(c=>c&&typeof c.id==='string') : [];
    const passed=checks.filter(c=>c.passed===true).length;
    const summary=container.querySelector('summary'),list=container.querySelector('[data-preparation-checks]');
    summary.textContent=valid ? `현장 시험 준비 · 확인 ${passed}/${checks.length} · 자동 구동 보류` : '현장 시험 준비 · 점검 자료 수신 대기';
    const key=JSON.stringify(checks.map(c=>[c.id,c.passed===true]));
    if (container.dataset.preparationKey===key) return;
    container.dataset.preparationKey=key;
    const fragment=container.ownerDocument.createDocumentFragment();
    for (const c of checks) {
      const item=container.ownerDocument.createElement('li');
      item.textContent=`${c.passed===true?'확인':'미완료'} · ${labels[c.id]||c.id}`;
      item.className=c.passed===true?'field-check-passed':'field-check-pending';
      fragment.appendChild(item);
    }
    list.replaceChildren(fragment);
  }
  function observationView(value) {
    if (!value || value.runtime_authority!==false || value.motor_command_sent!==false ||
        value.autonomy_output_enabled!==false || typeof value.active!=='boolean') {
      throw new Error('관찰 전용 상태를 확인할 수 없습니다.');
    }
    const decision=value.current_decision || {};
    const proposed=[decision.proposed_left_percent ?? 0,decision.proposed_right_percent ?? 0];
    const applied=[decision.applied_left_percent ?? 0,decision.applied_right_percent ?? 0];
    if (proposed.some(n=>typeof n!=='number'||!Number.isFinite(n)||n<0||n>30) ||
        applied.some(n=>n!==0)) throw new Error('관찰 전용 출력 계약이 일치하지 않습니다.');
    const age=value.input_age || {};
    const ms=n=>Number.isSafeInteger(n)&&n>=0 ? `${(n/1e6).toFixed(1)} ms` : '확인 불가';
    const trajectory=Array.isArray(decision.trajectory) ? decision.trajectory.slice(0,16) : [];
    return {
      active:value.active,
      state:`OBSERVE_ONLY · ${value.state || '상태 대기'}`,
      reason:value.first_failure_reason || value.stop_reason || decision.reason || '입력 대기',
      decision:decision.decision || 'HOLD',
      proposed:`좌 ${proposed[0].toFixed(1)}% / 우 ${proposed[1].toFixed(1)}% · 추정 제안`,
      applied:'좌 0% / 우 0% · 모터 전송 없음',
      speed:`기존 속도 옵션 ${Number.isFinite(value.speed_percent)?value.speed_percent:'—'}% / 관찰 선호 ${Number.isFinite(value.cruise_percent)?value.cruise_percent:'—'}% · 실측 m/s 교정 없음`,
      freshness:`LiDAR source ${ms(age.scan_source_ns)} / receipt ${ms(age.scan_receipt_ns)} · IMU source ${ms(age.imu_source_ns)} / receipt ${ms(age.imu_receipt_ns)}`,
      trajectory:`센서 평면 계획 ${trajectory.length}점 · BODY·장착·제동 미검증`,
      source:`${value.source?.context?.source_clock_domain || '원본 시계 검증 대기'} / ${value.source?.context?.receipt_clock_domain || 'receipt 검증 대기'} · ${value.native_source?.enabled===false?'legacy WEB WS 미검증':(value.native_source?.status?.owner?.first_failure_reason || value.native_source?.error || value.native_source?.reason || value.native_source?.state || 'native 입력 상태 확인 대기')}`
    };
  }

  function observationController({request,display,error,busy}) {
    let revision=0,polling=false,actions=0,chain=Promise.resolve();
    async function refresh() {
      if (polling||actions) return;
      polling=true;const current=revision;
      try {
        const value=await request('/api/autonomy/observe');
        if (current===revision) display(observationView(value));
      } catch (failure) {if(current===revision)error(failure.message);}
      finally {polling=false;}
    }
    function command(kind) {
      if (!['start','stop'].includes(kind)) throw new Error('관찰 작업이 잘못되었습니다.');
      const current=++revision;actions++;busy(true);
      // A queued Stop follows the Start response. Late polls/Start responses
      // cannot repaint a newer Stop intent. No action is retried automatically.
      chain=chain.catch(()=>{}).then(async()=>{
        try {
          const value=await request(`/api/autonomy/observe/${kind}`,{});
          if(current===revision)display(observationView(value));
        } catch (failure) {if(current===revision)error(failure.message);}
        finally {actions--;if(!actions)busy(false);}
      });
      return chain;
    }
    return {refresh,start:()=>command('start'),stop:()=>command('stop')};
  }

  async function observationRequest(path,payload) {
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),5000);
    try {
      const response=await fetch(path,{method:payload===undefined?'GET':'POST',
        credentials:'same-origin',cache:'no-store',signal:abort.signal,
        headers:payload===undefined?{}:{'Content-Type':'application/json'},
        body:payload===undefined?undefined:JSON.stringify(payload)});
      const type=response.headers?.get?.('content-type')||'';
      let value;
      try{
        if(type&&!type.toLowerCase().includes('json'))throw new Error('not_json');
        value=await response.json();
      }catch(error){if(error?.name==='AbortError')throw error;throw new Error(`HTTP ${response.status}`);}  // S41: proxy HTML error pages
      if(!response.ok)throw new Error(value.error || `HTTP ${response.status}`);
      return value;
    } finally {clearTimeout(timer);}
  }

  function mountObservation(container) {
    if (!container || container._autonomyObservation || !container.ownerDocument?.createElement) return;
    const document=container.ownerDocument;
    const panel=document.createElement('section');panel.className='field-observation';
    const title=document.createElement('h3');title.textContent='자율주행 관찰 · OBSERVE_ONLY';panel.appendChild(title);
    const notice=document.createElement('p');
    notice.textContent='센서·판단·추정 제안만 관찰합니다. 기존 속도 옵션을 30% 이하로 설정한 뒤 시작하세요. 지도·수동 조종은 별개이며 이 관찰의 모터 적용은 항상 0입니다.';
    panel.appendChild(notice);
    const controls=document.createElement('div');controls.className='field-observation-actions';
    const start=document.createElement('button'),stop=document.createElement('button');
    for(const button of [start,stop])button.type='button';
    start.className='action-button secondary';start.textContent='관찰 Start';
    stop.className='action-button danger';stop.textContent='관찰 Stop';
    controls.appendChild(start);controls.appendChild(stop);panel.appendChild(controls);
    const nodes={};
    for(const [key,label] of [['state','상태'],['decision','열 판단'],['reason','HOLD / 사유'],
        ['proposed','제안'],['applied','관찰 적용'],['speed','속도 옵션'],['freshness','입력 경과'],
        ['trajectory','계획'],['source','시계 출처']]) {
      const row=document.createElement('p'),name=document.createElement('strong'),value=document.createElement('span');
      name.textContent=label+' · ';value.textContent=key==='applied'?'좌 0% / 우 0% · 모터 전송 없음':'상태 수신 대기';
      row.appendChild(name);row.appendChild(value);panel.appendChild(row);nodes[key]=value;
    }
    const status=document.createElement('p');status.className='field-observation-notice';
    status.setAttribute('role','status');status.textContent='기본 OFF · 관찰 Start는 자동 구동 허가가 아닙니다.';panel.appendChild(status);
    const control=observationController({request:observationRequest,
      display:model=>{for(const key of Object.keys(nodes))nodes[key].textContent=model[key];
        status.textContent=model.active?'관찰 중 · BODY·장착·제동·실차 정확도 미검증':'관찰 OFF · 마지막 판단은 현재 입력으로 사용하지 않습니다.';},
      error:message=>{status.textContent=`관찰 상태 확인 실패 · ${message} · 마지막 표시 유지`;},
      busy:pending=>{start.disabled=pending;status.textContent=pending?'관찰 작업 응답 확인 중 · 모터 적용 0':status.textContent;}});
    // This optional display never calls Start/Stop on sensors or motors itself.
    import('./native-accepted-map.mjs').then(module=>module.mountNativeAcceptedMap(container)).catch(()=>{});
    start.addEventListener('click',()=>control.start().then(()=>container._nativeMapView?.start()));
    stop.addEventListener('click',()=>{container._nativeMapView?.stop();return control.stop();});
    container.appendChild(panel);container._autonomyObservation=control;
    const refresh=()=>{if(!document.hidden&&container.offsetParent!==null)control.refresh();};
    setInterval(refresh,1000);refresh();
  }

  const api={render,observationView,observationController};
  if (typeof window!=='undefined') window.FieldPreparation=api;
  if (typeof module!=='undefined') module.exports=api;
  if (typeof document!=='undefined') {
    const mount=()=>mountObservation(document.getElementById('fieldPreparationPanel'));
    if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount,{once:true});else mount();
  }
})();
