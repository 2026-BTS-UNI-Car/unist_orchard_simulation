/* One recorder-owned conversion queue; this view never starts recordings. */
(() => {
  const panel = document.getElementById('recordingMediaJobs');
  if (!panel) return;
  const list = panel.querySelector('[data-media-list]');
  const note = panel.querySelector('[data-media-note]');
  const rows = new Map();
  const labels = {
    queued:'대기', waiting_priority:'센서·조종 작업 대기', running:'변환 중',
    paused:'일시정지', cancelled:'중단됨', complete:'완료', failed:'실패',
    starting:'준비', verifying_raw_sources:'원본 확인', exporting_native_frames:'원본 영상 추출',
    encoding_adaptive_video:'재생 영상 변환', verifying_generated_video:'결과 확인',
    recording_active:'녹화 우선', rover_control_active:'조종 우선', live_mapping_active:'실시간 지도 우선',
    recorded_mapping_active:'기록 지도 계산 대기', operator_paused:'사용자 일시정지',
    mapping_status_unavailable:'지도 상태 확인 대기', rover_status_unavailable:'조종 상태 확인 대기'
  };
  let loading=false, sequence=0;
  const describe = value => labels[value] || value || '—';
  function rowFor(id) {
    if (rows.has(id)) return rows.get(id);
    const row=document.createElement('div');row.className='media-job';
    const text=document.createElement('span');text.className='media-job-description';row.append(text);
    const controls=document.createElement('span');controls.className='media-job-controls';row.append(controls);
    const item={row,text,buttons:[]};
    for (const [action,label] of [['pause','일시정지'],['resume','재개'],['cancel','중단']]) {
      const button=document.createElement('button');button.type='button';button.className='action-button secondary';
      button.textContent=label;controls.append(button);item.buttons.push({button,action});
      button.addEventListener('click',async()=>{
        item.buttons.forEach(x=>x.button.disabled=true);
        try {
          const response=await fetch('/api/recordings/media-jobs/action',{method:'POST',credentials:'same-origin',
            headers:{'Content-Type':'application/json'},body:JSON.stringify({recording_id:id,action}),signal:AbortSignal.timeout(4000)});
          const result=await response.json();if (!response.ok || !result.ok) throw new Error('action_failed');
        } catch {note.textContent='요청을 완료하지 못했습니다. 현재 상태를 다시 확인하세요.';}
        await refresh();
      });
    }
    rows.set(id,item);list.append(row);return item;
  }
  async function refresh() {
    if (loading) return;
    loading=true;const token=++sequence;
    try {
      const response=await fetch('/api/recordings/media-jobs',{credentials:'same-origin',signal:AbortSignal.timeout(4000)});
      const value=await response.json();if(!response.ok || !value.ok || !Array.isArray(value.jobs)) throw new Error('unavailable');
      if(token!==sequence)return;
      note.textContent=value.jobs.length?'동시에 영상 하나만 처리합니다. 녹화·조종·지도 계산 중에는 대기합니다.':'변환 작업 없음';
      const visible=new Set();
      for(const job of value.jobs.slice(0,24)) {
        if(typeof job.recording_id!=='string')continue;
        visible.add(job.recording_id);const item=rowFor(job.recording_id);
        item.text.textContent=`${job.recording_id} · ${describe(job.state)} · ${describe(job.stage)}${job.reason?' · '+describe(job.reason):''}`;
        for(const {button,action} of item.buttons) {
          button.hidden=job.state==='complete' || (action==='pause' && ['paused','failed','cancelled'].includes(job.state))
            || (action==='resume' && !['paused','failed','cancelled'].includes(job.state)) || (action==='cancel' && job.state==='cancelled');
          button.disabled=false;
        }
      }
      for(const [id,item] of rows)if(!visible.has(id)){item.row.remove();rows.delete(id);}
    } catch {note.textContent='영상 변환 서비스에 연결할 수 없습니다.';}
    finally {loading=false;}
  }
  panel.addEventListener('toggle',()=>{if(panel.open)refresh();});
  panel.querySelector('[data-media-refresh]').addEventListener('click',refresh);
  setInterval(()=>{if(panel.open && !document.hidden && panel.getClientRects().length)refresh();},3000);
})();
