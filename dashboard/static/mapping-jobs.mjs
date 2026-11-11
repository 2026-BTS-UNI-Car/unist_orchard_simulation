// Structured text only: no server error/recording ID becomes executable HTML.
const stages={idle:'유휴',drawing:'그리는 중',waiting_for_sensors:'센서 대기',
  waiting_for_geometry:'위치 다시 찾는 중',needs_reset:'새 지도 필요',worker_error:'처리 오류',
  registering_scan:'스캔 정합',queued_scan:'스캔 처리 대기',waiting_sensors:'센서 대기',
  waiting_live_priority:'실시간 매핑 우선 · 기록 대기',verifying_source:'원본 확인',
  localizing:'위치 추정·지도 저장',paused:'일시정지',waiting_cursor:'재생 시각 대기',
  stopping:'작업 정리 중',stopped:'중단됨',complete:'완료',failed:'오류'};
export function jobLabel(job){
  if(job.kind==='native-observer')return `추가 지도 관찰 · ${{idle:'대기',starting:'준비 중',running:'관찰 중',stopping:'정리 중',stopped:'멈춤',failed:'오류'}[job.state]||'연결 대기'} · 완성 지도 ${job.progress?.complete_batches||0}회 · 주행 제어 없음`;
  const head=job.kind==='live'?'실시간 지도':`기록 ${job.recording_id}`;
  const state=stages[job.state]||job.state||stages[job.stage]||job.stage||'대기';
  const elapsed=Number.isFinite(job.last_progress_age_s)?` · 최근 지도 갱신 ${job.last_progress_age_s.toFixed(1)}초 전`:'';
  const progress=job.kind==='archive'?` · ${Number(job.processed_t_s||0).toFixed(1)} / ${Number(job.requested_t_s||0).toFixed(1)}초 요청`:` · 입력 ${job.input_scans||0} / 지도 ${job.scan_count||0} scans · 큐 ${job.queue_scans||0}`;
  const delay=Number.isFinite(job.diagnostic?.receipt_to_map_ms)?` · 처리 지연 ${job.diagnostic.receipt_to_map_ms.toFixed(0)}ms`:'';
  const trouble=job.error||job.reason;
  return `${head} · ${state}${progress}${delay}${elapsed}${trouble?` · ${trouble}`:''}`;
}
export function renderJobs(container,value,onStop){
  container.replaceChildren();
  for(const job of value.jobs||[]){
    const row=document.createElement('div');row.className='mapping-job-row';
    const label=document.createElement('p');label.textContent=jobLabel(job);row.append(label);
    if(job.active||job.tracking||job.processing){
      const button=document.createElement('button');button.type='button';button.className='action-button secondary';
      button.textContent='이 작업 중단';button.setAttribute('aria-label',`${job.kind==='native-observer'?'추가 지도 관찰':job.kind==='live'?'실시간 지도':job.recording_id} 매핑 중단`);
      button.addEventListener('click',async()=>{button.disabled=true;try{await onStop(job);}finally{button.disabled=false;}});
      row.append(button);
    }
    container.append(row);
  }
}
