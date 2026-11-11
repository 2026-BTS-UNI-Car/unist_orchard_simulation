/* Read-only display of field commissioning evidence. No requests/commands. */
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
  const api={render};
  if (typeof window!=='undefined') window.FieldPreparation=api;
  if (typeof module!=='undefined') module.exports=api;
})();
