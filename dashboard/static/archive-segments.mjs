// Disconnected coordinate frames remain separate. A preview is never treated
// as a transform linking them; navigation seeks all recorded sensors together.
export function visibleSegments(catalog, time) {
  return (catalog || []).filter(row => row.start_s <= time).map(row => ({
    segment:row.segment, start_s:row.start_s, end_s:Math.min(row.end_s,time),
  }));
}

export class ArchiveSegmentGallery {
  constructor(map, seek) {
    this.map=map;this.seek=seek;this.previews=new Map();this.epoch=null;
    this.element=document.getElementById('liveArchiveSegments');
    this.key='';
  }
  reset() {this.previews.clear();this.epoch=null;this.key='';this.element?.replaceChildren();}
  beforeCommit(previous,next) {
    if(next.map_epoch && this.epoch!==next.map_epoch){this.reset();this.epoch=next.map_epoch;}
    if(previous.segment!=null && previous.segment!==next.segment && previous.map_epoch===next.map_epoch && previous.grid){
      try{
        const image=document.createElement('canvas');image.width=160;image.height=96;
        image.getContext('2d').drawImage(this.map.canvas,0,0,160,96);
        this.previews.set(previous.segment,{url:image.toDataURL('image/png'),time:previous.frame_t_s});
      }catch{} // A missing thumbnail never hides a retained source map.
    }
  }
  update(snapshot,time) {
    if(!this.element)return;
    const rows=visibleSegments(snapshot.segment_catalog,time);
    this.rows=rows;
    const key=JSON.stringify([snapshot.map_epoch,snapshot.segment,rows.map(r=>r.segment),[...this.previews.keys()]]);
    if(key===this.key){
      [...this.element.children].forEach((button,i)=>{
        button.querySelector('span').textContent=this.label(rows[i]);
      });return;
    }
    this.key=key;
    this.element.replaceChildren();
    this.element.hidden=rows.length<2;
    for(const row of rows){
      const button=document.createElement('button');button.type='button';
      button.className='map-segment-card';button.setAttribute('aria-pressed',String(row.segment===snapshot.segment));
      const preview=this.previews.get(row.segment);
      if(preview && preview.time<=time){
        const image=document.createElement('img');image.src=preview.url;image.alt='';button.append(image);
      }
      const label=document.createElement('span');
      label.textContent=this.label(row);
      button.append(label);button.addEventListener('click',()=>{
        const current=this.rows.find(value=>value.segment===row.segment);
        if(current)this.seek(current.end_s);
      });this.element.append(button);
    }
  }
  label(row){return `구간 ${row.segment+1} · ${row.start_s.toFixed(1)}–${row.end_s.toFixed(1)}초`;}
}
