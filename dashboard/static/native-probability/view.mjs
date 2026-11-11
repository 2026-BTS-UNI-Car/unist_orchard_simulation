// SPDX-License-Identifier: Apache-2.0
// Reusable canvas component. No Worker/fetch until explicit start().
import {inversePoint,worldAt,pan,zoomAt,fit} from './display_geometry.mjs';
export class NativeProbabilityView {
  #canvas;#ctx;#status;#worker=null;#factory;#makeCanvas;#makeImageData;#models=[];#bounds=null;#identity=null;
  #camera={scale:5,tx:0,ty:0};#autoFit=true;#active=false;#drag=null;#visible=new Map();#palette='gray';#onView;#onInspect;
  #inspectBusy=false;#inspectPending=null;#inspectRequest=0;#resizeObserver=null;#listeners=[];
  constructor({canvas,pollMs,statusElement=null,onView=()=>{},onInspect=()=>{},
    workerFactory=url=>new Worker(url,{type:'module'}),
    makeCanvas=()=>document.createElement('canvas'),makeImageData=(bytes,w,h)=>new ImageData(bytes,w,h)}={}) {
    if(!canvas?.getContext)throw Error('canvas_required');this.#canvas=canvas;this.#ctx=canvas.getContext('2d');if(!this.#ctx)throw Error('canvas_2d_required');
    if(pollMs!==undefined&&(!Number.isSafeInteger(pollMs)||pollMs<100||pollMs>5000))throw Error('client_budget');
    this.#factory=()=>{const url=new URL('./native_worker.mjs',import.meta.url);if(pollMs!==undefined)url.searchParams.set('pollMs',String(pollMs));return workerFactory(url);};
    this.#status=statusElement;this.#makeCanvas=makeCanvas;this.#makeImageData=makeImageData;this.#onView=onView;this.#onInspect=onInspect;
    this.#bind();this.#message('Inactive - optional native submap display');
  }
  state(){return {active:this.#active,identity:this.#identity,camera:{...this.#camera},models:this.#models.length,autoFit:this.#autoFit};}
  #message(text){if(this.#status)this.#status.textContent=text;}
  start(){if(this.#active)return;this.#active=true;
    try{const worker=this.#worker=this.#factory();worker.onmessage=e=>{if(this.#active&&worker===this.#worker)this.#receive(e.data);};
      worker.onerror=()=>this.#message('Stale - Worker failure; previous complete view retained');
      worker.postMessage({kind:'palette',palette:this.#palette});worker.postMessage({kind:'start'});
    }catch(error){this.#active=false;this.#worker?.terminate();this.#worker=null;this.#message('Stale - '+error.message);}
  }
  stop(){this.#active=false;try{this.#worker?.postMessage({kind:'stop'});}finally{this.#worker?.terminate();this.#worker=null;}
    this.#inspectBusy=false;this.#inspectPending=null;this.#message('Stale - stopped; previous complete view retained');}
  refresh(){if(this.#active)this.#worker.postMessage({kind:'refresh'});}
  setPalette(palette){if(!['gray','positive'].includes(palette))throw Error('palette_mode');this.#palette=palette;this.#worker?.postMessage({kind:'palette',palette});}
  setSubmapVisible(trajectory,index,visible){if(typeof visible!=='boolean')throw Error('visibility');
    if(this.#identity)this.#visible.set(this.#identity.publisher_epoch+':'+trajectory+':'+index,visible);this.render();}
  #fitCurrent(){if(this.#bounds){this.#camera=fit(this.#bounds,this.#canvas.clientWidth,this.#canvas.clientHeight);this.render();}}
  fit(){this.#autoFit=false;this.#fitCurrent();} // Explicit Fit once retains this viewport on future batches.
  setAutoFit(enabled){if(typeof enabled!=='boolean')throw Error('auto_fit_boolean');this.#autoFit=enabled;if(enabled)this.#fitCurrent();}
  resize(){const dpr=globalThis.devicePixelRatio||1;this.#canvas.width=Math.round(this.#canvas.clientWidth*dpr);this.#canvas.height=Math.round(this.#canvas.clientHeight*dpr);this.render();}
  #shown(m){return this.#visible.get(this.#identity.publisher_epoch+':'+m.metadata.trajectory_id+':'+m.metadata.submap_index)!==false;}
  render(){const dpr=globalThis.devicePixelRatio||1,c=this.#ctx,v=this.#camera;c.setTransform(1,0,0,1,0,0);c.clearRect(0,0,this.#canvas.width,this.#canvas.height);c.imageSmoothingEnabled=false;
    for(const m of this.#models)if(this.#shown(m)){const [a,b,d,e,x,y]=m.affine;
      c.setTransform(dpr*v.scale*a,-dpr*v.scale*b,dpr*v.scale*d,-dpr*v.scale*e,dpr*(v.scale*x+v.tx),dpr*(-v.scale*y+v.ty));c.drawImage(m.image,0,0);}}
  #receive(data){
    try {
      if(data?.kind==='state'){const s=data.state;this.#message((s.producerStopped===true&&this.#identity?'Retained complete - producer stopped; ':s.stale?'Stale - ':'Latest complete - ')+(s.reason??'native submaps; display only'));}
      else if(data?.kind==='error')this.#message('Stale - '+data.reason);
      else if(data?.kind==='commit')this.#swap(data.event);
      else if(data?.kind==='inspect'){
        this.#inspectBusy=false;if(data.request===this.#inspectRequest)this.#onInspect(data.cell);
        this.#sendInspect();
      }
    }catch(error){this.#message('Stale - '+error.message+'; previous complete view retained');}
  }
  #swap(event){
    if(!event?.prepared||!Array.isArray(event.prepared.models)||event.prepared.models.length>256)throw Error('prepared_models');
    const models=[];let cells=0;
    // Validate the complete batch budget before creating any detached canvas.
    for(const m of event.prepared.models){const [w,h]=m.metadata.crop_dimensions_n0_n1;
      if(!Number.isInteger(w)||!Number.isInteger(h)||w<1||h<1||w>16384||h>16384||(cells+=w*h)>4194304||
        !(m.rgba instanceof ArrayBuffer)||m.rgba.byteLength!==w*h*4||m.affine.length!==6||!m.affine.every(Number.isFinite))throw Error('prepared_render_budget');
    }
    const bounds=event.prepared.bounds;
    if(bounds!==null&&(!Array.isArray(bounds)||bounds.length!==4||!bounds.every(Number.isFinite)||bounds[0]>bounds[2]||bounds[1]>bounds[3]))throw Error('prepared_bounds');
    const camera=this.#autoFit&&bounds?fit(bounds,this.#canvas.clientWidth,this.#canvas.clientHeight):this.#camera;
    if(![camera.scale,camera.tx,camera.ty].every(Number.isFinite)||camera.scale<=0)throw Error('prepared_camera');
    // Build all detached canvases before replacing any visible membership.
    for(const m of event.prepared.models){const [w,h]=m.metadata.crop_dimensions_n0_n1;
      const image=this.#makeCanvas();image.width=w;image.height=h;
      const context=image.getContext('2d');if(!context)throw Error('submap_canvas_2d');
      context.putImageData(this.#makeImageData(new Uint8ClampedArray(m.rgba),w,h),0,0);const {rgba,...rest}=m;models.push({...rest,image});
    }
    this.#models=models;this.#bounds=bounds;this.#camera=camera;this.#identity=Object.freeze({...event.identity});
    const liveKeys=new Set(models.map(m=>this.#identity.publisher_epoch+':'+m.metadata.trajectory_id+':'+m.metadata.submap_index));
    this.#visible=new Map([...this.#visible].filter(([key])=>liveKeys.has(key)));
    this.#inspectPending=null;this.#inspectRequest++;this.resize();
    this.#message((event.producerStopped===true?'Retained complete - producer stopped; ':event.stale?'Stale - ':'Latest complete - ')+models.length+' independent submaps; no fused posterior');
    this.#onView({identity:this.#identity,submaps:models.map(m=>({...m.metadata,display_visible:this.#shown(m)})),stale:event.stale});
  }
  #sendInspect(){if(!this.#inspectBusy&&this.#inspectPending&&this.#worker){const p=this.#inspectPending;this.#inspectPending=null;this.#inspectBusy=true;this.#worker.postMessage(p);}}
  #hover(event){if(!this.#identity||!this.#worker)return;const box=this.#canvas.getBoundingClientRect(),[x,y]=worldAt(this.#camera,event.clientX-box.left,event.clientY-box.top);
    const request=++this.#inspectRequest,candidates=[];
    for(const m of [...this.#models].reverse())if(this.#shown(m)){const [u,v]=inversePoint(m.affine,x,y),i0=Math.floor(u),i1=Math.floor(v),[w,h]=m.metadata.crop_dimensions_n0_n1;
      if(i0>=0&&i1>=0&&i0<w&&i1<h)candidates.push({trajectory_id:m.metadata.trajectory_id,submap_index:m.metadata.submap_index,index:i0+w*i1});}
    this.#inspectPending=candidates.length?{kind:'inspect',request,identity:this.#identity,candidates}:null;
    if(!candidates.length)this.#onInspect(null);this.#sendInspect();
  }
  #listen(name,fn,options){this.#canvas.addEventListener(name,fn,options);this.#listeners.push([name,fn,options]);}
  #bind(){
    this.#listen('pointerdown',e=>{this.#drag={x:e.clientX,y:e.clientY};this.#canvas.setPointerCapture(e.pointerId);});
    this.#listen('pointerup',()=>{this.#drag=null;});this.#listen('pointercancel',()=>{this.#drag=null;});
    this.#listen('pointermove',e=>{if(this.#drag){if(e.clientX!==this.#drag.x||e.clientY!==this.#drag.y)this.#autoFit=false;this.#camera=pan(this.#camera,e.clientX-this.#drag.x,e.clientY-this.#drag.y);this.#drag={x:e.clientX,y:e.clientY};this.render();}this.#hover(e);});
    this.#listen('wheel',e=>{e.preventDefault();this.#autoFit=false;const b=this.#canvas.getBoundingClientRect();this.#camera=zoomAt(this.#camera,e.clientX-b.left,e.clientY-b.top,Math.exp(-e.deltaY*.001));this.render();},{passive:false});
    if(typeof ResizeObserver!=='undefined'){this.#resizeObserver=new ResizeObserver(()=>this.resize());this.#resizeObserver.observe(this.#canvas);}
  }
  destroy(){this.stop();for(const [name,fn,o]of this.#listeners)this.#canvas.removeEventListener(name,fn,o);this.#resizeObserver?.disconnect();this.#listeners=[];}
}
