// Native 2-bit cells, compressed before base64. Browser-native streaming
// decompression avoids per-tile image codecs and preserves the exact XOR chain.
export async function decodeArchiveWindow(value) {
  if(value.cell_codec!=='xor2-deflate-v1')return value;
  const size=value.cell_blob_bytes;
  if(!Number.isInteger(size)||size<0||size>64*1024*1024||size%1024||
     typeof value.cell_blob!=='string'||value.cell_blob.length>24*1024*1024)
    throw new Error('Invalid compressed map window');
  const data=Uint8Array.from(atob(value.cell_blob),c=>c.charCodeAt(0));
  const reader=new Blob([data]).stream().pipeThrough(new DecompressionStream('deflate')).getReader();
  const cells=new Uint8Array(size);let offset=0;
  try{
    while(true){
      const {value:chunk,done}=await reader.read();if(done)break;
      if(offset+chunk.length>size)throw new Error('Map window exceeds declared cell bytes');
      cells.set(chunk,offset);offset+=chunk.length;
    }
  }finally{await reader.cancel().catch(()=>{});reader.releaseLock();}
  if(offset!==size)throw new Error('Truncated compressed map window');
  const decode=frame=>({...frame,tiles:(frame.tiles||[]).map(tile=>{
    const start=tile.cells;
    if(!['packed2-v1','xor2-v1'].includes(tile.encoding)||!Number.isInteger(start)||start<0||start%1024||start+1024>size)
      throw new Error('Invalid map cell block offset');
    return {...tile,cells:btoa(String.fromCharCode(...cells.subarray(start,start+1024)))};
  })});
  const {cell_blob,cell_blob_bytes,...window}=value;
  return {...window,cell_codec:'xor2-v1',seed:decode(value.seed),events:value.events.map(decode)};
}
