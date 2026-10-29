export type ShaftPoint = {axis:number;sequence:string;segment:string;monotonic_ns:string;utc_ns:string;time_ms:number;position_cdeg:number;gap:boolean;ordinal?:number};
export type ShaftAxisStatus = {axis:number;limit_bytes:number;file_bytes:number;retained_records:number;fresh:boolean;age_seconds:number|null;recording:boolean;error:string|null;first_ms:number|null;last_ms:number|null;latest:ShaftPoint|null;corrupt_records:number};
export type ShaftStatus = {recording:boolean;connected:boolean;source_dropped:number;missing_samples:number;writer_dropped:number;queued:number;saved:number;cursor:number;server_time_ms:number;axes:ShaftAxisStatus[]};
export type ShaftFrame = {cursor:number;expired:boolean;samples:ShaftPoint[];status:ShaftStatus};
export type ShaftHistory = {axes:{axis:number;samples:ShaftPoint[];next_cursor:number|null}[];status:ShaftStatus};
export type YScale = {auto:boolean;minimum:number;maximum:number};
export const traceBase = "/api/v1/diagnostics/trace";
export const emptyScales = ():YScale[] => Array.from({length:6},()=>({auto:true,minimum:-1,maximum:1}));
export function positionText(point:ShaftPoint|null|undefined) {return point ? (point.position_cdeg/100).toFixed(2)+"°" : "—";}
export function valueAt(points:ShaftPoint[],time:number):ShaftPoint|null {
    let lo=0,hi=points.length;
    while(lo<hi) {const mid=(lo+hi)>>>1;if(points[mid].time_ms<=time) lo=mid+1;else hi=mid;}
    const before=points[lo-1],after=points[lo];
    if(before?.time_ms===time) return before;
    if(!before||!after||after.gap||after.segment!==before.segment||after.time_ms-before.time_ms>80) return null;
    return time-before.time_ms<=after.time_ms-time?before:after;
}
export function reduceTrace(points:ShaftPoint[],maximum:number):ShaftPoint[] {
    if(points.length<=maximum) return points;
    const buckets=Math.max(1,Math.floor(maximum/2)),result:ShaftPoint[]=[];
    for(let i=0;i<buckets;i++) {
        const chunk=points.slice(Math.floor(i*points.length/buckets),Math.floor((i+1)*points.length/buckets));
        let min=0,max=0;
        chunk.forEach((p,j)=>{if(p.position_cdeg<chunk[min].position_cdeg) min=j;if(p.position_cdeg>chunk[max].position_cdeg) max=j;});
        const gap=chunk.some(p=>p.gap);
        for(const j of [...new Set([min,max])].sort((a,b)=>a-b)) result.push({...chunk[j],gap:chunk[j].gap||gap});
    }
    return result;
}
export function mergeTrace(old:ShaftPoint[],incoming:ShaftPoint[],start:number):ShaftPoint[] {
    const values=new Map<string,ShaftPoint>();
    for(const p of [...old,...incoming]) if(p.time_ms>=start) values.set(`${p.segment}/${p.sequence}`,p);
    return reduceTrace([...values.values()].sort((a,b)=>a.time_ms-b.time_ms),4000);
}
export function yRange(points:ShaftPoint[],scale:YScale):[number,number] {
    if(!scale.auto) return [scale.minimum,scale.maximum];
    if(!points.length) return [-1,1];
    let min=Infinity,max=-Infinity;
    for(const p of points) {min=Math.min(min,p.position_cdeg/100);max=Math.max(max,p.position_cdeg/100);}
    const padding=Math.max(.05,(max-min)*.1);return [min-padding,max+padding];
}
