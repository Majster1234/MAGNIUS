import {expect,it,vi} from "vitest";
import {ShaftHistoryCache} from "./shaftHistoryCache";
import type {ShaftHistory,ShaftPoint} from "./shaftTrace";
const point=(time:number,q=0):ShaftPoint=>({axis:1,sequence:String(time),segment:"1",monotonic_ns:String(time*1000000),utc_ns:String(time*1000000),time_ms:time,position_cdeg:q,gap:false});
const data=(start:number,end:number):ShaftHistory=>({status:{recording:true,connected:true,server_time_ms:1000000,source_dropped:0,missing_samples:0,writer_dropped:0,queued:0,saved:0,cursor:0,axes:[]},
    axes:Array.from({length:6},(_,i)=>({axis:i+1,samples:i===0?Array.from({length:11},(_,j)=>point(start+(end-start)*j/10)):[],next_cursor:null}))});
const flush=async()=>{for(let i=0;i<20;i++) await Promise.resolve();};

it("pobiera widok i sąsiednie okna, a przesuwanie wewnątrz bufora nie wymaga HTTP",async()=>{
    const read=vi.fn(async(a:number,b:number)=>data(a,b)),cache=new ShaftHistoryCache(read,()=>{},()=>{},()=>{});
    cache.setView(10000,20000,false,1000000);await flush();
    expect(read.mock.calls.map(r=>r[0]).sort((a,b)=>a-b)).toEqual([0,10000,20000,30000]);
    expect(cache.select(0,10000,[])[0].length).toBeGreaterThan(0);
    cache.setView(12000,22000,false,1000000);await flush();expect(read).toHaveBeenCalledTimes(4);
    expect(cache.loading).toBe(false);cache.dispose();
});
it("podczas ciągłego przesuwania nie anuluje pobrania i utrzymuje najwyżej dwa zapytania",async()=>{
    const pending:{a:number;b:number;signal:AbortSignal;resolve:(d:ShaftHistory)=>void}[]=[];
    const read=vi.fn((a:number,b:number,signal:AbortSignal)=>new Promise<ShaftHistory>(resolve=>pending.push({a,b,signal,resolve})));
    const cache=new ShaftHistoryCache(read,()=>{},()=>{},()=>{});
    cache.setView(10000,20000,false,1000000);
    for(let t=21000;t<=30000;t+=1000) cache.setView(t-10000,t,false,1000000);
    expect(pending).toHaveLength(2);expect(pending.every(p=>!p.signal.aborted)).toBe(true);
    pending[0].resolve(data(pending[0].a,pending[0].b));await flush();
    expect(cache.requestsInFlight).toBe(2);expect(pending.length).toBe(3);
    expect(pending[2].a).toBe(30000); // najnowszy widok, bez kolejki dawnych ruchów
    cache.dispose();expect(pending.every(p=>p.signal.aborted||p===pending[0])).toBe(true);
});
it("ogranicza cache do ośmiu przedziałów i ponownie wykorzystuje sąsiadów przy zmianie kierunku",async()=>{
    const read=vi.fn(async(a:number,b:number)=>data(a,b)),cache=new ShaftHistoryCache(read,()=>{},()=>{},()=>{});
    for(let t=20000;t<200000;t+=10000) {cache.setView(t-10000,t,false,1000000);await flush();expect(cache.retainedTiles).toBeLessThanOrEqual(8);}
    const count=read.mock.calls.length;cache.setView(170000,180000,false,1000000);await flush();expect(read).toHaveBeenCalledTimes(count);
    cache.dispose();
});
it("po zmianie skali ignoruje spóźnioną odpowiedź i pozwala ponowić odczyt po reconnect",async()=>{
    const pending:{a:number;b:number;signal:AbortSignal;resolve:(d:ShaftHistory)=>void}[]=[],changed=vi.fn();
    const cache=new ShaftHistoryCache((a,b,signal)=>new Promise(resolve=>pending.push({a,b,signal,resolve})),changed,()=>{},()=>{});
    cache.setView(10000,20000,false,1000000);cache.setView(15000,20000,false,1000000);
    expect(pending[0].signal.aborted).toBe(true);
    pending[0].resolve(data(10000,20000));await flush();expect(changed).not.toHaveBeenCalled();
    expect(cache.retainedTiles).toBe(0);cache.setView(15000,20000,false,1000000,1);
    expect(pending[2].signal.aborted).toBe(true);cache.dispose();
});
it("nie łączy linii przez zakres, którego jeszcze nie pobrano; pozostawia brakujące osie puste",async()=>{
    const cache=new ShaftHistoryCache(async(a,b)=>data(a,b),()=>{},()=>{},()=>{});
    cache.setView(0,10000,false,1000000);await flush();
    const visible=cache.select(0,50000,[[point(45000),point(45040)]]);
    expect(visible[0].find(p=>p.time_ms===45000)?.gap).toBe(true);
    expect(visible[0].find(p=>p.time_ms===45040)?.gap).toBe(false);
    expect(visible.slice(1).every(p=>p.length===0)).toBe(true);cache.dispose();
});
it("zachowuje szczyty, segmenty i usuwa duplikat na styku przedziałów",async()=>{
    const cache=new ShaftHistoryCache(async(a,b)=>{
        const d=data(a,b);d.axes[0].samples[5].position_cdeg=12345;d.axes[0].samples[6]={...d.axes[0].samples[6],segment:'2',gap:true};return d;
    },()=>{},()=>{},()=>{});
    cache.setView(10000,20000,false,1000000);await flush();
    const points=cache.select(10000,20000,[])[0];expect(points.filter(p=>p.time_ms===20000)).toHaveLength(1);
    expect(Math.max(...points.map(p=>p.position_cdeg))).toBe(12345);expect(points.some(p=>p.gap&&p.segment==='2')).toBe(true);cache.dispose();
});
it("odświeża niepełny przedział po przejściu z live do historii",async()=>{
    let now=15500;
    const read=vi.fn(async(a:number,b:number)=>{const d=data(a,b);d.status.server_time_ms=now;return d;});
    const cache=new ShaftHistoryCache(read,()=>{},()=>{},()=>{});
    cache.setView(5000,15000,true,now);await flush();const before=read.mock.calls.length;
    now=19000;cache.setView(8000,18000,true,now);await flush();expect(read).toHaveBeenCalledTimes(before);
    cache.setView(8000,18000,false,now);await flush();expect(read.mock.calls.length).toBe(before+1);cache.dispose();
});
it("błąd HTTP nie zapętla pobierania; zamknięcie anuluje zapytania i opóźniony retry",async()=>{
    vi.useFakeTimers();
    const error=vi.fn(),read=vi.fn(async()=>{throw new Error('Dysk niedostępny');});
    const cache=new ShaftHistoryCache(read,()=>{},()=>{},error);
    try {
        cache.setView(10000,20000,false,1000000);await flush();expect(read).toHaveBeenCalledTimes(4);expect(error).toHaveBeenCalledTimes(4);
        cache.dispose();await vi.advanceTimersByTimeAsync(2000);expect(read).toHaveBeenCalledTimes(4);
    } finally {cache.dispose();vi.useRealTimers();}
});
