import {mergeTrace,type ShaftHistory,type ShaftPoint} from "./shaftTrace";

type Tile={start:number;end:number;through:number;axes:ShaftPoint[][]};
type View={start:number;end:number;span:number;live:boolean;now:number;token:number};
type Read=(start:number,end:number,signal:AbortSignal)=>Promise<ShaftHistory>;

function slice(points:ShaftPoint[],start:number,end:number) {
    let low=0,high=points.length;
    while(low<high) {const mid=(low+high)>>>1;if(points[mid].time_ms<start) low=mid+1;else high=mid;}
    const first=low;high=points.length;
    while(low<high) {const mid=(low+high)>>>1;if(points[mid].time_ms<=end) low=mid+1;else high=mid;}
    return points.slice(first,low);
}

/** Stałe przedziały o szerokości okna, z marginesem po obu stronach.
 * Ruch myszy zmienia priorytety; nie anuluje użytecznego pobrania ani nie tworzy kolejki. */
export class ShaftHistoryCache {
    static readonly capacity=8;
    static readonly concurrency=2;
    private tiles=new Map<number,Tile>();
    private pending=new Map<number,AbortController>();
    private failed=new Set<number>();
    private retry:ReturnType<typeof setTimeout>|undefined;
    private view:View|null=null;
    private generation=0;
    private disposed=false;
    constructor(private read:Read,private changed:()=>void,private result:(data:ShaftHistory)=>void,private error:(error:unknown)=>void) {}

    setView(start:number,end:number,live:boolean,now:number,token=0) {
        if(this.disposed||!Number.isFinite(start)||!Number.isFinite(end)||end<=start) return;
        const span=end-start,reset=this.view!==null&&(span!==this.view.span||token!==this.view.token);
        this.view={start,end,span,live,now,token};
        if(reset) {
            this.generation++;for(const controller of this.pending.values()) controller.abort();
            this.pending.clear();this.tiles.clear();this.failed.clear();clearTimeout(this.retry);this.retry=undefined;
        }
        for(const key of this.wanted()) {
            const tile=this.tiles.get(key);
            if(tile) {this.tiles.delete(key);this.tiles.set(key,tile);}
        }
        this.pump();
    }
    private visible() {
        if(!this.view) return [];
        const {start,end,span}=this.view;
        return [...new Set([Math.floor(start/span),Math.floor(end/span)])];
    }
    private wanted() {
        if(!this.view) return [];
        const visible=this.visible(),first=visible[0],last=visible.at(-1)!;
        return [...new Set([...visible,first-1,last+1])].filter(key=>key*this.view!.span<=this.view!.now);
    }
    private usable(key:number) {
        const tile=this.tiles.get(key),view=this.view!;
        // Bieżące próbki uzupełnia WS. Historia odświeża niepełny przedział po wyjściu z live.
        return !!tile&&(view.live||tile.through>=Math.min(tile.end,view.end,view.now));
    }
    get loading() {return this.visible().some(key=>!this.usable(key)&&!this.failed.has(key)&&key*this.view!.span<=this.view!.now);}
    get retainedTiles() {return this.tiles.size;}
    get requestsInFlight() {return this.pending.size;}

    private pump() {
        if(this.disposed||!this.view) return;
        for(const key of this.wanted()) {
            if(this.pending.size>=ShaftHistoryCache.concurrency) break;
            if(this.usable(key)||this.pending.has(key)||this.failed.has(key)) continue;
            const controller=new AbortController(),generation=this.generation,start=key*this.view.span,end=start+this.view.span;
            this.pending.set(key,controller);
            this.read(start,end,controller.signal).then(data=>{
                if(this.disposed||generation!==this.generation) return;
                this.tiles.set(key,{start,end,through:Math.min(end,data.status.server_time_ms),axes:Array.from({length:6},(_,i)=>
                    [...(data.axes.find(a=>a.axis===i+1)?.samples??[])].slice(0,2000).sort((a,b)=>a.time_ms-b.time_ms))});
                const wanted=new Set(this.wanted());
                for(const old of this.tiles.keys()) {
                    if(this.tiles.size<=ShaftHistoryCache.capacity) break;
                    if(!wanted.has(old)) this.tiles.delete(old);
                }
                this.result(data);
            }).catch(error=>{
                if(this.disposed||generation!==this.generation||controller.signal.aborted) return;
                this.failed.add(key);this.error(error);
                if(!this.retry) this.retry=setTimeout(()=>{
                    this.retry=undefined;this.failed.clear();this.pump();
                },1000);
            }).finally(()=>{
                if(this.disposed||generation!==this.generation) return;
                this.pending.delete(key);this.changed();this.pump();
            });
        }
    }

    select(start:number,end:number,live:ShaftPoint[][]):ShaftPoint[][] {
        const tiles=[...this.tiles.values()].filter(t=>t.end>=start&&t.start<=end);
        return Array.from({length:6},(_,axis)=>{
            const recent=live[axis]??[];
            const intervals=tiles.map(t=>({start:t.start,end:t.through}));
            if(recent.length) intervals.push({start:recent[0].time_ms,end:recent.at(-1)!.time_ms});
            const covered:{start:number;end:number}[]=[];
            for(const interval of intervals.sort((a,b)=>a.start-b.start)) {
                if(interval.end<interval.start) continue;
                const last=covered.at(-1);
                if(last&&interval.start<=last.end) last.end=Math.max(last.end,interval.end);
                else covered.push({...interval});
            }
            const points=mergeTrace(tiles.flatMap(t=>slice(t.axes[axis],start,end)),slice(recent,start,end),start);
            let region=0;
            return points.map((point,i)=>{
                if(!i) return point;
                const previous=points[i-1];
                while(region<covered.length&&covered[region].end<previous.time_ms) region++;
                const interval=covered[region];
                return !interval||interval.start>previous.time_ms||interval.end<point.time_ms?{...point,gap:true}:point;
            });
        });
    }
    dispose() {
        this.disposed=true;this.generation++;clearTimeout(this.retry);
        for(const controller of this.pending.values()) controller.abort();
        this.pending.clear();this.tiles.clear();
    }
}
