import { expect, it, vi } from "vitest";
import { PhysicalTcpJog, type TcpJogStatus } from "./physicalTcpJog";
const ready: TcpJogStatus = { fresh:true,ready:true,busy:false,state:"ready",message:"Gotowy",axis:0,direction:0,frame:"base",impulse:false,
    progress:0,requested_step:0,velocity:0,max_linear_velocity:.02,max_angular_velocity:.2,model_id:"m",configuration_revision:"r",generation:0 };
function setup() {
    let now=0,sequence=0;const sent:Record<string,unknown>[]=[],error=vi.fn();
    const socket={readyState:1,send:(raw:string)=>sent.push(JSON.parse(raw))};
    const jog=new PhysicalTcpJog(socket,()=>++sequence,()=>now,error);jog.receive({...ready},true);
    return {jog,sent,socket,error,time:(value:number)=>{now=value;},status:(value:Partial<TcpJogStatus>)=>jog.receive({...ready,...value},true)};
}
it.each([false,true])("LIVE: mm i stopnie przechodzą do SI; puszczenie hamuje również impuls=%s", impulse=>{
    for(const [axis,speed,step,expected] of [["x",10,1,.01],["rx",5,5,5*Math.PI/180]] as const) {
        const s=setup();expect(s.jog.start(axis,1,"tool",speed,impulse?step:undefined)).toBe(true);
        expect(s.sent[0]).toMatchObject({space:"tcp",action:"start",frame:"tool",velocity:expected,impulse});
        expect(s.jog.start("y",1,"base",10)).toBe(false);
        s.jog.ack(1,true);s.status({ready:false,busy:true,generation:1});s.time(50);s.jog.pulse();
        expect(s.sent[1]).toMatchObject({action:"keepalive",sequence:2});
        s.jog.stop();expect(s.sent.at(-1)).toMatchObject({action:"stop",emergency:false});expect(s.jog.held).toBeNull();expect(s.jog.busy).toBe(true);
        s.status({generation:1,state:"stopped"});expect(s.jog.busy).toBe(false);
    }
});
it.each(["ack","stale","socket","revision","hidden"])("utrata warunku %s zatrzymuje i nie wznawia automatycznie", reason=>{
    const s=setup();s.jog.start("z",-1,"base",10);s.jog.ack(1,true);s.status({ready:false,busy:true,generation:1});
    if(reason==="ack") {s.time(50);s.jog.pulse();s.time(150);s.jog.pulse();}
    if(reason==="stale") {s.time(250);s.jog.pulse();}
    if(reason==="socket") {s.socket.readyState=3;s.jog.pulse();}
    if(reason==="revision") s.status({configuration_revision:"other"});
    if(reason==="hidden") s.jog.stop(true);
    expect(s.jog.held).toBeNull();s.status({generation:1,state:"stopped"});const count=s.sent.length;
    s.time(500);s.jog.pulse();expect(s.sent).toHaveLength(count);
    if(reason!=="socket") expect(s.sent.at(-1)).toMatchObject({action:"stop",emergency:true});
});
it("blokuje brak gotowości i przekroczenie fizycznego limitu",()=>{
    const s=setup();s.status({ready:false});expect(s.jog.start("x",1,"base",1)).toBe(false);
    s.status({ready:true});expect(s.jog.start("x",1,"base",21)).toBe(false);expect(s.sent).toEqual([]);
});
