import {expect,it} from "vitest";
import {ControlMessages,describeControlNotice} from "./controlMessages";
import {motionReady,motionStatusFresh,type MotionStatus} from "./motionStatus";

it.each(["connection","freshness","missing","enable","calibration","profile","configuration","program","limits","unreachable","singularity","planner","executor","jog","waiting_position","waiting_feedback","waiting_budget","partial","uncertain_start","stop","fault"])("opis %s zawiera znaczenie, rozwiązanie i oryginalny szczegół",code=>{
    const n=describeControlNotice({source:"test",code,severity:"error",message:"Szczegół sterownika J3"});
    expect(n.title).toBeTruthy();expect(n.description).toBeTruthy();expect(n.remedy).toBeTruthy();expect(n.details).toBe("Szczegół sterownika J3");
});
it("nie przypisuje nieznanemu błędowi niepotwierdzonej przyczyny",()=>{
    const n=describeControlNotice({source:"motion-prepare",severity:"error",message:"Nieznany błąd #42"});
    expect(n.description).toContain("nie została jednoznacznie");expect(n.context).toBe("Robot fizyczny");expect(n.details).toContain("#42");expect(n.remedy).toContain("stan");
});
it("aktywna blokada przeżywa limit historii i wymaga jawnie potwierdzonej zmiany warunku",()=>{
    const store=new ControlMessages();
    store.condition("calibration-1",{source:"calibration-1",severity:"warning",code:"calibration",message:"J1"});
    for(let i=0;i<150;i++)store.report({source:"result",severity:"info",message:String(i)});
    expect(store.log.entries).toHaveLength(100);expect(store.active).toHaveLength(1);
    store.report({source:"calibration-1",severity:"info",message:"Inna operacja zakończona"});expect(store.active).toHaveLength(1);
    store.condition("calibration-1",null);expect(store.active).toHaveLength(0);expect(store.log.entries.at(-1)?.message).toContain("problem ustąpił");
});
it("dedyplikuje statusy i procenty; następny wynik operacji usuwa dawny błąd",()=>{
    const store=new ControlMessages();
    for(let i=0;i<100;i++)store.report({source:"simulation",severity:"info",message:"Obliczanie PTP",progress:i});
    expect(store.log.entries).toHaveLength(1);expect(store.statuses[0].progress).toBe(99);
    store.report({source:"motion-prepare",severity:"error",message:"Cel nieosiągalny"});expect(store.active[0].kind).toBe("result");
    store.report({source:"motion-prepare",severity:"info",message:"Program przygotowany"});expect(store.active).toHaveLength(0);expect(store.log.entries).toHaveLength(3);
});
it("brak oceny nie jest przedstawiany jako ustąpienie problemu",()=>{
    const store=new ControlMessages();store.condition("model",{source:"model",code:"singularity",severity:"warning",message:"Blisko"});
    store.condition("model",null,"Ocena niedostępna — brak danych");expect(store.log.entries.at(-1)?.message).toContain("brak danych");expect(store.log.entries.at(-1)?.message).not.toContain("ustąpił");
});
it("świeżość wygasa przy 750 ms, błędzie i nieprawidłowym zegarze",()=>{
    expect(motionStatusFresh(1000,1749)).toBe(true);expect(motionStatusFresh(1000,1750)).toBe(false);
    expect(motionStatusFresh(1000,999)).toBe(false);expect(motionStatusFresh(-Infinity,1000)).toBe(false);expect(motionStatusFresh(1000,1001,true)).toBe(false);
});
it("gotowość wymaga właściwego przygotowania, programu, typu i świeżego statusu",()=>{
    const s={state:"ready",busy:false,program_id:"p",motion_type:"LIN",preparation:{id:"new",state:"ready",preview_plan_id:"v",program_id:"p",motion_type:"LIN"}} as MotionStatus;
    expect(motionReady(s,"v","LIN","new",true)).toBe(true);
    for(const changed of [{preparation:{...s.preparation,id:"old"}},{preparation:{...s.preparation,id:undefined}},{program_id:""},{program_id:"other"},{motion_type:"PTP"},{busy:undefined},{busy:true},{state:"unknown"},{state:"running"}])expect(motionReady({...s,...changed} as MotionStatus,"v","LIN","new",true)).toBe(false);
    expect(motionReady(s,"v","LIN","new",false)).toBe(false);expect(motionReady(s,"other","LIN","new",true)).toBe(false);
});
