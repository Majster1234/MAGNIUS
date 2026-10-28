import {expect,it} from "vitest";
import {dirtyKeys,documentFromDraft,emptyDocument,fieldErrors,mergeAppliedDraft,toDraft,validateDocument,withExample} from "./configurationData";
it("migruje stare snapshoty bez limitów TCP do null i waliduje nowy profil",()=>{
    const legacy=emptyDocument();for(const key of Object.keys(legacy.values)) if(key.startsWith('tcp.')) delete legacy.values[key];
    const normalized=validateDocument(legacy);expect(normalized.values['tcp.max_linear_velocity']).toBeNull();
    const draft=toDraft(normalized);expect(dirtyKeys(draft,legacy).size).toBe(0);
    draft['tcp.max_linear_velocity']='100';draft['tcp.linear_acceleration']='5';
    expect(fieldErrors(draft)['tcp.linear_acceleration']).toContain('10 s');
    draft['tcp.linear_acceleration']='50';draft['tcp.max_angular_velocity']='30';draft['tcp.angular_acceleration']='60';
    expect(fieldErrors(draft)).toEqual({});expect(documentFromDraft(draft).values['tcp.max_linear_velocity']).toBe(100);
    draft['tcp.angular_acceleration']='0';expect(fieldErrors(draft)['tcp.angular_acceleration']).toBeDefined();
});
it("zachowuje null, zero i jednostki JSON bez dodatkowej konwersji",()=>{
    const draft=toDraft(emptyDocument());draft['joint.1.origin_x']='125';draft['joint.1.origin_y']='0';draft['joint.1.origin_yaw']='90';
    const document=documentFromDraft(draft,'Robot testowy');
    expect(validateDocument(JSON.parse(JSON.stringify(document)))).toEqual(document);
    expect(document.values['joint.1.origin_x']).toBe(125);expect(document.values['joint.1.origin_y']).toBe(0);
    expect(document.values['joint.1.origin_z']).toBeNull();expect(document.values['joint.1.origin_yaw']).toBe(90);
});
it("podświetla zmianę do ROS i usuwa po przywróceniu wartości także z innym formatem liczby",()=>{
    const baseline=emptyDocument();baseline.values['joint.1.origin_x']=100;
    const draft=toDraft(baseline);draft['joint.1.origin_x']='100.0';expect(dirtyKeys(draft,baseline).size).toBe(0);
    draft['joint.1.origin_x']='101';expect(dirtyKeys(draft,baseline).has('joint.1.origin_x')).toBe(true);
    draft['joint.1.origin_x']='';expect(dirtyKeys(draft,baseline).has('joint.1.origin_x')).toBe(true);
    draft['joint.1.origin_x']='100';expect(dirtyKeys(draft,baseline).size).toBe(0);
});
it("przykład uzupełnia geometrię bez fikcyjnego przełożenia, limitów i utraty nazw",()=>{
    const draft=toDraft(emptyDocument());draft['robot.name']='Mój robot';draft['joint.1.gear_ratio']='50';
    const example=withExample(draft);expect(example['joint.3.origin_z']).toBe('250');expect(example['joint.4.origin_x']).toBe('220');
    expect(example['joint.1.gear_ratio']).toBe('50');expect(example['joint.1.lower']).toBe('');expect(example['robot.name']).toBe('Mój robot');
});
it("odrzuca złą wersję, jednostki, typy, zakresy i nazwy ram",()=>{
    for(const change of [{schema_version:2},{units:'m-rad-kg'},{extra:1}])expect(()=>validateDocument({...emptyDocument(),...change})).toThrow();
    for(const [key,value] of [['joint.1.gear_ratio',0],['joint.1.direction',0],['joint.1.velocity',NaN],['tool.mass',-1],['robot.base_frame','link_3'],['joint.1.origin_x',true]]){
        const document=emptyDocument();document.values[key as string]=value as number;expect(()=>validateDocument(document)).toThrow();
    }
    const draft=toDraft(emptyDocument());draft['joint.2.lower']='720';draft['joint.2.upper']='360';
    expect(fieldErrors(draft)['joint.2.upper']).toBeTruthy();
});
it("przyjmuje wyliczony offset bez utraty edycji wykonanych po wysłaniu konfiguracji",()=>{
    const applied=emptyDocument();applied.values['joint.1.gear_ratio']=3;applied.values['joint.1.zero_offset']=60;
    const submitted=toDraft(applied);submitted['joint.1.zero_offset']='0';
    const current={...submitted,'joint.2.velocity':'12'};
    const merged=mergeAppliedDraft(current,submitted,applied);
    expect(merged['joint.1.zero_offset']).toBe('60');expect(merged['joint.2.velocity']).toBe('12');
    expect(dirtyKeys(merged,applied)).toEqual(new Set(['joint.2.velocity']));
    expect(mergeAppliedDraft({...current,'joint.1.zero_offset':'5'},submitted,applied)['joint.1.zero_offset']).toBe('5');
});

it("migruje regułę awarii do true i zapisuje świadome wyłączenie jako boolean",()=>{
    const key="safety.disable_enable_on_critical_fault", legacy=emptyDocument();delete legacy.values[key];
    const normalized=validateDocument(legacy);expect(normalized.values[key]).toBe(true);
    const draft=toDraft(normalized);expect(dirtyKeys(draft,legacy).size).toBe(0);
    draft[key]="false";
    expect(dirtyKeys(draft,legacy).has(key)).toBe(true);
    expect(documentFromDraft(draft).values[key]).toBe(false);
    expect(validateDocument(documentFromDraft(draft)).values[key]).toBe(false);
    for(const invalid of [null,0,1,"false"]) expect(()=>validateDocument({...normalized,values:{...normalized.values,[key]:invalid}})).toThrow();
});
