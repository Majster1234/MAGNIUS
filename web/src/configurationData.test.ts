import {expect,it} from "vitest";
import {dirtyKeys,documentFromDraft,emptyDocument,fieldErrors,toDraft,validateDocument,withExample} from "./configurationData";
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
