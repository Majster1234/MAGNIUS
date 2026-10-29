import {expect,it} from "vitest";
import {emptyScales,mergeTrace,positionText,reduceTrace,valueAt,yRange,type ShaftPoint} from "./shaftTrace";
const point=(n:number,q=0,gap=false):ShaftPoint=>({axis:1,sequence:String(n),segment:"1",monotonic_ns:String(n*40000000),utc_ns:String(n*40000000),time_ms:n*40,position_cdeg:q,gap});
it("utrzymuje setne stopnia, brak danych i niezależne skale",()=>{
    expect(positionText(point(1,-563))).toBe("-5.63°");expect(positionText(point(1,0))).toBe("0.00°");expect(positionText(null)).toBe("—");
    const scales=emptyScales();scales[0].auto=false;scales[0].minimum=-12;scales[0].maximum=20;
    expect(yRange([],scales[0])).toEqual([-12,20]);expect(scales[1].auto).toBe(true);
    expect(yRange([point(1,100)],scales[1])).toEqual([.95,1.05]);
});
it("min/max zachowuje krótkie szczyty, kolejność i przerwy",()=>{
    const points=Array.from({length:5000},(_,i)=>point(i));points[2345].position_cdeg=12345;points[2346].position_cdeg=-8765;points[4000].gap=true;
    const reduced=reduceTrace(points,100);expect(reduced.length).toBeLessThanOrEqual(100);
    expect(Math.max(...reduced.map(p=>p.position_cdeg))).toBe(12345);expect(Math.min(...reduced.map(p=>p.position_cdeg))).toBe(-8765);
    expect(reduced.some(p=>p.gap)).toBe(true);expect(reduced.map(p=>p.time_ms)).toEqual(reduced.map(p=>p.time_ms).sort((a,b)=>a-b));
});
it("scala historię bez duplikacji i nie zastępuje braku pomiaru zerem",()=>{
    const merged=mergeTrace([point(1),point(2)],[point(2),point(3,-20)],60);
    expect(merged.map(p=>p.sequence)).toEqual(["2","3"]);expect(valueAt(merged,240)).toBeNull();expect(valueAt([],10)).toBeNull();
    expect(valueAt(merged,120)?.position_cdeg).toBe(-20);
    expect(valueAt([point(1,12),point(5,24,true)],80)).toBeNull();
    expect(valueAt([point(1),point(2)],81)).toBeNull();
    expect(valueAt([point(1),point(2,-10)],70)?.position_cdeg).toBe(-10);
});
