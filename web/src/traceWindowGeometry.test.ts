import {describe,expect,it} from "vitest";
import {fitTraceWindow,initialTraceWindow,maximizedTraceWindow} from "./traceWindowGeometry";

describe("okno wykresów",()=>{
    it("otwiera się przy prawej krawędzi, zostawiając dostęp do menu",()=>{
        const rect=initialTraceWindow({width:1920,height:1080});
        expect({width:rect.width,height:rect.height}).toEqual({width:640,height:600});
        expect(rect.x).toBeGreaterThan(224);
        expect(rect.x+rect.width).toBeLessThan(1920);
        expect(rect.y+rect.height).toBeLessThan(1080);
        expect(initialTraceWindow({width:1366,height:768}).x).toBeGreaterThan(224);
    });
    it("utrzymuje całe okno na ekranie po przesunięciu i zmianie monitora",()=>{
        const rect=fitTraceWindow({x:1500,y:-300,width:1100,height:790},{width:760,height:600});
        expect(rect).toEqual({x:8,y:8,width:744,height:584});
    });
    it("pozwala zmniejszyć okno tylko do czytelnego rozmiaru",()=>{
        expect(fitTraceWindow({x:12,y:12,width:1,height:1},{width:1920,height:1080})).toEqual({x:12,y:12,width:420,height:320});
        expect(initialTraceWindow({width:320,height:280})).toEqual({x:8,y:8,width:304,height:264});
    });
    it("maksymalizacja dopasowuje się do bieżącego ekranu",()=>{
        expect(maximizedTraceWindow({width:1920,height:1080})).toEqual({x:8,y:8,width:1904,height:1064});
        expect(maximizedTraceWindow({width:760,height:600})).toEqual({x:8,y:8,width:744,height:584});
    });
});
