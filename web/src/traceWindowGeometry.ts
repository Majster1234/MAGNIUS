export type TraceWindowRect={x:number;y:number;width:number;height:number};
export type TraceViewport={width:number;height:number};
const margin=8;
export function fitTraceWindow(rect:TraceWindowRect,viewport:TraceViewport):TraceWindowRect {
    const maxWidth=Math.max(1,viewport.width-margin*2),maxHeight=Math.max(1,viewport.height-margin*2);
    const width=Math.min(maxWidth,Math.max(Math.min(420,maxWidth),rect.width));
    const height=Math.min(maxHeight,Math.max(Math.min(320,maxHeight),rect.height));
    return {width,height,x:Math.max(margin,Math.min(rect.x,viewport.width-margin-width)),y:Math.max(margin,Math.min(rect.y,viewport.height-margin-height))};
}
export function initialTraceWindow(viewport:TraceViewport):TraceWindowRect {
    const width=Math.min(640,Math.max(420,viewport.width-256),viewport.width-32),height=Math.min(600,viewport.height-80);
    return fitTraceWindow({width,height,x:viewport.width-width-16,y:64},viewport);
}
export function maximizedTraceWindow(viewport:TraceViewport):TraceWindowRect {
    return {x:margin,y:margin,width:Math.max(1,viewport.width-margin*2),height:Math.max(1,viewport.height-margin*2)};
}
