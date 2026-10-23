// Ograniczony test/porównanie doczytywania modułu trace. Wyłącznie atrapy.
const {chromium}=require('playwright');
const assert=require('assert/strict'),path=require('path');
(async()=>{
 const pinned=process.argv.includes('--pinned'),baseline=process.argv.includes('--baseline'),browser=await chromium.launch({headless:true});
 const dist=path.resolve(__dirname,'../dist'),base='/api/v1/diagnostics/trace',now=1792776000000;
 const samples=Array.from({length:6},(_,a)=>Array.from({length:3001},(_,i)=>({axis:a+1,sequence:String(i+1),segment:'1',
  monotonic_ns:String(BigInt(i+1)*40000000n),utc_ns:String(BigInt(now-120000+i*40)*1000000n),time_ms:now-120000+i*40,
  position_cdeg:a*100+Math.round(Math.sin(i/17)*25),gap:i===0})));
 const status={recording:true,connected:true,server_time_ms:now,source_dropped:0,missing_samples:0,writer_dropped:0,queued:0,saved:18006,cursor:18006,
  axes:samples.map((p,i)=>({axis:i+1,limit_bytes:20000000,file_bytes:20000000,retained_records:p.length,fresh:true,age_seconds:0,
   recording:true,error:null,first_ms:p[0].time_ms,last_ms:now,latest:p.at(-1),corrupt_records:0}))};
 const runs=[];
 try {for(let run=0;run<7;run++) {
  const context=await browser.newContext({viewport:{width:1920,height:1080}}),requests=[],errors=[];let held=false;
  await context.addInitScript(()=>{
   window.tracePaint={lines:0};let lines=0;
   const prototype=CanvasRenderingContext2D.prototype,clear=prototype.clearRect,line=prototype.lineTo,stroke=prototype.stroke;
   prototype.clearRect=function(...args){if(this.canvas.getAttribute('aria-label')==='Przebieg pozycji wału J1')lines=0;return clear.apply(this,args);};
   prototype.lineTo=function(...args){if(this.canvas.getAttribute('aria-label')==='Przebieg pozycji wału J1'&&this.strokeStyle==='#1aa79c')lines++;return line.apply(this,args);};
   prototype.stroke=function(...args){if(this.canvas.getAttribute('aria-label')==='Przebieg pozycji wału J1'&&this.strokeStyle==='#1aa79c')window.tracePaint={lines,at:performance.now()};return stroke.apply(this,args);};
  });
  await context.routeWebSocket('ws://robot.test/**',socket=>{
   if(!socket.url().endsWith(base+'/ws'))return;
   const timer=setInterval(()=>socket.send(JSON.stringify({cursor:18006,expired:false,samples:[],status})),50);
   socket.send(JSON.stringify({cursor:18006,expired:false,samples:[],status}));socket.onClose(()=>clearInterval(timer));
  });
  await context.route('http://robot.test/**',async route=>{
   const u=new URL(route.request().url());assert.equal(route.request().method(),'GET');
   if(u.pathname===base+'/history') {
    const lo=Number(u.searchParams.get('start_ms')),hi=Number(u.searchParams.get('end_ms'));
    requests.push({start:lo,end:hi,held});await new Promise(resolve=>setTimeout(resolve,80));
    return route.fulfill({contentType:'application/json',body:JSON.stringify({status,axes:samples.map((p,i)=>({axis:i+1,samples:p.filter(q=>q.time_ms>=lo&&q.time_ms<=hi).slice(0,2000),next_cursor:null}))})}).catch(()=>{});
   }
   if(u.pathname.startsWith('/api/'))return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Atrapa"}'});
   const name=u.pathname==='/'?'index.html':u.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
   return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
  });
  const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
  try {
   await page.goto('http://robot.test/#diagnostics');
   if(pinned) await page.getByRole('button',{name:'Przypnij wykres J1',exact:true}).click();
   await page.waitForFunction(()=>window.tracePaint.lines>20);
   await page.waitForTimeout(600); // ten sam czas na przygotowanie bufora przed przeciąganiem
   const client=await context.newCDPSession(page);await client.send('Performance.enable');
   const metrics=async()=>Object.fromEntries((await client.send('Performance.getMetrics')).metrics.map(p=>[p.name,p.value]));
   const before=await metrics(),started=performance.now(),canvas=page.locator('.shaft-chart[data-axis="1"] canvas'),box=await canvas.boundingBox();
   const span=box.width-76,x=box.x+80,y=box.y+70;let frames=0,ready=0;
   await page.mouse.move(x,y);held=true;await page.mouse.down();
   for(let step=1;step<=24;step++) {
    await page.mouse.move(x+step*span/12,y);await page.waitForTimeout(25);
    if(step>=13){frames++;if(await page.evaluate(()=>window.tracePaint.lines>20))ready++;}
   }
   const readyBeforeRelease=await page.evaluate(()=>window.tracePaint.lines>20),released=performance.now();
   await page.mouse.up();held=false;
   await page.waitForFunction(()=>window.tracePaint.lines>20);
   const releaseWait=performance.now()-released,elapsed=performance.now()-started,after=await metrics();
   runs.push({frames,ready,readyBeforeRelease,release_wait_ms:releaseWait,requests:requests.length,requests_while_held:requests.filter(r=>r.held).length,
    renderer_task_cpu_percent:(after.TaskDuration-before.TaskDuration)/(elapsed/1000)*100});
   if(!baseline){assert.equal(readyBeforeRelease,true,'Dane muszą być narysowane przy nadal wciśniętej myszy');assert.equal(ready,frames,'Margines ma przygotować dane przed wejściem do widoku');assert(requests.length<=10,'Brak zapytań przy każdym ruchu myszy');}
   assert.deepEqual(errors,[]);
  } finally {await context.close();}
 }} finally {await browser.close();}
 console.log(JSON.stringify({mode:baseline?'before':'after',window:pinned?'floating':'docked',runs,fixture:{runs:7,steps:24,step_ms:25,http_delay_ms:80,samples_per_axis:3001,axes:6,initial_window_seconds:10}}));
})().catch(e=>{console.error(e);process.exitCode=1;});
