// Wyłącznie atrapy HTTP/WebSocket. Żadnego połączenia z CAN ani wykonywania PTP.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path'), assert = require('assert/strict');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
 const browser = await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context = await browser.newContext({viewport:{width:1440,height:1000}});
 const dist=process.env.ROBOT_WEB_TEST_DIST || path.resolve(__dirname,'../dist');
 const sockets=new Set(), errors=[], writes=[], plans=new Map();let serial=0, tick=0, advancing=false, loseStatus=false, modelId='model';
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Atrapa śladów',configuration_revision:'initial',
  axes:Array.from({length:6},(_,i)=>({axis:i+1,available:[0,1,2,4].includes(i),prepared:true,enable_known:true,enabled:true,
   position_known:true,position_age_seconds:0,position_degrees:0,speed_known:true,speed_rpm:0,can_jog:true,can_zero:true,motion:'zatrzymany',
   joint_configured:true,calibration_valid:true,joint_position_known:true,joint_position_degrees:[0,-20,25,0,30,0][i],
   joint_lower_degrees:-180,joint_upper_degrees:180,jog_min_velocity_degrees_s:0,jog_max_velocity_degrees_s:80}))};
 let motion={program_id:'',state:'idle',mode:'streamed',motion_type:'PTP',message:'',execution_key:'',participating:[true,true,true,false,true,false],tcp_verified:false,
  busy:false,progress:0,duration:3,elapsed:0,start_positions:state.axes.map(a=>a.joint_position_degrees*Math.PI/180),target_positions:Array(6).fill(.3)};
 const timer=setInterval(()=>{
  if(advancing)state.axes[0].joint_position_degrees+=1;
  state.state_monotonic_ns=String(++tick);
  for(const socket of sockets)socket.send(JSON.stringify({type:'state',state}));
 },50);
 context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
 await context.routeWebSocket('ws://robot.test/**',socket=>{
  if(!socket.url().endsWith('/manual/ws'))return;sockets.add(socket);socket.onClose(()=>sockets.delete(socket));
  socket.onMessage(raw=>{const value=JSON.parse(raw);if(value.type!=='trace')writes.push(value);});
 });
 await context.route('http://robot.test/**',async route=>{
  const req=route.request(),url=new URL(req.url()),json=(value,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(value)});
  if(url.pathname==='/api/v1/motion/status')return loseStatus?json({detail:'Przerwa statusu'},503):json({...motion,preparation:{state:'idle'}});
  if(url.pathname.startsWith('/api/v1/manual/axes') || (url.pathname.startsWith('/api/v1/motion/') && req.method()!=='GET')){
   writes.push(url.pathname);return json({detail:'Niedozwolone polecenie w teście'},422);
  }
  if(url.pathname==='/api/v1/configuration/system')return json({revision:'settings',ignore_missing_drivers:true,allow_missing_motion:true});
  if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  if(url.pathname==='/api/v1/planning/context')return json({model_id:modelId,configuration_revision:'initial',settings_revision:'settings',ignore_missing_drivers:true,planner_available:true,
   available:state.axes.map(a=>a.available),preview_seed_degrees:state.axes.map(a=>a.joint_position_degrees),seed_sources:state.axes.map(a=>a.available?'measured':'zero'),
   profiles:state.axes.map(()=>({configured:true,lower:-180,upper:180,velocity:80}))});
  if(url.pathname==='/api/v1/planning/moves'){
   const data=req.postDataJSON(),id=String(++serial),q=data.seed_degrees.map(v=>v*Math.PI/180),target=data.target_degrees.map(v=>v*Math.PI/180);
   plans.set(id,{id,model_id:'model',configuration_revision:'initial',settings_revision:'settings',motion_type:'PTP',simulation_only:true,simulated_start:data.simulated_start,
    execution_ready:false,execution_message:'',mks_compatible:false,mks_message:'',mks_samples:[],start_positions:q,target_positions:target,locked:Array(6).fill(false),
    trajectory:{joint_names:Array.from({length:6},(_,i)=>`joint_${i+1}`),points:[q,target].map((positions,i)=>({positions,velocities:Array(6).fill(0),accelerations:Array(6).fill(0),time_from_start:{sec:i,nanosec:0}}))}});
   return json({id});
  }
  if(url.pathname.startsWith('/api/v1/planning/jobs/'))return json({status:req.method()==='DELETE'?'cancelled':'succeeded',phase:'Gotowe',progress:1,plan_id:url.pathname.split('/').at(-1)});
  if(url.pathname.startsWith('/api/v1/planning/plans/'))return json(plans.get(url.pathname.split('/').at(-1)));
  if(url.pathname.startsWith('/api/'))return json({detail:'Brak w atrapie'},503);
  const file=path.resolve(dist,url.pathname==='/'?'index.html':url.pathname.slice(1));assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try {
  const page=await context.newPage();await page.goto('http://robot.test/#manual');
  const canvas=page.locator('.robot-preview canvas'), clear=page.getByRole('button',{name:'Wyczyść ślad',exact:true});
  await page.waitForFunction(()=>!!document.querySelector('.robot-preview canvas')?.dataset.executionTrace);
  const trace=async()=>JSON.parse(await canvas.getAttribute('data-execution-trace'));
  const blue=async()=>JSON.parse(await canvas.getAttribute('data-motion-trace'));
  assert.equal((await trace()).points,0);assert(await clear.isDisabled());
  await page.getByRole('button',{name:'Planowanie',exact:true}).click();
  const jog=page.getByRole('button',{name:'JOG J1 prawo',exact:true});
  await jog.hover();await page.mouse.down();await delay(200);await page.mouse.up();
  await page.getByRole('button',{name:'PTP',exact:true}).click();
  await page.waitForFunction(()=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.motionTrace).visible);
  assert.equal((await blue()).color,'#4da3ff');assert.equal((await trace()).points,0,'Planowanie nie zapisuje śladu fizycznego');
  motion={...motion,program_id:'p1',execution_key:'k1',state:'running',busy:true};advancing=true;
  await page.waitForFunction(()=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).points>=6);
  assert.equal((await trace()).color,'#ffc857');assert((await trace()).partial);assert((await trace()).visible);
  const ptpSegments=(await trace()).segments;await delay(600);
  assert.equal((await trace()).segments,ptpSegments,'Kolejne próbki PTP tworzą ciągłą linię');
  await page.locator('.control-issue[data-source="model-trace-partial"]').waitFor();
  assert((await blue()).visible,'Plan i historia są widoczne jednocześnie');
  const colors=await page.locator('.preview-trail-legend').evaluate(el=>[...el.querySelectorAll('.trail-planned,.trail-executed')].map(s=>getComputedStyle(s,'::before').backgroundColor));
  assert.deepEqual(colors,['rgb(77, 163, 255)','rgb(255, 200, 87)']);
  await page.screenshot({path:'/tmp/robot-ptp-trails.png',fullPage:true});
  motion={...motion,state:'succeeded',busy:false};await delay(450);const completed=await trace();await delay(250);
  assert.equal((await trace()).points,completed.points,'Sukces zamraża historię mimo kolejnych odczytów');
  motion={...motion,program_id:'p2',execution_key:'k2',motion_type:'LIN',state:'running',busy:true};
  await page.waitForFunction(old=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).segments>old,completed.segments);
  const linSegments=(await trace()).segments;await delay(600);
  assert.equal((await trace()).segments,linSegments,'Kolejne próbki LIN tworzą ciągłą linię');
  state.axes[0].joint_position_known=false;await delay(150);const gap=await trace();await delay(150);assert.equal((await trace()).points,gap.points);
  state.axes[0].joint_position_known=true;
  await page.waitForFunction(old=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).segments>old,gap.segments);
  loseStatus=true;await delay(500);const lost=await trace();await delay(200);assert.equal((await trace()).points,lost.points);
  loseStatus=false;
  await page.waitForFunction(old=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).segments>old,lost.segments);
  advancing=false;await delay(150);const planId=(await blue()).id;
  await clear.click();await delay(200);assert((await trace()).points<=1);assert.equal((await trace()).edges,0);
  assert.equal((await blue()).id,planId,'Czyszczenie złota zachowuje plan');
  advancing=true;await page.waitForFunction(()=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).points>=4);
  advancing=false;motion={...motion,state:'failed',busy:false};await delay(450);const stopped=await trace();
  await page.getByRole('button',{name:'Zakończ planowanie',exact:true}).click();await delay(200);
  assert.equal((await trace()).points,stopped.points);assert((await trace()).visible,'Wyjście z planowania zachowuje złoto');
  assert(!(await blue()).visible);
  const previousCanvas=await canvas.elementHandle();modelId='another-model';
  await page.waitForFunction(old=>!old.isConnected,previousCanvas);
  await page.waitForFunction(()=>!!document.querySelector('.robot-preview canvas')?.dataset.executionTrace);
  assert.equal((await trace()).points,0,'Zmiana modelu usuwa historię poprzedniego FK');
  motion={...motion,program_id:'p3',execution_key:'k3',state:'running',busy:true};advancing=true;
  await page.waitForFunction(()=>JSON.parse(document.querySelector('.robot-preview canvas').dataset.executionTrace).points>=4);
  advancing=false;motion={...motion,state:'succeeded',busy:false};await delay(450);
  await page.reload();await page.waitForFunction(()=>!!document.querySelector('.robot-preview canvas')?.dataset.executionTrace);
  assert.equal((await trace()).points,0,'Odświeżenie usuwa historię sesji');
  assert.deepEqual(writes,[]);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',checks:['niebieski plan i złota historia jednocześnie','legenda kolorów','niepełne pomiary','wiele PTP','sukces i STOP/błąd','przerwa enkodera i statusu bez mostu','czyszczenie podczas PTP bez usuwania planu','historia po wyjściu z planowania','reset po zmianie modelu i odświeżeniu','zero poleceń sterowania']}));
 } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
