// PTP/LIN, trzy modele i globalne ustawienie; wszystkie API są atrapami bez CAN.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path'), assert = require('assert/strict');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
 const browser = await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context = await browser.newContext({viewport:{width:1366,height:768}});
 const dist=path.resolve(__dirname,'../dist'), commands=[], writes=[], errors=[], sockets=new Set(), requests=[], plans=new Map(), jobs=new Map(), events=[];
 let settings={revision:'initial',ignore_missing_drivers:false}, seed=Array(6).fill(0), slow=false, cancelling=false, delayedPlan=false;
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Podgląd bez napędów',configuration_revision:'initial',
  axes:Array.from({length:6},(_,i)=>({axis:i+1,available:false,position_known:false,position_age_seconds:0,position_degrees:0,prepared:false,enable_known:true,enabled:false,
   can_jog:false,can_zero:false,speed_known:true,speed_rpm:0,motion:'zatrzymany'}))};
 const timer=setInterval(()=>{for(const socket of sockets) socket.send(JSON.stringify({type:'state',state}));},50);
 context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
 await context.routeWebSocket('ws://robot.test/**',socket=>{if(!socket.url().endsWith('/manual/ws'))return;sockets.add(socket);socket.onClose(()=>sockets.delete(socket));socket.onMessage(raw=>{const v=JSON.parse(raw);if(v.type!=='trace')commands.push(v);});});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),json=(data,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(data)});
  if(url.pathname.startsWith('/api/v1/manual/axes')){writes.push(url.pathname);return route.abort();}
  if(url.pathname==='/api/v1/configuration/system'){
   if(request.method()==='PUT'){settings={revision:String(Number(settings.revision)||0)+requests.length+'changed',ignore_missing_drivers:request.postDataJSON().ignore_missing_drivers};}
   return json(settings);
  }
  if(url.pathname.startsWith('/api/v1/configuration'))return json({detail:'Brak konfiguracji szkicu w tej atrapie'},503);
  if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  if(url.pathname==='/api/v1/planning/context')return json({model_id:'model',configuration_revision:'initial',settings_revision:settings.revision,ignore_missing_drivers:settings.ignore_missing_drivers,
   planner_available:true,available:state.axes.map(a=>a.available),preview_seed_degrees:seed,seed_sources:state.axes.map(a=>a.available?'measured':'zero'),
   profiles:Array.from({length:6},()=>({configured:false,lower:-3600,upper:3600,velocity:180}))});
  if(url.pathname==='/api/v1/planning/moves'){
   assert(!cancelling,'Nowe obliczenie musi czekać na zakończenie anulowania');
   const data=request.postDataJSON();requests.push(data);const id=String(requests.length);events.push('post'+id);
   assert(data.seed_degrees.every(Number.isFinite));assert(data.target_degrees.every(Number.isFinite));
   const q=data.seed_degrees.map(v=>v*Math.PI/180),target=data.target_degrees.map(v=>v*Math.PI/180);
   plans.set(id,{id,model_id:'model',configuration_revision:'initial',settings_revision:data.settings_revision,simulation_only:true,simulated_start:data.simulated_start,start_positions:q,locked:Array(6).fill(false),
    execution_ready:false,execution_message:'Tylko symulacja',mks_compatible:false,mks_message:'Podgląd',mks_samples:[],motion_type:data.motion_type,
    trajectory:{joint_names:Array.from({length:6},(_,i)=>`joint_${i+1}`),points:[q,target].map((positions,i)=>({positions,velocities:Array(6).fill(0),accelerations:Array(6).fill(0),time_from_start:{sec:0,nanosec:i*600000000}}))}});
   jobs.set(id,{status:slow?'running':'succeeded',progress:slow?.2:1,phase:'TOTG',plan_id:id});return json({id,status:'running'});
  }
  if(url.pathname.startsWith('/api/v1/planning/jobs/')){
   const id=url.pathname.split('/').at(-1);
   if(request.method()==='DELETE'){events.push('cancelBegin'+id);cancelling=true;await delay(180);cancelling=false;events.push('cancelEnd'+id);jobs.set(id,{status:'cancelled',message:'Anulowano'});}
   return json(jobs.get(id));
  }
  if(url.pathname.startsWith('/api/v1/planning/plans/')){
   const plan=structuredClone(plans.get(url.pathname.split('/').at(-1)));
   if(delayedPlan){plan.trajectory.points[1].positions.fill(1.5);await delay(600);}
   return json(plan);
  }
  const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try{
  const config=await context.newPage();await config.goto('http://robot.test/#configuration');
  const toggle=config.getByRole('switch',{name:'Ignoruj brak sterowników — tylko planowanie',exact:true});
  await toggle.waitFor();await toggle.click();await config.waitForFunction(()=>document.querySelector('[role="switch"]').checked);
  assert(settings.ignore_missing_drivers);
  const page=await context.newPage();await page.goto('http://robot.test/#manual');
  await page.getByRole('button',{name:'Planowanie',exact:true}).click();
  const panel=page.getByLabel('Podgląd ruchu PTP/LIN',{exact:true}),canvas=page.locator('.robot-preview canvas');
  assert.equal(await panel.getByRole('button',{name:/Oblicz|Odtwórz/}).count(),0);
  assert.equal(await canvas.getAttribute('data-motion-joint-angles'),'');
  async function jog(axis){const b=page.getByRole('button',{name:`JOG J${axis} prawo`,exact:true});await b.hover();await page.mouse.down();await delay(180);await page.mouse.up();await delay(100);}
  await jog(6);const target=await canvas.getAttribute('data-planned-joint-angles');assert(JSON.parse(target)[5]>.1);
  await panel.getByRole('button',{name:'PTP',exact:true}).click();await panel.getByText(/PTP · 0.60 s · pętla/).waitFor();
  await page.waitForFunction(()=>document.querySelector('.robot-preview canvas').dataset.motionOpacity==='.55');
  assert.equal(await canvas.getAttribute('data-planning-opacity'),'.3');assert.equal(await canvas.getAttribute('data-planned-joint-angles'),target);
  const samples=[];for(let i=0;i<16;i++){samples.push(JSON.parse(await canvas.getAttribute('data-motion-joint-angles'))[5]);await delay(100);}
  const end=JSON.parse(target)[5];assert(samples.some(v=>Math.abs(v-end)<.015),'Pauza na celu');assert(samples.some(v=>v<end*.25),'Reset na start');
  await page.screenshot({path:'/tmp/robot-ptp-lin-playing.png',fullPage:true});
  assert.equal(requests.length,1,'Pętla nie powtarza obliczeń');
  seed=[25,0,0,0,0,0];state.axes[0].available=state.axes[0].position_known=true;state.axes[0].position_degrees=25;await delay(1200);
  await jog(2);await panel.getByText(/PTP · 0.60 s · pętla/).waitFor();assert.equal(requests.length,2);assert.deepEqual(requests[1].seed_degrees,Array(6).fill(0));
  assert.equal(requests[1].motion_type,'PTP');
  await panel.getByRole('button',{name:'LIN',exact:true}).click();await panel.getByText(/LIN · 0.60 s · pętla/).waitFor();assert.equal(requests.at(-1).seed_degrees[0],25);
  assert.equal(requests.at(-1).tcp_velocity_mm_s,50);assert.equal(requests.at(-1).tcp_acceleration_mm_s2,100);
  assert.equal(requests.at(-1).tcp_angular_velocity_degrees_s,10);assert(!('preview_velocity_degrees_s' in requests.at(-1)));
  await panel.getByText('Prędkość podglądu',{exact:true}).click();
  const feed=panel.getByRole('spinbutton',{name:'Prędkość TCP mm/s',exact:true});
  const changedFeed=page.waitForRequest(r=>r.url().endsWith('/planning/moves') && r.postDataJSON().tcp_velocity_mm_s===25);
  await feed.fill('25');await changedFeed;
  await panel.getByText(/LIN · 0.60 s · pętla/).waitFor();
  assert.equal(requests.at(-1).tcp_velocity_mm_s,25);assert.equal(requests.at(-1).seed_degrees[0],25);
  slow=true;await panel.getByRole('button',{name:'PTP',exact:true}).click();await panel.getByText(/TOTG · 20%/).waitFor();
  const old=String(requests.length);await jog(3);await delay(600);assert(events.indexOf('cancelEnd'+old)<events.indexOf('post'+(Number(old)+1)),events.join(','));
  await panel.getByRole('button',{name:'Zatrzymaj podgląd',exact:true}).click();await delay(250);slow=false;
  await panel.getByRole('button',{name:'PTP',exact:true}).click();await panel.getByText(/PTP · 0.60 s · pętla/).waitFor();
  await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await delay(100);
  const frozen=await canvas.getAttribute('data-motion-joint-angles');await delay(300);assert.equal(await canvas.getAttribute('data-motion-joint-angles'),frozen);
  delayedPlan=true;const pendingPlan=page.waitForRequest(request=>request.url().includes('/planning/plans/'));
  await panel.getByRole('button',{name:'PTP',exact:true}).click();await pendingPlan;await jog(4);delayedPlan=false;
  for(let i=0;i<15;i++){const pose=JSON.parse(await canvas.getAttribute('data-motion-joint-angles'));assert(pose.every(q=>Math.abs(q)<1.2),'Spóźniony wynik nie może uruchomić animacji');await delay(100);}
  await panel.getByText(/PTP · 0.60 s · pętla/).waitFor();
  await config.bringToFront();await toggle.click();await config.waitForFunction(()=>!document.querySelector('[role="switch"]').checked);await page.bringToFront();await delay(1400);
  assert(await page.getByRole('button',{name:'JOG J6 prawo',exact:true}).isDisabled());
  assert(!await page.getByRole('button',{name:'JOG J1 prawo',exact:true}).isDisabled());
  assert.equal(await panel.getByRole('button',{name:'PTP',exact:true}).getAttribute('aria-pressed'),'false');
  await page.screenshot({path:'/tmp/robot-ptp-lin.png',fullPage:true});
  assert.deepEqual(commands,[]);assert.deepEqual(writes,[]);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',checks:['wspólny suwak dwóch klientów','JOG bez sześciu napędów','trzy modele WebGL','stały cel','PTP i LIN','pętla z pauzą i resetem','start niezależny od enkoderów','automatyczne przeliczenie po JOG','oczekiwanie na anulowanie','blur','przywrócenie ograniczeń osi','brak komend sprzętowych']}));
 }finally{clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
