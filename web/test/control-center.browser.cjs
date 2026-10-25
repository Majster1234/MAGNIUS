// Wyłącznie emulowane HTTP/WebSocket, bez ROS i CAN.
const {chromium}=require('playwright');
const path=require('path'),fs=require('fs'),assert=require('assert/strict');
const wait=ms=>new Promise(r=>setTimeout(r,ms)),copy=x=>structuredClone(x);
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1366,height:768}});
 const dist=process.env.ROBOT_WEB_TEST_DIST || path.resolve(__dirname,'../dist'),writes=[],errors=[],sockets=new Set(),plans=new Map();
 let jobs=0,preparations=0,preparation={state:'idle'},failPrepare='',unavailable=false,delayStatus=0,delaying=false,unknownStart=false,failStop=false,partial=false,stopDelay=0,prepareDelay=0;
 let motion={state:'idle',program_id:'',mode:'streamed',message:'Oczekiwanie',busy:false,progress:0,duration:0,elapsed:0,execution_key:'',participating:Array(6).fill(false),tcp_verified:true,start_positions:Array(6).fill(0),target_positions:Array(6).fill(0)};
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Sterowanie gotowe',configuration_revision:'initial',axes:Array.from({length:6},(_,i)=>({axis:i+1,available:true,position_known:true,position_age_seconds:0,position_degrees:0,prepared:true,enable_known:true,enabled:true,can_jog:true,can_zero:true,speed_known:true,speed_rpm:0,motion:'zatrzymany',joint_configured:false}))};
 const timer=setInterval(()=>{for(const socket of sockets)socket.send(JSON.stringify({type:'state',state}));},50);
 await context.routeWebSocket('ws://robot.test/**',socket=>{sockets.add(socket);socket.onClose(()=>sockets.delete(socket));socket.onMessage(raw=>{const data=JSON.parse(raw);if(data.type!=='trace')throw Error('Nieoczekiwana komenda JOG');});});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),method=request.method(),json=(value,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(value)});
  if(method!=='GET')writes.push({path:url.pathname,body:request.postData()?request.postDataJSON():null});
  if(url.pathname==='/api/v1/motion/status'){
   const value=copy({...motion,preparation});
   if(unavailable)return json({detail:'Brak odpowiedzi wykonawcy'},503);
   if(delayStatus){const delay=delayStatus;delayStatus=0;delaying=true;await wait(delay);delaying=false;}
   return json(value);
  }
  if(url.pathname==='/api/v1/configuration/system')return json({revision:'settings',ignore_missing_drivers:false,allow_missing_motion:partial});
  if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  if(url.pathname==='/api/v1/planning/context')return json({model_id:'model',configuration_revision:'initial',settings_revision:'settings',planner_available:true,ignore_missing_drivers:false,available:Array(6).fill(true),preview_seed_degrees:Array(6).fill(0),seed_sources:Array(6).fill('measured'),profiles:Array.from({length:6},()=>({configured:false,lower:-3600,upper:3600,velocity:80})),tcp_step_available:true});
  if(url.pathname==='/api/v1/planning/moves'){
   const body=request.postDataJSON(),id='plan-'+(++jobs),start=body.seed_degrees.map(x=>x*Math.PI/180),target=body.target_degrees.map(x=>x*Math.PI/180);
   plans.set(id,{id,model_id:'model',configuration_revision:'initial',settings_revision:'settings',simulation_only:true,simulated_start:Array(6).fill(false),execution_ready:false,execution_message:'Symulacja',mks_compatible:true,mks_message:'OK',mks_samples:[],locked:Array(6).fill(false),motion_type:body.motion_type,start_positions:start,target_positions:target,trajectory:{joint_names:Array.from({length:6},(_,i)=>`joint_${i+1}`),points:[start,target].map((positions,i)=>({positions,velocities:Array(6).fill(0),accelerations:Array(6).fill(0),time_from_start:{sec:i,nanosec:0}}))}});
   return json({id,status:'running'});
  }
  if(url.pathname.startsWith('/api/v1/planning/jobs/'))return json(method==='DELETE'?{status:'cancelled'}:{status:'succeeded',phase:'TOTG',progress:1,plan_id:url.pathname.split('/').at(-1)});
  if(url.pathname.startsWith('/api/v1/planning/plans/'))return json(plans.get(url.pathname.split('/').at(-1)));
  if(url.pathname==='/api/v1/motion/prepare'){
   if(failPrepare){const detail=failPrepare;failPrepare='';return json({detail},422);}
   const selected=plans.get(request.postDataJSON().plan_id);assert(selected);
   preparation={id:'preparation-'+(++preparations),state:'ready',preview_plan_id:selected.id,program_id:'program-'+preparations,motion_type:selected.motion_type};
   motion={...motion,state:'ready',busy:false,program_id:preparation.program_id,motion_type:selected.motion_type,duration:2,progress:0,elapsed:0,participating:Array(6).fill(true).map((v,i)=>v && (!partial || i<2)),tcp_verified:!partial,message:'Gotowy',execution_key:'',target_positions:Array(6).fill(.2)};
   const result=copy(preparation),delay=prepareDelay;prepareDelay=0;if(delay)await wait(delay);return json(result,202);
  }
  if(url.pathname==='/api/v1/motion/prepared'){
   const plan=copy(plans.get(preparation.preview_plan_id));plan.simulation_only=false;return json({program_id:motion.program_id,plan});
  }
  if(url.pathname==='/api/v1/motion/execute'){
   const body=request.postDataJSON();
   if(unknownStart){const timeout=unknownStart==='timeout';unknownStart=false;setTimeout(()=>{motion={...motion,state:'running',busy:true,execution_key:body.execution_key};},1200);return timeout?json({detail:'Przekroczony czas odpowiedzi wykonania'},408):route.abort('failed');}
   motion={...motion,state:'running',busy:true,execution_key:body.execution_key};return json({...motion,preparation},202);
  }
  if(url.pathname==='/api/v1/motion/stop'){
   if(failStop){failStop=false;return json({detail:'Żądanie STOP nie dotarło'},503);}
   const finish=()=>{motion={...motion,state:'failed',busy:false,message:'STOP operatora'};};if(stopDelay)setTimeout(finish,stopDelay);else finish();return json({stopped:true});
  }
  if(url.pathname.startsWith('/api/')){assert.equal(method,'GET');return json({detail:'Niedostępne w atrapie'},503);}
  assert.equal(method,'GET');const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try{
  const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));await page.goto('http://robot.test/#manual');
  const center=page.getByRole('region',{name:'Centrum komunikatów',exact:true}),physical=page.getByRole('region',{name:'Wykonanie fizycznego PTP/LIN',exact:true});
  const issue=source=>center.locator(`.control-issue[data-source="${source}"]`),stop=page.getByRole('button',{name:'STOP robota',exact:true});
  await center.waitFor();await page.getByRole('button',{name:'Planowanie',exact:true}).waitFor();
  await wait(600);assert.equal(writes.length,0,'Otwarcie panelu i komunikatów nie uruchamia operacji');
  state.axes[0]={...state.axes[0],joint_configured:true,calibration_valid:false,joint_position_known:false,joint_position_degrees:0,joint_lower_degrees:-180,joint_upper_degrees:180,configuration_message:'Odniesienie niepotwierdzone'};
  await issue('profile-1').waitFor();assert((await issue('profile-1').textContent()).includes('Co zrobić:'));
  await issue('profile-1').getByText('Szczegóły techniczne',{exact:true}).click();assert.equal(writes.length,0);await issue('profile-1').getByRole('link',{name:'Otwórz konfigurację →'}).click();await page.waitForURL('**/#config');assert.equal(writes.length,0,'Otwarcie zalecanej konfiguracji nie wysyła komend');await page.goto('http://robot.test/#manual');await issue('profile-1').waitFor();
  state.axes[0].calibration_valid=state.axes[0].joint_position_known=true;state.axes[0].configuration_message='';await issue('profile-1').waitFor({state:'detached'});
  state.axes[1].position_age_seconds=1;await issue('axis-feedback').waitFor();state.axes[1].position_age_seconds=0;await issue('axis-feedback').waitFor({state:'detached'});await page.getByRole('button',{name:'Planowanie',exact:true}).click();
  async function choose(type){await page.getByRole('button',{name:type,exact:true}).click();await center.locator('.control-current').getByText(new RegExp(`${type} · 1.00 s · pętla`)).waitFor();}
  async function prepare(type){await physical.getByRole('button',{name:`Przygotuj ${type} robota`,exact:true}).click();await page.waitForFunction(()=>!!document.querySelector('.motion-summary')?.dataset.programId);await page.waitForFunction(()=>!document.querySelector('.motion-confirm input')?.disabled);}
  await choose('PTP');failPrepare='Cel poza limitami osi J1';await physical.getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();
  await issue('motion-prepare').waitFor();assert((await issue('motion-prepare').textContent()).includes('Popraw wskazane pole'));
  await prepare('PTP');await issue('motion-prepare').waitFor({state:'detached'});
  assert.equal(writes.filter(w=>w.path.endsWith('/execute')).length,0);
  prepareDelay=1400;await physical.getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();await wait(100);await stop.click();await wait(350);await prepare('PTP');
  const newestProgram=motion.program_id;await wait(1100);assert.equal(await physical.locator('.motion-summary').getAttribute('data-program-id'),newestProgram,'Stara odpowiedź przygotowania nie zastępuje nowego programu');assert(await physical.getByRole('checkbox').isEnabled());
  await physical.getByRole('checkbox').check();delayStatus=1300;while(!delaying)await wait(20);
  await physical.getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();
  await wait(850);assert(await physical.getByRole('checkbox').isDisabled(),'Status starszy niż750 ms blokuje zatwierdzenie');assert(await stop.isEnabled());
  await page.waitForFunction(()=>!document.querySelector('.motion-confirm input')?.disabled);
  assert(!await physical.getByRole('checkbox').isChecked(),'Powrót świeżych danych nie przywraca zgody');
  assert.equal(await physical.locator('.motion-summary').getAttribute('data-program-id'),motion.program_id,'Spóźniony status nie przywraca starego programu');
  unavailable=true;await issue('motion-connection').waitFor();assert(await physical.getByRole('checkbox').isDisabled());
  unavailable=false;await issue('motion-connection').waitFor({state:'detached'});assert(!await physical.getByRole('checkbox').isChecked());
  await physical.getByRole('checkbox').focus();await page.keyboard.press('Space');assert(await physical.getByRole('checkbox').isChecked());await physical.getByRole('button',{name:'Wykonaj PTP',exact:true}).focus();await page.keyboard.press('Enter');await wait(350);assert(motion.busy);
  const historyCount=await center.locator('.history-entry').count(),announcement=await center.getByRole('status').textContent();
  for(let i=1;i<=4;i++){motion.progress=i/10;await wait(300);}assert.equal(await center.locator('.history-entry').count(),historyCount,'Procenty postępu nie dopisują historii');assert.equal(await center.getByRole('status').textContent(),announcement,'Czytnik nie ogłasza procentów postępu');
  for(const socket of [...sockets])socket.close({code:1001,reason:'Utrata połączenia w teście'});await issue('connection-issue').waitFor();assert(motion.busy,'Zerwanie panelu nie oznacza zatrzymania programu');assert(await physical.getByRole('checkbox').isDisabled());await issue('connection-issue').waitFor({state:'detached'});assert(motion.busy);
  failStop=true;await stop.click();await issue('motion-stop').waitFor();assert(motion.busy);assert((await issue('motion-stop').textContent()).includes('fizycznego zatrzymania'));
  stopDelay=800;await stop.click();await issue('motion-stop').waitFor({state:'detached'});assert(motion.busy,'Przyjęcie STOP nie jest potwierdzeniem bezczynności');assert((await center.locator('.history-entry').first().textContent()).includes('Oczekiwanie na aktualny stan'));await wait(1100);assert(!motion.busy);stopDelay=0;assert.equal(await issue('motion-failed').count(),0,'STOP operatora jest informacją, nie nową awarią');
  await choose('LIN');partial=true;await prepare('LIN');await issue('motion-partial').getByRole('heading',{name:'Próba LIN — tor TCP nie jest potwierdzony',exact:true}).waitFor();
  await physical.getByRole('checkbox').check();unknownStart='timeout';await physical.getByRole('button',{name:'Wykonaj LIN',exact:true}).click();
  await issue('motion-execute').getByRole('heading',{name:'Wynik żądania startu jest niepotwierdzony',exact:true}).waitFor();
  assert(await physical.getByRole('button',{name:'Przygotuj LIN robota',exact:true}).isDisabled());assert(await physical.getByRole('checkbox').isDisabled());
  await issue('motion-execute').waitFor({state:'detached'});assert(motion.busy);
  assert.equal(writes.filter(w=>w.path.endsWith('/execute')).length,2,'Nie powtórzono automatycznie startu');
  await stop.click();await wait(400);assert(!motion.busy);
  const layouts=[];
  for(const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]){
   await page.setViewportSize(viewport);await wait(200);await page.evaluate(()=>window.scrollTo(0,0));
   const bounds=await stop.boundingBox();assert(bounds.y>=0 && bounds.y+bounds.height<=viewport.height,JSON.stringify({viewport,bounds}));
   assert(await stop.isEnabled());assert.equal(await page.locator('.motion-workflow').evaluate(e=>e.scrollWidth>e.clientWidth+1),false,'Karta ruchu nie ma poziomego przepełnienia');
   layouts.push({viewport,stop:bounds});
  }
  await page.setViewportSize({width:1366,height:768});await page.getByRole('button',{name:'Rozwiń menu',exact:true}).click();
  await page.evaluate(()=>document.body.style.zoom='1.25');await page.evaluate(()=>window.scrollTo(0,document.body.scrollHeight));await wait(300);
  const zoomStop=await stop.boundingBox();assert(zoomStop.y>=0 && zoomStop.y+zoomStop.height<=768,'STOP dostępny przy125% i rozwiniętym menu');
  assert(await stop.isEnabled());await stop.focus();assert(await stop.evaluate(e=>e===document.activeElement));
  await page.evaluate(()=>{document.body.style.zoom='1';window.scrollTo(0,0);});await page.getByRole('button',{name:'Zwiń menu',exact:true}).click();
  await page.evaluate(()=>{document.querySelector('.manual-controls-row').scrollTop=0;});await wait(300);await page.screenshot({path:'/tmp/robot-control-center.png',fullPage:true});
  assert.deepEqual(errors,[]);console.log(JSON.stringify({result:'OK',layouts,checks:['wspólne opisy i rozwiązania','kalibracja i potwierdzone ustąpienie','PTP/LIN przygotowanie i zgoda','błąd i ponowienie','opóźniony status i750 ms','utrata łączności','niepełny zestaw','niepotwierdzony start bez ponawiania','błąd STOP i odczyt wyniku','deduplikacja postępu','trzy rozdzielczości','menu i125%','fokus klawiatury','brak automatycznych komend']}));
 }finally{clearInterval(timer);await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
