// PTP/LIN, trzy modele i globalne ustawienie; wszystkie API są atrapami bez CAN.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path'), assert = require('assert/strict');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
 const browser = await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context = await browser.newContext({viewport:{width:1366,height:768}});
 const dist=process.env.ROBOT_WEB_TEST_DIST || path.resolve(__dirname,'../dist'), commands=[], writes=[], errors=[], sockets=new Set(), requests=[], plans=new Map(), jobs=new Map(), events=[];
 let settings={revision:'initial',ignore_missing_drivers:false,allow_missing_motion:false}, seed=Array(6).fill(0), slow=false, cancelling=false, delayedPlan=false;
 let motion={program_id:'',state:'idle',mode:'',message:'Oczekiwanie',execution_key:'',participating:Array(6).fill(false),tcp_verified:false,busy:false,progress:0,duration:0,start_positions:Array(6).fill(0),target_positions:Array(6).fill(0)}, preparation={state:'idle'};const physicalWrites=[];
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Podgląd bez napędów',configuration_revision:'initial',
  axes:Array.from({length:6},(_,i)=>({axis:i+1,available:false,position_known:false,position_age_seconds:0,position_degrees:0,prepared:false,enable_known:true,enabled:false,
   can_jog:false,can_zero:false,speed_known:true,speed_rpm:0,motion:'zatrzymany'}))};
 const timer=setInterval(()=>{for(const socket of sockets) socket.send(JSON.stringify({type:'state',state}));},50);
 context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
 await context.routeWebSocket('ws://robot.test/**',socket=>{if(!socket.url().endsWith('/manual/ws'))return;sockets.add(socket);socket.onClose(()=>sockets.delete(socket));socket.onMessage(raw=>{const v=JSON.parse(raw);if(v.type!=='trace')commands.push(v);});});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),json=(data,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(data)});
  if(url.pathname==='/api/v1/motion/status')return json({...motion,preparation});
  if(url.pathname==='/api/v1/motion/prepare'){
   if(!settings.allow_missing_motion)return json({detail:'Tryb niepełnego zestawu jest wyłączony'},422);
   const selected=plans.get(request.postDataJSON().plan_id);
   preparation={id:'prep-'+requests.length,state:'ready',program_id:'prepared',preview_plan_id:selected.id,motion_type:selected.motion_type};
   motion={...motion,program_id:'prepared',state:'ready',motion_type:selected.motion_type,mode:'streamed',participating:[true,true,false,false,false,false],duration:2,start_positions:Array(6).fill(.1),target_positions:Array(6).fill(.2)};
   return json(preparation,202);
  }
  if(url.pathname==='/api/v1/motion/prepared'){
   const plan=structuredClone(plans.get(preparation.preview_plan_id));plan.simulation_only=false;
   plan.start_positions=Array(6).fill(.1);plan.target_positions=[.2,.2,.1,.1,.1,.1];
   plan.trajectory.points[0].positions=Array(6).fill(.1);plan.trajectory.points[1].positions=plan.target_positions;
   return json({program_id:motion.program_id,plan});
  }
  if(url.pathname==='/api/v1/motion/execute'){
   physicalWrites.push(request.postDataJSON());motion={...motion,state:'running',busy:true,execution_key:request.postDataJSON().execution_key};return json(motion,202);
  }
  if(url.pathname==='/api/v1/motion/stop'){
   physicalWrites.push('STOP');motion={...motion,state:'failed',busy:false,message:'Zatrzymano'};return json({stopped:true});
  }
  if(url.pathname.startsWith('/api/v1/manual/axes')){writes.push(url.pathname);return route.abort();}
  if(url.pathname==='/api/v1/configuration/system'){
   if(request.method()==='PUT'){settings={...settings,...request.postDataJSON(),revision:String(Number(settings.revision)||0)+requests.length+'changed'};}
   return json(settings);
  }
  if(url.pathname.startsWith('/api/v1/configuration'))return json({detail:'Brak konfiguracji szkicu w tej atrapie'},503);
  if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  if(url.pathname==='/api/v1/planning/context')return json({model_id:'model',configuration_revision:'initial',settings_revision:settings.revision,ignore_missing_drivers:settings.ignore_missing_drivers,
   planner_available:true,available:state.axes.map(a=>a.available),preview_seed_degrees:seed,seed_sources:state.axes.map(a=>a.available?'measured':'zero'),
   profiles:Array.from({length:6},()=>({configured:false,lower:-3600,upper:3600,velocity:80}))});
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
  const partial=config.getByRole('switch',{name:'Zezwalaj na ruch bez wszystkich silników',exact:true});
  const preview=config.getByRole('switch',{name:'Ignoruj brak sterowników — tylko planowanie',exact:true});
  await partial.waitFor();assert(!await partial.isChecked());await preview.click();
  assert(settings.ignore_missing_drivers && !settings.allow_missing_motion);
  let page=await context.newPage();await page.goto('http://robot.test/#manual');
  await page.getByRole('button',{name:'Planowanie',exact:true}).click();
  const velocity=page.getByRole('spinbutton',{name:'Prędkość PTP °/s',exact:true});
  const acceleration=page.getByRole('spinbutton',{name:'Przyspieszenie PTP °/s²',exact:true});
  assert.equal(await velocity.inputValue(),'6','Nowy panel nie wybiera automatycznie zwiększonej prędkości');
  await velocity.fill('81');assert(await page.getByRole('button',{name:'PTP',exact:true}).isDisabled());
  await velocity.fill('');assert(await page.getByRole('button',{name:'PTP',exact:true}).isDisabled());
  await velocity.fill('80');await acceleration.fill('0');assert(await page.getByRole('button',{name:'PTP',exact:true}).isDisabled());
  assert.equal(requests.length,0,'Błędne parametry nie rozpoczynają obliczeń');
  await acceleration.fill('125');
  await page.getByRole('button',{name:'PTP',exact:true}).click();
  await page.locator(".control-current").getByText(/PTP · 0.60 s · pętla/).waitFor();
  assert.equal(requests.at(-1).preview_velocity_degrees_s,80);
  assert.equal(requests.at(-1).preview_acceleration_degrees_s2,125);
  await page.screenshot({path:'/tmp/robot-ptp-parameters.png',fullPage:true});
  const physical=()=>page.getByRole('region',{name:'Wykonanie fizycznego PTP/LIN',exact:true});
  await physical().getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();
  await page.locator('.control-issue[data-source="motion-prepare"]').waitFor();
  assert.deepEqual(physicalWrites,[]);
  await config.bringToFront();await partial.click();assert(settings.allow_missing_motion && settings.ignore_missing_drivers);
  await page.bringToFront();await delay(1400);
  await page.getByRole('button',{name:'PTP',exact:true}).click();
  await page.locator(".control-current").getByText(/PTP · 0.60 s · pętla/).waitFor();
  await physical().getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();
  await page.locator('.control-issue[data-source="motion-partial"]').waitFor();
  const execute=physical().getByRole('button',{name:'Wykonaj PTP',exact:true});
  assert(await execute.isDisabled());
  await physical().getByRole('button',{name:'Podgląd przygotowanego PTP',exact:true}).click();
  await page.locator(".control-current").getByText(/Podgląd przygotowanego PTP ·/).waitFor();
  await delay(150);
  const q=JSON.parse(await page.locator('.robot-preview canvas').getAttribute('data-motion-joint-angles'));
  assert(Math.abs(q[2]-.1)<1e-6,'Podgląd nie porusza pominiętą osią: '+JSON.stringify(q));
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).check();
  const computed=requests.length;
  await velocity.fill('40');await acceleration.fill('200');await delay(350);
  assert(await execute.isDisabled(),'Zmiana parametrów unieważnia przygotowane wykonanie');
  assert(await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).isDisabled());
  assert.equal(requests.length,computed,'Zmiana nie uruchamia samoczynnie obliczeń ani ruchu');
  assert.deepEqual(physicalWrites,[]);
  await page.getByRole('button',{name:'PTP',exact:true}).click();
  await page.locator(".control-current").getByText(/PTP · 0.60 s · pętla/).waitFor();
  assert.equal(requests.at(-1).preview_velocity_degrees_s,40);
  assert.equal(requests.at(-1).preview_acceleration_degrees_s2,200);
  await physical().getByRole('button',{name:'Przygotuj PTP robota',exact:true}).click();
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).waitFor();
  assert(!await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).isChecked());
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).check();
  await execute.click();await delay(350);assert(motion.busy);assert.equal(physicalWrites.length,1);
  await page.close();assert(motion.busy);assert.equal(physicalWrites.length,1,'Zamknięcie panelu nie zatrzymuje programu');
  page=await context.newPage();await page.goto('http://robot.test/#manual');
  await page.getByRole('button',{name:'STOP robota',exact:true}).waitFor();
  for(const [state,label] of [['waiting_position','Oczekiwanie na pozycję osi'],['waiting_feedback','Oczekiwanie na pomiary'],['waiting_budget','Oczekiwanie na budżet CAN']]) {
   motion={...motion,state,progress:.4,elapsed:4,message:'Oczekiwanie grupy J1, J2'};
   await physical().getByText(label,{exact:true}).waitFor();
   assert((await physical().textContent()).includes('2.00 s') && (await physical().textContent()).includes('4.00 s'));
   assert(await physical().getByRole('button',{name:/^Wykonaj /}).isDisabled());
   assert(await page.getByRole('button',{name:'STOP robota',exact:true}).isEnabled());
   assert.equal(physicalWrites.length,1,'Oczekiwanie nie ponawia START ani nie wysyła STOP');
  }
  await page.getByRole('button',{name:'STOP robota',exact:true}).click();await delay(250);
  assert(!motion.busy);assert.deepEqual(physicalWrites.slice(1),['STOP']);assert.deepEqual(errors,[]);
  await page.getByRole('button',{name:'Planowanie',exact:true}).click();
  await page.getByRole('button',{name:'LIN',exact:true}).click();
  await page.locator(".control-current").getByText(/LIN · 0.60 s · pętla/).waitFor();
  await physical().getByRole('button',{name:'Przygotuj LIN robota',exact:true}).click();
  await page.locator('.control-issue[data-source="motion-partial"]').getByRole('heading',{name:'Próba LIN — tor TCP nie jest potwierdzony',exact:true}).waitFor();
  assert.equal(motion.motion_type,'LIN');
  assert(await physical().getByRole('button',{name:'Wykonaj LIN',exact:true}).isDisabled());
  await physical().getByRole('button',{name:'Podgląd przygotowanego LIN',exact:true}).click();
  await page.locator(".control-current").getByText(/Podgląd przygotowanego LIN ·/).waitFor();
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).check();
  await page.getByText('Prędkość podglądu LIN',{exact:true}).click();
  const previousWrites=physicalWrites.length;
  await page.getByRole('spinbutton',{name:'Prędkość TCP mm/s',exact:true}).fill('12');await delay(350);
  assert(await physical().getByRole('button',{name:'Wykonaj LIN',exact:true}).isDisabled());
  assert.equal(physicalWrites.length,previousWrites,'Zmiana posuwu nie uruchamia robota');
  // Przygotowany podgląd zatrzymuje automatyczną pętlę; nowy wybór daje nowe limity.
  await page.getByRole('button',{name:'LIN',exact:true}).click();
  await page.locator(".control-current").getByText(/LIN · 0.60 s · pętla/).waitFor();
  assert.equal(requests.at(-1).tcp_velocity_mm_s,12);
  await physical().getByRole('button',{name:'Przygotuj LIN robota',exact:true}).click();
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).waitFor();
  assert(!await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).isChecked());
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).check();
  await page.getByRole('button',{name:'PTP',exact:true}).click();
  await page.locator(".control-current").getByText(/PTP · 0.60 s · pętla/).waitFor();
  assert(await physical().getByRole('button',{name:'Wykonaj PTP',exact:true}).isDisabled());
  await page.getByRole('button',{name:'LIN',exact:true}).click();
  await page.locator(".control-current").getByText(/LIN · 0.60 s · pętla/).waitFor();
  await physical().getByRole('button',{name:'Przygotuj LIN robota',exact:true}).click();
  await physical().getByRole('checkbox',{name:/Zatwierdzam fizyczny ruch/}).check();
  await physical().getByRole('button',{name:'Wykonaj LIN',exact:true}).click();await delay(350);
  assert(motion.busy && motion.motion_type==='LIN');
  assert.equal(physicalWrites.length,previousWrites+1,'LIN uruchomiony dokładnie raz po zatwierdzeniu');
  await page.getByRole('button',{name:'STOP robota',exact:true}).click();await delay(250);
  assert(!motion.busy);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',checks:['fizyczny LIN: przygotowanie/podgląd/zgoda/wykonanie/STOP','posuw LIN i zmiana typu unieważniają zgodę','parametry PTP i zakres80°/s','puste i nieprawidłowe wartości','unieważnienie programu i zgody po zmianie','nowy plan z nową prędkością/przyspieszeniem','dwa niezależne suwaki','lista osi i niepotwierdzone TCP','jawne zatwierdzenie','wykonanie po zamknięciu panelu','stany oczekiwania bez ponawiania ruchu','ponowne połączenie i STOP']}));
 }finally{clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
