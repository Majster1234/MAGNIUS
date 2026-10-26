// Wyłącznie atrapy HTTP/WebSocket. Brak ROS, CAN i rzeczywistych poleceń ruchu.
const {chromium}=require('playwright');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000}});
 const dist=process.env.ROBOT_WEB_TEST_DIST || path.resolve(__dirname,'../dist'),sockets=new Set(),commands=[],errors=[];
 const tcp={fresh:true,ready:true,busy:false,state:'ready',message:'JOG TCP gotowy',axis:0,direction:0,frame:'base',impulse:false,progress:0,
  requested_step:0,velocity:0,max_linear_velocity:.02,max_angular_velocity:.2,model_id:'model',configuration_revision:'initial',generation:0};
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Atrapa',configuration_revision:'initial',tcp_jog:tcp,
  axes:Array.from({length:6},(_,i)=>({axis:i+1,available:true,prepared:true,enable_known:true,enabled:true,position_known:true,position_age_seconds:0,
   position_degrees:0,speed_known:true,speed_rpm:0,can_jog:true,can_zero:true,motion:'zatrzymany',joint_configured:true,calibration_valid:true,
   joint_position_known:true,joint_position_degrees:[15,-20,40,15,30,-10][i],joint_lower_degrees:-180,joint_upper_degrees:180,jog_min_velocity_degrees_s:.1,jog_max_velocity_degrees_s:20}))};
 const stopped=()=>{tcp.busy=state.busy=false;tcp.ready=true;tcp.state='stopped';tcp.message='Zatrzymanie TCP potwierdzone';};
 const timer=setInterval(()=>{for(const socket of sockets)socket.send(JSON.stringify({type:'state',state}));},50);
 await context.routeWebSocket('ws://robot.test/**',socket=>{
  sockets.add(socket);socket.onClose(()=>sockets.delete(socket));
  socket.onMessage(raw=>{
   const cmd=JSON.parse(raw);if(cmd.type==='trace')return;assert.equal(cmd.space,'tcp');commands.push(cmd);
   if(cmd.action==='start'){tcp.busy=state.busy=true;tcp.ready=false;tcp.state='running';tcp.message='JOG TCP w ruchu';tcp.generation++;tcp.impulse=cmd.impulse;}
   if(cmd.action==='stop')stopped();
   socket.send(JSON.stringify({type:'tcp_ack',sequence:cmd.sequence,accepted:true,message:'Przyjęto'}));
  });
 });
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),json=value=>route.fulfill({contentType:'application/json',body:JSON.stringify(value)});
  assert.equal(request.method(),'GET');
  if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  if(url.pathname==='/api/v1/planning/context')return json({model_id:'model',configuration_revision:'initial',settings_revision:'s',planner_available:true,tcp_step_available:true,
   available:Array(6).fill(true),preview_seed_degrees:state.axes.map(a=>a.joint_position_degrees),seed_sources:Array(6).fill('measured'),profiles:Array.from({length:6},()=>({configured:true,lower:-180,upper:180,velocity:20}))});
  if(url.pathname==='/api/v1/configuration/system')return json({revision:'s',ignore_missing_drivers:false,allow_missing_motion:false});
  if(url.pathname==='/api/v1/motion/status')return json({program_id:'',state:'idle',mode:'streamed',message:'',execution_key:'',busy:false,progress:0,duration:0,
   start_positions:Array(6).fill(0),target_positions:Array(6).fill(0),participating:Array(6).fill(false),tcp_verified:false,preparation:{state:'idle'}});
  if(url.pathname.startsWith('/api/'))return route.fulfill({status:503,contentType:'application/json',body:'{}'});
  const file=path.resolve(dist,url.pathname==='/'?'index.html':url.pathname.slice(1));assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html'});
 });
 try {
  const page=await context.newPage();page.on('pageerror',error=>errors.push(error.message));await page.goto('http://robot.test/#manual');
  const x=page.getByRole('button',{name:'TCP X plus',exact:true}),rotation=page.getByRole('button',{name:'TCP RZ minus',exact:true});
  await x.waitFor();await page.waitForFunction(()=>!document.querySelector('[aria-label="TCP X plus"]').disabled);
  await page.getByRole('spinbutton',{name:'Prędkość przesuwu TCP',exact:true}).fill('10');
  await page.getByRole('spinbutton',{name:'Prędkość obrotu TCP',exact:true}).fill('5');
  const press=async button=>{await button.hover();await page.mouse.down();await delay(230);};
  const release=async()=>{await page.mouse.up();await page.waitForFunction(()=>!document.querySelector('[aria-label="Tryb TCP"]').disabled);};
  await press(x);assert(commands.filter(c=>c.action==='keepalive').length>=1);assert(await page.getByLabel('Układ TCP').isDisabled());
  assert(await page.getByRole('button',{name:'JOG J1 prawo',exact:true}).isDisabled());await release();
  assert.deepEqual(commands[0].velocity,.01);assert(commands.some(c=>c.action==='stop' && !c.emergency));
  await page.getByLabel('Układ TCP').selectOption('tool');await page.getByLabel('Tryb TCP').selectOption('step');
  await press(rotation);const start=commands.findLast(c=>c.action==='start');
  assert.equal(start.frame,'tool');assert.equal(start.impulse,true);assert(Math.abs(start.velocity-5*Math.PI/180)<1e-12);assert(Math.abs(start.step-Math.PI/180)<1e-12);
  await release();assert.equal(commands.at(-1).action,'stop');assert.equal(commands.at(-1).emergency,false);
  await press(x);await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await page.mouse.up();await delay(180);
  assert(commands.some(c=>c.action==='stop' && c.emergency));
  const count=commands.filter(c=>c.action==='start').length;await delay(300);assert.equal(commands.filter(c=>c.action==='start').length,count);
  tcp.ready=false;tcp.message='Brak skalibrowanej J6';await delay(150);assert(await x.isDisabled());
  assert.deepEqual(errors,[]);assert(commands.every((c,i)=>!i || c.sequence>commands[i-1].sequence));
  console.log(JSON.stringify({result:'OK',checks:['LIVE Baza i Narzędzie','mm/s i stopnie/s do SI','ciągły i impulsowy wymagają przytrzymania','podtrzymanie','blokady konkurencyjnego JOG','STOP po blur','brak samoczynnego wznowienia','brak osi blokuje start']}));
 } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
