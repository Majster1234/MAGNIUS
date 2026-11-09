// HTTP, WebSocket, ROS i napędy są emulowane; brak rzeczywistych komend CAN.
const {chromium}=require('playwright'),{execFileSync}=require('child_process');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const dist=process.env.ROBOT_WEB_TEST_DIST || path.resolve(__dirname,'../dist');
const schema=path.resolve(__dirname,'../../ros2_ws/src/robot_configuration');
const document=JSON.parse(execFileSync('python3',['-c','import sys,json;sys.path.insert(0,sys.argv[1]);from robot_configuration.schema import empty_document;print(json.dumps(empty_document()))',schema]));
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000}}),sockets=new Set(),writes=[],errors=[],snapshots=[];
 let status={active:{revision:'initial',document},operation:{id:'',phase:'idle',message:'Gotowy',success:true}};
 const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Atrapa',configuration_revision:'initial',
  axes:Array.from({length:6},(_,i)=>({axis:i+1,available:true,prepared:false,enable_known:true,enabled:false,position_known:true,position_age_seconds:0,position_degrees:0,
   speed_known:true,speed_rpm:0,can_jog:true,can_zero:true,motion:'zatrzymany',joint_configured:true,calibration_valid:true,joint_position_known:true,joint_position_degrees:0,
   joint_lower_degrees:-180,joint_upper_degrees:180,jog_min_velocity_degrees_s:.1,jog_max_velocity_degrees_s:20}))};
 const timer=setInterval(()=>{for(const socket of sockets)socket.send(JSON.stringify({type:'state',state}));},50);
 await context.routeWebSocket('ws://robot.test/**',socket=>{sockets.add(socket);socket.onClose(()=>sockets.delete(socket));socket.onMessage(raw=>assert.equal(JSON.parse(raw).type,'trace'));});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),method=request.method(),body=request.postData()?request.postDataJSON():undefined;
  const json=(value,code=200)=>route.fulfill({status:code,contentType:'application/json',body:JSON.stringify(value)});
  if(method!=='GET')writes.push({path:url.pathname,body});
  if(url.pathname==='/api/v1/configuration')return json(status);
  if(url.pathname==='/api/v1/configuration/system')return json({revision:'s',ignore_missing_drivers:false,allow_missing_motion:false});
  if(url.pathname==='/api/v1/configuration/snapshots'){
   if(method==='GET')return json({items:snapshots,next_cursor:null});
   snapshots.push({id:'snap',name:body.document.name,created_at:body.document.created_at});return json(snapshots[0],201);
  }
  if(url.pathname==='/api/v1/configuration/apply'){
   assert.equal(body.expected_revision,status.active.revision);
   status={active:{revision:'r1',document:body.document},operation:{id:'apply',phase:'done',message:'Zapisano limity TCP',success:true}};
   state.configuration_revision='r1';return json(status,202);
  }
  if(url.pathname==='/api/v1/manual/enable-all'){
   assert.equal(method,'POST');assert.equal(typeof body.enabled,'boolean');state.busy=true;state.operation=body.enabled?'enable_all':'disable_all';
   setTimeout(()=>{for(const axis of state.axes){axis.enabled=body.enabled;axis.prepared=body.enabled;}state.busy=false;state.operation='idle';state.message='Zbiorcze ENABLE potwierdzone';},250);
   return json({accepted:true,message:'Zbiorcze ENABLE przyjęte'},202);
  }
  if(url.pathname==='/api/v1/robot/description'){
   let xml=fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8');
   xml=xml.replace(/configuration_revision="[^"]*"/g,'').replace('<robot ',`<robot configuration_revision="${state.configuration_revision}" `);
   return route.fulfill({contentType:'application/xml',body:xml});
  }
  if(url.pathname==='/api/v1/planning/context')return json({model_id:'m',configuration_revision:state.configuration_revision,settings_revision:'s',planner_available:true,
   available:Array(6).fill(true),preview_seed_degrees:Array(6).fill(0),seed_sources:Array(6).fill('measured'),profiles:Array.from({length:6},()=>({configured:true,lower:-180,upper:180,velocity:20}))});
  if(url.pathname.startsWith('/api/')){assert.equal(method,'GET');return json({},503);}
  const file=path.resolve(dist,url.pathname==='/'?'index.html':url.pathname.slice(1));assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html'});
 });
 try {
  const page=await context.newPage();page.on('pageerror',error=>errors.push(error.message));await page.goto('http://robot.test/#configuration');
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  await page.getByText('Fizyczny JOG TCP — wymagane limity',{exact:true}).click();
  const fields=[['Maksymalna prędkość przesuwu TCP','100'],['Maksymalna prędkość obrotu TCP','30'],['Przyspieszenie i hamowanie przesuwu TCP','500'],['Przyspieszenie i hamowanie obrotu TCP','60']];
  for(const [label,value] of fields)await page.getByRole('spinbutton',{name:new RegExp(label)}).fill(value);
  await page.getByRole('button',{name:'Zapisz snapshot',exact:true}).click();
  await page.getByRole('button',{name:'Zapisz do ROS',exact:true}).click();
  await page.getByText('Bez zmian w szkicu',{exact:true}).waitFor();
  for(const write of writes.filter(w=>w.body?.document))assert.equal(write.body.document.values['tcp.max_linear_velocity'],100);
  await page.screenshot({path:'/tmp/tcp-settings-config.png',fullPage:true});
  await page.goto('http://robot.test/#manual');
  const enable=page.getByRole('button',{name:'Załącz wszystkie',exact:true}),disable=page.getByRole('button',{name:'Wyłącz wszystkie',exact:true});
  await page.waitForFunction(()=>![...document.querySelectorAll('button')].find(b=>b.textContent==='Załącz wszystkie')?.disabled);
  await enable.click();await delay(80);assert(await enable.isDisabled());
  await page.waitForFunction(()=>document.querySelectorAll('.manual-drive .badge.ok').length===6);
  await disable.click();await page.waitForFunction(()=>document.querySelectorAll('.manual-drive .badge.warn').length===6);
  const group=writes.filter(w=>w.path==='/api/v1/manual/enable-all');assert.deepEqual(group.map(w=>w.body.enabled),[true,false]);
  await page.screenshot({path:'/tmp/tcp-settings-controls.png',fullPage:true});
  await page.getByRole('button',{name:'Planowanie',exact:true}).click();assert(await enable.isDisabled());assert(await disable.isDisabled());
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',checks:['4 limity TCP w konfiguracji','snapshot i zapis ROS zachowują wartości','jedno żądanie na zbiorcze ENABLE','włączenie i wyłączenie 6 osi','blokady operacji i planowania','brak komend ruchu']}));
 } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
