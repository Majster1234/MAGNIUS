// Testuje wyłącznie atrapy HTTP/WS. Nie łączy się z ROS ani CAN.
const {chromium}=require('playwright');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const dist=path.resolve(__dirname,'../dist');
(async()=>{
  const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const context=await browser.newContext({viewport:{width:1600,height:1000}});
  const sockets=new Set(),writes=[],errors=[];
  const state={fresh:true,busy:true,fault:true,active_axis:0,max_rpm:30,operation:'fault',message:'STOP wysłany',
    recovery_state:'verifying_disabled',recovery_message:'Oczekiwanie na ENABLE=0 i postój',recovery_pending_mask:4,
    axes:Array.from({length:6},(_,i)=>({axis:i+1,available:true,prepared:false,enable_known:true,enabled:false,
      position_known:true,position_age_seconds:0,position_degrees:12+i,speed_known:true,speed_rpm:0,
      can_jog:false,can_zero:false,motion:'zatrzymany'}))};
  const update=()=>{for(const socket of sockets) socket.send(JSON.stringify({type:'state',state}));};
  const timer=setInterval(update,50);
  await context.routeWebSocket('ws://robot.test/**',socket=>{
    if(!socket.url().endsWith('/api/v1/manual/ws')) return;
    sockets.add(socket);socket.onClose(()=>sockets.delete(socket));
    socket.onMessage(raw=>assert.equal(JSON.parse(raw).type,'trace'));update();
  });
  await context.route('http://robot.test/**',async route=>{
    const request=route.request(),url=new URL(request.url());
    if(request.method()==='POST') {
      writes.push({path:url.pathname,body:request.postData()?request.postDataJSON():null});
      return route.fulfill({status:202,contentType:'application/json',body:'{"accepted":true,"message":"Przyjęte"}'});
    }
    if(url.pathname==='/api/v1/robot/description') return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
    if(url.pathname.startsWith('/api/')) return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Atrapa odzyskiwania"}'});
    const file=path.resolve(dist,url.pathname==='/'?'index.html':url.pathname.slice(1));assert(file.startsWith(dist+path.sep));
    return route.fulfill({path:file,contentType:file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html'});
  });
  try {
    const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
    await page.goto('http://robot.test/#manual');
    await page.locator('.manual-recovery').getByText(/oczekujące osie: J3/).waitFor();
    assert(await page.getByRole('button',{name:'Załącz wszystkie',exact:true}).isDisabled());
    assert(await page.getByRole('button',{name:'Wyłącz wszystkie',exact:true}).isEnabled());
    assert(await page.getByRole('button',{name:'Reset CAN',exact:true}).isEnabled());
    for(let j=1;j<=6;j++) assert(await page.getByRole('button',{name:`JOG J${j} prawo`,exact:true}).isDisabled());
    await page.getByRole('button',{name:'Wyłącz wszystkie',exact:true}).click();
    assert.deepEqual(writes.at(-1),{path:'/api/v1/manual/enable-all',body:{enabled:false}});
    state.recovery_state='recovery_failed';state.recovery_message='J3: odrzucony F5';update();
    await page.locator('.manual-recovery').getByText('Odzyskiwanie nieudane',{exact:true}).waitFor();
    state.axes[0].position_degrees=42;update();await page.getByText('42.0°',{exact:true}).waitFor();
    for(const width of [1280,1366,1920]) {
      await page.setViewportSize({width,height:1000});
      const header=await page.locator('.manual-header').boundingBox(),banner=await page.locator('.manual-recovery').boundingBox();
      const workspace=await page.locator('.manual-workspace').boundingBox();
      assert(banner.y>=header.y+header.height-1);assert(workspace.y>=banner.y+banner.height-1);
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    }
    state.busy=false;state.fault=false;state.recovery_state='idle_disabled';
    state.recovery_message='Gotowy do ponownego załączenia';state.recovery_pending_mask=0;update();
    await page.getByRole('button',{name:'Załącz wszystkie',exact:true}).waitFor({state:'visible'});
    await page.waitForFunction(()=>!Array.from(document.querySelectorAll('button')).find(b=>b.textContent==='Załącz wszystkie')?.disabled);
    for(const axis of state.axes) assert.equal(axis.enabled,false);
    assert.deepEqual(errors,[]);console.log('Recovery UI: etapy, blokady ruchu, odczyty, ENABLE=0 i układ 1280/1366/1920 OK');
  } finally {clearInterval(timer);await browser.close();}
})().catch(error=>{console.error(error);process.exit(1);});
