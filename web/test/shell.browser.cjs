// Nawigacja i odczyty statusu na atrapach. Brak ROS/CAN i poleceń sprzętowych.
const {chromium}=require('playwright');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
(async()=>{
  const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const context=await browser.newContext({viewport:{width:1366,height:768}}),dist=path.resolve(__dirname,'../dist');
  const errors=[],writes=[],commands=[],sockets=new Set();let manualConnections=0;
  const component=(id,values={})=>({id,state:'ok',message:'Pomiar testowy',sampled_at:new Date().toISOString(),values});
  const status={schema_version:1,updated_at:new Date().toISOString(),age_seconds:0,fresh:true,
    network:{address:'robot.test',port:8080,interface:'eth0',subnet:'LAN',url:'http://robot.test'},
    components:Object.fromEntries(['api','ros','host','can','controllers','drives'].map(id=>[id,component(id,id==='can'?{interface:'can0',bus_state:'ERROR-ACTIVE',bitrate:500000}:{})]))};
  const can={schema_version:1,updated_at:new Date().toISOString(),age_seconds:0,fresh:true,interface:'can0',
    summary:{state:'ok',message:'CAN działa',values:{interface:'can0',bus_state:'ERROR-ACTIVE',bitrate:500000,device_count:4,command_period_ms:20,encoder_period_ms:40,planned_utilization_limit_percent:40,stop_result:"ACK odebrane; kasowanie celu wymaga odbioru"}},
    scan:{state:'ok',message:'Skan zakończony',values:{state:'done'}},
    devices:[1,2,5,9].map(id=>({id,state:'ok',message:'Odpowiada',values:{online:true,model:'MKS SERVO42D',enable:false}}))};
  const manual={fresh:true,busy:false,fault:false,recording:false,active_axis:0,max_rpm:30,operation:'idle',message:'Test nawigacji',
    axes:Array.from({length:6},(_,i)=>({axis:i+1,available:[1,2,5].includes(i+1),position_known:[1,2,5].includes(i+1),position_degrees:0,
      position_age_seconds:0,enable_known:true,enabled:false,prepared:false,can_jog:false,can_zero:false,motion:'zatrzymany'}))};
  function publish(){for(const item of sockets)item.socket.send(JSON.stringify(item.kind==='manual'?{type:'state',state:manual}:item.kind==='can'?can:status));}
  const timer=setInterval(publish,50);
  context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
  await context.routeWebSocket('ws://robot.test/**',socket=>{
    const kind=socket.url().endsWith('/manual/ws')?'manual':socket.url().endsWith('/can/ws')?'can':'status';
    if(kind==='manual')manualConnections++;
    const item={socket,kind};sockets.add(item);publish();socket.onClose(()=>sockets.delete(item));
    socket.onMessage(raw=>{const value=JSON.parse(raw);if(value.type!=='trace')commands.push(value);});
  });
  await context.route('http://robot.test/**',async route=>{
    const request=route.request(),url=new URL(request.url());
    if(request.method()!=='GET'){writes.push(url.pathname);return route.abort();}
    if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
    if(url.pathname.startsWith('/api/'))return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Brak usługi w teście nawigacji"}'});
    const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
    return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
  });
  try{
    const page=await context.newPage();await page.goto('http://robot.test/');
    const nav=page.getByRole('navigation',{name:'Sekcje panelu'}),sidebar=page.getByRole('complementary',{name:'Nawigacja aplikacji'});
    await page.getByText('3/6 dostępnych',{exact:true}).waitFor();
    assert.equal(await page.locator('.joints [data-present="true"]').count(),3);
    assert.equal(await page.locator('.joints [data-present="true"]').allTextContents().then(items=>items.map(s=>s.trim()).join(',')),'J1,J2,J5');
    assert(await page.getByText('can0 · OK (ERROR-ACTIVE)',{exact:true}).isVisible());
    assert.equal(await page.locator('.manual-tile').count(),0);
    await page.evaluate(()=>{window.navigationSentinel='same-document';window.resizeEvents=0;window.addEventListener('resize',()=>window.resizeEvents++);});
    for(const [id,label] of [['can','Magistrala CAN'],['diagnostics','Diagnostyka'],['manual','Sterowanie ręczne'],['config','Konfiguracja'],['architecture','Architektura'],['overview','Przegląd systemu']]){
      await nav.getByRole('link',{name:label,exact:true}).click();await page.getByRole('heading',{name:label,exact:true}).waitFor();
      assert.equal(new URL(page.url()).hash,`#${id}`);
      assert.equal(await nav.locator('a.active').count(),1);assert.equal(await nav.getByRole('link',{name:label,exact:true}).getAttribute('aria-current'),'page');
      assert.equal(await page.locator('.app-topbar').count(),1);assert(await sidebar.isVisible());
      assert.equal(await page.evaluate(()=>window.navigationSentinel),'same-document');
      assert.equal(await page.locator('.breadcrumbs > span:last-child').textContent(),label);
      if(id==='can'){
        can.scan.values.scanning=true;can.scan.values.checked=7;publish();
        await page.getByText('Adres 7 / 20',{exact:true}).waitFor();
        can.scan.values.scanning=false;publish();
        const details=page.locator('.can-bus-details');const closed=await details.boundingBox();assert(closed.height<70,'Zwinięte szczegóły nie tworzą pustej karty');
        await details.getByText('Szczegóły komunikacji',{exact:true}).click();assert((await details.boundingBox()).height>closed.height);
        assert(await page.getByText('ACK odebrane; kasowanie celu wymaga odbioru',{exact:true}).isVisible());
        assert((await details.locator('.metric').filter({has:page.getByText('Cykl komend',{exact:true})}).textContent()).includes('20'));
        assert((await details.locator('.metric').filter({has:page.getByText('Okres enkoderów',{exact:true})}).textContent()).includes('40'));
        assert(await page.getByRole('columnheader',{name:'Oś robota',exact:true}).isVisible());
        assert(await page.locator('.device-row').filter({hasText:'MKS SERVO42D'}).nth(2).getByRole('cell',{name:'J5',exact:true}).isVisible());
        await page.screenshot({path:'/tmp/robot-shell-can.png',fullPage:true});
      }
      if(id==='manual'){
        await page.locator('.preview-canvas canvas').waitFor();await page.getByText('Połączono · 20 Hz',{exact:true}).waitFor();
        assert.equal(Math.round((await sidebar.boundingBox()).width),68);
        await page.locator('.preview-canvas canvas').evaluate(el=>el.dataset.identity='same-canvas');
        const before=await page.locator('.preview-canvas').boundingBox(),resizeBefore=await page.evaluate(()=>window.resizeEvents);
        await page.getByRole('button',{name:'Rozwiń menu',exact:true}).click();
        await page.waitForFunction(n=>window.resizeEvents>n,resizeBefore);
        assert.equal(Math.round((await sidebar.boundingBox()).width),224);
        assert.equal(await page.locator('.preview-canvas canvas').getAttribute('data-identity'),'same-canvas');
        await page.getByRole('button',{name:'Zwiń menu',exact:true}).click();
        await page.waitForFunction(()=>document.querySelector('.sidebar').getBoundingClientRect().width===68);
        assert.equal(Math.round((await page.locator('.preview-canvas').boundingBox()).width),Math.round(before.width));
        assert.equal(manualConnections,1,'Zwijanie menu nie otwiera nowego strumienia sterowania');
        await page.getByRole('button',{name:'Planowanie',exact:true}).click();
        await page.locator('.app-topbar').getByText('PLANOWANIE · bliźniak',{exact:true}).waitFor();
        assert.equal(await page.locator('.jog-controls .motion-indicator').count(),0);
        for(const group of await page.locator('.tcp-joystick-group').all()){
          const cross=await group.locator('.tcp-cross').boundingBox(),depth=await group.locator('.tcp-depth').boundingBox();
          assert(depth.x>=cross.x+cross.width);const buttons=group.locator('.tcp-depth button');
          assert((await buttons.nth(0).getAttribute('aria-label')).endsWith('plus'));
          assert((await buttons.nth(1).getAttribute('aria-label')).endsWith('minus'));
        }
        assert.equal(await page.locator('.preview-viewport .cartesian-planning').count(),0);
        await page.screenshot({path:'/tmp/robot-shell-manual.png',fullPage:true});
      }
    }
    can.devices.find(device=>device.id===5).values.online=false;publish();await page.getByText('2/6 dostępnych',{exact:true}).waitFor();
    can.fresh=false;publish();await page.waitForFunction(()=>[...document.querySelectorAll('.joints > span')].every(el=>el.dataset.present==='unknown'));
    await nav.getByRole('link',{name:'Architektura',exact:true}).click();await page.screenshot({path:'/tmp/robot-shell-architecture.png',fullPage:true});
    assert.equal(await page.locator('.architecture-cards .card').count(),6);
    assert.deepEqual(writes,[]);assert.deepEqual(commands,[]);assert.deepEqual(errors,[]);
    console.log(JSON.stringify({result:'OK',checks:['5 tras bez przeładowania','aktywny link i wspólny topbar','mini-rail i resize bez restartu WebSocket/3D','3/6 osi z CAN i wygaszanie danych','ERROR-ACTIVE jako OK','szczegóły CAN bez pustej wysokości','CAN ID→J','Z/Rz osobno','symulacja poza sceną','architektura 6 kart','brak komend sprzętowych']}));
  }finally{clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
