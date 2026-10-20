// Wszystkie HTTP/WS są atrapami. Brak połączeń ROS/CAN i komend sprzętowych.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path'), assert = require('assert/strict');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
  const browser = await chromium.launch({ headless:true, args:['--use-angle=swiftshader','--enable-unsafe-swiftshader'] });
  const context = await browser.newContext({ viewport:{width:1366,height:768} });
  const dist = path.resolve(__dirname,'../dist'), sockets = new Set(), commands=[], writes=[], errors=[], requests=[], stepRequests=[], events=[], jobs=new Map(), plans=new Map();
  let settings={revision:'s0',ignore_missing_drivers:false}, slow=false, delayed=false, fail=false, cancelling=false, stepDurationMs=600;
  const state={fresh:true,busy:false,fault:false,active_axis:0,max_rpm:30,operation:'idle',message:'Test TCP',configuration_revision:'initial',
    axes:Array.from({length:6},(_,i)=>({axis:i+1,available:i<2,position_known:i<2,position_age_seconds:0,position_degrees:i+5,
      prepared:true,enable_known:true,enabled:true,can_jog:i<2,can_zero:false,speed_known:true,speed_rpm:0,motion:'zatrzymany'}))};
  const publish=()=>{for(const socket of sockets)socket.send(JSON.stringify({type:'state',state}));};
  const timer=setInterval(publish,50);
  context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
  await context.routeWebSocket('ws://robot.test/**',socket=>{
    if(!socket.url().endsWith('/manual/ws'))return;
    sockets.add(socket);socket.onClose(()=>sockets.delete(socket));socket.onMessage(raw=>{const data=JSON.parse(raw);if(data.type!=='trace')commands.push(data);});
  });
  await context.route('http://robot.test/**',async route=>{
    const request=route.request(),url=new URL(request.url()),json=(data,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(data)});
    if(url.pathname.startsWith('/api/v1/manual/')){writes.push(url.pathname);return route.abort();}
    if(url.pathname==='/api/v1/robot/description')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
    if(url.pathname==='/api/v1/planning/context')return json({model_id:'m',configuration_revision:'initial',settings_revision:settings.revision,
      ignore_missing_drivers:settings.ignore_missing_drivers,planner_available:true,tcp_jog_segmented:true,tcp_step_available:true,available:state.axes.map(a=>a.available),
      preview_seed_degrees:state.axes.map(a=>a.position_degrees),seed_sources:state.axes.map(a=>a.available?'measured':'last'),
      profiles:Array.from({length:6},()=>({configured:false,lower:-3600,upper:3600,velocity:180}))});
    if(url.pathname==='/api/v1/planning/tcp-jog'){
      assert(!cancelling,'TCP musi czekać na anulowanie PTP/LIN');
      const data=request.postDataJSON();requests.push(data);events.push('tcp');
      const q=[...data.seed_degrees];q[['x','y','z','rx','ry','rz'].indexOf(data.axis)]+=.1*data.direction;
      if(delayed)await delay(600);
      return fail?json({detail:'Brak rozwiązania IK'},422):json({...data,positions_degrees:q,tcp:[0,0,0,0,0,0],simulation_only:true,execution_ready:false});
    }
    if(url.pathname==='/api/v1/planning/tcp-step'){
      assert(!cancelling);const data=request.postDataJSON();stepRequests.push(data);
      const id='step'+stepRequests.length,q=data.seed_degrees.map(v=>v*Math.PI/180),target=[...q];
      target[['x','y','z','rx','ry','rz'].indexOf(data.axis)]+=data.step*data.direction*Math.PI/180;
      plans.set(id,{id,model_id:'m',configuration_revision:'initial',settings_revision:data.settings_revision,simulation_only:true,execution_ready:false,
        motion_type:'LIN',start_positions:q,trajectory:{joint_names:Array.from({length:6},(_,i)=>`joint_${i+1}`),
        points:[q,target].map((positions,i)=>({positions,velocities:Array(6).fill(0),accelerations:Array(6).fill(0),time_from_start:{sec:Math.floor(i*stepDurationMs/1000),nanosec:(i*stepDurationMs%1000)*1000000}}))}});
      jobs.set(id,{status:'succeeded',progress:1,phase:'LIN',plan_id:id});if(delayed)await delay(600);return json({id,status:'running'});
    }
    if(url.pathname==='/api/v1/planning/moves'){
      assert(!cancelling);const data=request.postDataJSON(),id=String(jobs.size+1);events.push('move');
      const q=data.seed_degrees.map(v=>v*Math.PI/180),target=data.target_degrees.map(v=>v*Math.PI/180);
      plans.set(id,{id,model_id:'m',configuration_revision:'initial',settings_revision:data.settings_revision,simulation_only:true,execution_ready:false,
        motion_type:data.motion_type,locked:Array(6).fill(false),start_positions:q,trajectory:{joint_names:Array.from({length:6},(_,i)=>`joint_${i+1}`),
        points:[q,target].map((positions,i)=>({positions,velocities:Array(6).fill(0),accelerations:Array(6).fill(0),time_from_start:{sec:0,nanosec:i*600000000}}))}});
      jobs.set(id,{status:slow?'running':'succeeded',progress:slow?.2:1,phase:'IK',plan_id:id});return json({id,status:'running'});
    }
    if(url.pathname.startsWith('/api/v1/planning/jobs/')){
      const id=url.pathname.split('/').at(-1);
      if(request.method()==='DELETE'){cancelling=true;events.push('cancel');await delay(120);cancelling=false;events.push('cancelled');jobs.set(id,{status:'cancelled'});}
      return json(jobs.get(id));
    }
    if(url.pathname.startsWith('/api/v1/planning/plans/'))return json(plans.get(url.pathname.split('/').at(-1)));
    if(url.pathname.startsWith('/api/'))return json({detail:'Brak atrapy'},503);
    const file=path.resolve(dist,url.pathname==='/'?'index.html':url.pathname.slice(1));assert(file.startsWith(dist+path.sep));
    return route.fulfill({path:file,contentType:file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.svg')?'image/svg+xml':'text/html'});
  });
  try {
    const page=await context.newPage();await page.goto('http://robot.test/#manual');
    const tcp=page.getByRole('region',{name:'Sterowanie TCP',exact:true}),preview=page.getByRole('region',{name:'Wizualizacja robota',exact:true});
    const button=(axis='X',sign='plus')=>tcp.getByRole('button',{name:`TCP ${axis} ${sign}`,exact:true});
    const target=()=>page.locator('.robot-preview canvas').getAttribute('data-planned-joint-angles');
    const hold=async(axis='X',sign='plus')=>{await button(axis,sign).hover();await page.mouse.down();};
    const waitRequests=async count=>{for(let i=0;i<100&&requests.length<count;i++)await delay(30);assert(requests.length>=count,'Brak kroku TCP');};
    await preview.locator('canvas').waitFor();assert(await button().isDisabled());
    assert((await tcp.textContent()).includes('J3'));
    for(const axis of state.axes){axis.available=axis.position_known=axis.can_jog=true;}publish();await delay(100);
    assert(await button().isDisabled());assert((await tcp.textContent()).includes('6/6'));
    for(const axis of state.axes.slice(2)){axis.available=axis.position_known=axis.can_jog=false;}publish();await delay(100);
    await preview.getByRole('button',{name:'Planowanie',exact:true}).click();assert(await button().isDisabled());
    assert(await page.getByRole('button',{name:'JOG J1 prawo',exact:true}).isEnabled());
    settings={revision:'s1',ignore_missing_drivers:true};await button().waitFor();
    await page.waitForFunction(()=>!document.querySelector('[aria-label="TCP X plus"]').disabled);
    const actual=await page.locator('.robot-preview canvas').getAttribute('data-joint-angles');
    await hold();await waitRequests(2);assert(await page.getByRole('button',{name:'JOG J1 prawo',exact:true}).isDisabled());
    await page.mouse.up();await delay(100);const retained=await target();await delay(160);assert.equal(await target(),retained);
    assert.equal(await page.locator('.robot-preview canvas').getAttribute('data-joint-angles'),actual);
    state.axes[0].position_degrees=-123;publish();await delay(100);assert.equal(await target(),retained);
    await tcp.getByLabel('Układ TCP',{exact:true}).selectOption('tool');
    const beforeTool=requests.length;await hold('RZ','minus');await waitRequests(beforeTool+1);await page.mouse.up();await delay(100);
    assert.equal(requests.at(-1).frame,'tool');assert.equal(requests.at(-1).axis,'rz');assert.equal(requests.at(-1).velocity,5);
    await tcp.getByLabel('Prędkość przesuwu TCP',{exact:true}).fill('100');
    const fastBefore=requests.length;await hold('X','plus');await waitRequests(fastBefore+1);await page.mouse.up();await delay(100);
    assert.equal(requests.at(-1).velocity,100);assert.equal(requests[fastBefore].duration_s,.05,'100 mm/s zadaje 5 mm na 50 ms');
    await tcp.getByLabel('Prędkość przesuwu TCP',{exact:true}).fill('10');
    // Ten sam cel jest dalej edytowany przez JOG, następnie przez TCP.
    const jog=page.getByRole('button',{name:'JOG J1 prawo',exact:true});const beforeJog=await target();
    await jog.hover();await page.mouse.down();await delay(160);assert(await button().isDisabled());await page.mouse.up();
    assert.notEqual(await target(),beforeJog);
    // TCP czeka na potwierdzone anulowanie poprzednich obliczeń.
    slow=true;const ptp=page.getByLabel('Podgląd ruchu PTP/LIN',{exact:true});
    await ptp.getByRole('button',{name:'PTP',exact:true}).click();await ptp.getByText(/PTP · IK/).waitFor();
    const beforeCancel=requests.length;await hold();await waitRequests(beforeCancel+1);
    assert(events.indexOf('cancelled')<events.lastIndexOf('tcp'));slow=false;await page.mouse.up();
    await ptp.getByText(/PTP · 0.60 s/).waitFor();
    // Spóźnione odpowiedzi po puszczeniu, blur i zmianie ustawień nie zmieniają celu.
    await ptp.getByRole('button',{name:'Zatrzymaj podgląd',exact:true}).click();
    assert.equal(await tcp.getByRole('button').count(),12);
    for(const axis of ['X','Y','Z','RX','RY','RZ'])for(const sign of ['plus','minus']){
      const count=requests.length;await hold(axis,sign);await waitRequests(count+1);await page.mouse.up();await delay(70);
      assert.equal(requests.at(-1).axis,axis.toLowerCase());assert.equal(requests.at(-1).direction,sign==='plus'?1:-1);
    }
    delayed=true;let count=requests.length;await hold();await waitRequests(count+1);await page.mouse.up();
    const released=await target();await delay(750);assert.equal(await target(),released);
    count=requests.length;await hold();await waitRequests(count+1);await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await page.mouse.up();
    const blurred=await target();await delay(750);assert.equal(await target(),blurred);
    count=requests.length;await hold();await waitRequests(count+1);await page.evaluate(()=>window.dispatchEvent(new Event('pointercancel')));await page.mouse.up();
    const cancelled=await target();await delay(750);assert.equal(await target(),cancelled);
    count=requests.length;await hold();await waitRequests(count+1);for(const socket of [...sockets])socket.close();
    await page.getByText('Brak połączenia',{exact:true}).waitFor();await page.mouse.up();const disconnected=await target();
    await page.waitForFunction(()=>!document.querySelector('[aria-label="TCP X plus"]').disabled);await delay(150);
    assert.equal(await target(),disconnected);assert.equal(requests.length,count+1,'Powrót połączenia nie wznawia TCP');
    count=requests.length;await hold();await waitRequests(count+1);settings={revision:'s2',ignore_missing_drivers:false};
    await page.waitForFunction(()=>document.querySelector('[aria-label="TCP X plus"]').disabled);await page.mouse.up();
    const invalidated=await target();await delay(750);assert.equal(await target(),invalidated);
    delayed=false;settings={revision:'s3',ignore_missing_drivers:true};
    await page.waitForFunction(()=>!document.querySelector('[aria-label="TCP X plus"]').disabled);
    fail=true;count=requests.length;await hold();await waitRequests(count+1);await page.getByRole('log').getByText('Brak rozwiązania IK',{exact:true}).waitFor();await page.mouse.up();
    const rejected=await target();await delay(150);assert.equal(await target(),rejected);fail=false;
    // Kokpit: tabela obok modelu, TCP pod tabelą; zachowane wszystkie statusy.
    for(const size of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]){
      await page.setViewportSize(size);
      const table=await page.locator('.manual-table').boundingBox(),panel=await tcp.boundingBox(),model=await preview.boundingBox();
      await page.screenshot({path:`/tmp/robot-tcp-${size.width}.png`,fullPage:true});
      assert.equal(await page.locator('.manual-table th').count(),6);assert(table.x>=model.x+model.width);assert(panel.y>=table.y+table.height);
      assert(panel.y+panel.height<=size.height);
      for(const direction of await tcp.locator('.tcp-direction').all()){const box=await direction.boundingBox();assert.equal(box.width,44);assert.equal(box.height,44);}
      const overflow=await page.locator('.manual-table td,.manual-table th').evaluateAll(cells=>cells.filter(c=>c.scrollWidth>c.clientWidth+1).map(c=>({text:c.innerText,width:c.clientWidth,scroll:c.scrollWidth})));
      assert.deepEqual(overflow,[],JSON.stringify(overflow));assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
      assert((await page.locator('.manual-table tbody tr').first().boundingBox()).height<=50);
      assert.equal(await page.locator('.position-planned-thumb').count(),6);
      const geometry=()=>page.locator('.manual-table th,.position-track').evaluateAll(elements=>elements.map(e=>{const r=e.getBoundingClientRect();return [r.x,r.width];}));
      const before=await geometry();await preview.getByRole('button',{name:'Zakończ planowanie',exact:true}).click();assert.deepEqual(await geometry(),before);assert(await button().isDisabled());
      await preview.getByRole('button',{name:'Planowanie',exact:true}).click();assert.deepEqual(await geometry(),before);
      await page.screenshot({path:`/tmp/robot-tcp-${size.width}.png`,fullPage:true});
    }
    // Geometria desktop została sprawdzona powyżej. Interakcje wykonujemy w bazowym
    // oknie testu, aby programowy WebGL nie wprowadzał przerw zegara watchdog.
    await page.setViewportSize({width:1366,height:768});await delay(200);
    await tcp.getByLabel('Tryb TCP',{exact:true}).selectOption('step');
    const slider=tcp.getByRole('slider',{name:'Wielkość impulsu TCP',exact:true}),stop=tcp.getByRole('button',{name:'Zatrzymaj impuls',exact:true});
    assert.equal(await slider.getAttribute('max'),'4');assert.equal(await slider.getAttribute('step'),'1');
    for(const axis of ['X','RZ'])for(const [index,amount] of [.1,1,5,10,20].entries()){
      await slider.focus();await slider.press('Home');for(let i=0;i<index;i++)await slider.press('ArrowRight');
      assert.equal(await slider.getAttribute('aria-valuetext'),`${amount} mm / ${amount}°`);
      const before=JSON.parse(await target()),count=stepRequests.length;
      await button(axis).click();await page.waitForFunction(()=>!document.querySelector('.tcp-step-stop').disabled);
      assert(await page.getByRole('button',{name:'JOG J1 prawo',exact:true}).isDisabled());
      await page.waitForFunction(()=>document.querySelector('.tcp-step-stop').disabled);
      // Oczekujemy publikacji końcowej pozy w scenie, zamiast zakładać czas renderu WebGL.
      const joint=axis==='X'?0:5;
      await page.waitForFunction(({joint,expected})=>{
        const angles=JSON.parse(document.querySelector('.robot-preview canvas').dataset.plannedJointAngles);
        return Math.abs(angles[joint]-expected)<1e-12;
      },{joint,expected:before[joint]+amount*Math.PI/180},{timeout:3000});
      const after=JSON.parse(await target());
      assert.equal(stepRequests.length,count+1);assert.equal(stepRequests.at(-1).step,amount);assert.equal(stepRequests.at(-1).axis,axis.toLowerCase());
      assert(Math.abs(after[joint]-before[joint]-amount*Math.PI/180)<1e-6,`Jeden klik dochodzi do celu po puszczeniu: ${axis}, ${amount}`);
      await delay(120);assert.equal(await target(),JSON.stringify(after),'Brak zapętlenia impulsu');
    }
    const heldStepCount=stepRequests.length;await hold();await delay(1100);await page.mouse.up();
    assert.equal(stepRequests.length,heldStepCount+1,'Przytrzymanie nie powtarza impulsu');
    // Dłuższa trajektoria atrapy pozwala kliknąć STOP przed końcem także w programowym WebGL.
    stepDurationMs=2000;
    await button().click();await page.waitForFunction(()=>!document.querySelector('.tcp-step-stop').disabled);
    await delay(150);await stop.click();await delay(80);const stopped=await target();await delay(250);assert.equal(await target(),stopped);
    delayed=true;const late=stepRequests.length;await button().click();
    for(let i=0;i<100&&stepRequests.length===late;i++)await delay(20);
    await stop.click();const beforeLate=await target();await delay(850);assert.equal(await target(),beforeLate,'Stop odrzuca spóźnione zadanie');delayed=false;
    for(const size of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]){
      await page.setViewportSize(size);assert((await page.locator('.manual-screen').boundingBox()).height<=size.height,'Tryb impulsowy mieści się na desktop');
      assert(await page.locator('.manual-controls-row').evaluate(el=>el.scrollHeight<=el.clientHeight+1),'Impuls nie wymaga przewijania sterowania');
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
      await page.screenshot({path:`/tmp/robot-tcp-step-${size.width}.png`,fullPage:true});
    }
    assert.deepEqual(commands,[]);assert.deepEqual(writes,[]);assert.deepEqual(errors,[]);
    console.log(JSON.stringify({result:'OK',checks:['TCP 12 kierunków','baza/narzędzie','wspólny cel JOG/TCP','anulowanie PTP/LIN','release/blur/rewizja','błąd IK','1280/1366/1920','blokada fizyczna 2/6 i 6/6','brak komend sprzętowych']}));
  } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
