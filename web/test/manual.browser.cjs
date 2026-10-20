// Test przeglądarki na atrapach HTTP/WS. Żadne żądanie nie dociera do CAN.
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');
const assert = require('assert/strict');
const dist = path.resolve(__dirname, '../dist');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
  const browser = await chromium.launch({headless: true, args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const context = await browser.newContext({viewport: {width: 1600, height: 1000}});
  const commands = [], traces = [], errors = [], sockets = new Set();
  const state = {fresh:true, busy:false, fault:false, recording:true, state_monotonic_ns:'123456', active_axis:0, max_rpm:30, operation:'idle', message:'Stan testowy',
    axes: Array.from({length:6}, (_,i) => ({axis:i+1, available:[1,2,4].includes(i+1), prepared:false,
      enable_known:[1,2,4].includes(i+1), enabled:false, position_known:[1,2,4].includes(i+1), position_degrees:123.4, position_age_seconds:0, speed_known:true, speed_rpm:0,
      can_jog:false, can_zero:false, motion:'zatrzymany'}))};
  let owner = null, pauseStates = false, modelRequests = 0, descriptionRevision = '';
  function update() {
    for (const a of state.axes) {a.can_jog = a.available && a.prepared && a.enabled && !state.busy; a.can_zero = a.available && a.prepared && !state.busy;}
    if (!pauseStates) for (const socket of sockets) socket.send(JSON.stringify({type:'state', state}));
  }
  const timer = setInterval(update, 50);
  context.on('page', page => page.on('pageerror', error => errors.push(error.message)));
  await context.route('http://robot.test/**', async route => {
    const url = new URL(route.request().url());
    if (url.pathname.startsWith('/api/v1/planning/')) return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Planner wyłączony w teście fizycznego JOG"}'});
    if (url.pathname === '/api/v1/robot/description') {
      modelRequests++;
      if (modelRequests === 1) return route.fulfill({status:503,body:'Oczekiwanie na ROS'});
      const xml=fs.readFileSync(path.resolve(__dirname,'../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8');
      return route.fulfill({body:descriptionRevision?xml.replace('<robot ',`<robot configuration_revision="${descriptionRevision}" `):xml,contentType:'application/xml'});
    }
    if (url.pathname.startsWith('/api/v1/manual/axes/')) {
      const axis = Number(url.pathname.split('/')[5]);
      const a = state.axes[axis-1];
      if (url.pathname.endsWith('/enable')) {
        const enabled = route.request().postDataJSON().enabled;
        setTimeout(() => {a.enabled = enabled; a.prepared = enabled; update();}, 130);
      } else {a.position_degrees = 0;}
      return route.fulfill({status:202,contentType:'application/json',body:JSON.stringify({accepted:true})});
    }
    const name = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
    const file = path.resolve(dist, name);
    assert(file.startsWith(dist + path.sep));
    return route.fulfill({path:file, contentType: name.endsWith('.js') ? 'application/javascript' : name.endsWith('.css') ? 'text/css' : name.endsWith('.svg') ? 'image/svg+xml' : 'text/html'});
  });
  await context.routeWebSocket('ws://robot.test/**', socket => {
    if (!socket.url().endsWith('/api/v1/manual/ws')) return;
    sockets.add(socket); update();
    socket.onClose(() => sockets.delete(socket));
    socket.onMessage(raw => {
      const cmd = JSON.parse(raw);
      if (cmd.type === 'trace') {traces.push(cmd);return;}
      commands.push(cmd);
      let accepted = true;
      if (cmd.action === 'start') {
        accepted = !owner && state.axes[cmd.axis-1].enabled;
        if (accepted) {owner=socket; state.busy=true; state.active_axis=cmd.axis; state.operation='jog'; state.axes[cmd.axis-1].motion='rozpędza się';}
      }
      if (cmd.action === 'keepalive') accepted = owner === socket;
      if (cmd.action === 'stop' && owner === socket) {
        state.operation='stopping'; state.axes[cmd.axis-1].motion='hamuje';
        setTimeout(() => {owner=null;state.busy=false;state.active_axis=0;state.operation='idle';state.axes[cmd.axis-1].motion='zatrzymany';update();},150);
      }
      socket.send(JSON.stringify({type:'ack',sequence:cmd.sequence,accepted,message:accepted?'Przyjęte':'Zajęty'}));update();
    });
  });
  try {
    const page = await context.newPage(); await page.goto('http://robot.test/#manual');
    await page.getByText('Połączono · 20 Hz').waitFor();
    const preview=page.getByRole('region',{name:'Wizualizacja robota',exact:true});
    const canvas=preview.locator('canvas');
    await canvas.waitFor();
    await page.waitForFunction(()=>!!document.querySelector('.preview-canvas canvas')?.dataset.jointAngles);
    assert(modelRequests>=2,'Model ma być ponownie pobrany po 503');
    assert(await canvas.evaluate(el=>!!el.getContext('webgl2')),'Model ma być faktycznie renderowany w WebGL');
    assert(await preview.getByText('Pozycja częściowa — 3/6 osi',{exact:true}).isVisible());
    assert.equal(await preview.locator('.preview-joint-label').count(),7);
    const beforePose=await canvas.screenshot();
    state.axes[0].position_degrees=-450;update();
    await page.waitForFunction(()=>Math.abs(JSON.parse(document.querySelector('.preview-canvas canvas').dataset.jointAngles)[0]+2.5*Math.PI)<.001);
    assert(!beforePose.equals(await canvas.screenshot()),'Zmiana enkodera ma zmienić obraz modelu');
    assert.equal(await preview.getByRole('button').count(),2,'Pozostają domek i wejście do planowania');
    assert.equal(await preview.locator('.preview-readings').count(),0,'Pozycje nie mogą być zdublowane przy modelu');
    const home=preview.getByRole('button',{name:'Dopasuj widok',exact:true});
    assert.equal(await home.locator('svg').count(),1);
    const beforeCamera=await canvas.screenshot();
    const modelBox=await canvas.boundingBox();
    await page.mouse.move(modelBox.x+modelBox.width/2,modelBox.y+modelBox.height/2);
    await page.mouse.down();await page.mouse.move(modelBox.x+modelBox.width/2+110,modelBox.y+modelBox.height/2+50,{steps:8});await page.mouse.up();
    await delay(80);assert(!beforeCamera.equals(await canvas.screenshot()),'Mysz ma obracać kamerę');
    const orbited=await canvas.screenshot();const commandCount=commands.length;
    await home.click();await delay(80);
    assert(!orbited.equals(await canvas.screenshot()),'Domek ma dopasowywać kamerę');
    assert.equal(commands.length,commandCount,'Dopasowanie kamery nie wysyła komend do silników');
    const row = axis => page.locator('tbody tr').nth(axis-1);
    assert.equal(await page.locator('tbody tr').count(),6);
    // Planowanie jest w pełni lokalne: nie wymaga ENABLE i nie wysyła komend CAN.
    const planning=preview.getByRole('button',{name:'Planowanie',exact:true});
    await planning.click(); await preview.getByText('Planowanie — JOG / TCP',{exact:true}).waitFor();
    await page.waitForFunction(()=>!!document.querySelector('.preview-canvas canvas')?.dataset.plannedJointAngles);
    assert(await row(1).getByRole('button',{name:'Załącz',exact:true}).isDisabled());
    assert(await row(1).getByRole('button',{name:'Ustaw zero'}).isDisabled());
    const planArrow=page.getByRole('button',{name:'JOG J1 prawo'}), beforePlanCommands=commands.length;
    const planBox=await planArrow.boundingBox(); await page.mouse.move(planBox.x+14,planBox.y+14); await page.mouse.down(); await delay(160); await page.mouse.up();
    await page.waitForFunction(()=>document.querySelector('.planned-position')?.textContent !== 'Plan: -450.0°');
    assert.equal(commands.length,beforePlanCommands,'JOG bliźniaka nie używa WebSocketu sterowania');
    assert(await row(3).getByRole('button',{name:'JOG J3 prawo'}).isDisabled(),'Niedostępna oś nie steruje bliźniakiem');
    state.axes[0].position_degrees=-600;update();await delay(80);
    assert.notEqual(await row(1).locator('.planned-position').textContent(),'Plan: -600.0°','Enkoder nie nadpisuje planu');
    await preview.getByRole('button',{name:'Od pozycji robota',exact:true}).click();
    await page.getByText('Plan: -600.0°',{exact:true}).waitFor();
    await preview.getByRole('button',{name:'Zakończ planowanie',exact:true}).click();
    await planning.waitFor();
    const layouts = [];
    for (const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080},{width:2560,height:1440},{width:1600,height:1000}]) {
      await page.setViewportSize(viewport);
      const panel = await page.locator('.manual-screen').boundingBox();
      const header = await page.locator('.manual-header').boundingBox();
      const title = await page.getByRole('heading',{name:'Sterowanie ręczne',exact:true}).boundingBox();
      const recording = await page.getByText('Nagrywanie diagnostyki').boundingBox();
      const background = await page.evaluate(()=>getComputedStyle(document.documentElement).backgroundColor);
      const channels = background.match(/\d+/g).slice(0,3).map(Number);
      assert(channels.every(c=>c<45), 'Tło panelu ma być ciemne');
      await page.screenshot({path:`/tmp/robot-manual-layout-${viewport.width}.png`,fullPage:true});
      assert(panel.height <= viewport.height, `Podgląd i sterowanie mają mieścić się w wysokości desktop: ${JSON.stringify({panel,viewport})}`);
      const previewBox=await preview.boundingBox(), tableBox=await page.locator('.manual-table').boundingBox();
      const consoleBox=await page.getByRole('log',{name:'Konsola zdarzeń'}).boundingBox();
      assert(previewBox.width>=400 && previewBox.width<viewport.width*.51,'Podgląd zajmuje lewą część kokpitu obok stałego menu');
      assert(consoleBox.y>=previewBox.y+previewBox.height && consoleBox.y+consoleBox.height<=viewport.height,'Konsola mieści się pod modelem');
      assert(consoleBox.height<=84,'Konsola ma wysokość do czterech linii');
      assert(tableBox.x>=previewBox.x+previewBox.width && tableBox.y<previewBox.y+2,'Tabela jest po prawej, obok podglądu');
      const tcpBox=await page.locator('.tcp-controls').boundingBox();
      assert(tcpBox.y>=tableBox.y+tableBox.height && tcpBox.y+tcpBox.height<=viewport.height,'Cały panel TCP mieści się pod tabelą');
      assert(await page.locator('.manual-controls-row').evaluate(el=>el.scrollHeight<=el.clientHeight+1),'Kontrolki nie wymagają przewijania');
      assert.equal(Math.round((await canvas.boundingBox()).width),Math.round(previewBox.width-2),'Scena wykorzystuje całą szerokość panelu');
      const homeBox=await home.boundingBox();
      assert(homeBox.x>previewBox.x && homeBox.x+homeBox.width<previewBox.x+previewBox.width && homeBox.y>previewBox.y+34,'Domek ma być na scenie 3D');
      assert((await page.locator('.manual-table').boundingBox()).height <= 330,'Tabela sterowania ma pozostać zwarta');
      const jogBox=await row(1).locator('.jog-arrow').first().boundingBox();
      assert(jogBox.width>=36 && jogBox.height>=36,'JOG ma ergonomiczny obszar naciśnięcia');
      assert((await row(1).boundingBox()).height <= 50, 'Wiersze mieszczą duże przyciski JOG');
      const topbar=await page.locator('.app-topbar').boundingBox();
      assert(topbar.y<=1 && header.y>=topbar.y+topbar.height && title.y<header.y+header.height && recording.y<topbar.y+topbar.height,'Wspólny pasek statusów znajduje się nad tytułem');
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth), 'Panel musi mieścić się w szerokości ekranu');
      layouts.push({...viewport,panel_height:panel.height,row_height:(await row(1).boundingBox()).height});
    }
    const eventConsole=page.getByRole('log',{name:'Konsola zdarzeń'});
    const initialCount=await eventConsole.locator('.console-entry').count();
    await delay(300);
    assert.equal(await eventConsole.locator('.console-entry').count(),initialCount,'Powtarzane statusy nie dopisują logów');
    const infoBackground=await eventConsole.locator('.console-info').last().evaluate(el=>getComputedStyle(el).backgroundColor);
    assert.equal(infoBackground,'rgba(0, 0, 0, 0)','Informacje nie mają tła');
    const warning=eventConsole.locator('.console-warning').last();
    assert.equal(await warning.evaluate(el=>getComputedStyle(el).backgroundColor),'rgba(0, 0, 0, 0)','Ostrzeżenie ma ciemne tło konsoli');
    assert.equal(await warning.evaluate(el=>getComputedStyle(el).borderLeftColor),'rgb(245, 158, 11)','Ostrzeżenie ma bursztynową krawędź');
    state.fault=true;state.message='Testowa awaria komunikacji';update();
    const fault=eventConsole.locator('.console-error').filter({hasText:'Testowa awaria komunikacji'});
    await fault.waitFor();
    assert.equal(await fault.evaluate(el=>getComputedStyle(el).borderLeftColor),'rgb(239, 68, 68)','Awaria ma czerwoną krawędź');
    assert.equal(await fault.locator('.console-level').textContent(),'AWARIA');
    assert.match(await fault.locator('time').textContent(),/\d{2}:\d{2}:\d{2}[.,]\d{3}/,'Zdarzenia mają czas z milisekundami');
    state.fault=false;state.message='Stan testowy';update();
    for(let i=0;i<115;i++){state.message=`Komunikat testowy ${i}`;update();await delay(20);}
    assert.equal(await eventConsole.locator('.console-entry').count(),100,'Historia ma stałe ograniczenie pamięci');
    await delay(50);
    assert(await eventConsole.evaluate(el=>el.scrollHeight-el.clientHeight-el.scrollTop<12),'Nowe wpisy są przewijane do końca');
    await eventConsole.evaluate(el=>{el.scrollTop=0;el.dispatchEvent(new Event('scroll'));});
    state.message='Wpis podczas czytania historii';update();await delay(80);
    assert.equal(await eventConsole.evaluate(el=>el.scrollTop),0,'Czytanie historii nie jest przerywane automatycznym przewijaniem');
    await eventConsole.evaluate(el=>{el.scrollTop=el.scrollHeight;el.dispatchEvent(new Event('scroll'));});
    assert.equal(await row(3).locator('.position-thumb').count(), 0, 'Brak pomiaru nie oznacza pozycji zero');
    for (const [position, percent] of [[-3600,0],[0,50],[3500,98.61111111111111]]) {
      state.axes[0].position_degrees=position; update(); await delay(65);
      const meter = row(1).getByRole('meter');
      assert.equal(Number(await meter.getAttribute('aria-valuenow')), position);
      assert(Math.abs(parseFloat(await meter.locator('.position-thumb').evaluate(el=>el.style.left)) - percent) < .001);
    }
    assert(await row(3).getByRole('button',{name:'Załącz',exact:true}).isDisabled());
    await row(1).getByRole('button',{name:'Załącz',exact:true}).click();
    // Żądanie jest przyjęte, ale ENABLE nie jest jeszcze potwierdzony.
    assert(await row(1).getByText('Wyłączony',{exact:true}).isVisible());
    await row(1).getByText('Włączony',{exact:true}).waitFor();
    await row(2).getByRole('button',{name:'Załącz',exact:true}).click();
    await row(2).getByText('Włączony',{exact:true}).waitFor();
    await row(1).getByRole('button',{name:'Ustaw zero'}).click();
    await row(1).getByText('0.0°',{exact:true}).waitFor();
    const arrow = page.getByRole('button',{name:'JOG J1 prawo'});
    const box = await arrow.boundingBox(); await page.mouse.move(box.x+20,box.y+20); await page.mouse.down();
    await delay(240);
    assert(commands.some(c=>c.action==='keepalive'));
    assert(await arrow.isEnabled(), 'Przytrzymana strzałka nie może zostać wyłączona po aktualizacji busy');
    assert(await page.getByRole('button',{name:'JOG J2 prawo'}).isDisabled());
    // Krótka przerwa publikacji statusów, nadal potwierdzane podtrzymania.
    pauseStates=true; await delay(250);
    assert(await page.getByText('Połączono · 20 Hz').isVisible());
    assert(await row(1).getByText('Połączony',{exact:true}).isVisible());
    assert(!commands.some(c=>c.action==='stop'));
    pauseStates=false; update();
    const another = await context.newPage(); await another.goto('http://robot.test/#manual');
    await another.getByText('Połączono · 20 Hz').waitFor();
    assert(await another.getByRole('button',{name:'JOG J2 prawo'}).isDisabled());
    await page.mouse.up(); await delay(250);
    assert(commands.some(c=>c.action==='stop'));
    assert(await arrow.isEnabled());
    // Utrata fokusu podczas kolejnego przytrzymania.
    await page.mouse.move(box.x+20,box.y+20); await page.mouse.down(); await delay(100);
    const stops = commands.filter(c=>c.action==='stop').length;
    await page.evaluate(()=>window.dispatchEvent(new Event('blur')));
    await delay(100); assert(commands.filter(c=>c.action==='stop').length > stops);
    await page.mouse.up(); await delay(200);
    // Zanik danych zatrzymuje JOG; powrót sam nie wysyła START.
    await page.mouse.move(box.x+20,box.y+20); await page.mouse.down(); await delay(100);
    state.fresh=false; update(); await delay(250); await page.mouse.up();
    const starts = commands.filter(c=>c.action==='start').length;
    state.fresh=true; update(); await delay(250);
    assert.equal(commands.filter(c=>c.action==='start').length, starts);
    // Dłuższy zanik danych zatrzymuje ruch, ale nie udaje zerwania WebSocket.
    await page.mouse.move(box.x+16,box.y+16); await page.mouse.down(); await delay(100);
    const beforeGap = commands.filter(c=>c.action==='stop').length;
    pauseStates=true; await delay(650);
    assert(commands.filter(c=>c.action==='stop').length > beforeGap);
    assert(await page.getByText('Połączono · dane nieaktualne').isVisible());
    assert(await row(1).getByText('Dane nieaktualne',{exact:true}).isVisible());
    assert.equal(await row(1).getByRole('meter').getAttribute('data-stale'), 'true');
    assert(await row(1).getByText('ostatni pomiar',{exact:true}).isVisible());
    assert.equal(await row(1).getByText('Niedostępny',{exact:true}).count(), 0);
    assert(await arrow.isDisabled());
    assert(await preview.getByText('Pozycja częściowa — 0/6 osi',{exact:true}).isVisible());
    const frozenAngles=await canvas.getAttribute('data-joint-angles');await delay(100);
    assert.equal(await canvas.getAttribute('data-joint-angles'),frozenAngles,'Brak danych zamraża model');
    await page.mouse.up(); const beforeRecovery = commands.filter(c=>c.action==='start').length;
    pauseStates=false; update(); await delay(200);
    assert.equal(commands.filter(c=>c.action==='start').length, beforeRecovery);
    state.axes[1].position_degrees=3000; state.axes[3].position_degrees=-3000; update(); await delay(70);
    await page.screenshot({path: process.env.ROBOT_SCREENSHOT || '/tmp/robot-manual-desktop.png',fullPage:true});
    assert.equal(sockets.size,2,'Każdy panel współdzieli swój istniejący strumień pozycji z podglądem');
    // Nowa rewizja nie może wyrenderować opisu modelu z innej transakcji.
    await planning.click(); await preview.getByText('Planowanie — JOG / TCP',{exact:true}).waitFor();
    descriptionRevision='ahead-of-runtime';state.configuration_revision='profile-test';
    Object.assign(state.axes[0],{joint_configured:true,calibration_valid:true,joint_position_known:true,joint_position_degrees:45,joint_lower_degrees:-90,joint_upper_degrees:90,jog_min_velocity_degrees_s:.12,jog_max_velocity_degrees_s:3.6});update();
    await page.getByRole('log').getByText('Plan unieważniony przez zmianę konfiguracji. Skopiuj ponownie pozycję robota.',{exact:true}).waitFor();
    assert(await page.getByRole('button',{name:'JOG J1 prawo'}).isDisabled(),'Zmiana rewizji blokuje JOG bliźniaka do synchronizacji');
    await preview.getByRole('button',{name:'Zakończ planowanie',exact:true}).click(); await planning.waitFor();
    await page.getByRole('log').getByText('Oczekiwanie na model zgodny z rewizją sterowania ROS…',{exact:true}).waitFor();
    assert(await arrow.isEnabled(),'Synchronizacja modelu nie blokuje niezależnego sterowania');
    descriptionRevision='profile-test';
    await page.waitForFunction(()=>{const canvas=document.querySelector('.preview-canvas canvas');return canvas?.dataset.jointAngles&&Math.abs(JSON.parse(canvas.dataset.jointAngles)[0]-Math.PI/4)<.001;});
    assert.equal(Number(await row(1).getByRole('meter').getAttribute('aria-valuenow')),45);
    assert.equal(Number(await row(1).getByRole('meter').getAttribute('aria-valuemin')),-90);
    assert(await row(1).getByText('°/s przeg.',{exact:true}).isVisible());
    const mappedBox=await arrow.boundingBox();await page.mouse.move(mappedBox.x+12,mappedBox.y+12);await page.mouse.down();await delay(130);await page.mouse.up();await delay(250);
    const jointStart=commands.find(command=>command.action==='start'&&command.space==='joint');
    assert(jointStart&&jointStart.rpm===0&&jointStart.velocity_degrees_s<=3.6&&jointStart.revision==='profile-test','JOG przegubu używa jawnych jednostek i rewizji');
    assert(commands.some(command=>command.action==='stop'&&command.space==='joint'));
    // Alarmy używają pozycji przegubu, bez dodatkowych komend ani odpytywania CAN.
    const beforeLimits=commands.length;
    const indicator=side=>preview.locator(`.preview-limit-label[data-axis="1"][data-side="${side}"]`);
    for(const [position,side,level,critical] of [
      [74.9,'upper',null,false],[75,'upper','warning',false],[84.9,'upper','warning',false],
      [85,'upper','danger',false],[89,'upper','danger',true],[90,'upper','danger',true],
      [90.1,'upper','exceeded',true],[-75,'lower','warning',false],[-85,'lower','danger',false],[-91,'lower','exceeded',true],
    ]) {
      state.axes[0].joint_position_degrees=position;update();await delay(100);
      assert.equal(await indicator(side).getAttribute('data-level'),level,`Próg poświaty przy ${position}°`);
      assert.equal(await indicator(side==='lower'?'upper':'lower').getAttribute('data-level'),null,'Świeci tylko odpowiednia granica');
      if(level){
        assert(await indicator(side).isVisible());
        assert.equal(await indicator(side).getAttribute('data-critical'),String(critical));
        assert.equal(Number(await indicator(side).getAttribute('data-limit-degrees')),side==='lower'?-90:90);
        const rgb=(await indicator(side).evaluate(el=>getComputedStyle(el).color)).match(/\d+/g).map(Number);
        assert(level==='warning'?rgb[0]>rgb[2]&&rgb[1]>rgb[2]:rgb[0]>rgb[1]&&rgb[0]>rgb[2],'Kolor ostrzeżenia odpowiada progowi');
      }
    }
    assert.equal(await preview.locator('.preview-limit-label[data-axis="3"][data-level]').count(),0,'Nie ma fikcyjnych ostrzeżeń brakującej osi');
    await delay(150);
    const pulseStart=await canvas.screenshot();let pulseChanged=false;
    for(let i=0;i<3&&!pulseChanged;i++){await delay(180);pulseChanged=!pulseStart.equals(await canvas.screenshot());}
    assert(pulseChanged,'Poświata przekroczenia pulsuje w rzeczywistym WebGL przy nieruchomej osi');
    state.fresh=false;update();await delay(100);
    assert.equal(await indicator('lower').getAttribute('data-level'),'exceeded');
    assert.equal(await indicator('lower').getAttribute('data-fresh'),'false');
    assert((await indicator('lower').textContent()).includes('ostatni pomiar'),'Nieaktualny alarm nie udaje powrotu do bezpiecznego zakresu');
    const staleGlow=await canvas.screenshot();await delay(180);
    assert(staleGlow.equals(await canvas.screenshot()),'Nieaktualna poświata nie animuje zamrożonej sceny');
    state.fresh=true;state.axes[0].joint_position_degrees=0;update();await delay(100);
    assert.equal(await preview.locator('.preview-limit-label[data-axis="1"][data-level]').count(),0,'Po powrocie do środka zakresu poświata znika');
    assert.equal(commands.length,beforeLimits,'Alarmy są wyłącznie wizualne');
    const fallback=await context.newPage();
    await fallback.addInitScript(()=>{HTMLCanvasElement.prototype.getContext=()=>null;});
    await fallback.goto('http://robot.test/#manual');
    await fallback.getByText('Podgląd 3D niedostępny — sprawdź model lub obsługę WebGL.',{exact:true}).waitFor();
    assert(await fallback.getByRole('button',{name:'JOG J1 prawo',exact:true}).isEnabled());
    await fallback.close();
    assert.deepEqual(errors, []);
    assert(traces.some(t=>t.event==='client_stop'));
    assert(traces.some(t=>t.event==='state_received'&&t.state_monotonic_ns==='123456'));
    console.log(JSON.stringify({result:'OK',axes:6,clients:2,commands:commands.length,layouts,checks:['URDF rendered in WebGL','encoder changes image','matching model/configuration revision','joint JOG degrees per second and joint position','limit glow thresholds and side','actual WebGL glow pulse at fixed joint position','frozen stale limit indicator','no control commands from limit glow','mouse camera and home overlay','split cockpit preview without duplicated positions','bounded event console, severity colors, timestamps, deduplication and scroll','503 retry','partial pose and frozen stale pose','no extra position websocket','WebGL fallback preserves JOG','dark theme','compact control table','top title and statuses','position meter','ENABLE readback','missing axes','zero','hold/release','exclusive jog','blur','250 ms state jitter','500 ms stale stop','no auto restart']}));
  } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
