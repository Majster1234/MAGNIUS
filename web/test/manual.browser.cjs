// Test przeglądarki na atrapach HTTP/WS. Żadne żądanie nie dociera do CAN.
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');
const assert = require('assert/strict');
const dist = path.resolve(__dirname, '../dist');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
(async () => {
  const browser = await chromium.launch({headless: true});
  const context = await browser.newContext({viewport: {width: 1600, height: 1000}});
  const commands = [], traces = [], errors = [], sockets = new Set();
  const state = {fresh:true, busy:false, fault:false, recording:true, state_monotonic_ns:'123456', active_axis:0, max_rpm:30, operation:'idle', message:'Stan testowy',
    axes: Array.from({length:6}, (_,i) => ({axis:i+1, available:[1,2,4].includes(i+1), prepared:false,
      enable_known:[1,2,4].includes(i+1), enabled:false, position_known:[1,2,4].includes(i+1), position_degrees:123.4, speed_known:true, speed_rpm:0,
      can_jog:false, can_zero:false, motion:'zatrzymany'}))};
  let owner = null, pauseStates = false;
  function update() {
    for (const a of state.axes) {a.can_jog = a.available && a.prepared && a.enabled && !state.busy; a.can_zero = a.available && a.prepared && !state.busy;}
    if (!pauseStates) for (const socket of sockets) socket.send(JSON.stringify({type:'state', state}));
  }
  const timer = setInterval(update, 50);
  context.on('page', page => page.on('pageerror', error => errors.push(error.message)));
  await context.route('http://robot.test/**', async route => {
    const url = new URL(route.request().url());
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
    const row = axis => page.locator('tbody tr').nth(axis-1);
    assert.equal(await page.locator('tbody tr').count(),6);
    const layouts = [];
    for (const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080},{width:1600,height:1000}]) {
      await page.setViewportSize(viewport);
      const panel = await page.locator('.manual-screen').boundingBox();
      const header = await page.locator('.manual-header').boundingBox();
      const title = await page.getByRole('heading',{name:'Sterowanie ręczne',exact:true}).boundingBox();
      const recording = await page.getByText('Nagrywanie diagnostyki').boundingBox();
      const background = await page.evaluate(()=>getComputedStyle(document.documentElement).backgroundColor);
      const channels = background.match(/\d+/g).slice(0,3).map(Number);
      assert(channels.every(c=>c<45), 'Tło panelu ma być ciemne');
      assert(panel.height <= 290, 'Sześć osi z nagłówkiem ma zajmować około 1/3 wysokości ekranu desktop');
      assert((await row(1).boundingBox()).height <= 30, 'Wiersze nie powinny przekraczać 30 px');
      assert(header.y <= 1 && title.y < header.y+header.height && recording.y < header.y+header.height, 'Tytuł i statusy muszą być na samej górze');
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth), 'Panel musi mieścić się w szerokości ekranu');
      layouts.push({...viewport,panel_height:panel.height,row_height:(await row(1).boundingBox()).height});
    }
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
    await page.mouse.up(); const beforeRecovery = commands.filter(c=>c.action==='start').length;
    pauseStates=false; update(); await delay(200);
    assert.equal(commands.filter(c=>c.action==='start').length, beforeRecovery);
    state.axes[1].position_degrees=3000; state.axes[3].position_degrees=-3000; update(); await delay(70);
    await page.screenshot({path: process.env.ROBOT_SCREENSHOT || '/tmp/robot-manual-desktop.png',fullPage:true});
    assert.deepEqual(errors, []);
    assert(traces.some(t=>t.event==='client_stop'));
    assert(traces.some(t=>t.event==='state_received'&&t.state_monotonic_ns==='123456'));
    console.log(JSON.stringify({result:'OK',axes:6,clients:2,commands:commands.length,layouts,checks:['dark theme','compact panel','top title and statuses','position meter','ENABLE readback','missing axes','zero','hold/release','exclusive jog','blur','250 ms state jitter','500 ms stale stop','no auto restart']}));
  } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
