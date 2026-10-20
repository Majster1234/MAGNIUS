// Każde HTTP/WS przechwycone lokalnie: brak połączenia z ROS i CAN.
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');
const assert = require('assert/strict');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
const dist = path.resolve(__dirname, '../dist');

(async () => {
  const browser = await chromium.launch({ headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const context = await browser.newContext({ viewport: { width: 1366, height: 768 } });
  const sockets = new Set(), commands = [], writes = [], errors = [];
  const state = { fresh: true, busy: false, fault: false, active_axis: 0, max_rpm: 30, operation: 'idle', message: 'Plan testowy',
    axes: Array.from({ length: 6 }, (_, i) => ({ axis: i + 1, available: [1,2,4].includes(i + 1), prepared: false,
      enable_known: true, enabled: false, position_known: [1,2,4].includes(i + 1), position_degrees: i * 10,
      position_age_seconds: 0, speed_known: true, speed_rpm: 0, can_jog: false, can_zero: false, motion: 'zatrzymany' })) };
  let paused = false, modelRevision = '';
  const publish = () => { if (!paused) for (const socket of sockets) socket.send(JSON.stringify({ type: 'state', state })); };
  const timer = setInterval(publish, 50);
  context.on('page', page => page.on('pageerror', error => errors.push(error.message)));
  await context.route('http://robot.test/**', async route => {
    const url = new URL(route.request().url());
    if (url.pathname.startsWith('/api/v1/planning/')) return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Planner wyłączony w teście lokalnego JOG"}'});
    if (url.pathname.startsWith('/api/v1/manual/axes/')) {
      writes.push(url.pathname);
      return route.fulfill({ status: 202, contentType: 'application/json', body: '{"accepted":true}' });
    }
    if (url.pathname === '/api/v1/robot/description') {
      let xml = fs.readFileSync(path.resolve(__dirname, '../../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'), 'utf8');
      if (modelRevision) xml = xml.replace('<robot ', `<robot configuration_revision="${modelRevision}" `);
      return route.fulfill({ body: xml, contentType: 'application/xml' });
    }
    const name = url.pathname === '/' ? 'index.html' : url.pathname.slice(1), file = path.resolve(dist, name);
    assert(file.startsWith(dist + path.sep));
    return route.fulfill({ path: file, contentType: name.endsWith('.js') ? 'application/javascript' : name.endsWith('.css') ? 'text/css' : name.endsWith('.svg') ? 'image/svg+xml' : 'text/html' });
  });
  await context.routeWebSocket('ws://robot.test/**', socket => {
    if (!socket.url().endsWith('/api/v1/manual/ws')) return;
    sockets.add(socket); publish();
    socket.onClose(() => sockets.delete(socket));
    socket.onMessage(raw => {
      const command = JSON.parse(raw); if (command.type === 'trace') return;
      commands.push(command); socket.send(JSON.stringify({ type: 'ack', sequence: command.sequence, accepted: true }));
    });
  });
  try {
    const page = await context.newPage(); await page.goto('http://robot.test/#manual');
    const preview = page.getByRole('region', { name: 'Wizualizacja robota', exact: true });
    const canvas = preview.locator('canvas'), start = preview.getByRole('button', { name: 'Planowanie', exact: true });
    const row = id => page.locator('tbody tr').nth(id - 1);
    const arrow = (id, direction = 'prawo') => page.getByRole('button', { name: `JOG J${id} ${direction}`, exact: true });
    const planned = async id => Number((await row(id).locator('.planned-position').textContent()).match(/-?[\d.]+/)[0]);
    const hold = async id => { const box = await arrow(id).boundingBox(); await page.mouse.move(box.x + 12, box.y + 12); await page.mouse.down(); };
    const sync = preview.getByRole('button', { name: 'Od pozycji robota', exact: true });
    const geometry = async () => page.locator('.manual-table').evaluate(el => {
      const rect = node => {const r=node.getBoundingClientRect();return [r.x,r.y,r.width,r.height].map(x=>Math.round(x*100)/100);};
      return {headers:[...el.querySelectorAll('thead th')].map(rect),tracks:[...el.querySelectorAll('.position-track')].map(rect)};
    });
    await canvas.waitFor(); await start.waitFor();
    assert.equal(await page.locator('.manual-table thead th').first().textContent(),'JOG');
    assert.equal(await row(1).locator('td').first().locator('.jog-arrow').count(),2);
    assert.equal(await row(1).locator('.jog-axis-id').count(),0,'Numer osi nie jest powtarzany między strzałkami');
    const manualGeometry=await geometry();
    state.busy = true; publish(); await delay(70); assert(await start.isDisabled());
    state.busy = false; publish(); await start.click();
    await page.waitForFunction(() => !!document.querySelector('.preview-canvas canvas')?.dataset.plannedJointAngles);
    assert.deepEqual(await geometry(),manualGeometry,'Kolumny i skale pozostają w miejscu po włączeniu planowania');
    assert.equal(await row(1).locator('.position-planned-thumb').count(),1);
    assert.equal(await row(3).locator('.position-planned-thumb').count(),0,'Nieznana pozycja bliźniaka nie otrzymuje wymyślonego znacznika');
    assert.equal(await canvas.getAttribute('data-planning-opacity'), '.3');
    assert(await row(1).getByRole('button', { name: 'Załącz', exact: true }).isDisabled());
    assert(await row(1).getByRole('button', { name: 'Wyłącz', exact: true }).isDisabled());
    assert(await row(1).getByRole('button', { name: 'Ustaw zero', exact: true }).isDisabled());
    assert(await arrow(3).isDisabled()); assert(await arrow(1).isEnabled());
    const physicalBefore = JSON.parse(await canvas.getAttribute('data-joint-angles'));
    const imageBefore = await canvas.screenshot();
    await hold(1); await delay(90); assert(await arrow(2).isDisabled());assert(await row(1).evaluate(el=>el.classList.contains('manual-planned-active')));await delay(90); await page.mouse.up();
    assert((await planned(1)) > 5); await delay(70);
    assert.deepEqual(JSON.parse(await canvas.getAttribute('data-joint-angles')), physicalBefore);
    assert(!imageBefore.equals(await canvas.screenshot()), 'Ruch bliźniaka zmienia obraz WebGL');
    const retained = await planned(1);
    const retainedMarker=await row(1).locator('.position-planned-thumb').getAttribute('data-position');
    assert(Number(retainedMarker)>0,'Niebieski znacznik śledzi JOG bliźniaka');
    state.axes[0].position_degrees = -50; publish(); await delay(100);
    assert.equal(await planned(1), retained, 'Kolejne enkodery nie nadpisują planu');
    assert.equal(await row(1).locator('.position-planned-thumb').getAttribute('data-position'),retainedMarker);
    assert(Math.abs(JSON.parse(await canvas.getAttribute('data-joint-angles'))[0] + 50 * Math.PI / 180) < .001);
    await sync.click(); assert.equal(await planned(1), -50);
    await hold(1); await delay(60); await page.evaluate(() => window.dispatchEvent(new Event('blur'))); await page.mouse.up();
    const blurred = await planned(1); await delay(100); assert.equal(await planned(1), blurred);
    state.axes[0].position_age_seconds = .25; publish(); await delay(70); assert(await arrow(1).isDisabled()); assert(await arrow(2).isEnabled());
    state.axes[0].position_age_seconds = 0; publish(); await delay(70);
    await hold(1); await delay(60); paused = true; await delay(300); await page.mouse.up();
    const frozen = await planned(1); assert(await arrow(1).isDisabled()); await delay(250); assert.equal(await planned(1), frozen);
    paused = false; publish(); await delay(80); assert.equal(await planned(1), frozen);
    state.axes[2].available = true; state.axes[2].position_known = true; state.axes[2].position_degrees = 45; publish();
    await row(3).getByText('Plan: 45.0°', { exact: true }).waitFor(); assert(await arrow(3).isEnabled());
    await hold(3); await delay(60); state.axes[2].available = false; publish(); await delay(70); await page.mouse.up();
    const disconnected = await planned(3); assert(await arrow(3).isDisabled()); await delay(80); assert.equal(await planned(3), disconnected);
    modelRevision = 'mismatch'; state.configuration_revision = 'new-profile'; publish();
    await preview.getByText('Plan wymaga ponownej synchronizacji', { exact: true }).waitFor(); assert(await arrow(2).isDisabled());
    await page.getByRole('log').getByText('Oczekiwanie na model zgodny z rewizją sterowania ROS…', { exact: true }).waitFor();
    assert(await sync.isDisabled());
    modelRevision = 'new-profile';
    await page.waitForFunction(() => !Array.from(document.querySelectorAll('button')).find(b => b.textContent === 'Od pozycji robota')?.disabled);
    assert(await arrow(2).isDisabled()); await sync.click(); assert(await arrow(2).isEnabled());
    const localPos = await planned(2); const socket = [...sockets][0]; socket.close();
    await page.getByText('Brak połączenia', { exact: true }).waitFor(); assert(await arrow(2).isDisabled());
    await page.getByText('Połączono · 20 Hz', { exact: true }).waitFor(); assert.equal(await planned(2), localPos);
    await page.screenshot({ path: '/tmp/robot-planning-desktop.png', fullPage: true });
    for (const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]) {
      await page.setViewportSize(viewport);
      const plannedGeometry=await geometry();
      await preview.getByRole('button',{name:'Zakończ planowanie',exact:true}).click();
      assert.deepEqual(await geometry(),plannedGeometry,'Wyłączenie planowania nie przesuwa tabeli');
      assert.equal(await page.locator('.position-planned-thumb').count(),0);
      await start.click();await row(1).locator('.position-planned-thumb').waitFor();
      assert.deepEqual(await geometry(),plannedGeometry,'Ponowne planowanie zachowuje szerokości kolumn');
      assert((await preview.boundingBox()).x<(await page.locator('.manual-table').boundingBox()).x);
      assert(await page.locator('.manual-screen').evaluate(el=>el.classList.contains('mode-planning')));
      assert.equal(await page.locator('.manual-mode').textContent(),'PLANOWANIE · bliźniak');
      assert((await row(1).boundingBox()).height <= 50, 'Wiersz pozostaje zwarty także w planowaniu');
      const overflow=await page.locator('.manual-table td').evaluateAll(cells=>cells.filter(cell=>cell.scrollWidth>cell.clientWidth+1).map(cell=>({text:cell.innerText,width:cell.clientWidth,scroll:cell.scrollWidth})));
      assert.deepEqual(overflow,[], 'Zawartość mieści się w stałych kolumnach: '+JSON.stringify(overflow));
      assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
    }
    assert.deepEqual(commands, [], 'Planowanie nie wysłało start/keepalive/stop'); assert.deepEqual(writes, [], 'Planowanie nie wysłało ENABLE/zero');
    await preview.getByRole('button', { name: 'Zakończ planowanie', exact: true }).click();
    assert.equal(await page.locator('.planned-position').count(), 0); assert(await arrow(1).isDisabled());
    // Wyjście nie uruchamia napędów. Dopiero nowy gest po gotowości sprzętu trafia do API.
    state.axes[0].can_jog = true; state.axes[0].enabled = true; publish(); await delay(70);
    assert.deepEqual(commands, []); await hold(1); await delay(60); await page.mouse.up();
    assert(commands.some(command => command.action === 'start')); assert(commands.some(command => command.action === 'stop'));
    assert.deepEqual(errors, []);
    console.log(JSON.stringify({ result: 'OK', checks: ['dwa modele WebGL', 'krycie 30%', 'JOG bez ENABLE', 'brak komend sprzętowych', 'niezależność od enkoderów', 'synchronizacja', 'jedna oś', 'blur', 'wiek 250 ms', 'luka danych', 'utrata i powrót sterownika', 'rewizja i zgodny URDF', 'rozłączenie WS', 'zwarty desktop', 'powrót do fizycznego JOG'] }));
  } finally { clearInterval(timer); await context.close(); await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
