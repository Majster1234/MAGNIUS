// SPRZĘTOWY test długiej sesji teach pendanta (RUCH ROBOTA): prawdziwa strona #pendant w Chromium jak telefon
// (dotyk wielopunktowy przez CDP: zezwolenie + drążek, spowolnienie CPU), obserwator stanu przez /api/v1/manual/ws.
// Ruchy TCP tam i z powrotem (X/Y/Z/RX/RY/RZ, trzy prędkości), co jakiś czas powrót do pozy startowej JOG-iem osi.
// Zerowanie MKS i załączenie napędów wykonuje wcześniej scripts/pendant_session.py (ten plik zakłada ENABLE=1).
// Uruchomienie: node test/pendant-session.hw.cjs --minutes 10 --out ../log/pendant-sessions/X [--cpu 4] [--seed 1]
// Opis: docs/teach_pendant.md („Test długiej sesji”).
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');

const arg = (name, fallback) => { const i = process.argv.indexOf(`--${name}`); return i > 0 ? process.argv[i + 1] : fallback; };
const BASE = arg('url', 'http://192.168.1.69:8080');
const MINUTES = Number(arg('minutes', '5'));
const CPU = Number(arg('cpu', '4'));
const OUT = path.resolve(arg('out', path.resolve(__dirname, '../../log/pendant-sessions/' + new Date().toISOString().replace(/[:.]/g, '-'))));
const HOME = JSON.parse(arg('home', '[0,-30,40,0,45,0]'));
let seed = Number(arg('seed', '1'));
const random = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
fs.mkdirSync(OUT, { recursive: true });
const logFile = fs.createWriteStream(path.join(OUT, 'session.jsonl'));
const t0 = Date.now();
const log = (event, fields = {}) => { const row = { t: (Date.now() - t0) / 1000, event, ...fields }; logFile.write(JSON.stringify(row) + '\n'); if (!['state'].includes(event)) console.log(row.t.toFixed(2), event, JSON.stringify(fields).slice(0, 220)); };

// ---------- obserwator stanu (osobne połączenie, nic nie wysyła) ----------
let latest = null, previous = null;
const observer = { states: 0, faults: [], tcpEnds: [] };
function observe() {
  const ws = new WebSocket(BASE.replace('http', 'ws') + '/api/v1/manual/ws');
  ws.onmessage = event => {
    const data = JSON.parse(event.data); if (data.type !== 'state') return;
    const s = data.state; observer.states++;
    const tcp = s.tcp_jog || {};
    const compact = { fault: s.fault, fresh: s.fresh, recovery: s.recovery_state, message: s.message, tcp_state: tcp.state, tcp_busy: tcp.busy,
      tcp_ready: tcp.ready, tcp_message: tcp.message, generation: tcp.generation, enabled: s.axes.map(a => a.enabled), q: s.axes.map(a => +(a.joint_position_degrees ?? NaN).toFixed(2)) };
    if (previous) {
      if (compact.fault && !previous.fault) { observer.faults.push({ t: (Date.now() - t0) / 1000, message: s.message }); log('FAULT', { message: s.message }); }
      if (previous.tcp_busy && !compact.tcp_busy) { observer.tcpEnds.push({ t: (Date.now() - t0) / 1000, state: compact.tcp_state, message: compact.tcp_message }); log('tcp_end', { state: compact.tcp_state, message: compact.tcp_message }); }
      if (compact.tcp_state !== previous.tcp_state) log('tcp_state', { state: compact.tcp_state, message: compact.tcp_message });
      if (compact.enabled.some((e, i) => e !== previous.enabled[i])) log('enable', { enabled: compact.enabled });
      if (compact.recovery !== previous.recovery) log('recovery', { state: compact.recovery, message: s.recovery_message });
    }
    previous = compact; latest = compact;
  };
  ws.onclose = () => setTimeout(observe, 500);
}
observe();

(async () => {
  const browser = await chromium.launch({ headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const context = await browser.newContext({ viewport: { width: 844, height: 390 }, deviceScaleFactor: 2, hasTouch: true, isMobile: true });
  const page = await context.newPage();
  page.on('dialog', dialog => { log('dialog', { message: dialog.message() }); void dialog.accept(); });
  page.on('pageerror', error => log('pageerror', { message: error.message }));
  page.on('console', message => { if (message.text().startsWith('[pd]')) log('pendant', { text: message.text().slice(5) }); });
  await page.goto(`${BASE}/#pendant`);
  await page.waitForSelector('.pd-deadman');
  // Komunikaty pendanta (pasek ostrzeżeń) do dziennika.
  await page.evaluate(() => { let last = ''; setInterval(() => { const text = document.querySelector('.pd-warn')?.textContent ?? ''; if (text !== last) { last = text; if (text) console.log('[pd] ' + text); } }, 100); });
  const cdp = await context.newCDPSession(page);
  if (CPU > 1) await cdp.send('Emulation.setCPUThrottlingRate', { rate: CPU });
  log('start', { url: BASE, minutes: MINUTES, cpu: CPU, home: HOME });
  for (let i = 0; i < 100 && !latest; i++) await delay(100);

  // ---------- dotyk ----------
  const touches = new Map();
  const points = () => [...touches.values()].map(p => ({ x: p.x, y: p.y, id: p.id, radiusX: 8, radiusY: 8, force: 1 }));
  const touchDown = async (id, x, y) => { touches.set(id, { id, x, y }); await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: points() }); };
  const touchMove = async (id, x, y) => { Object.assign(touches.get(id), { x, y }); await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: points() }); };
  const touchUp = async id => { const p = touches.get(id); touches.delete(id); await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [{ x: p.x, y: p.y, id: p.id }] }); };
  const box = async selector => { const b = await page.locator(selector).boundingBox(); return { x: b.x + b.width / 2, y: b.y + b.height / 2, r: b.width / 2 }; };
  const DEADMAN = 0, STICK = 1;
  async function deadman(on) {
    if (on && !touches.has(DEADMAN)) { const b = await box('.pd-deadman'); await touchDown(DEADMAN, b.x, b.y); await delay(120); }
    if (!on && touches.has(DEADMAN)) await touchUp(DEADMAN);
  }
  // Wychylenie drążka (dx, dy w [-1, 1], y w górę dodatni) narastające w 80 ms jak kciuk.
  async function stickTo(dx, dy) {
    const b = await box('.pd-stick'), r = b.r - 30;
    if (!touches.has(STICK)) await touchDown(STICK, b.x, b.y);
    const from = touches.get(STICK);
    for (let k = 1; k <= 4; k++) { await touchMove(STICK, from.x + (b.x + dx * r - from.x) * k / 4, from.y + (b.y - dy * r - from.y) * k / 4); await delay(20); }
  }
  async function stickRelease() { if (touches.has(STICK)) { const b = await box('.pd-stick'); await touchMove(STICK, b.x, b.y); await delay(20); await touchUp(STICK); } }
  const click = async (text, scope = '') => { await page.locator(`${scope} button`, { hasText: text }).first().click(); await delay(80); };

  // ---------- odzyskanie po awarii: ponowne ENABLE przyciskiem pendanta ----------
  async function recoverIfNeeded() {
    if (!latest) return;
    if (!latest.fault && latest.enabled.every(Boolean)) return;
    await stickRelease(); await deadman(false);
    log('recover_wait', { recovery: latest.recovery, enabled: latest.enabled });
    for (let i = 0; i < 300 && (latest.fault || latest.recovery !== 'idle_disabled' && !latest.enabled.every(Boolean)); i++) await delay(100);
    if (!latest.enabled.every(Boolean)) {
      for (let attempt = 0; attempt < 10 && !latest.enabled.every(Boolean); attempt++) {
        const button = page.locator('.pd-enable');
        if (await button.count()) { await button.click(); log('enable_click'); }
        for (let i = 0; i < 40 && !latest.enabled.every(Boolean); i++) await delay(100);
      }
    }
    for (let i = 0; i < 50 && !latest.tcp_ready; i++) await delay(100);
    log('recovered', { enabled: latest.enabled, tcp_ready: latest.tcp_ready });
  }

  // ---------- powrót do pozy startowej JOG-iem osi (tryb „Osie”) ----------
  async function home(tolerance = 2) {
    await stickRelease();
    await click('Osie', '.pd-right'); await click('Szybko', '.pd-presets');
    for (let joint = 1; joint <= 6; joint++) {
      const error = () => HOME[joint - 1] - latest.q[joint - 1];
      if (Math.abs(error()) <= tolerance) continue;
      const pair = Math.floor((joint - 1) / 2), horizontal = (joint - 1) % 2 === 0;
      await page.locator('.pd-chip').nth(pair).click(); await delay(80);
      await deadman(true);
      const sign = Math.sign(error());
      await stickTo(horizontal ? sign * .8 : 0, horizontal ? 0 : sign * .8);
      const started = Date.now();
      while (Date.now() - started < 30000 && Math.sign(error()) === sign && Math.abs(error()) > tolerance && !latest.fault) await delay(20);
      await stickRelease(); await delay(600);
      log('home_joint', { joint, error: +error().toFixed(2) });
      await recoverIfNeeded();
    }
    await deadman(false);
    await click('TCP', '.pd-right');
  }

  // ---------- sesja ----------
  const PAIRS = [['x', 'y'], ['z', 'rz'], ['rx', 'ry']];
  const SPEEDS = ['Wolno', 'Średnio', 'Szybko'];
  const holds = [];
  await recoverIfNeeded();
  await home();
  const end = Date.now() + MINUTES * 60000;
  let cycle = 0;
  while (Date.now() < end) {
    cycle++;
    await recoverIfNeeded();
    const drift = latest.q.map((q, j) => Math.abs(q - HOME[j]));
    if (Math.max(...drift) > 25) { log('rehome', { drift: drift.map(d => +d.toFixed(1)) }); await home(); }
    const pair = Math.floor(random() * 3), horizontal = random() < .5, speed = SPEEDS[Math.floor(random() * 3)];
    const axis = PAIRS[pair][horizontal ? 0 : 1], duration = 500 + Math.floor(random() * 1500);
    await page.locator('.pd-chip').nth(pair).click(); await delay(60); await click(speed, '.pd-presets');
    await deadman(true);
    for (const direction of [1, -1]) {
      const generationBefore = latest.generation;
      const on = (Date.now() - t0) / 1000;
      await stickTo(horizontal ? direction * .85 : 0, horizontal ? 0 : direction * .85);
      await delay(duration);
      const off = (Date.now() - t0) / 1000;
      await stickRelease();
      holds.push({ cycle, axis, direction, speed, on, off, generationBefore });
      log('hold', { cycle, axis, direction, speed, on, off });
      await delay(450 + Math.floor(random() * 300));
    }
    if (random() < .5) await deadman(false);
    await delay(200 + Math.floor(random() * 400));
  }
  await stickRelease(); await deadman(false); await delay(1500);

  // ---------- ocena: każdy przytrzymany ruch powinien trwać do puszczenia drążka ----------
  const rows = fs.readFileSync(path.join(OUT, 'session.jsonl'), 'utf8').trim().split('\n').map(l => JSON.parse(l));
  const states = rows.filter(r => r.event === 'tcp_state');
  const interruptions = [], noStart = [];
  for (const h of holds) {
    const during = states.filter(s => s.t > h.on && s.t <= h.off + .1);
    const started = during.find(s => ['arming', 'running', 'braking', 'settling'].includes(s.state));
    if (!started) { noStart.push(h); continue; }
    const early = states.find(s => s.t > started.t && s.t < h.off - .15 && ['braking', 'settling', 'stopped', 'failed', 'ready'].includes(s.state));
    if (early) interruptions.push({ ...h, at: early.t, state: early.state, message: early.message });
  }
  const summary = { minutes: MINUTES, cpu: CPU, holds: holds.length, no_start: noStart.length, interruptions: interruptions.length, faults: observer.faults,
    interruption_reasons: interruptions.reduce((a, x) => { const k = (x.message || x.state).replace(/[-\d.,]+/g, '#').slice(0, 90); a[k] = (a[k] || 0) + 1; return a; }, {}),
    no_start_examples: noStart.slice(0, 5), pendant_notes: rows.filter(r => r.event === 'pendant').map(r => r.text).reduce((a, k) => { k = k.replace(/[-\d.,]+/g, '#').slice(0, 90); a[k] = (a[k] || 0) + 1; return a; }, {}) };
  fs.writeFileSync(path.join(OUT, 'summary.json'), JSON.stringify(summary, null, 2));
  console.log(JSON.stringify(summary, null, 2));
  await browser.close(); logFile.end(); process.exit(0);
})().catch(error => { console.error(error); log('error', { message: String(error) }); process.exit(1); });
