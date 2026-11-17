(() => {
  "use strict";
  const $ = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => [...r.querySelectorAll(s)];
  const fmt = (v, d) => v.toLocaleString("pl-PL", { minimumFractionDigits: d, maximumFractionDigits: d });
  const reduce = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const ico = (id) => `<svg class="ico"><use href="#${id}"/></svg>`;
  const badge = (cls, text, sm) => `<span class="badge ${cls}${sm ? " sm" : ""}"><i class="dot"></i>${text}</span>`;

  // ---------- Dane przykładowe (wspólne dla C1–C3) ----------
  const axes = [
    { j: 1, val: -10.1, min: -171, max: 171, shaft: -253.70, rtt: 1.17, rx: 412, tx: 206, hz: 50, to: 0, last: 4 },
    { j: 2, val: 4.5, min: -200, max: 30, shaft: 113.82, rtt: 0.68, rx: 410, tx: 205, hz: 50, to: 0, last: 6 },
    { j: 3, val: -6.5, min: -196, max: 57, shaft: -161.26, rtt: 1.00, rx: 411, tx: 205, hz: 50, to: 0, last: 3 },
    { j: 4, val: 36.1, min: -180, max: 180, shaft: 903.21, rtt: 0.70, rx: 409, tx: 204, hz: 50, to: 1, last: 5 },
    { j: 5, val: 58.3, min: -100, max: 100, shaft: 1458.92, rtt: 1.30, rx: 412, tx: 206, hz: 50, to: 0, last: 4 },
    { j: 6, val: -44.6, min: -180, max: 180, shaft: -1115.38, rtt: 0.98, rx: 410, tx: 205, hz: 50, to: 0, last: 7 },
  ];
  // [grupa, stan, nazwa, typ, pid, cpu(live base), ram, czas pracy, komunikat]
  const procs = [
    ["System", "ok", "mk-OptiPlex-3050", "host", "—", 35, "2,6G", "6 h 55 min", "Ubuntu 22.04.5 LTS · 6.8.0-138-generic"],
    ["System", "ok", "robot-web-status.service", "systemd", "48629", null, "—", "—", "active (running)"],
    ["System", "ok", "robot-can@can0.service", "systemd", "—", null, "—", "—", "active (exited)"],
    ["System", "ok", "can0 / gs_usb", "SocketCAN", "—", null, "—", "—", "ERROR-ACTIVE · 1 Mbit/s · błędy 0/0 · bus-off 0"],
    ["ROS 2", "ok", "can_diagnostics", "robot_can_diagnostics", "48672", 11.1, "38M", "1 h 26 min", "install/stop-recovery-ros"],
    ["ROS 2", "ok", "configuration", "robot_configuration", "48676", 9.1, "61M", "1 h 26 min", "install/robot_configuration"],
    ["ROS 2", "ok", "fault_snapshot", "robot_web_bridge", "48680", 0.5, "55M", "1 h 26 min", "install/stop-recovery-ros"],
    ["ROS 2", "ok", "monitor", "robot_system_monitor", "48670", 0.5, "59M", "1 h 26 min", "install/robot_system_monitor"],
    ["ROS 2", "ok", "planner", "robot_motion_planner", "48678", 0.5, "39M", "1 h 26 min", "install/stop-recovery-ros"],
    ["ROS 2", "ok", "state_publisher", "robot_state_publisher", "48684", 1.4, "27M", "1 h 26 min", "/opt/ros/humble"],
    ["ROS 2", "ok", "Diagnostyka CAN", "mks_servo_can", "—", null, "—", "—", "Skanowanie zakończone"],
    ["ROS 2", "off", "ros2_control", "kontrolery", "—", null, "—", "—", "Kontrolery nieuruchomione"],
    ["ROS 2", "ok", "Graf ROS", "węzły", "—", null, "—", "—", "7 węzłów"],
    ["Backend", "ok", "robot_web_bridge", "FastAPI", "48674", 25, "184M", "1 h 26 min", "uvicorn http://192.168.1.69:8080"],
    ["Backend", "ok", "Klienci WebSocket", "WebSocket", "—", null, "—", "—", "status 2 · CAN 2 · ręczne 0"],
    ["Backend", "ok", "Snapshot statusu", "REST/WS", "—", null, "—", "—", "wiek 0,2 s"],
    ["Frontend", "ok", "Build panelu", "web/dist", "—", null, "—", "—", "c192281+dirty"],
    ["Frontend", "ok", "Ta karta", "przeglądarka", "—", null, "—", "—", "status połączony · CAN połączony"],
  ];
  const events = [
    ["19:31:02", "warn", "J4 · CAN 4", "Przekroczony czas odczytu pozycji (1×)", "rozwiązane"],
    ["18:05:44", "plan", "can0", "Restart interfejsu (Reset CAN z panelu)", "info"],
    ["17:40:10", "plan", "robot-web-status", "Restart usługi po aktualizacji", "info"],
    ["15:35:43", "plan", "Diagnostyka", "Start ciągłego zapisu przebiegów", "info"],
  ];
  const kpis = [
    ["CPU", "35,0", "%", 35, null, "35,4,1"], ["RAM", "2,6", "/ 7,6 G", 34], ["CAN", "10,6", "/ 60 %", 17.7, 100, "10.6,0.4,1"],
    ["RTT", "9,9", "/ 24 ms", 41, null, "9.9,0.6,1"], ["Ramki/s", "2 464", "RX", null, null, "2464,30,0"], ["Błędy CAN", "0", "RX/TX", null],
    ["Napędy", "6/6", "odpowiada", null], ["Procesy", "17/18", "OK", null],
  ];
  const pct = (v, a) => ((v - a.min) / (a.max - a.min) * 100).toFixed(1) + "%";
  const signed = (v) => (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v);
  const fill = (name, fn) => $$(`[data-fill="${name}"]`).forEach((el) => { el.innerHTML = fn(el); });

  // ---------- Sterowanie ręczne: wiersze osi ----------
  const axisRows = $("#axisRows");
  if (axisRows) axisRows.innerHTML = axes.map((a) => `
    <div class="axrow">
      <div class="ax-id"><span class="ax j${a.j}">J${a.j}</span><small>CAN ${a.j}</small><span class="badge ok sm"><i class="dot"></i>Połączony</span></div>
      <div class="jog">
        <button class="btn icon" type="button" title="JOG J${a.j} −">${ico("i-left")}</button>
        <div class="jog-val"><b class="num" data-live="${a.val},0.04,1">${fmt(a.val, 1)}</b><span class="deg">°</span>
          <div class="track" style="--c:var(--j${a.j})"><span class="zero" style="left:${pct(0, a)}"></span><i style="left:${pct(a.val, a)}"></i></div>
          <div class="lims num"><span>${signed(a.min)}°</span><span>${signed(a.max)}°</span></div></div>
        <button class="btn icon" type="button" title="JOG J${a.j} +">${ico("i-right")}</button>
      </div>
      <div><span class="badge"><i class="dot"></i>nieznany</span></div>
      <div><span class="seg sm"><button type="button" aria-pressed="true">Wył</button><button type="button" aria-pressed="false">Zał</button></span></div>
      <div><button class="btn sm" type="button" disabled>Zeruj</button></div>
      <div><span class="field sm" style="--w:46px"><input value="7,19" aria-label="Prędkość J${a.j}"><span class="unit">°/s</span></span></div>
    </div>`).join("");

  // ---------- Diagnostyka: wskaźniki ----------
  fill("kpis", () => kpis.map(([l, v, u, m, lim, live]) => `
    <div class="kpi"><span class="lbl">${l}</span><span class="kpi-v"><b class="num"${live ? ` data-live="${live}"` : ""}>${v}</b><em>${u}</em></span>
    ${m != null ? `<span class="meter" style="--v:${m}%${l === "CAN" ? ";--l:100%" : ""}"><i></i></span>` : ""}</div>`).join(""));
  fill("resources", () => kpis.slice(0, 4).map(([l, v, u, m, , live]) => `
    <div class="bar"><span class="lbl">${l}</span><span class="meter" style="--v:${m}%"><i></i></span><b class="num"${live ? ` data-live="${live}"` : ""}>${v}</b><em>${u.replace("/ ", "/")}</em></div>`).join(""));

  // ---------- Diagnostyka: urządzenia CAN ----------
  fill("can", (el) => axes.map((a) => el.dataset.cols === "full"
    ? `<tr data-axis-row="${a.j}"><td class="mono">${a.j}</td><td><span class="ax j${a.j}">J${a.j}</span></td><td>MKS SERVO42D CAN</td><td>${badge("ok", "Odpowiada", 1)}</td><td>${badge("", "Zatrzymany", 1)}</td><td>${badge("", "Wyłączony", 1)}</td><td class="n" data-live="${a.rtt},0.15,2">${fmt(a.rtt, 2)}</td><td class="n">${a.to}</td></tr>`
    : `<tr data-axis-row="${a.j}"><td><span class="ax j${a.j}">J${a.j}</span></td><td>${badge("ok", "OK", 1)}</td><td>${badge("", "Wył", 1)}</td><td class="muted">Zatrz.</td><td class="n" data-live="${a.rtt},0.15,2">${fmt(a.rtt, 2)}</td><td class="n">${a.to}</td></tr>`).join(""));
  fill("cancomm", () => axes.map((a) => `<tr><td><span class="ax j${a.j}">J${a.j}</span></td><td class="n" data-live="${a.rx},6,0">${a.rx}</td><td class="n" data-live="${a.tx},3,0">${a.tx}</td><td class="n">${a.hz}</td><td class="n">${a.to}</td><td class="n"><span data-live="${a.last},3,0">${a.last}</span> ms temu</td></tr>`).join(""));
  const canStats = [["Interfejs", badge("ok", "can0 · aktywny", 1)], ["Bitrate", `<b class="big num">1 000 000</b><em>bit/s</em>`], ["Obciążenie", `<b class="big num" data-live="10.6,0.4,1">10,6</b><em>/ 60 %</em>`], ["Obieg odczytu", `<b class="big num" data-live="2.1,0.2,1">2,1</b><em>ms</em>`], ["Błędy · bus-off", `<b class="big num">0 · 0</b>`]];
  fill("canstats", () => canStats.map(([l, v]) => `<div><span class="lbl">${l}</span>${v}</div>`).join(""));
  const canKv = [["Stan", "ERROR-ACTIVE"], ["Bitrate", "1 Mbit/s"], ["Obciążenie", `<span data-live="10.6,0.4,1">10,6</span> / 60 %`], ["Obieg odczytu", `<span data-live="2.1,0.2,1">2,1</span> ms`], ["Ramki RX", `<span data-live="18389818,180,0" data-inc>18 389 818</span>`], ["Ramki TX", `<span data-live="9258974,90,0" data-inc>9 258 974</span>`], ["Błędy RX/TX", "0 / 0"], ["Bus-off", "0"], ["Urządzenia", "6 · MKS SERVO42D"]];
  fill("cankv", () => canKv.map(([k, v]) => `<div class="kv"><span>${k}</span><b class="num">${v}</b></div>`).join(""));

  // ---------- Diagnostyka: procesy (tabela pogrupowana) ----------
  fill("proc", (el) => {
    const cols = el.dataset.cols || "full";
    let group = "", html = "";
    procs.forEach(([g, st, name, type, pid, cpu, ram, up, msg]) => {
      if (g !== group) { group = g; html += `<tr class="group-row" data-group="${g}"><td colspan="8">${g === "Backend" ? "Backend Python" : g}</td></tr>`; }
      const cpuCell = cpu == null ? "—" : `<span data-live="${cpu},${Math.max(0.2, cpu * 0.12).toFixed(2)},${cpu >= 20 ? 0 : 1}">${fmt(cpu, cpu >= 20 ? 0 : 1)}</span>`;
      const stb = st === "ok" ? badge("ok", "OK", 1) : badge("", "OFF", 1);
      if (cols === "compact") html += `<tr data-group="${g}" data-proc="${name}"><td>${stb}</td><td class="mono">${name}</td><td class="n">${cpuCell}</td><td class="muted ell">${msg}</td></tr>`;
      else html += `<tr data-group="${g}" data-proc="${name}"><td>${stb}</td><td class="mono">${name}</td><td class="c-typ">${type}</td><td class="n c-pid">${pid}</td><td class="n">${cpuCell}</td><td class="n c-ram">${ram}</td>${cols === "full" ? `<td class="n c-up">${up}</td>` : ""}<td class="ell">${msg}</td></tr>`;
    });
    return html;
  });

  // ---------- Menu: panel stanu (CPU per wątek, RAM, CAN) ----------
  const cores = [42, 28, 39, 31];
  fill("cores", () => cores.map((v, i) => `<div class="core"><span class="core-bar"><i style="--v:${v}%"></i></span><small class="num">${v}</small><small class="core-n">W${i}</small></div>`).join(""));
  const gauges = { ram: [34, 0.3, 0], can: [10.6, 0.4, 1] };
  function tickGauges() {
    const now = cores.map((b) => Math.max(2, Math.min(98, b + (Math.random() * 2 - 1) * 9)));
    $$(".core").forEach((el, i) => { $("i", el).style.setProperty("--v", now[i].toFixed(0) + "%"); $("small", el).textContent = now[i].toFixed(0); });
    $$('[data-gauge="cpu"] .ns-row b').forEach((b) => { b.textContent = fmt(now.reduce((x, y) => x + y, 0) / now.length, 0); });
    Object.entries(gauges).forEach(([k, [base, amp, dec]]) => {
      const v = base + (Math.random() * 2 - 1) * amp;
      $$(`[data-gauge="${k}"]`).forEach((g) => { $(".ns-row b", g).textContent = fmt(v, dec); g.style.setProperty("--v", v.toFixed(1) + "%"); });
    });
  }

  // ---------- Przegląd: pozycje osi ----------
  fill("ovaxes", () => axes.map((a) => `<div class="ova j${a.j}"><span class="ax j${a.j}">J${a.j}</span><b class="num" data-live="${a.val},0.04,1">${fmt(a.val, 1)}</b><span class="deg">°</span>
    <div class="track" style="--c:var(--j${a.j})"><span class="zero" style="left:${pct(0, a)}"></span><i style="left:${pct(a.val, a)}"></i></div><span class="muted num">0,0 °/s</span></div>`).join(""));

  // ---------- Diagnostyka: zdarzenia ----------
  fill("events", (el) => {
    const lim = +el.dataset.limit || events.length;
    return `<div class="ev-now">${ico("i-check")}Brak aktywnych problemów</div>` + events.slice(0, lim).map(([t, cls, src, txt, state]) =>
      `<div class="ev ${cls}"><span class="num ev-t">${t}</span><i class="ev-dot"></i><span class="ev-src">${src}</span><span class="ev-txt">${txt}</span><span class="ev-st">${state}</span></div>`).join("");
  });

  // ---------- Diagnostyka: wykresy (siatka) i paski osi (oscyloskop) ----------
  fill("charts", (el) => axes.map((a) => `
    <article class="chart-card" data-chart-card="${a.j}">
      <div class="chart-h"><span class="ax j${a.j}">J${a.j}</span><small class="muted">CAN ${a.j}</small>
        <b class="val num" data-live="${a.shaft},0.01,2">${fmt(a.shaft, 2)}</b><span class="deg">°</span>
        ${badge("ok", "Świeże", 1)}<button class="btn icon sm" type="button" title="Odepnij oś do okna">${ico("i-pin")}</button></div>
      <div class="chart" data-axis="${a.j}"${el.hasAttribute("data-mini") ? " data-mini" : ""} style="--c:var(--j${a.j})"></div>
    </article>`).join(""));
  fill("strips", () => axes.map((a) => `
    <div class="srow j${a.j}" data-chart-card="${a.j}">
      <div class="s-info">
        <div class="s-top"><span class="ax j${a.j}">J${a.j}</span><small>CAN ${a.j}</small><b class="val num" data-live="${a.shaft},0.01,2">${fmt(a.shaft, 2)}</b><span class="deg">°</span></div>
        <div class="s-badges">${badge("ok", "Odpowiada", 1)}${badge("", "ENABLE wył", 1)}${badge("", "Zatrzymany", 1)}</div>
        <div class="s-meta num">RTT <b data-live="${a.rtt},0.15,2">${fmt(a.rtt, 2)}</b> ms · odczyt ${a.hz} Hz · timeouty ${a.to}</div>
      </div>
      <div class="chart" data-axis="${a.j}" data-mini data-strip style="--c:var(--j${a.j})"></div>
      <div class="s-act"><button class="btn icon sm" type="button" title="Odepnij oś do okna">${ico("i-pin")}</button><button class="btn icon sm" type="button" title="CSV osi">${ico("i-download")}</button></div>
    </div>`).join(""));

  // ---------- Diagnostyka: mapa systemu (drzewo) ----------
  fill("tree", () => {
    const node = (target, st, label, meta) => `<button class="tnode" type="button" data-target="${target}"><i class="tdot ${st}"></i><span class="tl">${label}</span><span class="tm num">${meta || ""}</span></button>`;
    let h = `<div class="tgroup">${ico("i-can")}Magistrala can0<span class="tm num">RTT <span data-live="9.9,0.6,1">9,9</span> ms</span></div>` + node("tab:can", "ok", "can0 / gs_usb", "ERROR-ACTIVE");
    h += axes.map((a) => node(`chart:${a.j}`, "ok", `<span class="ax j${a.j}">J${a.j}</span> MKS SERVO42D`, `<span data-live="${a.rtt},0.15,2">${fmt(a.rtt, 2)}</span> ms`)).join("");
    let group = "";
    procs.forEach(([g, st, name, , , cpu]) => {
      if (g !== group) { group = g; h += `<div class="tgroup">${ico(g === "System" ? "i-overview" : g === "ROS 2" ? "i-arch" : g === "Backend" ? "i-config" : "i-palette")}${g === "Backend" ? "Backend Python" : g}</div>`; }
      h += node(`proc:${name}`, st, name, cpu == null ? "" : `${fmt(cpu, cpu >= 20 ? 0 : 1)} %`);
    });
    return h;
  });

  // ---------- Konfiguracja, System wizualny ----------
  const machine = [
    ["Translacja X", "mm", [0, 0, 0, 220, 80, 60]], ["Translacja Y", "mm", [0, 0, 0, 0, 0, 0]],
    ["Translacja Z", "mm", [100, 100, 250, 0, 0, 0]], ["Rotacja X (roll)", "°", [0, 0, 0, 0, 0, 0]],
    ["Rotacja Y (pitch)", "°", [0, 0, 0, 0, 0, 0]], ["Rotacja Z (yaw)", "°", [0, 0, 0, 0, 0, 0]],
  ];
  const mrows = $("#machineRows");
  if (mrows) mrows.innerHTML = machine.map(([n, u, v]) =>
    `<tr><td>${n}</td><td class="muted">${u}</td>${v.map((x, i) => `<td><input class="cell-in" value="${x}" aria-label="${n} J${i + 1}"></td>`).join("")}</tr>`).join("");
  const iconGrid = $("#iconGrid");
  if (iconGrid) iconGrid.innerHTML = $$("symbol").map((s) => `<span class="icon-cell">${ico(s.id)}<small>${s.id.slice(2)}</small></span>`).join("");

  // ---------- Podglądy 3D ----------
  const tpl = $("#tplRobot");
  $$("[data-robot]").forEach((host) => {
    host.prepend(tpl.content.cloneNode(true));
    const L = [40, 330], F = [320, 430], R = [620, 320], B = [340, 236];
    const lerp = (p, q, t) => [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
    let d = "";
    for (let i = 0; i <= 10; i++) {
      const t = i / 10, a = lerp(L, B, t), b = lerp(F, R, t), c = lerp(L, F, t), e = lerp(B, R, t);
      d += `M${a[0]} ${a[1]}L${b[0]} ${b[1]}M${c[0]} ${c[1]}L${e[0]} ${e[1]}`;
    }
    $(".floor", host).innerHTML = `<path d="${d}"/>`;
  });

  // ---------- Wykresy: budowa i odświeżanie ----------
  const N = 200, series = {};
  const sample = (a, i) => a.shaft + ((a.j === 1 || a.j === 5) ? (Math.random() < 0.5 ? 0 : 0.02) : 0) + Math.sin(i / 25) * 0.003;
  axes.forEach((a) => { series[a.j] = Array.from({ length: N }, (_, i) => sample(a, i)); });
  const chartEls = $$(".chart[data-axis]");
  chartEls.forEach((c) => {
    const a = axes[c.dataset.axis - 1], mini = c.hasAttribute("data-mini");
    const ys = mini ? [0.05, 0, -0.05] : [0.05, 0.025, 0, -0.025, -0.05];
    const step = 120 / (ys.length - 1);
    c.innerHTML = ys.map((o, i) => `<span class="yl num" style="top:${(i * 100 / (ys.length - 1)).toFixed(1)}%">${fmt(a.shaft + o, 2)}</span>`).join("") +
      `<svg viewBox="0 0 400 120" preserveAspectRatio="none">${ys.map((_, i) => `<line class="gl" x1="0" x2="400" y1="${i * step}" y2="${i * step}"/>`).join("")}
       <polyline class="ref" points="0,60 400,60"/><polyline class="ln"/></svg>` +
      (mini ? "" : `<div class="xl num"><span>0 s</span><span>2,5 s</span><span>5 s</span><span>7,5 s</span><span>10 s</span></div>`);
  });
  function drawCharts() {
    chartEls.forEach((c) => {
      if (!c.offsetParent) return;
      const a = axes[c.dataset.axis - 1], s = series[a.j], lo = a.shaft - 0.06, hi = a.shaft + 0.06;
      $("polyline.ln", c).setAttribute("points", s.map((v, i) => `${(i / (N - 1) * 400).toFixed(1)},${(120 - (v - lo) / (hi - lo) * 120).toFixed(1)}`).join(" "));
    });
  }
  let t = N;
  if (!reduce) setInterval(() => { axes.forEach((a) => { series[a.j].shift(); series[a.j].push(sample(a, t)); }); t++; drawCharts(); }, 250);

  // ---------- Nawigacja ----------
  const app = $("#app"), titles = {};
  $$(".nav-link").forEach((a) => { titles[a.dataset.go] = a.textContent.trim(); });
  function go(id) {
    if (!$(`.screen[data-screen="${id}"]`)) id = "overview";
    $$(".screen").forEach((s) => s.classList.toggle("active", s.dataset.screen === id));
    $$(".nav-link").forEach((a) => a.classList.toggle("active", a.dataset.go === id));
    // Jak w aplikacji: menu zwinięte na ekranach ruchu, rozwinięte na pozostałych (przycisk zmienia do zmiany strony).
    app.classList.toggle("collapsed", id === "manual" || id === "programming");
    $("#crumb").textContent = titles[id];
    if (location.hash !== "#" + id) history.replaceState(null, "", "#" + id);
    $$(".mock-switch a").forEach((a) => { a.href = a.dataset.file + "#" + id; });
    requestAnimationFrame(drawCharts);
  }
  $$("[data-go]").forEach((a) => a.addEventListener("click", (e) => { e.preventDefault(); go(a.dataset.go); }));
  addEventListener("hashchange", () => go(location.hash.slice(1)));
  $("[data-act=collapse]").addEventListener("click", () => app.classList.toggle("collapsed"));

  // ---------- Zakładki ----------
  function selectTab(group, tab) {
    const name = group.dataset.tabs;
    $$(".tab", group).forEach((x) => x.setAttribute("aria-selected", String(x === tab)));
    $$(`[data-panel^="${name}:"]`).forEach((p) => { p.hidden = p.dataset.panel !== `${name}:${tab.dataset.tab}`; });
    requestAnimationFrame(drawCharts);
  }
  $$("[data-tabs]").forEach((group) => $$(".tab", group).forEach((tab) => tab.addEventListener("click", () => { if (!tab.disabled) selectTab(group, tab); })));

  // Filtr grup procesów
  $$("[data-proc-filter]").forEach((group) => $$(".tab", group).forEach((tab) => tab.addEventListener("click", () => {
    $$(".tab", group).forEach((x) => x.setAttribute("aria-selected", String(x === tab)));
    const g = tab.dataset.group;
    $$("tr[data-group]", group.closest(".card")).forEach((tr) => { tr.hidden = g !== "all" && tr.dataset.group !== g; });
  })));

  // Mapa systemu → szczegóły
  $$(".tnode").forEach((n) => n.addEventListener("click", () => {
    $$(".tnode").forEach((x) => x.classList.toggle("sel", x === n));
    const [kind, id] = n.dataset.target.split(":");
    const tabs = $('[data-tabs="dg"]');
    const pick = (t) => selectTab(tabs, $(`.tab[data-tab="${t}"]`, tabs));
    $$(".hl").forEach((x) => x.classList.remove("hl"));
    if (kind === "chart") { pick("tr"); $(`[data-panel="dg:tr"] [data-chart-card="${id}"]`).classList.add("hl"); }
    else if (kind === "proc") { pick("proc"); const row = $(`[data-panel="dg:proc"] tr[data-proc="${id}"]`); row.classList.add("hl"); row.scrollIntoView({ block: "nearest" }); }
    else pick(id);
  }));

  // ---------- Kontrolki ----------
  $$(".seg").forEach((seg) => $$("button", seg).forEach((b) => b.addEventListener("click", () => {
    $$("button", seg).forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    if (b.dataset.mode) {
      const screen = b.closest(".screen");
      screen.dataset.mode = b.dataset.mode;
      $$("[data-mode-label]", screen).forEach((el) => { el.textContent = b.dataset.mode === "live" ? el.dataset.liveText : el.dataset.planText; });
    }
  })));
  $$(".switch").forEach((s) => s.addEventListener("click", () => s.setAttribute("aria-checked", String(s.getAttribute("aria-checked") !== "true"))));
  $$(".btn.stop").forEach((b) => b.addEventListener("click", () => document.body.classList.add("stopped")));
  $("#stopRelease").addEventListener("click", () => document.body.classList.remove("stopped"));

  // ---------- Wartości na żywo ----------
  const live = $$("[data-live]").map((el) => {
    const [base, amp, dec] = el.dataset.live.split(",").map(Number);
    return { el, base, amp, dec, inc: el.hasAttribute("data-inc") };
  });
  function tick() {
    live.forEach((o) => {
      if (o.inc) o.base += o.amp * (0.6 + Math.random() * 0.8);
      const v = o.inc ? o.base : o.base + (Math.random() * 2 - 1) * o.amp;
      o.el.textContent = fmt(v, o.dec);
    });
  }
  tick(); tickGauges();
  if (!reduce) { setInterval(tick, 500); setInterval(tickGauges, 1000); }

  go(location.hash.slice(1) || "overview");
})();
