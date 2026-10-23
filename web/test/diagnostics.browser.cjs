// Cały podgląd na atrapach; bez CAN, ROS i poleceń sprzętowych.
const {chromium}=require('playwright');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
(async()=>{
 const pinningOnly=process.argv.includes('--pinning-only'),wheelOnly=process.argv.includes('--wheel-only');
 const browser=await chromium.launch({headless:true}),context=await browser.newContext({viewport:{width:1920,height:1080},acceptDownloads:true});
 const dist=path.resolve(__dirname,'../dist'),errors=[],writes=[],reads=[],sockets=new Set();
 const now=Date.now(),base='/api/v1/diagnostics/trace';let sequence=500,broadcasting=true,serverTime=now,traceConnections=0;
 const make=(axis,n,gap=false)=>({axis,sequence:String(n),segment:n<480?'1':'2',monotonic_ns:String(n*40000000),utc_ns:String(BigInt(now-20000+n*40)*1000000n),
  time_ms:now-20000+n*40,position_cdeg:n===470?12345:n===471?-8765:axis*100,gap:gap||n===480});
 const samples=Array.from({length:5},(_,i)=>Array.from({length:500},(_,n)=>make(i+1,n+1))).flat();
 const status=()=>({recording:true,connected:true,server_time_ms:serverTime,source_dropped:0,missing_samples:1,writer_dropped:0,queued:0,saved:2500,cursor:sequence,
   axes:Array.from({length:6},(_,i)=>({axis:i+1,limit_bytes:20000000,file_bytes:i===5?0:20000000,retained_records:i===5?0:500,fresh:i!==5,age_seconds:i===5?null:0,
    recording:true,error:null,first_ms:i===5?null:now-20000,last_ms:i===5?null:now,latest:i===5?null:make(i+1,500),corrupt_records:i===1?1:0}))});
 function publish(){if(!broadcasting)return;for(const s of sockets)s.send(JSON.stringify({type:'shaft_trace',cursor:++sequence,expired:false,samples:[],status:status()}));}
 const timer=setInterval(publish,50);
 context.on('page',page=>page.on('pageerror',e=>errors.push(e.message)));
 await context.routeWebSocket('ws://robot.test/**',socket=>{if(!socket.url().endsWith(base+'/ws'))return;traceConnections++;sockets.add(socket);publish();socket.onClose(()=>sockets.delete(socket));socket.onMessage(value=>writes.push(value));});
 await context.route('http://robot.test/**',async route=>{
  const req=route.request(),u=new URL(req.url());if(req.method()!=='GET'){writes.push(u.pathname);return route.abort();}
  if(u.pathname.startsWith(base)) {
   reads.push(u.pathname+u.search);
   if(u.pathname.endsWith('export.csv'))return route.fulfill({contentType:'text/csv',headers:{'Content-Disposition':'attachment; filename="shaft-J1.csv"'},body:'axis,position_degrees\n1,-5.63\n'});
   if(u.pathname.endsWith('/history')) {
    const lo=Number(u.searchParams.get('start_ms')??0),hi=Number(u.searchParams.get('end_ms')??Infinity);
    return route.fulfill({contentType:'application/json',body:JSON.stringify({status:status(),axes:Array.from({length:6},(_,i)=>({axis:i+1,samples:samples.filter(p=>p.axis===i+1&&p.time_ms>=lo&&p.time_ms<=hi),next_cursor:500}))})});
   }
   return route.fulfill({contentType:'application/json',body:JSON.stringify(status())});
  }
  if(u.pathname.startsWith('/api/'))return route.fulfill({status:503,contentType:'application/json',body:'{"detail":"Atrapa"}'});
  const name=u.pathname==='/'?'index.html':u.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try {
  const page=await context.newPage();await page.goto('http://robot.test/');
  await page.locator('.diagnostics-tile').click();await page.getByRole('heading',{name:'Diagnostyka',exact:true}).waitFor();
  assert.equal(new URL(page.url()).hash,'#diagnostics');assert.equal(await page.locator('.shaft-chart').count(),6);
  const displayedEnd=async(axis=1)=>Number(new URL(await page.locator(`.shaft-chart[data-axis="${axis}"]`).getByRole('link',{name:'CSV zakresu',exact:true}).getAttribute('href'),'http://robot.test').searchParams.get('end_ms'));
  const panel=page.locator('.shaft-chart[data-axis="1"]');
  if(wheelOnly) {
   await page.setViewportSize({width:1440,height:900});
   const module=page.locator('.app-content > .diagnostics-screen');
   await module.getByText('Zapis ciągły aktywny',{exact:true}).waitFor();
   const canvas=panel.locator('canvas');
   const waitEnd=async expected=>page.waitForFunction(value=>Math.abs(Number(new URL(document.querySelector('.shaft-chart[data-axis="1"] a').href).searchParams.get('end_ms'))-value)<2,expected);
   async function nativeWheel(target,dx) {
    await target.scrollIntoViewIfNeeded();const box=await target.boundingBox(),width=await target.evaluate(el=>el.clientWidth-76);
    const old=await displayedEnd(),scroll=await page.evaluate(()=>scrollY);
    await page.mouse.move(box.x+box.width*.5,box.y+70);await page.mouse.wheel(dx,0);
    await waitEnd(old+dx/width*10000);
    assert.equal(await page.evaluate(()=>scrollY),scroll,'Pozioma rolka nie przewija strony');
    return displayedEnd();
   }
   const first=await nativeWheel(canvas,-120);assert(first<now,'Rolka w lewo odsłania starsze dane');
   assert.equal(await module.getByRole('button',{name:'Na żywo',exact:true}).getAttribute('aria-pressed'),'false');
   const next=await nativeWheel(canvas,60);assert(next>first,'Rolka w prawo odsłania późniejsze dane');
   for(let axis=1;axis<=6;axis++) assert.equal(await displayedEnd(axis),next,'Wspólny czas sześciu osi');
   const width=await canvas.evaluate(el=>el.clientWidth-76);
   async function dispatch(deltas,shift) {
    const old=await displayedEnd();
    const prevented=await canvas.evaluate((el,events)=>events.map(data=>{const event=new WheelEvent('wheel',{bubbles:true,cancelable:true,...data});el.dispatchEvent(event);return event.defaultPrevented;}),deltas);
    assert(prevented.every(Boolean),'Obsłużone poziome zdarzenia blokują natywne przewijanie');
    await waitEnd(old+shift);
   }
   await dispatch([{deltaX:-2,deltaMode:1}],-32/width*10000);
   await dispatch([{deltaX:1,deltaMode:2}],10000);
   await dispatch(Array.from({length:8},()=>({deltaX:-5})), -40/width*10000);
   const beforeIgnored=await displayedEnd();
   const ignored=await canvas.evaluate(el=>[{deltaX:0,deltaY:120},{deltaX:1,deltaY:120},{deltaX:0,deltaY:0},{deltaX:120,ctrlKey:true},{deltaX:120,metaKey:true}].map(data=>{
    const event=new WheelEvent('wheel',{bubbles:true,cancelable:true,...data});el.dispatchEvent(event);return event.defaultPrevented;
   }));
   assert(ignored.every(value=>!value),'Rolka pionowa i skróty przeglądarki pozostają obsługiwane natywnie');
   assert.equal(await displayedEnd(),beforeIgnored);
   await canvas.scrollIntoViewIfNeeded();const box=await canvas.boundingBox();await page.mouse.move(box.x+box.width/2,box.y+60);
   const beforeScroll=await page.evaluate(()=>scrollY);await page.mouse.wheel(0,160);
   await page.waitForFunction(before=>scrollY>before,beforeScroll);assert.equal(await displayedEnd(),beforeIgnored,'Pionowa rolka nie zmienia czasu');
   await module.getByRole('button',{name:'Na żywo',exact:true}).click();await waitEnd(now);
   await module.getByRole('button',{name:'Przypnij wykres J1',exact:true}).click();
   const floating=page.locator('.trace-window');await floating.waitFor();
   await nativeWheel(floating.locator('canvas'),-80);
   const pinnedEnd=await displayedEnd();
   for(let axis=2;axis<=6;axis++) assert.equal(await displayedEnd(axis),pinnedEnd,'Rolka w przypiętej osi przesuwa także osie modułu');
   assert.equal(await floating.locator('input[type=range]').count(),0,'Bez suwaka przewijania');
   assert(await floating.locator('.shaft-chart').evaluate(el=>el.scrollWidth<=el.clientWidth),'Bez poziomego paska przewijania wykresu');
   await page.locator('.sidebar a[href="#architecture"]').click();await module.waitFor({state:'detached'});
   await nativeWheel(floating.locator('canvas'),-40);
   await floating.getByRole('button',{name:'Na żywo',exact:true}).click();await waitEnd(now);
   assert.equal(await floating.getByLabel('Okno czasu [s]',{exact:true}).inputValue(),'10');
   assert.equal(traceConnections,1);assert.deepEqual(writes,[]);assert.deepEqual(errors,[]);
   console.log(JSON.stringify({result:'OK',scope:'wheel-only',checks:['pozioma rolka obu kierunków','wspólny czas osi','pauza i powrót live','piksele/linie/strony','sumowanie szybkich zdarzeń','pionowa rolka i skróty bez zmian','przypięta oś między modułami','brak suwaka i poziomego paska','jedno WS i brak poleceń sprzętowych']}));
   return;
  }
  if(!pinningOnly) {
  await page.getByText('Zapis ciągły aktywny',{exact:true}).waitFor();
  await page.getByText('Pominięte uszkodzone wpisy: 1',{exact:true}).waitFor();
  for(let axis=1;axis<=6;axis++) assert(await page.getByRole('img',{name:`Przebieg pozycji wału J${axis}`,exact:true}).isVisible());
  assert((await page.locator('.trace-controls').boundingBox()).height<100,'Pasek sterowania nie dziedziczy pustej wysokości karty');
  const sixth=await page.getByRole('img',{name:'Przebieg pozycji wału J6',exact:true}).boundingBox();
  assert(sixth.y+sixth.height<=1080,'Sześć przebiegów mieści się w oknie 1920×1080');
  await panel.getByText('1.00°',{exact:true}).waitFor();
  assert.equal(await page.locator('.shaft-chart[data-axis="6"] .shaft-value').textContent(),'—');
  assert.equal(await page.getByRole('spinbutton',{name:'Okno czasu [s]',exact:true}).inputValue(),'10');
  const toolbar=page.getByRole('group',{name:'Sterowanie wykresami',exact:true});
  assert.equal(await toolbar.locator('button svg').count(),7);
  for(const button of await toolbar.getByRole('button').all()) {
   assert.equal((await button.textContent()).trim(),'');assert(await button.getAttribute('title'));assert(await button.getAttribute('aria-label'));
  }
  const liveButton=page.getByRole('button',{name:'Na żywo',exact:true});

  await panel.locator('canvas').click();assert.equal(await liveButton.getAttribute('aria-pressed'),'true','Sam klik nie zatrzymuje podglądu');
  await page.getByRole('button',{name:'Powiększ czas',exact:true}).click();assert.equal(await page.getByRole('spinbutton',{name:'Okno czasu [s]',exact:true}).inputValue(),'5');
  await page.getByRole('button',{name:'Pomniejsz czas',exact:true}).click();
  await panel.getByRole('checkbox',{name:'Auto Y',exact:true}).uncheck();
  await page.getByRole('spinbutton',{name:'J1 minimum Y',exact:true}).fill('-20');await page.getByRole('spinbutton',{name:'J1 maksimum Y',exact:true}).fill('150');
  assert(await page.getByRole('spinbutton',{name:'J2 minimum Y',exact:true}).isDisabled());
  async function drag(axis,delta,outside=false) {
   const canvas=page.locator(`.shaft-chart[data-axis="${axis}"] canvas`),box=await canvas.boundingBox();
   const x=box.x+box.width/2,y=box.y+70,shift=outside?box.width/2+40:delta;
   const previous=await displayedEnd();
   await page.mouse.move(x,y);await page.mouse.down();await page.mouse.move(x+shift,y,{steps:8});
   assert.equal(await canvas.evaluate(el=>el.hasPointerCapture(1)),true,'Przeciąganie zachowuje capture poza wykresem');
   await page.mouse.up();
   assert.equal(await page.locator('canvas.is-dragging').count(),0);
   const actual=await displayedEnd(),expected=previous-shift/(box.width-76)*10000;
   assert(Math.abs(actual-expected)<30,`Przesunięcie wynika z pikseli/okna: ${actual-expected} ms`);
   for(let a=1;a<=6;a++)assert.equal(await displayedEnd(a),actual,'Wspólny czas sześciu osi');
   assert.equal(await liveButton.getAttribute('aria-pressed'),'false');return actual;
  }
  const older=await drag(3,90);assert(older<now);
  await page.waitForTimeout(180);assert.equal(await displayedEnd(),older,'Paczki live nie zmieniają przeglądanej historii');
  const newer=await drag(4,-45);assert(newer>older);
  await drag(1,0,true);
  await liveButton.click();assert.equal(await displayedEnd(),now);assert.equal(await liveButton.getAttribute('aria-pressed'),'true');
  await page.getByRole('button',{name:'Wstrzymaj podgląd',exact:true}).click();assert(await page.getByLabel('Koniec zakresu',{exact:true}).isVisible());
  const before=await page.getByLabel('Koniec zakresu',{exact:true}).inputValue();await page.getByRole('button',{name:'Wcześniej',exact:true}).click();
  await page.waitForFunction(value=>document.querySelector('input[type="datetime-local"]').value!==value,before);
  await page.getByRole('button',{name:'Cała historia',exact:true}).click();assert.equal(await page.getByRole('spinbutton',{name:'Okno czasu [s]',exact:true}).inputValue(),'20');
  const graph=await panel.locator('canvas').boundingBox();await page.mouse.move(graph.x+graph.width*.75,graph.y+60);
  await page.waitForFunction(()=>document.querySelector('.trace-cursor').textContent.startsWith('Kursor: '));
  await page.mouse.move(10,10);await page.getByRole('button',{name:'Na żywo',exact:true}).click();assert.equal(await page.locator('input[type="datetime-local"]').count(),0);
  assert.equal(await page.getByRole('spinbutton',{name:'Okno czasu [s]',exact:true}).inputValue(),'10','Na żywo resetuje szerokość okna');
  assert.equal(await page.getByRole('spinbutton',{name:'J1 minimum Y',exact:true}).inputValue(),'-20','Reset czasu zachowuje skalę Y');
  serverTime+=1000;publish();await page.waitForFunction(t=>Number(new URL(document.querySelector('.shaft-chart a').href).searchParams.get('end_ms'))===t,serverTime);
  const [download]=await Promise.all([page.waitForEvent('download'),panel.getByRole('link',{name:'CSV zakresu',exact:true}).click()]);
  assert.equal(download.suggestedFilename(),'shaft-J1.csv');assert(reads.some(p=>p.includes('export.csv?start_ms=')&&p.includes('&end_ms=')));
  broadcasting=false;await page.waitForFunction(()=>[...document.querySelectorAll('.shaft-value')].every(e=>e.textContent==='—'));
  await page.getByText('Stan zapisu nieaktualny',{exact:true}).waitFor();broadcasting=true;publish();
  await panel.getByText('1.00°',{exact:true}).waitFor();
  await page.screenshot({path:'/tmp/robot-shaft-trace.png',fullPage:true});
  }
  const module=page.locator('.app-content > .diagnostics-screen');
  await module.getByText('Zapis ciągły aktywny',{exact:true}).waitFor();
  await module.locator('.shaft-chart[data-axis="1"]').getByRole('checkbox',{name:'Auto Y',exact:true}).uncheck();
  await module.getByLabel('J1 minimum Y',{exact:true}).fill('-20');await module.getByLabel('J1 maksimum Y',{exact:true}).fill('150');
  await module.getByRole('button',{name:'Na żywo',exact:true}).click();
  assert.equal(await module.getByRole('button',{name:/^Przypnij wykres J/}).count(),6,'Pinezka jest przy każdej osi');
  await module.getByRole('button',{name:'Przypnij wykres J1',exact:true}).click();
  const floating=page.locator('.trace-window');await floating.waitFor();
  assert.equal(await floating.getAttribute('aria-label'),'Przypięty wykres J1');
  assert.equal(await floating.locator('.shaft-chart').count(),1);
  assert.equal(await module.locator('.shaft-chart').count(),5,'Pozostałe osie pozostają w Diagnostyce');
  assert.equal(await module.locator('.shaft-chart[data-axis="1"]').count(),0,'Oś odłączona nie jest zduplikowana');
  assert.equal(traceConnections,1,'Wspólne źródło trace bez ponownego WS');
  await module.getByRole('button',{name:'Przypnij wykres J2',exact:true}).click();
  assert.equal(await floating.getAttribute('aria-label'),'Przypięty wykres J2');
  assert.equal(await floating.locator('.shaft-chart').count(),1);
  assert.equal(await module.locator('.shaft-chart[data-axis="1"]').count(),1,'Poprzednia oś wraca do modułu');
  assert.equal(await module.locator('.shaft-chart[data-axis="2"]').count(),0);
  await module.getByRole('button',{name:'Przypnij wykres J1',exact:true}).click();
  for(let axis=1;axis<=6;axis++) assert.equal(await page.locator(`.shaft-chart[data-axis="${axis}"]`).count(),1,'Każda oś występuje dokładnie raz');
  assert.equal(await floating.getByLabel('J1 minimum Y',{exact:true}).inputValue(),'-20');
  await floating.getByRole('button',{name:'Wcześniej',exact:true}).click();
  assert.equal(await module.getByLabel('Koniec zakresu',{exact:true}).inputValue(),await floating.getByLabel('Koniec zakresu',{exact:true}).inputValue(),'Wspólny zakres obu podglądów');
  await floating.getByRole('button',{name:'Na żywo',exact:true}).click();
  await page.locator('.sidebar a[href="#architecture"]').click();
  await module.waitFor({state:'detached'});
  assert.equal(new URL(page.url()).hash,'#architecture');assert(await floating.isVisible());
  assert.equal(await module.count(),0,'Pozostałe osie nie są przenoszone do innych modułów');
  assert.equal(await page.locator('.shaft-chart').count(),1);
  await page.locator('.sidebar a[href="#can"]').click();assert(await floating.isVisible());assert.equal(traceConnections,1);
  serverTime+=1000;publish();
  await page.waitForFunction(t=>Number(new URL(document.querySelector('.shaft-chart a').href).searchParams.get('end_ms'))===t,serverTime);
  const initial=await floating.boundingBox(),beforeMove=await displayedEnd();
  const handle=floating.getByRole('button',{name:'Przesuń okno wykresów',exact:true}),hb=await handle.boundingBox();
  await page.mouse.move(hb.x+80,hb.y+hb.height/2);await page.mouse.down();await page.mouse.move(hb.x-20,hb.y+hb.height/2+35,{steps:8});await page.mouse.up();
  const moved=await floating.boundingBox();assert(Math.abs(moved.x-initial.x+100)<2);assert(Math.abs(moved.y-initial.y-35)<2);
  assert.equal(await displayedEnd(),beforeMove,'Przesuwanie okna nie przesuwa czasu');
  const resize=floating.getByRole('button',{name:'Zmień rozmiar okna wykresów',exact:true}),rb=await resize.boundingBox();
  await page.mouse.move(rb.x+rb.width/2,rb.y+rb.height/2);await page.mouse.down();await page.mouse.move(rb.x+rb.width/2-180,rb.y+rb.height/2-160,{steps:8});await page.mouse.up();
  const smaller=await floating.boundingBox();assert(Math.abs(smaller.width-moved.width+180)<2);assert(Math.abs(smaller.height-moved.height+160)<2);
  await resize.focus();await page.keyboard.press('ArrowRight');
  const resized=await floating.boundingBox();assert.equal(resized.width,smaller.width+10);
  await floating.locator('.trace-window-body').evaluate(el=>el.scrollTop=600);
  const controlBox=await floating.locator('.trace-controls').boundingBox(),windowBox=await floating.boundingBox();
  assert(controlBox.y>=windowBox.y&&controlBox.y+controlBox.height<windowBox.y+windowBox.height,'Sterowanie pozostaje na górze');
  const compactCanvas=await floating.locator('canvas').boundingBox();
  await floating.getByRole('button',{name:'Powiększ okno wykresów',exact:true}).click();
  const expanded=await floating.boundingBox();assert.equal(expanded.width,1904);assert.equal(expanded.height,1064);
  const expandedCanvas=await floating.locator('canvas').boundingBox();assert(expandedCanvas.height>compactCanvas.height,'Powiększenie okna zwiększa również sam wykres');
  assert.equal(await floating.locator('.shaft-chart').count(),1,'Maksymalizacja nadal pokazuje tylko przypiętą oś');
  await floating.getByRole('button',{name:'Przywróć rozmiar okna',exact:true}).click();assert.deepEqual(await floating.boundingBox(),resized);
  await floating.getByRole('button',{name:'Wróć do modułu Diagnostyka',exact:true}).click();
  await floating.waitFor({state:'detached'});assert.equal(new URL(page.url()).hash,'#diagnostics');
  assert.equal(await module.locator('.shaft-chart').count(),6);assert.equal(traceConnections,1);
  assert.equal(await module.getByLabel('J1 minimum Y',{exact:true}).inputValue(),'-20');
  await module.getByRole('button',{name:'Przypnij wykres J1',exact:true}).click();await floating.waitFor();
  assert.deepEqual(await floating.boundingBox(),resized,'Geometria pozostaje zapamiętana');
  const floatingCanvas=floating.locator('canvas');await floatingCanvas.scrollIntoViewIfNeeded();
  const fc=await floatingCanvas.boundingBox(),oldEnd=await displayedEnd();
  await page.mouse.move(fc.x+fc.width*.5,fc.y+60);await page.mouse.down();await page.mouse.move(fc.x+fc.width*.5+45,fc.y+60,{steps:8});await page.mouse.up();
  assert((await displayedEnd())<oldEnd,'Wykres nadal przewija historię');assert.deepEqual(await floating.boundingBox(),resized);
  await floating.getByRole('button',{name:'Na żywo',exact:true}).click();
  await page.locator('.sidebar nav a[href="#overview"]').click();assert(await floating.isVisible());
  await page.setViewportSize({width:760,height:600});
  await page.waitForFunction(()=>{const r=document.querySelector('.trace-window').getBoundingClientRect();return r.x>=8&&r.y>=8&&r.right<=innerWidth-8&&r.bottom<=innerHeight-8;});
  assert.equal(await floating.locator('.shaft-chart').count(),1);
  await floating.getByRole('button',{name:'Zamknij pływające okno',exact:true}).click();await floating.waitFor({state:'detached'});
  assert.equal(new URL(page.url()).hash,'#overview');
  await page.setViewportSize({width:1920,height:1080});await page.locator('.sidebar a[href="#diagnostics"]').click();
  await module.waitFor();
  assert.equal(await module.locator('.shaft-chart').count(),6,'Zamknięty wykres wraca do Diagnostyki');
  await module.getByRole('button',{name:'Przypnij wykres J6',exact:true}).click();await floating.waitFor();
  assert.equal(await floating.getAttribute('aria-label'),'Przypięty wykres J6');assert.equal(await module.locator('.shaft-chart').count(),5);
  assert.equal(await floating.locator('.shaft-value').textContent(),'—','Można przypiąć także oś bez danych');
  await page.screenshot({path:'/tmp/robot-shaft-single-floating.png',fullPage:true});
  await floating.getByRole('button',{name:'Zamknij pływające okno',exact:true}).click();await floating.waitFor({state:'detached'});assert.equal(await module.locator('.shaft-chart').count(),6);
  if(!pinningOnly) {
  await page.reload();await page.getByRole('heading',{name:'Diagnostyka',exact:true}).waitFor();
  assert.equal(await page.getByRole('spinbutton',{name:'J1 minimum Y',exact:true}).inputValue(),'-20');
  assert.equal(await page.getByRole('spinbutton',{name:'J1 maksimum Y',exact:true}).inputValue(),'150');
  await page.setViewportSize({width:760,height:1000});assert.equal(await page.locator('.shaft-chart').count(),6);
  assert(await page.getByRole('img',{name:'Przebieg pozycji wału J6',exact:true}).isVisible());
  }
  assert.deepEqual(writes,[]);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',scope:pinningOnly?'pinning-only':'diagnostics',checks:['jedna wybrana oś w oknie i pięć w module','zamiana przypiętej osi bez duplikatów','zachowanie skali i wspólnego czasu','nawigacja bez ponownego WS','przesuwanie/rozmiar/maksymalizacja pojedynczego wykresu','sterowanie u góry','powrót sześciu osi po zamknięciu','przypięcie osi bez danych','brak poleceń sprzętowych']}));
 } finally {clearInterval(timer);await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
