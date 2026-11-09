// Wyłącznie atrapy HTTP/WebSocket; bez połączenia z robotem.
const {chromium}=require('playwright');
const {execFileSync}=require('child_process');
const path=require('path'),assert=require('assert/strict');
const dist=path.resolve(__dirname,'../dist');
const schema=path.resolve(__dirname,'../../ros2_ws/src/robot_configuration');
const empty=JSON.parse(execFileSync('python3',['-c','import sys,json;sys.path.insert(0,sys.argv[1]);from robot_configuration.schema import empty_document;print(json.dumps(empty_document()))',schema]));
const clone=value=>JSON.parse(JSON.stringify(value));
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1366,height:768}});
 const writes=[],errors=[];
 empty.values['joint.1.lower']=-90;empty.values['joint.1.upper']=90;
 let status={active:{revision:'initial',document:empty},operation:{id:'',phase:'idle',message:'Konfiguracja gotowa',success:true}};
 let system={revision:'settings',ignore_missing_drivers:false,allow_missing_motion:false};
 await context.routeWebSocket('ws://robot.test/**',()=>{});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),method=request.method();
  const respond=(value,code=200)=>route.fulfill({status:code,contentType:'application/json',body:JSON.stringify(value)});
  if(url.pathname.startsWith('/api/')){
   if(method!=='GET')writes.push({path:url.pathname,method,body:request.postDataJSON()});
   if(url.pathname==='/api/v1/configuration/system'){
    if(method==='PUT')system={...system,...request.postDataJSON()};return respond(system);
   }
   if(url.pathname==='/api/v1/configuration')return respond(status);
   if(url.pathname==='/api/v1/configuration/snapshots')return respond({items:[],next_cursor:null});
   if(url.pathname==='/api/v1/configuration/apply'){
    const data=request.postDataJSON();assert.equal(data.expected_revision,status.active.revision);
    const accepted={...clone(status),operation:{id:'system-save',phase:'preparing',message:'Zapisywanie',success:null}};
    status=accepted;
    setTimeout(()=>{status={active:{revision:'saved',document:clone(data.document)},operation:{id:'system-save',phase:'done',message:'Zapisano',success:true}};},100);
    return respond(accepted,202);
   }
   return respond({detail:'Atrapa: brak źródła'},503);
  }
  assert.equal(method,'GET');
  const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);
  assert(file.startsWith(dist+path.sep));return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try{
  const page=await context.newPage();page.on('pageerror',error=>errors.push(error.message));
  await page.goto('http://robot.test/#configuration');
  const panel=page.getByRole('tabpanel');
  assert.equal(await panel.getByRole('switch').count(),3);
  assert.equal(await panel.getByRole('radio').count(),2);
  const help=page.getByRole('button',{name:'Opis: Sposób wysyłania PTP/LIN',exact:true});
  await help.focus();await page.getByRole('tooltip').waitFor();
  assert((await page.getByRole('tooltip').innerText()).includes('JOG zachowuje obecne działanie'));
  await page.screenshot({path:'/tmp/system-settings-tooltip.png'});
  await help.press('Escape');await page.getByRole('tooltip').waitFor({state:'hidden'});
  await page.getByRole('switch',{name:'Ignoruj brak sterowników — tylko planowanie',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('#system-ignore-missing')?.checked);
  await page.getByRole('switch',{name:'Zezwalaj na ruch bez wszystkich silników',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('#system-allow-missing')?.checked);
  assert.equal(writes.filter(w=>w.path.endsWith('/system')).length,2);
  assert.equal(writes.filter(w=>w.path.endsWith('/apply')).length,0);
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  const slider=page.getByRole('slider',{name:'Widok danych maszynowych'});await slider.focus();await slider.press('End');
  const lower=page.getByRole('spinbutton',{name:'Dolny limit przegubu J1 [°]',exact:true});await lower.fill('999');
  assert.equal(await lower.getAttribute('aria-invalid'),'true');
  await slider.press('Home');
  const transient=page.getByRole('spinbutton',{name:'Translacja X J2 [mm]',exact:true});await transient.fill('100');await transient.focus();await transient.press('ControlOrMeta+a');await transient.press('-');
  assert.equal(await transient.getAttribute('aria-invalid'),'true');
  await page.getByRole('tab',{name:/Ustawienia systemowe/}).click();
  await page.getByRole('switch',{name:'Wyłącz ENABLE wszystkich silników przy awarii krytycznej',exact:true}).uncheck();
  await page.getByRole('radio',{name:'Synchroniczny',exact:true}).check();
  assert.equal(writes.filter(w=>w.path.endsWith('/apply')).length,0);
  const save=page.getByRole('button',{name:'Zapisz ustawienia do ROS',exact:true});assert(await save.isEnabled());await save.click();
  await page.waitForFunction(()=>document.querySelector('#system-dispatch')?.closest('.system-setting')?.dataset.pending==='false');
  const applied=writes.find(w=>w.path.endsWith('/apply')).body.document.values;
  assert.equal(applied['joint.1.lower'],-90);assert.equal(applied['joint.2.origin_x'],null);
  assert.equal(applied['safety.disable_enable_on_critical_fault'],false);assert.equal(applied['motion.ptp_lin_dispatch_mode'],'synchronized');
  assert.equal(writes.filter(w=>w.path.endsWith('/apply')).length,1);
  await page.screenshot({path:'/tmp/system-settings-1366.png'});
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  await page.getByRole('slider',{name:'Widok danych maszynowych'}).press('End');
  assert.equal(await lower.inputValue(),'999');assert.equal(await lower.getAttribute('aria-invalid'),'true');
  await page.getByRole('slider',{name:'Widok danych maszynowych'}).press('Home');
  assert.equal(await transient.getAttribute('aria-invalid'),'true','Zapis opcji zachowuje walidację ukrytego szkicu');
  assert.equal(await page.getByRole('switch').count(),0,'Opcje globalne występują tylko w Ustawieniach systemowych');
  await page.getByRole('tab',{name:/Ustawienia systemowe/}).click();
  for(const width of [1280,1366,1920]){
   await page.setViewportSize({width,height:768});
   assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),`Poziome przepełnienie przy ${width}`);
   const rows=await panel.locator('.system-setting').count();assert.equal(rows,4);
  }
  assert.deepEqual(errors,[]);console.log(JSON.stringify({result:'OK',checks:['cztery globalne opcje','tooltip na fokus i Escape','niezależny zapis natychmiastowy','zapis tylko opcji profilu','zachowanie ukrytych zmian i walidacji geometrii','trzy szerokości bez przepełnienia','brak poleceń sprzętowych']}));
 }finally{await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
