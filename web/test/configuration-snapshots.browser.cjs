// Rzeczywiste WebGL i atrapiony współdzielony serwer; brak połączeń z CAN.
const {chromium}=require('playwright');
const {execFileSync}=require('child_process');
const path=require('path'),assert=require('assert/strict'),fs=require('fs');
const dist=path.resolve(process.env.ROBOT_WEB_DIST || path.join(__dirname,'../dist'));
const schema=path.resolve(__dirname,'../../ros2_ws/src/robot_configuration');
const empty=JSON.parse(execFileSync('python3',['-c','import sys,json;sys.path.insert(0,sys.argv[1]);from robot_configuration.schema import empty_document;print(json.dumps(empty_document()))',schema]));
const clone=value=>JSON.parse(JSON.stringify(value));
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1366,height:768},acceptDownloads:true});
 const errors=[],writes=[],snapshots=new Map();let revision=0, snapshotNumber=0;
 let status={active:{revision:'initial',document:empty},operation:{id:'',phase:'idle',message:'Konfiguracja gotowa',success:true}};
 context.on('page',page=>page.on('pageerror',error=>errors.push(error.message)));
 await context.routeWebSocket('ws://robot.test/**',()=>{});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),method=request.method();
  const respond=(value,code=200)=>route.fulfill({status:code,contentType:'application/json',body:JSON.stringify(value)});
  if(url.pathname.startsWith('/api/v1/configuration')){
   if(method!=='GET')writes.push({path:url.pathname,body:request.postData()?request.postDataJSON():null});
   if(url.pathname==='/api/v1/configuration')return respond(status);
   if(url.pathname==='/api/v1/configuration/validate')return respond(request.postDataJSON().document);
   if(url.pathname==='/api/v1/configuration/snapshots'){
    if(method==='GET')return respond({items:[...snapshots].map(([id,document])=>({id,name:document.name,created_at:document.created_at})),next_cursor:null});
    const document=clone(request.postDataJSON().document),id='snapshot-'+(++snapshotNumber);snapshots.set(id,document);
    return respond({id,name:document.name,created_at:document.created_at},201);
   }
   if(url.pathname.startsWith('/api/v1/configuration/snapshots/')){
    const id=url.pathname.split('/').at(-1);if(method==='DELETE'){snapshots.delete(id);return respond({deleted:true});}return respond(snapshots.get(id));
   }
   if(url.pathname.endsWith('/apply')){
    const data=request.postDataJSON();
    if(data.expected_revision!==status.active.revision)return respond({detail:'Konflikt rewizji'},409);
    const id='apply-'+(++revision), document=clone(data.document);
    status.operation={id,phase:'preparing',message:'Sprawdzanie konfiguracji',success:null};
    const accepted=clone(status);
    setTimeout(()=>{status={active:{revision:id,document},operation:{id,phase:'done',message:'Konfiguracja zapisana',success:true}};},250);
    return respond(accepted,202);
   }
   return respond({detail:'Nieznany endpoint'},404);
  }
  assert.equal(method,'GET','Test konfiguracji nie może wysyłać poleceń do napędów');
  const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);
  assert(file.startsWith(dist+path.sep));return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 try{
  const page=await context.newPage();await page.goto('http://robot.test/#configuration');
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  const field=(label,axis,unit)=>page.getByRole('spinbutton',{name:`${label} J${axis} [${unit}]`,exact:true});
  const slider=page.getByRole('slider',{name:'Widok danych maszynowych'});
  const canvas=page.locator('.configuration-preview canvas');await canvas.waitFor();
  await page.waitForFunction(()=>!!document.querySelector('.configuration-preview canvas')?.dataset.jointOrigins);
  assert(await canvas.evaluate(el=>!!el.getContext('webgl2')));
  assert.equal(await field('Translacja X',1,'mm').inputValue(),'');
  await page.getByRole('button',{name:'Wczytaj przykład',exact:true}).click();
  assert.equal(await field('Translacja Z',3,'mm').inputValue(),'250');
  assert.equal(await field('Translacja Z',3,'mm').getAttribute('data-dirty'),'true');
  await page.getByRole('textbox',{name:'Nazwa snapshotu'}).fill('Przykład startowy');
  await page.getByRole('button',{name:'Zapisz snapshot',exact:true}).click();
  await page.waitForFunction(()=>!!document.querySelector('select[aria-label="Zapisany snapshot"]')?.value);
  assert.equal(snapshots.size,1);assert.equal(writes.filter(item=>item.path.endsWith('/apply')).length,0);
  assert.equal(await field('Translacja Z',3,'mm').getAttribute('data-dirty'),'true','Snapshot nie zatwierdza konfiguracji ROS');
  await field('Translacja X',2,'mm').fill('100');
  await field('Rotacja Z (Yaw)',1,'°').fill('90');
  await page.waitForFunction(()=>{const origins=JSON.parse(document.querySelector('.configuration-preview canvas').dataset.jointOrigins);return Math.abs(origins[1][1]-.1)<1e-8 && Math.abs(origins[1][0])<1e-8;});
  const origins=JSON.parse(await canvas.getAttribute('data-joint-origins'));
  assert(Math.abs(origins[2][1]-.1)<1e-8,'Zmiana rodzica transformuje dalszy łańcuch');
  const x2=field('Translacja X',2,'mm');await x2.focus();await x2.press('ControlOrMeta+a');await x2.press('-');
  assert.equal(await x2.getAttribute('aria-invalid'),'true');
  assert.deepEqual(JSON.parse(await canvas.getAttribute('data-joint-origins')),origins,'Niepoprawna liczba zachowuje ostatnią geometrię');
  await x2.fill('100');
  await slider.focus();await slider.press('End');
  await field('Przełożenie silnik / przegub',1,':1').fill('50');
  await field('Dolny limit przegubu',1,'°').fill('-720');await field('Górny limit przegubu',1,'°').fill('1080');
  await page.locator('.configuration-bubble').filter({hasText:'J1 · 50:1 · -720°…1080° (5 obr.)'}).waitFor();
  await field('Dolny limit przegubu',1,'°').fill('1100');
  assert.equal(await field('Dolny limit przegubu',1,'°').getAttribute('aria-invalid'),'true');
  assert(await page.getByRole('button',{name:'Zapisz do ROS',exact:true}).isDisabled());
  assert(await page.locator('.configuration-bubble').filter({hasText:'J1 · 50:1 · -720°…1080° (5 obr.)'}).isVisible(),'Nieprawidłowe granice zachowują poprzedni podgląd');
  await field('Dolny limit przegubu',1,'°').fill('-720');
  await page.getByRole('button',{name:'Zapisz do ROS',exact:true}).click();
  assert.equal(await field('Dolny limit przegubu',1,'°').getAttribute('data-dirty'),'true');
  await page.waitForFunction(()=>document.querySelector('input[aria-label="Dolny limit przegubu J1 [°]"]')?.dataset.dirty==='false');
  await field('Dolny limit przegubu',1,'°').fill('-719');assert.equal(await field('Dolny limit przegubu',1,'°').getAttribute('data-dirty'),'true');
  await field('Dolny limit przegubu',1,'°').fill('-720.0');assert.equal(await field('Dolny limit przegubu',1,'°').getAttribute('data-dirty'),'false');
  const downloadPromise=page.waitForEvent('download');await page.getByRole('button',{name:'Eksportuj JSON',exact:true}).click();
  const download=await downloadPromise;await download.saveAs('/tmp/robot-configuration-export.json');
  const exported=JSON.parse(fs.readFileSync('/tmp/robot-configuration-export.json','utf8'));
  assert.equal(exported.units,'mm-deg-kg');assert.equal(exported.values['joint.1.gear_ratio'],50);assert.equal(exported.values['joint.1.direction'],null);
  const imported=clone(exported);imported.values['joint.2.origin_x']=160;
  await field('Dolny limit przegubu',1,'°').fill('-710');
  page.once('dialog',dialog=>dialog.dismiss());
  await page.locator('input[type=file]').setInputFiles({name:'snapshot.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(imported))});await delay(150);
  assert.equal(await field('Dolny limit przegubu',1,'°').inputValue(),'-710');
  page.once('dialog',dialog=>dialog.accept());
  await page.locator('input[type=file]').setInputFiles({name:'snapshot.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(imported))});
  await field('Dolny limit przegubu',1,'°').evaluate(el=>new Promise(resolve=>{const timer=setInterval(()=>{if(el.value==='-720'){clearInterval(timer);resolve();}},20);}));
  await slider.focus();await slider.press('Home');
  assert.equal(await field('Translacja X',2,'mm').inputValue(),'160');assert.equal(await field('Translacja X',2,'mm').getAttribute('data-dirty'),'true');
  const applyCount=writes.filter(item=>item.path.endsWith('/apply')).length;
  await page.locator('input[type=file]').setInputFiles({name:'bad.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify({...imported,schema_version:99}))});
  await page.getByRole('log').getByText('Nieobsługiwana wersja lub jednostki snapshotu',{exact:true}).waitFor();
  assert.equal(await field('Translacja X',2,'mm').inputValue(),'160');
  assert.equal(writes.filter(item=>item.path.endsWith('/apply')).length,applyCount,'Import nie stosuje konfiguracji');
  page.once('dialog',dialog=>dialog.accept());await page.getByRole('button',{name:'Wczytaj',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('input[aria-label="Translacja X J2 [mm]"]')?.value==='0');
  assert.equal(await field('Rotacja Z (Yaw)',1,'°').getAttribute('data-dirty'),'true');
  const another=await context.newPage();await another.goto('http://robot.test/#configuration');await another.getByRole('tab',{name:/Parametry robota/}).click();
  const anotherField=another.getByRole('spinbutton',{name:'Translacja X J2 [mm]',exact:true});
  await delay(150);await anotherField.fill('200');
  await page.getByRole('button',{name:'Zapisz do ROS',exact:true}).click();
  await another.getByText('Konflikt rewizji',{exact:true}).waitFor();
  assert.equal(await anotherField.inputValue(),'200');assert(await another.getByRole('button',{name:'Zapisz do ROS',exact:true}).isDisabled());
  another.once('dialog',dialog=>dialog.accept());await another.getByRole('button',{name:'Wczytaj aktywną',exact:true}).click();
  assert.equal(await anotherField.inputValue(),'0');
  const layouts=[];
  for(const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]){
   await page.setViewportSize(viewport);const table=await page.locator('.machine-panel').boundingBox(),model=await page.locator('.configuration-preview').boundingBox();
   assert(model.y>=table.y+table.height);assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
   layouts.push({...viewport,table_height:table.height,model_height:model.height});
  }
  await page.screenshot({path:'/tmp/robot-configuration-snapshots.png',fullPage:true});
  page.once('dialog',dialog=>dialog.accept());await page.getByRole('button',{name:'Usuń',exact:true}).click();await delay(100);assert.equal(snapshots.size,0);
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',layouts,checks:['shared JSON snapshots','snapshot preserves dirty fields','import/export and discard confirmation','revision conflict preserves draft','ROS apply clears highlights only after confirmation','WebGL chain transforms and RPY','joint limits and multi-turn bubbles','invalid inputs preserve scene','local model without CAN','empty system settings preserved']}));
 }finally{await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
