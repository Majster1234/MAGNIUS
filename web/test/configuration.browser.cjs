// Konfiguracja jest lokalnym szkicem; test nie łączy się z backendem ani CAN.
const {chromium}=require('playwright');
const path=require('path');
const assert=require('assert/strict');
const dist=path.resolve(__dirname,'../dist');
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1366,height:768}});
 const errors=[],apiWrites=[];
 context.on('page',page=>page.on('pageerror',e=>errors.push(e.message)));
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url());
  if(url.pathname.startsWith('/api/v1/configuration'))return route.fulfill({status:503,contentType:'application/json',body:JSON.stringify({detail:'Węzeł konfiguracji wyłączony w teście szkicu lokalnego'})});
  if(request.method()!=='GET'){apiWrites.push(url.pathname);return route.abort();}
  const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);
  assert(file.startsWith(dist+path.sep));
  await route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.svg')?'image/svg+xml':'text/html'});
 });
 await context.routeWebSocket('ws://robot.test/**',()=>{});
 try{
  const page=await context.newPage();await page.goto('http://robot.test/#configuration');
  await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  assert(await page.getByRole('link',{name:'Konfiguracja',exact:true}).evaluate(el=>el.classList.contains('selected')));
  assert.equal(await page.getByRole('tab',{name:/Ustawienia systemowe/}).getAttribute('aria-selected'),'true');
  assert.equal(await page.getByRole('tabpanel').locator('input,select').count(),1,'Ustawienia systemowe zawierają tylko suwak planowania');
  assert(await page.getByRole('switch').isDisabled());
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  const slider=page.getByRole('slider',{name:'Widok danych maszynowych'});
  const number=(label,axis,unit)=>page.getByRole('spinbutton',{name:`${label} J${axis} [${unit}]`,exact:true});
  const geometry=()=>page.getByRole('table',{name:'Geometria osi',exact:true});
  const limits=()=>page.getByRole('table',{name:'Przełożenia i ograniczenia osi',exact:true});
  assert.equal(await geometry().locator('tbody tr').count(),6);
  assert.equal(await geometry().getByRole('columnheader').count(),7);
  for(let axis=1;axis<=6;axis++){
   assert(await geometry().getByRole('columnheader',{name:`J${axis} CAN ${axis}`,exact:true}).isVisible());
   assert.equal(await number('Translacja X',axis,'mm').inputValue(),'');
  }
  await number('Translacja X',1,'mm').fill('0.25');
  await number('Translacja X',2,'mm').fill('0.5');
  await number('Translacja X',4,'mm').fill('0');
  await number('Rotacja Z (Yaw)',6,'°').fill('0.1');
  await slider.focus();await slider.press('ArrowRight');
  assert.equal(await slider.getAttribute('aria-valuetext'),'Przełożenia i ograniczenia');
  assert.equal(await limits().locator('tbody tr').count(),7);
  assert.equal(await number('Przełożenie silnik / przegub',1,':1').inputValue(),'');
  await number('Przełożenie silnik / przegub',1,':1').fill('50');
  await number('Przełożenie silnik / przegub',2,':1').fill('80');
  await number('Dolny limit przegubu',4,'°').fill('-1');
  await page.getByRole('combobox',{name:'Kierunek osi J3',exact:true}).selectOption('-1');
  await slider.press('ArrowLeft');
  assert.equal(await number('Translacja X',1,'mm').inputValue(),'0.25');
  assert.equal(await number('Translacja X',2,'mm').inputValue(),'0.5');
  assert.equal(await number('Translacja X',4,'mm').inputValue(),'0');
  await page.getByText('Narzędzie i TCP',{exact:true}).click();
  assert.equal(await page.getByLabel('Masa narzędzia').inputValue(),'');
  await page.getByLabel('Nazwa narzędzia',{exact:true}).fill('Chwytak');
  await page.getByLabel('Masa narzędzia').fill('1.2');
  await page.getByText('Narzędzie i TCP',{exact:true}).click();
  await page.getByRole('link',{name:'Przegląd systemu',exact:true}).click();
  await page.getByRole('link',{name:'Konfiguracja',exact:true}).click();
  // Stary szkic formularza pozostaje zgodny z nową tabelą; usunięte ustawienia znikają.
  await page.evaluate(()=>{
   const key='robot.configuration.draft.v2',draft=JSON.parse(sessionStorage.getItem(key));
   draft['system.port']='8080';draft['joint.6.gear_ratio']='123';sessionStorage.setItem(key,JSON.stringify(draft));
  });
  await page.reload();await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  assert.equal(await page.getByRole('tabpanel').locator('input,select').count(),1);
  await page.waitForFunction(()=>!Object.keys(JSON.parse(sessionStorage.getItem('robot.configuration.draft.v2'))).some(key=>key.startsWith('system.')));
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  assert.equal(await number('Translacja X',1,'mm').inputValue(),'0.25');
  assert.equal(await number('Rotacja Z (Yaw)',6,'°').inputValue(),'0.1');
  await slider.focus();await slider.press('ArrowRight');
  assert.equal(await number('Przełożenie silnik / przegub',1,':1').inputValue(),'50');
  assert.equal(await number('Przełożenie silnik / przegub',2,':1').inputValue(),'80');
  assert.equal(await number('Przełożenie silnik / przegub',6,':1').inputValue(),'123');
  assert.equal(await page.getByRole('combobox',{name:'Kierunek osi J3',exact:true}).inputValue(),'-1');
  await page.getByText('Narzędzie i TCP',{exact:true}).click();
  assert.equal(await page.getByLabel('Nazwa narzędzia',{exact:true}).inputValue(),'Chwytak');
  assert.equal(await page.getByLabel('Masa narzędzia').inputValue(),'1.2');
  await page.getByText('Narzędzie i TCP',{exact:true}).click();
  const layouts=[];
  for(const viewport of [{width:1280,height:800},{width:1366,height:768},{width:1920,height:1080}]){
   await page.setViewportSize(viewport);
   for(const key of ['Home','End']){
    await slider.press(key);
    const panel=await page.getByRole('region',{name:'Dane maszynowe',exact:true}).boundingBox();
    assert(panel.height<=(viewport.height-panel.y)/2,'Tabela z suwakiem ma zajmować najwyżej połowę wolnej wysokości');
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'Tabela musi mieścić się w szerokości ekranu');
    assert(await page.locator('.machine-table-scroll').evaluate(el=>el.scrollWidth<=el.clientWidth),'Sześć osi musi być widoczne bez przewijania w poziomie');
    layouts.push({...viewport,view:await slider.getAttribute('aria-valuetext'),panel_height:panel.height});
   }
  }
  await page.setViewportSize({width:1600,height:1000});
  await page.screenshot({path:'/tmp/robot-configuration-desktop.png',fullPage:true});
  // Wcześniejszy szkic SI musi zachować znaczenie, także dla narzędzia i limitów.
  await page.evaluate(()=>{
   const draft={'robot.name':'Robot rad','tool.name':'Chwytak','tool.mass':'1.2',
    'tool.tcp_x':'0.1','tool.tcp_y':'-0.05','tool.tcp_z':'0',
    'tool.tcp_roll':String(Math.PI/2),'tool.tcp_pitch':String(-Math.PI/4),'tool.tcp_yaw':'0',
    'tool.com_x':'0.001','tool.com_y':'-0.002','tool.com_z':'0'};
   for(let axis=1;axis<=6;axis++){
    const prefix=`joint.${axis}.`;
    for(const [key,value] of Object.entries({origin_x:axis*.001,origin_y:-axis*.002,origin_z:0,
     origin_roll:Math.PI/2,origin_pitch:0,origin_yaw:-Math.PI/4,zero_offset:Math.PI/6,
     lower:-Math.PI,upper:Math.PI,velocity:Math.PI/2,acceleration:Math.PI/4,gear_ratio:axis+50,direction:-1})){
     draft[prefix+key]=String(value);
    }
   }
   sessionStorage.removeItem('robot.configuration.draft.v2');
   sessionStorage.setItem('robot.configuration.draft.v1',JSON.stringify(draft));
  });
  await page.reload();await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  async function near(field,expected){assert(Math.abs(Number(await field.inputValue())-expected)<1e-10);}
  for(let axis=1;axis<=6;axis++){
   await near(number('Translacja X',axis,'mm'),axis);
   await near(number('Translacja Y',axis,'mm'),-2*axis);
   assert.equal(await number('Translacja Z',axis,'mm').inputValue(),'0');
   await near(number('Rotacja X (Roll)',axis,'°'),90);
   await near(number('Rotacja Z (Yaw)',axis,'°'),-45);
  }
  await slider.focus();await slider.press('End');
  for(let axis=1;axis<=6;axis++){
   await near(number('Offset zera przegubu',axis,'°'),30);
   await near(number('Dolny limit przegubu',axis,'°'),-180);
   await near(number('Górny limit przegubu',axis,'°'),180);
   await near(number('Maksymalna prędkość',axis,'°/s'),90);
   await near(number('Maksymalne przyspieszenie',axis,'°/s²'),45);
   assert.equal(await number('Przełożenie silnik / przegub',axis,':1').inputValue(),String(axis+50));
   assert.equal(await page.getByRole('combobox',{name:`Kierunek osi J${axis}`,exact:true}).inputValue(),'-1');
  }
  await page.getByText('Narzędzie i TCP',{exact:true}).click();
  for(const [name,unit,expected] of [['TCP X','mm',100],['TCP Y','mm',-50],['TCP Roll','°',90],['TCP Pitch','°',-45],['Środek masy X','mm',1],['Środek masy Y','mm',-2]]){
   await near(page.getByLabel(`${name}${unit}`,{exact:true}),expected);
  }
  assert.equal(await page.getByLabel('Masa narzędzia').inputValue(),'1.2');
  assert.equal(await page.getByLabel('Układ narzędzia',{exact:false}).inputValue(),'');
  await page.waitForFunction(()=>sessionStorage.getItem('robot.configuration.draft.v2')!==null&&sessionStorage.getItem('robot.configuration.draft.v1')===null);
  await page.reload();await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  await near(number('Translacja X',1,'mm'),1);
  await near(number('Rotacja X (Roll)',1,'°'),90);
  await slider.focus();await slider.press('End');
  await number('Górny limit przegubu',1,'°').fill('360');
  await page.reload();await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  await slider.focus();await slider.press('End');
  assert.equal(await number('Górny limit przegubu',1,'°').inputValue(),'360');
  await page.evaluate(()=>sessionStorage.setItem('robot.configuration.draft.v2','{uszkodzony JSON'));
  await page.reload();await page.getByRole('heading',{name:'Konfiguracja',exact:true}).waitFor();
  assert.equal(await page.getByRole('tabpanel').locator('input,select').count(),1);
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  assert.equal(await number('Translacja X',1,'mm').inputValue(),'');
  assert.deepEqual(apiWrites,[]);assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'OK',layouts,checks:['empty system settings','parameters in rows and six axes in columns','geometry/limits slider','blank unknown parameters','independent joints','tool fields collapsed','compatible draft survives navigation and reload','SI draft migrates to mm and degrees once','angular velocity and acceleration','tool TCP and center of mass conversion','mass ratio and direction unchanged','new values survive reload without conversion','corrupt draft recovery','half-height budget','no backend writes']}));
 }finally{await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
