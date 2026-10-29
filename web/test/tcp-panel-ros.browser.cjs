// Kliknięcia panelu -> produkcyjny backend WS -> typ JogTcp. Atrapa nie otwiera CAN.
// Wymaga środowiska ROS z wygenerowanym robot_control_interfaces.
const {chromium}=require('playwright'),{spawn}=require('node:child_process');
const path=require('node:path'),assert=require('node:assert/strict');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const root=path.resolve(__dirname,'../..'),pkg=path.join(root,'ros2_ws/src/robot_web_bridge');
const dist=process.env.ROBOT_WEB_TEST_DIST||path.join(root,'web/dist');
const server=spawn('python3',[path.join(pkg,'test/panel_tcp_fixture.py'),dist,path.join(root,'ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf')],
 {env:{...process.env,PYTHONPATH:pkg+path.delimiter+(process.env.PYTHONPATH||'')},stdio:['ignore','pipe','pipe']});
let errors='',output='';server.stderr.on('data',d=>errors+=d);
(async()=>{
 let browser;
 try {
  const port=await new Promise((resolve,reject)=>{
   const timer=setTimeout(()=>reject(Error('Brak fixture: '+errors)),10000);
   server.once('exit',()=>{clearTimeout(timer);reject(Error('Fixture zakończona: '+errors));});
   server.stdout.on('data',d=>{output+=d;const line=output.split('\n').find(l=>l.startsWith('{"port":'));if(line){clearTimeout(timer);resolve(JSON.parse(line).port);}});
  });
  const base=`http://127.0.0.1:${port}`;
  for(let i=0;i<100;i++){try{if((await fetch(base+'/healthz')).ok)break;}catch{}await delay(50);}
  browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),pageErrors=[];
  page.on('pageerror',e=>pageErrors.push(e.message));
  await page.goto(base+'/#manual');
  const x=page.getByRole('button',{name:'TCP X plus',exact:true});await x.waitFor();
  await page.waitForFunction(()=>!document.querySelector('[aria-label="TCP X plus"]').disabled);
  await page.getByRole('spinbutton',{name:'Prędkość przesuwu TCP',exact:true}).fill('2');
  const commands=async()=>{const response=await fetch(base+'/test/requests');assert(response.ok,'Brak danych atrapy');return response.json();};
  const press=async()=>{await x.hover();await page.mouse.down();await delay(300);};
  const release=async()=>{await page.mouse.up();await page.waitForFunction(()=>!document.querySelector('[aria-label="Tryb TCP"]').disabled);};
  await press();let sent=await commands();
  assert.equal(sent.filter(c=>c.action===0).length,1);assert(sent.filter(c=>c.action===1).length>=2);
  assert.equal(sent.filter(c=>c.action===2).length,0,'Panel nie może wysłać STOP podczas poprawnego przytrzymania');
  assert.equal(sent[0].step,0);assert.equal(sent[0].step_type,'float');assert.equal(sent[0].velocity_type,'float');
  await release();sent=await commands();assert(sent.some(c=>c.action===2&&!c.emergency));
  await page.getByLabel('Tryb TCP').selectOption('step');await press();await release();
  sent=await commands();assert(sent.some(c=>c.action===0&&c.impulse&&c.step===.001));
  await page.getByLabel('Tryb TCP').selectOption('continuous');await x.focus();await page.keyboard.down('Space');await delay(200);await page.keyboard.up('Space');
  await page.waitForFunction(()=>!document.querySelector('[aria-label="Tryb TCP"]').disabled);
  sent=await commands();assert.equal(sent.filter(c=>c.action===0).length,3);
  await press();await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await page.mouse.up();
  await page.getByLabel('Aktualne problemy').getByText('Właściciel sterowania zgłosił awarię: Awaryjne przerwanie JOG TCP przez panel/most',{exact:true}).waitFor();
  const count=(await commands()).filter(c=>c.action===0).length;await delay(300);assert.equal((await commands()).filter(c=>c.action===0).length,count);
  assert.deepEqual(pageErrors,[]);assert(!errors.includes('AssertionError'),errors);
  await page.screenshot({path:'/tmp/tcp-panel-ros.png',fullPage:true});
  console.log(JSON.stringify({result:'OK',checks:['kliknięcia rzeczywistego panelu','produkcyjny WebSocket backendu','rzeczywisty typ ROS JogTcp','ciągły step=0 bez rozłączenia','podtrzymanie i puszczenie','tryb impulsowy','klawiatura','blur zatrzymuje','widoczna przyczyna awarii','brak automatycznego wznowienia','bez CAN i fizycznego ruchu']}));
 } finally {if(browser)await browser.close();server.kill('SIGTERM');}
})().catch(e=>{console.error(e);process.exitCode=1;});
