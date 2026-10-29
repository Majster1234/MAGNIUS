// Rzeczywista przeglądarka, atrapiony właściciel konfiguracji, bez ROS/CAN.
const {chromium}=require('playwright');
const {execFileSync}=require('child_process');
const path=require('path'),assert=require('assert/strict');
const dist=path.resolve(process.env.ROBOT_WEB_DIST || path.join(__dirname,'../dist'));
const schema=path.resolve(__dirname,'../../ros2_ws/src/robot_configuration');
const document=JSON.parse(execFileSync('python3',['-c','import sys,json;sys.path.insert(0,sys.argv[1]);from robot_configuration.schema import empty_document;print(json.dumps(empty_document()))',schema]));
for(let a=1;a<=6;a++)for(const [k,v] of Object.entries({gear_ratio:1,direction:1,zero_offset:0,lower:-180,upper:180,velocity:30,acceleration:1000}))document.values[`joint.${a}.${k}`]=v;
const clone=x=>JSON.parse(JSON.stringify(x));
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({acceptDownloads:true});const writes=[],errors=[];
 let revision=0,status={active:{revision:'start',document},calibration_invalid:[false,true,false,false,false,false],operation:{id:'',phase:'idle',message:'Gotowe',success:true}};
 await context.routeWebSocket('ws://robot.test/**',()=>{});
 await context.route('http://robot.test/**',async route=>{
  const request=route.request(),url=new URL(request.url()),method=request.method();
  const respond=(body,code=200)=>route.fulfill({status:code,contentType:'application/json',body:JSON.stringify(body)});
  if(url.pathname==='/api/v1/configuration')return respond(status);
  if(url.pathname==='/api/v1/configuration/system')return respond({revision:'s',ignore_missing_drivers:false,allow_missing_motion:false});
  if(url.pathname==='/api/v1/configuration/snapshots')return respond({items:[],next_cursor:null});
  if(url.pathname==='/api/v1/configuration/apply'){
   const data=request.postDataJSON();writes.push(data);assert.equal(data.expected_revision,status.active.revision);
   const id='operation-'+(++revision),next=clone(data.document),invalid=clone(status.calibration_invalid);
   if(data.confirm_calibration_axes.length){for(const a of data.confirm_calibration_axes)invalid[a-1]=false;}
   else next.values['joint.1.zero_offset']=60;
   status.operation={id,phase:'preparing',message:'Trwa zapis',success:null};const response=clone(status);
   setTimeout(()=>{status={active:{revision:id,document:next},calibration_invalid:invalid,operation:{id,phase:'done',message:'Zapisano',success:true}};},600);
   return respond(response,202);
  }
  if(url.pathname.startsWith('/api/')){assert.equal(method,'GET');return respond({},503);}
  assert.equal(method,'GET','Brak komend ruchu');
  const name=url.pathname==='/'?'index.html':url.pathname.slice(1),file=path.resolve(dist,name);assert(file.startsWith(dist+path.sep));
  return route.fulfill({path:file,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html'});
 });
 try {
  const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));await page.goto('http://robot.test/#configuration');
  await page.getByRole('tab',{name:/Parametry robota/}).click();
  const slider=page.getByRole('slider',{name:'Widok danych maszynowych'});await slider.focus();await slider.press('End');
  const ratio=page.getByRole('spinbutton',{name:'Przełożenie silnik / przegub J1 [:1]',exact:true});
  const offset=page.getByRole('spinbutton',{name:'Offset zera przegubu J1 [°]',exact:true});
  const speed=page.getByRole('spinbutton',{name:'Maksymalna prędkość J3 [°/s]',exact:true});
  await page.waitForFunction(()=>document.querySelector('input[aria-label="Przełożenie silnik / przegub J1 [:1]"]')?.value==='1');
  await offset.fill('0.0');await ratio.fill('3');await page.getByRole('button',{name:'Zapisz do ROS',exact:true}).click();await speed.fill('31');
  await page.waitForFunction(()=>document.querySelector('input[aria-label="Offset zera przegubu J1 [°]"]')?.value==='60');
  assert.equal(await ratio.getAttribute('data-dirty'),'false');assert.equal(await offset.getAttribute('data-dirty'),'false');
  assert.equal(await speed.inputValue(),'31');assert.equal(await speed.getAttribute('data-dirty'),'true');
  assert.deepEqual(writes[0].confirm_calibration_axes,[]);assert.equal(writes[0].document.values['joint.1.zero_offset'],0);
  await page.getByRole('checkbox',{name:'Potwierdzam odniesienie J2',exact:true}).check();
  await page.getByRole('button',{name:'Potwierdź odniesienie wybranych osi'}).click();
  await page.getByRole('checkbox',{name:'Potwierdzam odniesienie J2',exact:true}).waitFor({state:'detached'});
  assert.deepEqual(writes[1].confirm_calibration_axes,[2]);assert.equal(writes[1].document.values['joint.3.velocity'],30);
  assert.equal(writes[1].document.values['joint.1.zero_offset'],60);assert.equal(await speed.inputValue(),'31');
  assert.equal(writes.length,2);assert.deepEqual(errors,[]);
  console.log('Configuration continuity: resolved offset, concurrent edits and selective confirmation OK; no CAN');
 } finally {await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
