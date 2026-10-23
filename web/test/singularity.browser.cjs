// Wyłącznie wizualizacja osobliwości w rzeczywistym WebGL; bez aplikacji, ROS i CAN.
const {chromium}=require('playwright');
const {build}=require('esbuild');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
(async()=>{
 const root=path.resolve(__dirname,'..'),errors=[],unexpected=[];
 const bundle=await build({stdin:{resolveDir:root,sourcefile:'singularity-fixture.ts',contents:`
  import {createPreviewScene} from './src/robotPreviewScene';
  import {robotSingularity} from './src/previewSingularity';
  import URDFLoader from 'urdf-loader';
  const xml=await (await fetch('/fixture.urdf')).text();
  window.frame={angles:Array(6).fill(0),valid:Array(6).fill(false),active:0,limits:Array.from({length:6},()=>[]),planning:false,plannedAngles:Array(6).fill(0)};
  window.scene=createPreviewScene(document.getElementById('scene'),xml,()=>window.frame);
  // Dobór pozy wyłącznie z zatwierdzonego URDF do sprawdzenia prezentacji każdego stanu.
  window.poseFor=(level,source)=>{
   const model=new URDFLoader().parse(xml);
   for(const j2 of [0,.4,-.5]) for(const j3 of [0,.5,-.7,-Math.PI/2,Math.PI/2]) for(const j5 of [0,Math.PI/12,Math.PI/4,Math.PI/2]) {
    const values=[0,j2,j3,.2,j5,0];values.forEach((q,i)=>model.setJointValue('joint_'+(i+1),q));model.updateMatrixWorld(true);
    const value=robotSingularity(model);if(value?.level===level&&(!source||value.source===source))return values;
   }
   throw Error('Brak pozy testowej '+level+'/'+source);
  };
 `},bundle:true,write:false,format:'esm',platform:'browser',target:'es2022'});
 const browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1100,height:800},reducedMotion:'no-preference'});
 await context.routeWebSocket('**/*',socket=>{unexpected.push(socket.url());socket.close();});
 await context.route('http://preview.test/**',route=>{
  const request=route.request(),url=new URL(request.url());
  if(request.method()!=='GET'){unexpected.push(request.method()+' '+url.pathname);return route.abort();}
  if(url.pathname==='/')return route.fulfill({contentType:'text/html',body:'<!doctype html><html lang="pl"><head><link rel="stylesheet" href="/style.css"></head><body><div id="scene" class="preview-canvas" style="width:1000px;height:700px;margin:40px;overflow:hidden"></div><script type="module" src="/fixture.js"></script></body></html>'});
  if(url.pathname==='/fixture.js')return route.fulfill({contentType:'application/javascript',body:bundle.outputFiles[0].text});
  if(url.pathname==='/style.css')return route.fulfill({contentType:'text/css',body:fs.readFileSync(path.join(root,'src/style.css'),'utf8')});
  if(url.pathname==='/fixture.urdf')return route.fulfill({contentType:'application/xml',body:fs.readFileSync(path.join(root,'../ros2_ws/src/robot_description/urdf/preview_6dof.example.urdf'),'utf8')});
  unexpected.push(url.pathname);return route.abort();
 });
 try {
  const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));await page.goto('http://preview.test/');
  const canvas=page.locator('#scene > canvas');await canvas.waitFor();
  await page.waitForFunction(()=>document.querySelector('#scene > canvas').dataset.singularities);
  const indicator=subject=>page.locator(`.preview-singularity[data-subject="${subject}"]`);
  const snapshot=async subject=>JSON.parse(await canvas.getAttribute('data-singularities')).find(value=>value.subject===subject);
  const waitState=(subject,level,anchor)=>page.waitForFunction(({subject,level,anchor})=>{
   const value=JSON.parse(document.querySelector('#scene > canvas').dataset.singularities).find(value=>value.subject===subject);
   return value.level===level&&value.anchor===anchor;
  },{subject,level,anchor});
  assert(await indicator('robot').evaluate(el=>el.hidden),'Brak pomiarów nie tworzy alarmu');
  await page.evaluate(()=>window.frame.valid.fill(true));await waitState('robot','critical','joint_5');
  assert.deepEqual((await snapshot('robot')).position.map(value=>Math.round(value*1e6)/1e6),[.3,0,.45],'Łuna jest w J5, a nie TCP (.44, 0, .45)');
  await indicator('robot').locator('.preview-singularity-bubble').waitFor({state:'visible'});
  assert((await indicator('robot').textContent()).includes('nadgarstek J4–J6'));
  async function matchesJoint(subject,jointName) {
   if(subject!=='robot')return;
   const result=await page.evaluate(jointName=>{
    const marker=document.querySelector('.preview-singularity[data-subject="robot"]');
    const joint=[...document.querySelectorAll('.preview-joint-label')].find(el=>el.textContent===jointName);
    return Math.hypot(parseFloat(marker.style.left)-parseFloat(joint.style.left),parseFloat(marker.style.top)-parseFloat(joint.style.top));
   },jointName);assert(result<.1,'Dymek i łuna są zakotwiczone przy tej samej osi');
  }
  await matchesJoint('robot','J5');
  const before=await canvas.screenshot(),opacity=(await snapshot('robot')).opacity;
  await page.waitForFunction(old=>JSON.parse(document.querySelector('#scene > canvas').dataset.singularities)[0].opacity!==old,opacity);
  let changed=false;for(let i=0;i<4&&!changed;i++){await page.waitForTimeout(170);changed=!before.equals(await canvas.screenshot());}
  assert(changed,'Łuna pulsuje także w WebGL przy nieruchomej pozie');
  await page.evaluate(()=>window.scene.view('top'));await page.waitForTimeout(120);await matchesJoint('robot','J5');
  const warning=await page.evaluate(()=>window.poseFor('warning','wrist'));
  await page.evaluate(values=>window.frame.angles=values,warning);await waitState('robot','warning','joint_5');
  assert.equal(await indicator('robot').evaluate(el=>getComputedStyle(el).getPropertyValue('--singularity-border').trim()),'#d5a33c');
  const arm=await page.evaluate(()=>window.poseFor('critical','arm'));
  await page.evaluate(values=>window.frame.angles=values,arm);await waitState('robot','critical','joint_3');await matchesJoint('robot','J3');
  assert((await indicator('robot').textContent()).includes('ramię'));
  assert.equal(await indicator('robot').evaluate(el=>getComputedStyle(el).getPropertyValue('--singularity-border').trim()),'#ff6379');
  const clear=await page.evaluate(()=>window.poseFor('clear'));
  await page.evaluate(values=>window.frame.angles=values,clear);await waitState('robot','clear',null);assert(await indicator('robot').evaluate(el=>el.hidden));
  await page.evaluate(()=>{window.frame.valid.fill(false);window.frame.planning=true;window.frame.plannedAngles.fill(0);window.frame.motionAngles=Array(6).fill(0);});
  await waitState('target','critical','joint_5');await waitState('motion','critical','joint_5');
  assert(await indicator('robot').evaluate(el=>el.hidden),'Nieaktualne pomiary ukrywają alarm robota');
  await indicator('target').locator('.preview-singularity-bubble').waitFor({state:'visible'});
  await indicator('motion').locator('.preview-singularity-bubble').waitFor({state:'visible'});
  assert.equal(await indicator('target').evaluate(el=>el.style.left),await indicator('motion').evaluate(el=>el.style.left));
  assert.equal(await indicator('target').evaluate(el=>el.style.top),await indicator('motion').evaluate(el=>el.style.top),'Dymki mają wspólny punkt osi, a osobne wysięgniki');
  await page.emulateMedia({reducedMotion:'reduce'});await page.waitForTimeout(180);
  const reduced=await snapshot('target');await page.waitForTimeout(240);assert.equal((await snapshot('target')).opacity,reduced.opacity,'Ograniczenie animacji wyłącza puls');
  await page.screenshot({path:'/tmp/robot-singularity-glow.png',fullPage:true});
  await page.evaluate(()=>window.frame.showMotionFrames=false);await page.waitForFunction(()=>document.querySelector('.preview-singularity[data-subject="motion"]').hidden);
  await page.evaluate(()=>{window.frame.plannedAngles[0]=null;});await page.waitForFunction(()=>document.querySelector('.preview-singularity[data-subject="target"]').hidden);
  await page.evaluate(()=>window.scene.dispose());assert.equal(await page.locator('#scene > *').count(),0);
  assert.deepEqual(errors,[]);assert.deepEqual(unexpected,[]);
  console.log(JSON.stringify({result:'OK',scope:'singularity-visualization',checks:['łuna w J5/J3 zamiast TCP','dymek zakotwiczony przy osi i po zmianie kamery','pulsowanie nieruchomej sceny w WebGL','żółty/czerwony stan i ukrycie poza strefą','cel/ruch bez fikcyjnych pomiarów robota','brak nakładania punktów kotwiczenia','ograniczenie animacji','ukrycie niepełnej pozy i sprzątanie zasobów','brak API/WS i poleceń sprzętowych']}));
 } finally {await context.close();await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
