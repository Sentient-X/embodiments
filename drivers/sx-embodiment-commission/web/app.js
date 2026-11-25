import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import URDFLoader from 'urdf-loader';
const $=id=>document.getElementById(id);
let robot, modelLoaded=false, latest=null, originalMaterials=new Map(), receivedStatus=null, receivedAt=0;
const jointOld=(status,name,stale)=>stale||(Number.isFinite(status.ages_s?.[name])&&status.ages_s[name]>1.5);
const scene=new THREE.Scene();
scene.background=new THREE.Color('#f2f5f3');
const camera=new THREE.PerspectiveCamera(38,1,.01,50);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
$('viewer').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
const reset=()=>{camera.position.set(1.15,-1.3,1.05);controls.target.set(0,0,.45);controls.update()};reset();$('reset').onclick=reset;
scene.add(new THREE.HemisphereLight(0xffffff,0x8a9b92,2.4));
const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(3,-4,5);scene.add(light);
const grid=new THREE.GridHelper(4,20,0xbdc9c1,0xdce3de);grid.rotation.x=Math.PI/2;scene.add(grid);
new ResizeObserver(()=>{const w=$('viewer').clientWidth,h=$('viewer').clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix()}).observe($('viewer'));
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera)});
function applyPose(){
 if(!robot||!latest)return;
 for(const [name,value] of Object.entries(latest.positions||{}))if(Number.isFinite(value)&&robot.joints[name])robot.setJointValue(name,value);
 const stale=Boolean(latest.stale)||(Number.isFinite(latest.age_s)?latest.age_s>1.5:!latest.updated_at_unix_ms||Date.now()-latest.updated_at_unix_ms>1500);
 robot.traverse(node=>{if(!node.isMesh)return;let link=node.parent;while(link&&!link.isURDFLink)link=link.parent;
 const joint=link?.parent;const name=joint?.name;const value=latest.positions?.[name], range=latest.ranges?.[name];
 const bad=Number.isFinite(value)&&range&&(value<range.lower||value>range.upper);
 if(!originalMaterials.has(node))originalMaterials.set(node,node.material);
 if(bad){if(!node.userData.red)node.userData.red=new THREE.MeshStandardMaterial({color:0xcc5c43,roughness:.65});node.material=node.userData.red}
 else if(jointOld(latest,name,stale)||(name in (latest.ranges||{})&&!Number.isFinite(value))){if(!node.userData.grey)node.userData.grey=new THREE.MeshStandardMaterial({color:0xa6b0aa,roughness:.8});node.material=node.userData.grey}
 else node.material=originalMaterials.get(node);
 });
}
async function loadModel(){try{
 const [xml,gltf]=await Promise.all([fetch('/model/robot.urdf').then(r=>{if(!r.ok)throw Error('Robot description unavailable');return r.text()}),new GLTFLoader().loadAsync('/model/pack.glb')]);
 const loader=new URDFLoader();loader.loadMeshCb=(_path,_manager,done)=>done(new THREE.Group());
 robot=loader.parse(xml);robot.ignoreLimits=true;
 // Reuse the platform's render pack: its mesh nodes are already in each named link frame.
 for(const name of Object.keys(robot.links)){const visual=gltf.scene.getObjectByName(name);if(visual)robot.links[name].add(visual.clone(true))}
 robot.traverse(n=>{if(n.isURDFJoint)n.ignoreLimits=true});scene.add(robot);modelLoaded=true;$('model-progress').textContent='Platform render pack · local assets';applyPose();
}catch(error){$('model-progress').textContent=error.message;$('notice').textContent='Robot model failed to load: '+error.message;$('notice').className='alert'}}
const degrees=v=>(v*180/Math.PI).toFixed(1)+'°';
function show(status){latest=status;let badCount=0;const positions=status.positions||{},ranges=status.ranges||{};
 const age=!Object.values(positions).some(Number.isFinite)?null:Number.isFinite(status.age_s)?Math.max(0,status.age_s):status.updated_at_unix_ms?Math.max(0,(Date.now()-status.updated_at_unix_ms)/1000):null;const stale=Boolean(status.stale)||age===null||age>1.5;
 $('mode').textContent=status.mode||'Idle';$('age').textContent=age===null?'No sample':stale?'Retained · '+age.toFixed(1)+' s old':'Recent · '+age.toFixed(1)+' s';
 const missing=Object.keys(ranges).filter(n=>!Number.isFinite(positions[n])).length;
 $('model-label').textContent=age===null?'Robot model · reference pose':stale?'Robot model · retained encoder pose':'Robot model · encoder pose'+(missing?' · '+missing+' unmeasured':'');
 let retainedCount=0;
 const body=$('joint-rows');body.replaceChildren();
 for(const name of [...new Set([...Object.keys(ranges),...Object.keys(positions)])].sort()){
 const q=positions[name],r=ranges[name],known=Number.isFinite(q),bad=known&&r&&(q<r.lower||q>r.upper);if(bad)badCount++;const retained=jointOld(status,name,stale);if(known&&retained)retainedCount++;
 const tr=document.createElement('tr');tr.className=bad?'bad':!known||retained?'unknown':'';
 for(const value of [name,known?degrees(q):'Not measured',r?degrees(r.lower)+' … '+degrees(r.upper):'Unavailable',bad?(retained?'Outside range · retained':'Outside range'):!known?'Not measured':retained?'Retained':'In range']){const td=document.createElement('td');td.textContent=value;tr.append(td)}body.append(tr);
 }
 if(!stale&&retainedCount){$('age').textContent='Partial · '+retainedCount+' retained';$('model-label').textContent='Robot model · mixed-age encoder pose'}
 $('outside').textContent=String(badCount);$('outside').style.color=badCount?'#ae4930':'';
 $('notice').className=status.error||badCount?'alert':'';
 $('notice').textContent=status.error||(badCount?`${badCount} sampled joint${badCount===1?' is':'s are'} outside the configured command range. Check the selected side’s encoder reference and physical pose. Readings on the isolated side do not block teaching.`:stale?'Read sensors for a current pose, or run teach mode with telemetry. The reference model is not a measured state.':'Joint feedback received. Engagement stays on the local keyboard.');
 $('refresh').disabled=String(status.mode).toLowerCase()==='observing';applyPose();
}
$('refresh').onclick=async()=>{try{const r=await fetch('/api/refresh',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});const d=await r.json();if(!r.ok)throw Error(d.error||'Sensor read refused');$('refresh').disabled=true}catch(e){$('notice').textContent=e.message;$('notice').className='alert'}};
async function poll(){
 try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error('Local status unavailable');receivedStatus=await r.json();receivedAt=performance.now();show(receivedStatus)}
 catch(e){if(receivedStatus){const elapsed=(performance.now()-receivedAt)/1000;show({...receivedStatus,stale:true,age_s:Number.isFinite(receivedStatus.age_s)?receivedStatus.age_s+elapsed:null,error:e.message})}else{$('notice').textContent=e.message;$('notice').className='alert'}}
 finally{setTimeout(poll,300)}
}
loadModel();poll();
