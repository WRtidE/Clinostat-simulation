const assert = require('node:assert/strict');
const { GimbalSimulation, DirectionCoverage, reduceCurvePoints, linearGravityAxis, wrap } = require('./simulation.js');
const near = (a, b, tolerance = 1e-8) => assert.ok(Math.abs(a-b)<tolerance, `${a} != ${b}`);
const uniform = new GimbalSimulation();
uniform.uniform=[1,2];uniform.update(10);
near(uniform.angles[0],85);near(uniform.angles[1],88);near(uniform.time,10);
uniform.uniform=[-10,0];uniform.update(2);near(uniform.angles[0],-35);near(uniform.angles[1],88);
near(wrap(-35),325);near(Math.hypot(...uniform.direction()),1);
uniform.angles=[90,0];uniform.direction().forEach((v,i)=>near(v,[0,-1,0][i]));
uniform.angles=[0,90];uniform.direction().forEach((v,i)=>near(v,[1,0,0][i]));
uniform.angles=[30,45];uniform.direction().forEach((v,i)=>near(v,[Math.SQRT1_2,-Math.SQRT1_2/2,Math.sqrt(3)*Math.SQRT1_2/2][i]));

// Fixed random targets allow a closed-form acceleration/arrival reference.
const ramp=new GimbalSimulation();ramp.setMode('random');ramp.reset();
ramp.angles=[0,0];ramp.targets=[.02,-.04];
const arrivals=[0,0];
ramp.chooseTarget=i=>{arrivals[i]++;ramp.targets[i]=i===0?-.01:.03;};
ramp.update(1);near(ramp.velocities[0],.01);near(ramp.velocities[1],-.01);
near(ramp.angles[0],.03);near(ramp.angles[1],-.03);
ramp.update(1);near(ramp.velocities[0],.02);near(ramp.velocities[1],-.02);
near(ramp.angles[0],.12);near(ramp.angles[1],-.12);
assert.deepEqual(arrivals,[1,0]);assert.deepEqual(ramp.targets,[-.01,-.04]);
ramp.update(.5);near(ramp.velocities[0],.015);near(ramp.velocities[1],-.025);
near(ramp.angles[0],.1725);near(ramp.angles[1],-.1875);

const a=new GimbalSimulation(),b=new GimbalSimulation();
for(const s of [a,b]){s.limits=[.1,.15];s.setMode('random');s.reset();}
for(let n=0;n<4000;n++){
  const before=[...a.velocities];a.update(.025);b.update(.025);
  a.velocities.forEach((v,i)=>{assert.ok(Math.abs(v)<=a.limits[i]+1e-8);assert.ok(Math.abs(v-before[i])<=.01*.025+1e-8);near(v,b.velocities[i]);});
  near(Math.hypot(...a.gravity()),1);
}
a.angles.forEach((v,i)=>near(v,b.angles[i]));
const expected=[...a.angles],expectedMean=a.meanGravity(),expectedCoverage=Array.from(a.coverage.bins);
a.reset();a.update(100);a.angles.forEach((v,i)=>near(v,expected[i],1e-7));
a.meanGravity().forEach((v,i)=>near(v,expectedMean[i],1e-8));
assert.deepEqual(Array.from(a.coverage.bins),expectedCoverage,'Fixed-time coverage must not depend on update step size');
a.seed=43;a.reset();a.update(100);assert.notDeepEqual(a.angles,expected);
const switchMode=new GimbalSimulation();switchMode.update(1);const current=[...switchMode.velocities];switchMode.setMode('random');assert.deepEqual(switchMode.velocities,current);
switchMode.update(1);switchMode.velocities.forEach((v,i)=>near(Math.abs(v-current[i]),.01));
switchMode.uniform=[4,-8];switchMode.setMode('uniform');assert.deepEqual(switchMode.velocities,[4,-8]);

const stopped=new GimbalSimulation();stopped.uniform=[0,0];
// Under tilted single-axis rotation the complete-cycle mean is analytical.
for(const target of [0,.001,.165,.378,.75,1]){
  const partial=new GimbalSimulation();const tilt=partial.configurePartialGravity(target);
  near(Math.cos(tilt*Math.PI/180),target);partial.update(30);
  partial.meanGravity().forEach((v,i)=>near(v,[0,-target,0][i],1e-9));near(partial.residualGravity(),target);
  near(Math.hypot(...partial.gravity()),1);partial.reset();near(partial.angles[0],tilt);
}
const earthCoverage=new GimbalSimulation(),moonCoverage=new GimbalSimulation();moonCoverage.setEnvironmentGravity(.165);
// Independent forward rotation must recover the same downward world field.
for(const magnitude of [.165,1,2])for(const angles of [[0,0],[80.503,120],[-43,297],[90,-90]]){
  const frameCheck=new GimbalSimulation();frameCheck.environmentGravity=magnitude;frameCheck.angles=angles;
  const v=frameCheck.gravity(),a=angles[0]*Math.PI/180,b=angles[1]*Math.PI/180;
  const inner=[v[0]*Math.cos(b)+v[2]*Math.sin(b),v[1],-v[0]*Math.sin(b)+v[2]*Math.cos(b)];
  const world=[inner[0],inner[1]*Math.cos(a)-inner[2]*Math.sin(a),inner[1]*Math.sin(a)+inner[2]*Math.cos(a)];
  world.forEach((component,i)=>near(component,frameCheck.worldGravity()[i]));
}
earthCoverage.update(60);moonCoverage.update(60);
assert.deepEqual(Array.from(moonCoverage.coverage.bins),Array.from(earthCoverage.coverage.bins),'Coverage uses unit direction, independently of gravity magnitude');
near(Math.hypot(...moonCoverage.gravity()),.165);
const highGravity=new GimbalSimulation();highGravity.setEnvironmentGravity(2);highGravity.configurePartialGravity(.378);highGravity.update(30);near(highGravity.residualGravity(),.378);
assert.throws(()=>highGravity.configurePartialGravity(3));assert.throws(()=>highGravity.configurePartialGravity(.1,0));assert.throws(()=>highGravity.setEnvironmentGravity(0));
assert.ok(linearGravityAxis([{time:1,residual:2}],0,2,true,{ceiling:2,target:.1}).upper>=2);
assert.ok(linearGravityAxis([{time:1,residual:4}],0,2,true,{ceiling:2,target:.1}).upper>=4);
assert.equal(stopped.meanGravity(),null);assert.equal(stopped.residualGravity(),null);
stopped.update(600);near(stopped.residualGravity(),1);
assert.equal(stopped.coverage.visitedCount,1);
assert.equal(stopped.coverage.sampleCount,12000);
stopped.meanGravity().forEach((v,i)=>near(v,stopped.gravity()[i]));
const quarter=new GimbalSimulation();quarter.angles=[0,0];quarter.uniform=[1,0];quarter.update(15);
quarter.meanGravity().forEach((v,i)=>near(v,[0,-2/Math.PI,2/Math.PI][i]));
const periodic=new GimbalSimulation();periodic.uniform=[1,2];periodic.update(60);
assert.ok(periodic.residualGravity()<1e-10,'Integer full rotations cancel the ideal cumulative gravity vector');
const locked=new GimbalSimulation();locked.angles=[0,0];locked.uniform=[1,1];locked.update(60);
near(locked.residualGravity(),.5); // Both axes turning alone does not guarantee cancellation.
locked.reset();assert.deepEqual(locked.gravityIntegral,[0,0,0]);assert.equal(locked.residualGravity(),null);
assert.equal(locked.coverage.percent,0);assert.equal(locked.coverage.sampleCount,0);

const allDirections=new DirectionCoverage();
for(let row=0;row<allDirections.rows;row++)for(let col=0;col<allDirections.columns;col++){
  const y=-1+(row+.5)*2/allDirections.rows,longitude=-Math.PI+(col+.5)*2*Math.PI/allDirections.columns,r=Math.sqrt(1-y*y);
  allDirections.observe([r*Math.cos(longitude),y,r*Math.sin(longitude)]);
}
assert.equal(allDirections.visitedCount,648);assert.equal(allDirections.percent,100);
const repeated=new DirectionCoverage();for(let n=0;n<100;n++)repeated.observe([0,1,0]);
assert.equal(repeated.visitedCount,1);assert.equal(repeated.sampleCount,100);
assert.equal(repeated.index([-1,0,0]),repeated.index([-1,0,-0]),'The longitude seam is the same direction');
const whole=new GimbalSimulation(),partitioned=new GimbalSimulation();
whole.update(60);for(let n=0;n<6000;n++)partitioned.update(.01);
assert.deepEqual(Array.from(whole.coverage.bins),Array.from(partitioned.coverage.bins));
const adjustable=new GimbalSimulation();adjustable.setMode('random');adjustable.reset();adjustable.targets=[1,-1];
adjustable.acceleration=.02;adjustable.update(1);near(adjustable.velocities[0],.02);near(adjustable.velocities[1],-.02);
adjustable.acceleration=.1;adjustable.update(1);near(adjustable.velocities[0],.12);near(adjustable.velocities[1],-.12);
assert.throws(()=>a.update(NaN));assert.throws(()=>a.update(-1));
const dense=Array.from({length:10000},(_,i)=>({time:i,residual:i<20?Math.exp(-i/4):.005+.002*Math.sin(i/19)}));
dense[1234].residual=.9;dense[1250].residual=.000001;
const reduced=reduceCurvePoints(dense,0,9999,100);
assert.equal(reduced[0],dense[0]);assert.equal(reduced[reduced.length-1],dense[dense.length-1]);
assert.ok(reduced.includes(dense[1234]));assert.ok(reduced.includes(dense[1250]));
assert.ok(reduced[1].time<100,'Early descent must remain in the first horizontal pixel');
assert.ok(reduced.length<=400);assert.ok(reduced.every(point=>dense.includes(point)));
for(let i=1;i<reduced.length;i++)assert.ok(reduced[i].time>reduced[i-1].time);
const detail=reduceCurvePoints(dense,1200,1300,500);assert.ok(detail.length>=101);assert.equal(detail[detail.length-1].time,1300);
const vectorPoints=Array.from({length:1000},(_,i)=>({time:i,mean:[0,0,0]}));vectorPoints[103].mean[1]=1;vectorPoints[110].mean[1]=-1;
const vectorReduced=reduceCurvePoints(vectorPoints,0,999,10,p=>p.mean[1]);assert.ok(vectorReduced.includes(vectorPoints[103]));assert.ok(vectorReduced.includes(vectorPoints[110]));
assert.deepEqual(linearGravityAxis(dense,0,9999),{upper:1,step:.2,decimals:1});
const smallAxis=linearGravityAxis([{time:1,residual:.0041}],0,2,true);
assert.ok(smallAxis.upper>.0041&&smallAxis.upper<.01);near(smallAxis.upper/smallAxis.step,Math.round(smallAxis.upper/smallAxis.step));
const zeroAxis=linearGravityAxis([{time:1,residual:0}],0,2,true);assert.ok(zeroAxis.upper>.001);
console.log('Passed: motion and gravity integration, configurable acceleration, coverage, extrema-preserving curve reduction, early descent, authentic data points and linear auto axes.');
