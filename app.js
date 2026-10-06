(() => {
  'use strict';
  const { GimbalSimulation, reduceCurvePoints, linearGravityAxis, RAD, wrap } = GimbalPhysics;
  const sim = new GimbalSimulation();
  const $ = id => document.getElementById(id);
  const scene = $('scene'), ctx = scene.getContext('2d');
  const chart = $('speedChart'), chartCtx = chart.getContext('2d');
  const gravityChart = $('gravityChart'), gravityCtx = gravityChart.getContext('2d');
  const coverageChart=$('coverageChart'),coverageCtx=coverageChart.getContext('2d');
  const coverageMap=$('coverageMap'),coverageMapCtx=coverageMap.getContext('2d');
  const coverageSphere=$('coverageSphere'),sphereCtx=coverageSphere.getContext('2d');
  let sphereWidth=0,sphereHeight=0,sphereCamera={yaw:.65,pitch:.25},spherePointer=null;
  const instantChart=$('instantComponentsChart'),instantCtx=instantChart.getContext('2d');
  const meanChart=$('meanComponentsChart'),meanCtx=meanChart.getContext('2d');
  let running = true, timeScale = 1, previousFrame = null, lastSample = 0;
  let history = [], trail = [], camera = { yaw: .58, pitch: .27, zoom: 1 };
  let width = 0, height = 0, chartWidth = 0, chartHeight = 0, dirtyChart = true;
  let pointer = null;
  let gravityWidth = 0, gravityHeight = 0, calculating = false;
  let gravityRecords=[],resultSnapshot=null;
  let accelerationRecords=[],instantWidth=0,instantHeight=0,meanWidth=0,meanHeight=0;
  const componentColors=['#2389a5','#cc6759','#8a9d46'];
  let targetEvaluation='ceiling',targetTolerance=.005,partialApplied=false;
  const gravityNumber=value=>value.toFixed(12).replace(/(\.\d*?[1-9])0+$|\.0+$/,'$1');
  let coverageWidth=0,coverageHeight=0,mapWidth=0,mapHeight=0;
  const colors = { outer: [38, 143, 172], inner: [233, 166, 74], metal: [130, 150, 162], dark: [49, 72, 87], sample: [215, 227, 232] };
  const add = (a, b) => a.map((v, i) => v + b[i]);
  const scale = (v, s) => v.map(x => x * s);
  const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const normalize = v => scale(v, 1 / (Math.hypot(...v) || 1));
  const rotateX = (v, a) => [v[0], v[1] * Math.cos(a) - v[2] * Math.sin(a), v[1] * Math.sin(a) + v[2] * Math.cos(a)];
  const rotateY = (v, a) => [v[0] * Math.cos(a) + v[2] * Math.sin(a), v[1], -v[0] * Math.sin(a) + v[2] * Math.cos(a)];
  const identity = v => v;
  const format = (v, decimals = 1) => (Math.abs(v) < .5 * 10 ** -decimals ? 0 : v).toFixed(decimals).replace('-', '−');
  const signedAngle = a => { const result = wrap(a + 180) - 180; return format(result); };
  const announce = text => { $('announcement').textContent = text; };
  const setPressed = (element, selected) => { element.classList.toggle('selected', selected); element.setAttribute('aria-pressed', String(selected)); };
  function resizeCanvas(canvas, context) {
    const rect = canvas.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    return [rect.width, rect.height];
  }
  const observer = new ResizeObserver(() => {
    [width, height] = resizeCanvas(scene, ctx);
    [chartWidth, chartHeight] = resizeCanvas(chart, chartCtx);
    [gravityWidth, gravityHeight] = resizeCanvas(gravityChart, gravityCtx);
    [coverageWidth,coverageHeight]=resizeCanvas(coverageChart,coverageCtx);
    [mapWidth,mapHeight]=resizeCanvas(coverageMap,coverageMapCtx);
    [sphereWidth,sphereHeight]=resizeCanvas(coverageSphere,sphereCtx);
    [instantWidth,instantHeight]=resizeCanvas(instantChart,instantCtx);
    [meanWidth,meanHeight]=resizeCanvas(meanChart,meanCtx);
    dirtyChart = true;
  });
  observer.observe($('canvasWrap')); observer.observe(chart); observer.observe(gravityChart);observer.observe(coverageChart);observer.observe(coverageMap);
  observer.observe(instantChart);observer.observe(meanChart);
  observer.observe(coverageSphere);

  function viewPoint(v) {
    return rotateX(rotateY(add(v, [0, .37, 0]), camera.yaw), camera.pitch);
  }
  function project(v) {
    const p = viewPoint(v), distance = 9;
    const factor = Math.min(width * .19, height * .205) * camera.zoom * distance / (distance - p[2]);
    return [width / 2 + p[0] * factor, height * .48 - p[1] * factor, p[2]];
  }
  function strokePath(points, color, lineWidth = 1, dash = []) {
    ctx.beginPath(); points.forEach((p, i) => { const q = project(p); if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]); });
    ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
  }
  function worldLabel(point, text, color) {
    const p = project(point); ctx.fillStyle = color; ctx.font = '11px "Segoe UI", "Microsoft YaHei", sans-serif'; ctx.textAlign = 'center'; ctx.fillText(text, p[0], p[1]);
  }
  let faces;
  function face(points, color, transform = identity, outline = false) {
    faces.push({ points: points.map(transform), color, outline });
  }
  function box(center, size, color, transform = identity) {
    const [x, y, z] = center, [sx, sy, sz] = size.map(v => v / 2);
    const p = [[x-sx,y-sy,z-sz],[x+sx,y-sy,z-sz],[x+sx,y+sy,z-sz],[x-sx,y+sy,z-sz],[x-sx,y-sy,z+sz],[x+sx,y-sy,z+sz],[x+sx,y+sy,z+sz],[x-sx,y+sy,z+sz]];
    [[0,3,2,1],[4,5,6,7],[0,4,7,3],[1,2,6,5],[3,7,6,2],[0,1,5,4]].forEach(indices => face(indices.map(i => p[i]), color, transform));
  }
  function ring(radius, thickness, depth, color, transform) {
    const n = 100, r1 = radius - thickness / 2, r2 = radius + thickness / 2, z = depth / 2;
    for (let i = 0; i < n; i++) {
      const a = i * 2 * Math.PI / n, b = (i + 1) * 2 * Math.PI / n;
      const p = (r, angle, zz) => [r * Math.cos(angle), r * Math.sin(angle), zz];
      face([p(r1,a,z),p(r2,a,z),p(r2,b,z),p(r1,b,z)], color, transform);
      face([p(r1,b,-z),p(r2,b,-z),p(r2,a,-z),p(r1,a,-z)], color, transform);
      face([p(r2,a,-z),p(r2,b,-z),p(r2,b,z),p(r2,a,z)], color, transform);
      face([p(r1,b,-z),p(r1,a,-z),p(r1,a,z),p(r1,b,z)], color, transform);
    }
  }
  function cylinder(center, radius, length, color, along = 'x', transform = identity) {
    const n = 24, local = (a, side) => {
      const v = along === 'x' ? [side * length / 2, radius * Math.cos(a), radius * Math.sin(a)] : [radius * Math.cos(a), side * length / 2, radius * Math.sin(a)];
      return add(v, center);
    };
    const cap1 = [], cap2 = [];
    for (let i = 0; i < n; i++) {
      const a = i * 2 * Math.PI / n, b = (i + 1) * 2 * Math.PI / n;
      face([local(a,-1),local(b,-1),local(b,1),local(a,1)], color, transform);
      cap1.push(local(a,-1)); cap2.unshift(local(a,1));
    }
    face(cap1, color, transform); face(cap2, color, transform);
  }
  function paintFaces() {
    const light = normalize([-.35, .8, 1]);
    faces.forEach(f => { f.projected = f.points.map(project); f.depth = f.projected.reduce((a, p) => a + p[2], 0) / f.points.length; });
    faces.sort((a, b) => a.depth - b.depth);
    for (const f of faces) {
      const p = f.projected;
      const normal = normalize(cross(add(f.points[1], scale(f.points[0], -1)), add(f.points[2], scale(f.points[0], -1))));
      const shading = .60 + .40 * Math.max(0, dot(normal, light));
      const color = f.color.map(v => Math.round(v * shading));
      ctx.beginPath(); p.forEach((point, i) => i === 0 ? ctx.moveTo(point[0],point[1]) : ctx.lineTo(point[0],point[1])); ctx.closePath();
      ctx.fillStyle = `rgb(${color.join(',')})`; ctx.fill();
      // A same-color seam prevents hairline gaps between ring segments.
      ctx.strokeStyle = ctx.fillStyle; ctx.lineWidth = .55; ctx.stroke();
    }
  }
  function drawScene() {
    if (!width || !height) return;
    ctx.clearRect(0,0,width,height);
    for (let x = -5; x <= 5; x += .5) strokePath([[x,-2.02,-5],[x,-2.02,5]], '#dce6ec70', .6);
    for (let z = -5; z <= 5; z += .5) strokePath([[-5,-2.02,z],[5,-2.02,z]], '#dce6ec70', .6);
    const shadow = project([0,-1.99,0]);
    ctx.save(); ctx.translate(shadow[0],shadow[1]); ctx.scale(1,.25);
    const gradient = ctx.createRadialGradient(0,0,0,0,0,160 * camera.zoom);
    gradient.addColorStop(0,'#24455425'); gradient.addColorStop(1,'#24455400');
    ctx.fillStyle = gradient; ctx.fillRect(-220*camera.zoom,-220*camera.zoom,440*camera.zoom,440*camera.zoom); ctx.restore();
    const a = wrap(sim.angles[0]) * RAD, b = wrap(sim.angles[1]) * RAD;
    const outer = v => rotateX(v,a), inner = v => outer(rotateY(v,b));
    if ($('showTrail').checked && trail.length > 1) {
      // This sphere records direction, not the translation of the sample center.
      for (let i = 1; i < trail.length; i++) strokePath([scale(trail[i-1],1.53),scale(trail[i],1.53)], `rgba(65,151,137,${.06 + .34 * i / trail.length})`,1.4);
    }
    faces = [];
    box([0,-1.96,0],[4.2,.14,1.55],colors.metal);
    box([0,-1.85,0],[4.06,.12,1.40],colors.sample);
    for (const side of [-1,1]) {
      box([side*1.88,-1.03,0],[.17,1.6,.36],colors.metal);
      box([side*1.88,-1.73,0],[.57,.2,.65],colors.dark);
      cylinder([side*1.88,0,0],.24,.38,colors.dark);
      cylinder([side*1.68,0,0],.12,.24,colors.metal);
    }
    ring(1.55,.17,.16,colors.outer,outer);
    ring(1.15,.14,.14,colors.inner,inner);
    for (const side of [-1,1]) {
      cylinder([0,side*1.33,0],.125,.34,colors.metal,'y',outer);
      cylinder([0,side*1.47,0],.17,.08,colors.dark,'y',outer);
      box([side*.66,0,0],[.90,.10,.10],colors.metal,inner);
    }
    box([0,0,0],[.48,.64,.48],colors.sample,inner);
    box([0,0,.25],[.36,.47,.025],colors.dark,inner);
    box([0,.08,.273],[.23,.22,.025],colors.outer,inner);
    box([0,-.14,.273],[.23,.04,.025],colors.inner,inner);
    paintFaces();
    if ($('showAxes').checked) {
      strokePath([[-2.34,0,0],[2.34,0,0]],'#2389a58c',1,[4,5]);
      strokePath([outer([0,-1.86,0]),outer([0,1.88,0])],'#d69a4699',1,[4,5]);
      worldLabel([2.46,0,0],'X / α','#29829b'); worldLabel(outer([0,2.0,0]),'Y / β','#bc8a43');
      strokePath([[0,0,0],inner([0,0,.82])],'#277e7299',1.5);
      const p = project(inner([0,0,.84])); ctx.beginPath(); ctx.arc(p[0],p[1],3.5,0,Math.PI*2); ctx.fillStyle = '#318f7f'; ctx.fill();
      worldLabel(inner([0,0,1.0]),'样品 +Z','#318f7f');
    }
    // Fixed world triad stays in the corner while the camera orbits.
    const ox = width - 47, oy = height - 62;
    [['X',[1,0,0],'#7c9eaf'],['Y',[0,1,0],'#72a095'],['Z',[0,0,1],'#b5a181']].forEach(([label,v,color]) => {
      const p = rotateX(rotateY(v,camera.yaw),camera.pitch);
      ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox+p[0]*22,oy-p[1]*22); ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.stroke();ctx.fillStyle=color;ctx.font='9px "Segoe UI"';ctx.textAlign='center';ctx.fillText(label,ox+p[0]*30,oy-p[1]*30+3);
    });
  }

  function record() {
    history.push({ time: sim.time, speeds: [...sim.velocities] });
    history = history.filter(p => p.time >= sim.time - 30);
    trail.push(sim.direction()); if (trail.length > 360) trail.shift();
    if(!accelerationRecords.length||sim.time-accelerationRecords[accelerationRecords.length-1].time>=.1-1e-8)accelerationRecords.push({time:sim.time,instant:sim.gravity()});
    if(sim.time > 0 && (!gravityRecords.length || sim.time-gravityRecords[gravityRecords.length-1].time >= .5-1e-8)) {
      gravityRecords.push(captureGravityPoint());
    }
    dirtyChart = true;
  }
  function drawChart() {
    if (!chartWidth) return;
    const c = chartCtx, w = chartWidth, h = chartHeight, left = 35, right = 10, top = 10, bottom = 24;
    const end = Math.max(30,sim.time), start = end-30, plotWidth = w-left-right, plotHeight = h-top-bottom;
    const max = Math.max(.02,...history.flatMap(p => p.speeds.map(Math.abs)),...(sim.mode==='uniform'?sim.uniform.map(Math.abs):[]));
    const magnitude = 10 ** Math.floor(Math.log10(max));
    const bound = Math.ceil(max / magnitude) * magnitude;
    const tickDecimals = Math.max(0, 1 - Math.floor(Math.log10(bound)));
    const px = t => left + (t-start)/30*plotWidth, py = v => top+plotHeight/2-v/bound*plotHeight/2;
    c.clearRect(0,0,w,h); c.font='9px "Segoe UI", "Microsoft YaHei"';c.textAlign='right';c.lineWidth=1;
    for (const value of [-bound,-bound/2,0,bound/2,bound]) {
      const y = py(value); c.beginPath();c.moveTo(left,y);c.lineTo(w-right,y);c.strokeStyle=value===0?'#dce5e9':'#edf1f3';c.setLineDash(value===0?[]:[3,5]);c.stroke();c.setLineDash([]);c.fillStyle='#9fadb5';c.fillText(format(value,tickDecimals),left-7,y+3);
    }
    c.textAlign='center';
    for (let i=0;i<=5;i++) { const t=start+i*6; const x=px(t); c.fillStyle='#a4b2b9';c.fillText(`${Math.round(t)}s`,x,h-7); }
    c.save();c.beginPath();c.rect(left-1,top-2,plotWidth+2,plotHeight+4);c.clip();
    ['#2389a5','#e6a047'].forEach((color,i) => {
      c.beginPath();history.forEach((p,j) => j===0?c.moveTo(px(p.time),py(p.speeds[i])):c.lineTo(px(p.time),py(p.speeds[i])));c.strokeStyle=color;c.lineWidth=2;c.stroke();
      if (history.length) {const p=history[history.length-1];c.beginPath();c.arc(px(p.time),py(p.speeds[i]),3,0,2*Math.PI);c.fillStyle=color;c.fill();}
    });c.restore();
    $('sampleCount').textContent=`${history.length} 个采样点`;dirtyChart=false;
    drawGravityChart();
    drawCoverageChart();drawCoverageMap();drawCoverageSphere();
    drawComponentCharts();
  }
  function drawGravityChart() {
    if(!gravityWidth)return;
    const points=currentGravityPoints();
    const window=Number($('gravityTimeWindow').value),start=window?Math.max(0,sim.time-window):0,end=Math.max(60,sim.time);
    const stats=renderGravityCurve(gravityCtx,gravityWidth,gravityHeight,points,end,{start,automatic:$('gravityAxisScale').value==='auto'});
    $('gravityPlotSummary').textContent=`显示 ${start.toFixed(1)}～${end.toFixed(1)} s · ${stats.sourceCount.toLocaleString('zh-CN')} 个记录点 · 按画面宽度保留真实峰谷`;
  }
  function renderGravityCurve(c,w,h,points,end,options={}) {
    const {start=0,automatic=false,forExport=false,target=sim.targetGravity,environment=sim.environmentGravity,evaluation=targetEvaluation,tolerance=targetTolerance}=options;
    const axis=linearGravityAxis(points,start,end,automatic,{ceiling:environment,target:target+(evaluation==='match'?tolerance:0)});
    const left=forExport?75:Math.max(40,axis.decimals*6+26),right=forExport?25:12,top=14,bottom=forExport?35:25;
    const plotWidth=w-left-right,plotHeight=h-top-bottom;
    const px=t=>left+(t-start)/Math.max(1e-9,end-start)*plotWidth;
    const py=v=>top+(1-Math.min(axis.upper,Math.max(0,v))/axis.upper)*plotHeight;
    c.clearRect(0,0,w,h);if(forExport){c.fillStyle='#fff';c.fillRect(0,0,w,h);}c.font=`${forExport?13:10}px "Segoe UI", "Microsoft YaHei", sans-serif`;c.textAlign='right';c.lineWidth=1;
    const ticks=Math.round(axis.upper/axis.step);
    for(let i=0;i<=ticks;i++){
      const value=(ticks-i)*axis.step,y=py(value);c.beginPath();c.moveTo(left,y);c.lineTo(w-right,y);c.setLineDash([2,5]);c.strokeStyle='#e7eef1';c.stroke();c.setLineDash([]);
      c.fillStyle='#8eA2ad';c.fillText(`${value.toFixed(axis.decimals)}${i===0?' g':''}`,left-7,y+3);
    }
    if(evaluation==='match'){c.fillStyle='#77b4a222';const upper=py(target+tolerance),lower=py(Math.max(0,target-tolerance));c.fillRect(left,upper,plotWidth,lower-upper);}
    const targetY=py(target);c.beginPath();c.moveTo(left,targetY);c.lineTo(w-right,targetY);c.setLineDash([4,4]);c.strokeStyle='#77b4a2';c.stroke();c.setLineDash([]);
    c.fillStyle='#398777';c.fillText(`目标 ${gravityNumber(target)} g${evaluation==='match'?` ± ${gravityNumber(tolerance)} g`:''}`,w-right-4,Math.max(top+11,targetY-6));
    for(let i=0;i<=4;i++){const t=start+(end-start)*i/4;c.textAlign=i===0?'left':i===4?'right':'center';c.fillStyle='#99aab4';c.fillText(forExport?t.toFixed(1):t>=120?`${(t/60).toFixed(1)}min`:`${Math.round(t)}s`,px(t),h-7);}
    c.save();c.beginPath();c.rect(left-1,top-1,plotWidth+2,plotHeight+2);c.clip();
    const reduced=reduceCurvePoints(points,start,end,plotWidth);
    c.beginPath();reduced.forEach((p,i)=>i===0?c.moveTo(px(p.time),py(p.residual)):c.lineTo(px(p.time),py(p.residual)));c.strokeStyle='#278675';c.lineWidth=1.7;c.lineJoin='round';c.stroke();c.restore();
    return {sourceCount:points.filter(p=>p.time>=start&&p.time<=end).length,drawnCount:reduced.length};
  }
  ['gravityTimeWindow','gravityAxisScale'].forEach(id=>$(id).addEventListener('change',()=>{dirtyChart=true;drawGravityChart();}));
  function updateReadouts() {
    $('outerAngle').innerHTML=`${signedAngle(sim.angles[0])}<span>°</span>`;
    $('innerAngle').innerHTML=`${signedAngle(sim.angles[1])}<span>°</span>`;
    $('outerActual').textContent=`${format(sim.velocities[0],3)} rpm`;
    $('innerActual').textContent=`${format(sim.velocities[1],3)} rpm`;
    $('outerTarget').textContent=`当前目标：${format(sim.targets[0],3)} rpm`;
    $('innerTarget').textContent=`当前目标：${format(sim.targets[1],3)} rpm`;
    $('direction').textContent=`(${sim.direction().map(v=>format(v,2)).join(', ')})`;
    const tenth = Math.floor((sim.time+1e-8)*10), minutes = Math.floor(tenth/600);
    $('clock').textContent=`${String(minutes).padStart(2,'0')}:${String(Math.floor(tenth/10)%60).padStart(2,'0')}.${tenth%10}`;
    const mean=sim.meanGravity(),residual=sim.residualGravity();
    $('residualGravity').textContent=residual===null?'— g':residual<1e-6?'< 0.000001 g':`${residual.toFixed(6)} g`;
    $('gravityDuration').textContent=`累计 ${format(sim.time)} s`;
    $('instantGravityVector').textContent=`(${sim.gravity().map(v=>format(v,3)).join(', ')})`;
    $('meanGravityVector').textContent=mean?`(${mean.map(v=>format(v,5)).join(', ')})`:'—';
    $('targetGravityValue').textContent=`${gravityNumber(sim.targetGravity)} g`;
    $('targetGravityRule').textContent=targetEvaluation==='match'?`偏差 ≤ ${gravityNumber(targetTolerance)} g`:`平均矢量 ≤ ${gravityNumber(sim.targetGravity)} g`;
    $('instantGravityMagnitude').textContent=`${gravityNumber(sim.environmentGravity)} g`;
    $('gravityTargetTag').textContent=targetEvaluation==='match'?`平均目标 ${gravityNumber(sim.targetGravity)} ± ${gravityNumber(targetTolerance)} g`:`平均矢量目标 ≤ ${gravityNumber(sim.targetGravity)} g`;
    $('environmentSI').textContent=`${(sim.environmentGravity*9.80665).toFixed(6)} m/s² · 1 g = 9.80665 m/s²`;
    $('targetToleranceControl').hidden=targetEvaluation!=='match';
    const fixedCeiling=Math.max(sim.environmentGravity,sim.targetGravity+(targetEvaluation==='match'?targetTolerance:0));
    $('gravityAxisScale').options[0].textContent=`固定 0～${gravityNumber(fixedCeiling)} g`;
    $('meanAxisScale').options[0].textContent=`固定 ±${gravityNumber(sim.environmentGravity)} g`;
    gravityChart.setAttribute('aria-label',`累计平均重力模长随时间变化，目标 ${gravityNumber(sim.targetGravity)} g，均匀线性纵轴`);
    instantChart.setAttribute('aria-label',`瞬时重力 X、Y、Z 分量，单位g，纵轴 ±${gravityNumber(sim.environmentGravity)} g`);
    const met=residual!==null && (targetEvaluation==='match'?Math.abs(residual-sim.targetGravity)<=targetTolerance:residual<=sim.targetGravity);
    $('gravityStatus').textContent=residual===null?'等待累计':met?'平均指标达标':'平均指标未达标';
    $('gravityStatus').classList.toggle('met',met);
    if(partialApplied){
      const fixed=sim.mode==='uniform'&&Math.abs(sim.uniform[0])<1e-10&&Math.abs(sim.uniform[1])>.000001;
      $('partialPlanSummary').textContent=fixed?`外轴固定 ${format(sim.angles[0],3)}° · 内轴 ${format(sim.uniform[1],3)} rpm · 周期 ${(60/Math.abs(sim.uniform[1])).toFixed(1)} s · 理论周期平均 ${gravityNumber(sim.environmentGravity*Math.abs(Math.cos(sim.angles[0]*RAD)))} g`:'运动参数已改变，当前为自定义方案，请以累计平均曲线评估。';
    }
    $('coverageValue').textContent=`${sim.coverage.percent.toFixed(2)}%`;
    $('coverageCells').textContent=`${sim.coverage.visitedCount} / ${sim.coverage.bins.length}`;
    $('coverageSamples').textContent=sim.coverage.sampleCount.toLocaleString('zh-CN');
    ['viewGravityResult','exportGravityPng','exportGravityCsv','exportComponentsPng'].forEach(id=>$(id).disabled=sim.time<=0||calculating);
  }
  function captureGravityPoint(){
    return {time:sim.time,residual:sim.residualGravity(),mean:sim.meanGravity(),instant:sim.gravity(),coverage:sim.coverage.percent,mode:sim.mode,speeds:[...sim.velocities],limits:[...sim.limits],acceleration:sim.acceleration,seed:sim.seed,environment:sim.environmentGravity,target:sim.targetGravity,evaluation:targetEvaluation,tolerance:targetTolerance};
  }
  function currentGravityPoints(){
    const points=gravityRecords.slice();if(sim.time<=0)return points;
    const current=captureGravityPoint();
    if(points.length&&Math.abs(points[points.length-1].time-current.time)<1e-8)points[points.length-1]=current;
    else points.push(current);
    return points;
  }
  function gravitySnapshot(){
    if(sim.time<=0||calculating)return null;
    const points=currentGravityPoints(),current=points[points.length-1];
    return {points,current,view:{start:Number($('gravityTimeWindow').value)?Math.max(0,current.time-Number($('gravityTimeWindow').value)):0,automatic:$('gravityAxisScale').value==='auto',environment:current.environment,target:current.target,evaluation:current.evaluation,tolerance:current.tolerance}};
  }
  function resultGravityText(residual){return residual<1e-6?'< 0.000001 g':`${residual.toFixed(6)} g`;}
  function resultCanvas(snapshot){
    const canvas=document.createElement('canvas');canvas.width=1800;canvas.height=1120;
    const c=canvas.getContext('2d');c.scale(2,2);c.fillStyle='#fff';c.fillRect(0,0,900,560);
    c.fillStyle='#24404f';c.font='22px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText('累计平均重力矢量的模随时间变化',40,42);
    c.font='12px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillStyle='#718894';c.fillText(`样品坐标系 · 显示 ${snapshot.view.start.toFixed(1)}～${snapshot.current.time.toFixed(1)} s · 线性纵轴 · 保留真实峰谷`,40,68);
    const p=snapshot.current;
    c.fillStyle='#365965';c.fillText(`累计时间：${p.time.toFixed(1)} s    累计平均模：${resultGravityText(p.residual)}    目标：${gravityNumber(p.target)} g${p.evaluation==='match'?` ± ${gravityNumber(p.tolerance)} g`:''}`,40,95);
    c.fillStyle='#718894';
    const settings=p.mode==='uniform'?`匀速模式，外轴 ${format(p.speeds[0],3)} rpm，内轴 ${format(p.speeds[1],3)} rpm`:`随机模式，外轴上限 ${format(p.limits[0],3)} rpm，内轴上限 ${format(p.limits[1],3)} rpm，加速度 ${p.acceleration} rpm/s，种子 ${p.seed}`;
    c.fillText(`结束设置：${settings}`,40,118);
    c.save();c.translate(40,139);renderGravityCurve(c,820,335,snapshot.points,Math.max(.1,p.time),{...snapshot.view,forExport:true});c.restore();
    c.textAlign='center';c.fillStyle='#54717f';c.font='13px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText('仿真时间 / s',450,495);
    c.textAlign='left';c.font='11px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillStyle='#8196a2';
    c.fillText(`全时段记录点：${snapshot.points.length}（CSV 保留全部）    环境重力 / 瞬时模长：${gravityNumber(p.environment)} g。`,40,526);
    c.fillText('本图评估理想旋转中心的运动学平均，不代表 ISS 真实微重力。',40,546);
    return canvas;
  }
  function downloadBlob(blob,filename){
    const url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download=filename;document.body.appendChild(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),10000);
  }
  function exportGravityPng(snapshot,status){
    if(!snapshot)return;
    const canvas=resultCanvas(snapshot),filename=`gravity-curve-${snapshot.current.time.toFixed(1)}s.png`;
    canvas.toBlob(blob=>{if(!blob){status.textContent='图片生成失败，请重试。';return;}downloadBlob(blob,filename);status.textContent=`已生成 ${filename}，请在下载列表中保存。`;},'image/png');
  }
  function exportGravityCsv(snapshot,status){
    if(!snapshot)return;
    const headers=['仿真时间(s)','累计平均矢量的模(g)','平均X(g)','平均Y(g)','平均Z(g)','瞬时X(g)','瞬时Y(g)','瞬时Z(g)','外轴实际转速(rpm)','内轴实际转速(rpm)','模式','外轴速度上限(rpm)','内轴速度上限(rpm)','加速度(rpm/s)','随机种子','覆盖率(%)','环境重力(g)','累计平均目标(g)','评估方式','允许偏差(g)'];
    const rows=snapshot.points.map(p=>[p.time.toFixed(6),p.residual.toFixed(12),...p.mean.map(v=>v.toFixed(12)),...p.instant.map(v=>v.toFixed(12)),...p.speeds.map(v=>v.toFixed(9)),p.mode==='uniform'?'匀速':'随机',...p.limits.map(v=>v.toFixed(9)),p.acceleration.toFixed(12),p.seed,p.coverage.toFixed(6),p.environment.toFixed(12),p.target.toFixed(12),p.evaluation==='match'?'接近目标':'不超过目标',p.tolerance.toFixed(12)].join(','));
    const filename=`gravity-curve-${snapshot.current.time.toFixed(1)}s.csv`;
    downloadBlob(new Blob(['\uFEFF',headers.join(','),'\r\n',rows.join('\r\n'),'\r\n'],{type:'text/csv;charset=utf-8'}),filename);status.textContent=`已生成 ${filename}，包含 ${snapshot.points.length} 个记录点。`;
  }
  $('viewGravityResult').addEventListener('click',()=>{
    if(sim.time<=0||calculating)return;
    running=false;previousFrame=null;updateRunState();updateReadouts();
    resultSnapshot=gravitySnapshot();$('gravityResultImage').src=resultCanvas(resultSnapshot).toDataURL('image/png');
    $('gravityResultSummary').textContent=`截至 ${resultSnapshot.current.time.toFixed(1)} s · 累计平均模 ${resultGravityText(resultSnapshot.current.residual)}`;
    $('resultExportStatus').textContent='';$('gravityResultDialog').showModal();
  });
  $('gravityResultDialog').addEventListener('close',()=>{resultSnapshot=null;$('gravityResultImage').removeAttribute('src');});
  $('exportGravityPng').addEventListener('click',()=>exportGravityPng(gravitySnapshot(),$('exportStatus')));
  $('exportGravityCsv').addEventListener('click',()=>exportGravityCsv(gravitySnapshot(),$('exportStatus')));
  $('resultExportPng').addEventListener('click',()=>exportGravityPng(resultSnapshot,$('resultExportStatus')));
  $('resultExportCsv').addEventListener('click',()=>exportGravityCsv(resultSnapshot,$('resultExportStatus')));
  function currentAccelerationPoints(){
    const points=accelerationRecords.slice(),current={time:sim.time,instant:sim.gravity()};
    if(points.length&&Math.abs(points[points.length-1].time-current.time)<1e-8)points[points.length-1]=current;else points.push(current);
    return points;
  }
  function drawComponentCharts(){
    const end=Math.max(60,sim.time),instantWindow=Number($('instantTimeWindow').value),meanWindow=Number($('meanTimeWindow').value);
    if(instantWidth)renderComponentCurve(instantCtx,instantWidth,instantHeight,currentAccelerationPoints(),end,{key:'instant',start:instantWindow?Math.max(0,sim.time-instantWindow):0});
    if(meanWidth)renderComponentCurve(meanCtx,meanWidth,meanHeight,currentGravityPoints(),end,{key:'mean',start:meanWindow?Math.max(0,sim.time-meanWindow):0,automatic:$('meanAxisScale').value==='auto'});
  }
  function renderComponentCurve(c,w,h,points,end,options){
    const {key,start=0,automatic=false,forExport=false}=options;
    let upper=sim.environmentGravity;
    if(automatic){let peak=0;for(const p of points)if(p.time>=start&&p.time<=end)peak=Math.max(peak,...p[key].map(Math.abs));upper=linearGravityAxis([{time:start,residual:peak}],start,end,true,{ceiling:sim.environmentGravity,target:0}).upper;}
    const step=upper/2;let decimals=1;while(decimals<9&&Math.abs(step-Number(step.toFixed(decimals)))>step*1e-8)decimals++;
    const left=forExport?75:Math.max(43,decimals*6+28),right=forExport?25:12,top=14,bottom=forExport?35:26,pw=w-left-right,ph=h-top-bottom;
    const px=t=>left+(t-start)/Math.max(1e-9,end-start)*pw,py=v=>top+(1-v/upper)/2*ph;
    c.clearRect(0,0,w,h);if(forExport){c.fillStyle='#fff';c.fillRect(0,0,w,h);}c.font=`${forExport?13:10}px "Segoe UI", "Microsoft YaHei", sans-serif`;c.textAlign='right';c.lineWidth=1;
    for(let i=0;i<=4;i++){
      const value=upper-i*step,y=py(value);c.beginPath();c.moveTo(left,y);c.lineTo(w-right,y);c.setLineDash(i===2?[]:[2,5]);c.strokeStyle=i===2?'#cfdde4':'#e7eef1';c.stroke();c.setLineDash([]);c.fillStyle='#8ea2ad';c.fillText(`${format(value,decimals)}${i===0?' g':''}`,left-7,y+3);
    }
    for(let i=0;i<=4;i++){
      const t=start+(end-start)*i/4;c.textAlign=i===0?'left':i===4?'right':'center';c.fillStyle='#99aab4';c.fillText(forExport?t.toFixed(1):t>=120?`${(t/60).toFixed(1)}min`:`${Math.round(t)}s`,px(t),h-7);
    }
    c.save();c.beginPath();c.rect(left-1,top-1,pw+2,ph+2);c.clip();
    for(let axis=0;axis<3;axis++){
      const reduced=reduceCurvePoints(points,start,end,pw,p=>p[key][axis]);c.beginPath();reduced.forEach((p,i)=>i===0?c.moveTo(px(p.time),py(p[key][axis])):c.lineTo(px(p.time),py(p[key][axis])));c.strokeStyle=componentColors[axis];c.lineWidth=1.4;c.lineJoin='round';c.stroke();
    }c.restore();
  }
  ['instantTimeWindow','meanTimeWindow','meanAxisScale'].forEach(id=>$(id).addEventListener('change',()=>{dirtyChart=true;drawComponentCharts();}));
  $('exportComponentsPng').addEventListener('click',()=>{
    const snapshot=gravitySnapshot();if(!snapshot)return;
    const current=snapshot.current,end=Math.max(.1,current.time),instantWindow=Number($('instantTimeWindow').value),meanWindow=Number($('meanTimeWindow').value);
    const instantStart=instantWindow?Math.max(0,current.time-instantWindow):0,meanStart=meanWindow?Math.max(0,current.time-meanWindow):0;
    const canvas=document.createElement('canvas');canvas.width=1800;canvas.height=1600;const c=canvas.getContext('2d');c.scale(2,2);c.fillStyle='#fff';c.fillRect(0,0,900,800);
    c.fillStyle='#24404f';c.font='22px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText('重力加速度三分量—时间变化曲线',40,42);
    c.font='12px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillStyle='#718894';c.fillText(`样品坐标系 · 单位 g · 截至 ${current.time.toFixed(1)} s · 平均值从重置开始累计`,40,69);
    function heading(text,y,start){
      c.textAlign='left';c.fillStyle='#365965';c.font='14px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText(text,40,y);c.font='11px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillStyle='#8196a2';c.fillText(`显示 ${start.toFixed(1)}～${current.time.toFixed(1)} s`,40,y+21);
      ['X','Y','Z'].forEach((label,i)=>{c.fillStyle=componentColors[i];c.fillRect(688+i*48,y-9,16,2);c.fillText(label,709+i*48,y-4);});
    }
    heading('瞬时重力加速度',104,instantStart);c.save();c.translate(40,140);renderComponentCurve(c,820,235,currentAccelerationPoints(),end,{key:'instant',start:instantStart,forExport:true});c.restore();
    heading('累计平均重力加速度',419,meanStart);c.save();c.translate(40,455);renderComponentCurve(c,820,235,snapshot.points,end,{key:'mean',start:meanStart,automatic:$('meanAxisScale').value==='auto',forExport:true});c.restore();
    c.textAlign='center';c.fillStyle='#54717f';c.font='13px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText('仿真时间 / s',450,398);c.fillText('仿真时间 / s',450,716);
    c.textAlign='left';c.fillStyle='#8196a2';c.font='11px "Microsoft YaHei", "Segoe UI", sans-serif';c.fillText('平均分量 = 各分量的时间积分 ÷ 累计时间；绘图保留原始峰谷，未进行平滑滤波。',40,756);c.fillText(`环境重力 / 瞬时模长：${gravityNumber(current.environment)} g；本图为理想旋转中心的运动学仿真。`,40,777);
    canvas.toBlob(blob=>{if(!blob){$('componentExportStatus').textContent='图片生成失败，请重试。';return;}const filename=`gravity-components-${current.time.toFixed(1)}s.png`;downloadBlob(blob,filename);$('componentExportStatus').textContent=`已生成 ${filename}，请在下载列表中保存。`;},'image/png');
  });
  function drawCoverageChart(){
    if(!coverageWidth)return;
    const c=coverageCtx,w=coverageWidth,h=coverageHeight,left=37,right=15,top=14,bottom=30;
    const pw=w-left-right,ph=h-top-bottom,end=Math.max(60,sim.time);
    const px=t=>left+t/end*pw,py=percent=>top+(1-percent/100)*ph;
    c.clearRect(0,0,w,h);c.font='10px "Segoe UI"';c.lineWidth=1;
    for(let i=0;i<=4;i++){
      const value=i*25,y=py(value);c.beginPath();c.moveTo(left,y);c.lineTo(w-right,y);c.setLineDash([2,5]);c.strokeStyle='#e7eef1';c.stroke();c.setLineDash([]);c.fillStyle='#8ea2ad';c.textAlign='right';c.fillText(`${value}%`,left-6,y+3);
    }
    for(let i=0;i<=4;i++){
      const t=end*i/4;c.textAlign=i===0?'left':i===4?'right':'center';c.fillStyle='#99aab4';c.fillText(t>=120?`${(t/60).toFixed(1)}min`:`${Math.round(t)}s`,px(t),h-8);
    }
    const points=[{time:0,coverage:0},...currentGravityPoints()];
    c.save();c.beginPath();c.rect(left,top,pw,ph);c.clip();c.beginPath();
    reduceCurvePoints(points,0,end,pw,'coverage').forEach((p,i)=>i===0?c.moveTo(px(p.time),py(p.coverage)):c.lineTo(px(p.time),py(p.coverage)));
    c.strokeStyle='#2389a5';c.lineWidth=1.8;c.stroke();
    c.lineTo(px(sim.time),py(0));c.lineTo(px(0),py(0));c.closePath();c.fillStyle='#2389a50d';c.fill();c.restore();
  }
  function mapGeometry(){return {left:40,right:20,top:14,bottom:30,pw:mapWidth-60,ph:mapHeight-44};}
  function coverageScale(){
    let min=Infinity,max=0;
    for(const count of sim.coverage.bins)if(count>0){min=Math.min(min,count);max=Math.max(max,count);}
    return {min:max?min:0,max};
  }
  function coverageColor(count,range){
    if(!count)return '#edf2f5';
    const intensity=range.max===range.min?.5:(count-range.min)/(range.max-range.min);
    const stops=[[250,234,179],[58,175,160],[19,62,105]],segment=intensity<.5?0:1,t=segment===0?intensity*2:(intensity-.5)*2;
    return `rgb(${stops[segment].map((v,i)=>Math.round(v+(stops[segment+1][i]-v)*t)).join(',')})`;
  }
  function updateCoverageLegend(range){
    $('coverageMinCount').textContent=`${range.min.toLocaleString('zh-CN')} 次`;
    $('coverageMaxCount').textContent=`${range.max.toLocaleString('zh-CN')} 次`;
    $('coverageScaleBar').style.background=range.max===0?'#edf2f5':range.min===range.max?'#3aafa0':'linear-gradient(90deg,#faeab3,#3aafa0,#133e69)';
    $('coverageScaleNote').textContent=!range.max?'暂无采样':range.min===range.max?'所有已覆盖网格的次数相同':'按当前已覆盖网格的最少～最多次数线性映射';
  }
  function drawCoverageMap(){
    if(!mapWidth)return;
    const c=coverageMapCtx,{left,top,pw,ph}=mapGeometry(),coverage=sim.coverage;
    const range=coverageScale(),cw=pw/coverage.columns,ch=ph/coverage.rows;
    updateCoverageLegend(range);
    c.clearRect(0,0,mapWidth,mapHeight);
    for(let row=0;row<coverage.rows;row++)for(let col=0;col<coverage.columns;col++){
      const count=coverage.bins[row*coverage.columns+col];
      c.fillStyle=coverageColor(count,range);
      c.fillRect(left+col*cw,top+(coverage.rows-1-row)*ch,Math.max(.5,cw-.55),Math.max(.5,ch-.55));
    }
    c.font='10px "Segoe UI"';c.fillStyle='#8ea2ad';c.textAlign='right';
    [1,.5,0,-.5,-1].forEach(value=>c.fillText(format(value),left-7,top+(1-value)/2*ph+3));
    c.textAlign='left';c.fillText('Y',left-23,top-4);
    for(let i=0;i<=4;i++){
      c.textAlign=i===0?'left':i===4?'right':'center';c.fillText(`${i*90-180}°`,left+pw*i/4,mapHeight-9);
    }
    const v=sim.gravityDirection(),longitude=Math.atan2(v[2],v[0]);
    const x=left+(longitude+Math.PI)/(2*Math.PI)*pw,y=top+(1-v[1])/2*ph;
    c.beginPath();c.arc(x,y,3.5,0,2*Math.PI);c.fillStyle='#e6a047';c.fill();c.strokeStyle='#fff';c.lineWidth=1.2;c.stroke();
  }
  coverageMap.addEventListener('pointermove',event=>{
    const rect=coverageMap.getBoundingClientRect(),{left,top,pw,ph}=mapGeometry();
    const x=event.clientX-rect.left,y=event.clientY-rect.top;
    if(x<left||x>left+pw||y<top||y>top+ph){$('coverageMapReadout').textContent='指向分布图可查看网格的方向范围及采样次数。';return;}
    const coverage=sim.coverage,col=Math.min(coverage.columns-1,Math.floor((x-left)/pw*coverage.columns));
    const row=Math.min(coverage.rows-1,Math.floor((1-(y-top)/ph)*coverage.rows));
    const angle=-180+col*360/coverage.columns,yMin=-1+row*2/coverage.rows;
    $('coverageMapReadout').textContent=`经度 ${angle}～${angle+360/coverage.columns}° · Y ${format(yMin,2)}～${format(yMin+2/coverage.rows,2)} · ${coverage.bins[row*coverage.columns+col].toLocaleString('zh-CN')} 次采样`;
  });
  coverageMap.addEventListener('pointerleave',()=>{$('coverageMapReadout').textContent='指向分布图可查看网格的方向范围及采样次数。';});
  const sphereHint='指向球面可查看网格采样次数；当前方向在背面时显示为空心圆与虚线。';
  const sphereCells=[];
  // Tessellate the same equal-area bins as the flat map, without resampling counts.
  for(let row=0;row<sim.coverage.rows;row++)for(let col=0;col<sim.coverage.columns;col++){
    const y0=-1+row*2/sim.coverage.rows,y1=y0+2/sim.coverage.rows;
    const a0=-Math.PI+col*2*Math.PI/sim.coverage.columns,a1=a0+2*Math.PI/sim.coverage.columns;
    const vertex=(a,y)=>{const r=Math.sqrt(Math.max(0,1-y*y));return [r*Math.cos(a),y,r*Math.sin(a)];},points=[];
    for(let n=0;n<4;n++)points.push(vertex(a0+(a1-a0)*n/4,y0));
    for(let n=0;n<4;n++)points.push(vertex(a1,y0+(y1-y0)*n/4));
    for(let n=0;n<4;n++)points.push(vertex(a1-(a1-a0)*n/4,y1));
    for(let n=0;n<4;n++)points.push(vertex(a0,y1-(y1-y0)*n/4));
    sphereCells.push(points);
  }
  const sphereView=v=>rotateX(rotateY(v,sphereCamera.yaw),sphereCamera.pitch);
  function sphereGeometry(){return {cx:sphereWidth/2,cy:sphereHeight/2,radius:Math.max(1,Math.min(sphereWidth/2-36,sphereHeight/2-28))};}
  function clipSphereFront(points){
    const result=[];
    for(let i=0;i<points.length;i++){
      const a=points[i],b=points[(i+1)%points.length],inside=a[2]>=0;
      if(inside)result.push(a);
      if(inside!==(b[2]>=0)){
        const t=a[2]/(a[2]-b[2]);result.push(normalize([a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,0]));
      }
    }return result;
  }
  function drawCoverageSphere(){
    if(!sphereWidth)return;
    const c=sphereCtx,{cx,cy,radius:r}=sphereGeometry(),coverage=sim.coverage,range=coverageScale();
    c.clearRect(0,0,sphereWidth,sphereHeight);
    c.beginPath();c.arc(cx,cy,r,0,2*Math.PI);c.fillStyle='#edf2f5';c.fill();
    c.save();c.clip();
    sphereCells.forEach((cell,index)=>{
      const points=clipSphereFront(cell.map(sphereView));if(points.length<3)return;
      c.beginPath();points.forEach((p,i)=>i===0?c.moveTo(cx+p[0]*r,cy-p[1]*r):c.lineTo(cx+p[0]*r,cy-p[1]*r));c.closePath();
      c.fillStyle=coverageColor(coverage.bins[index],range);c.fill();c.strokeStyle='#ffffff70';c.lineWidth=.5;c.stroke();
    });c.restore();
    c.beginPath();c.arc(cx,cy,r,0,2*Math.PI);c.strokeStyle='#c4d6de';c.lineWidth=1;c.stroke();
    c.font='11px "Segoe UI"';c.textAlign='center';
    [[1,0,0],[0,1,0],[0,0,1]].forEach((axis,i)=>{
      const v=sphereView(axis);c.beginPath();c.moveTo(cx,cy);c.lineTo(cx+v[0]*r*1.1,cy-v[1]*r*1.1);c.setLineDash(v[2]<0?[3,4]:[]);c.strokeStyle=componentColors[i]+'90';c.lineWidth=1;c.stroke();c.setLineDash([]);
      c.fillStyle=componentColors[i];c.fillText(['X','Y','Z'][i],cx+v[0]*r*1.19,cy-v[1]*r*1.19+4);
    });
    const current=sphereView(sim.gravityDirection()),x=cx+current[0]*r,y=cy-current[1]*r,behind=current[2]<0;
    c.beginPath();c.moveTo(cx,cy);c.lineTo(x,y);c.setLineDash(behind?[4,4]:[]);c.strokeStyle='#e6a047';c.lineWidth=1.5;c.stroke();c.setLineDash([]);
    c.beginPath();c.arc(x,y,5,0,2*Math.PI);c.fillStyle=behind?'#ffffffd9':'#e6a047';c.fill();c.strokeStyle=behind?'#e6a047':'#fff';c.lineWidth=2;c.stroke();
  }
  coverageSphere.addEventListener('pointerdown',event=>{if(spherePointer)return;spherePointer={id:event.pointerId,x:event.clientX,y:event.clientY};coverageSphere.setPointerCapture(event.pointerId);});
  coverageSphere.addEventListener('pointermove',event=>{
    if(spherePointer&&spherePointer.id===event.pointerId){
      sphereCamera.yaw+=(event.clientX-spherePointer.x)*.01;sphereCamera.pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,sphereCamera.pitch+(event.clientY-spherePointer.y)*.01));
      spherePointer.x=event.clientX;spherePointer.y=event.clientY;drawCoverageSphere();return;
    }
    const rect=coverageSphere.getBoundingClientRect(),{cx,cy,radius}=sphereGeometry(),x=(event.clientX-rect.left-cx)/radius,y=-(event.clientY-rect.top-cy)/radius;
    if(x*x+y*y>1){$('coverageSphereReadout').textContent=sphereHint;return;}
    const v=rotateY(rotateX([x,y,Math.sqrt(Math.max(0,1-x*x-y*y))],-sphereCamera.pitch),-sphereCamera.yaw),index=sim.coverage.index(v);
    $('coverageSphereReadout').textContent=`重力方向 (${v.map(value=>format(value,3)).join(', ')}) · 网格 ${Math.floor(index/sim.coverage.columns)+1} 行 / ${index%sim.coverage.columns+1} 列 · ${sim.coverage.bins[index].toLocaleString('zh-CN')} 次采样`;
  });
  ['pointerup','pointercancel','lostpointercapture'].forEach(name=>coverageSphere.addEventListener(name,()=>{spherePointer=null;}));
  coverageSphere.addEventListener('pointerleave',()=>{$('coverageSphereReadout').textContent=sphereHint;});
  coverageSphere.addEventListener('keydown',event=>{
    if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(event.key))return;event.preventDefault();
    sphereCamera.yaw+=event.key==='ArrowLeft'?-.15:event.key==='ArrowRight'?.15:0;
    sphereCamera.pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,sphereCamera.pitch+(event.key==='ArrowUp'?-.15:event.key==='ArrowDown'?.15:0)));drawCoverageSphere();
  });
  $('resetSphereView').addEventListener('click',()=>{sphereCamera={yaw:.65,pitch:.25};$('coverageSphereReadout').textContent=sphereHint;drawCoverageSphere();});
  function setMode(mode) {
    sim.setMode(mode);
    setPressed($('uniformMode'),mode==='uniform');setPressed($('randomMode'),mode==='random');
    $('uniformControls').hidden=mode!=='uniform';$('randomControls').hidden=mode!=='random';
    $('modeBadge').textContent=mode==='uniform'?'匀速模式':'随机模式';
    $('modeDescription').textContent=mode==='uniform'?'两轴以设定的转速持续旋转，负值表示反向。':'从当前转速追踪随机目标，各轴达到后立即切换下一目标。';
    $('chartNote').textContent=mode==='uniform'?'匀速模式下，两轴转速保持为设定值。':`变化率 ${sim.acceleration} rpm/s · 达到目标后立即切换下一目标。`;
    record();updateReadouts();announce(`已切换至${$('modeBadge').textContent}`);
  }
  $('uniformMode').addEventListener('click',()=>setMode('uniform'));
  $('randomMode').addEventListener('click',()=>setMode('random'));
  function syncUniformInputs(){
    ['outerSpeed','innerSpeed'].forEach((id,i)=>{$(id).value=sim.uniform[i];$(id+'Input').value=sim.uniform[i];$(id+'Value').textContent=`${format(sim.uniform[i],2)} rpm`;$(id+'Error').hidden=true;$(id+'Input').removeAttribute('aria-invalid');});
  }
  function syncEnvironmentPreset(){
    $('environmentPreset').value=['1','0.165','0.378'].includes(String(sim.environmentGravity))?String(sim.environmentGravity):'';
    $('environmentGravity').value=sim.environmentGravity;
  }
  function applyEnvironment(){
    const input=$('environmentGravity');if(!validParameter(input,$('environmentGravityError'),.000001,10))return;
    if(input.valueAsNumber===sim.environmentGravity)return;
    sim.setEnvironmentGravity(input.valueAsNumber);syncEnvironmentPreset();resetSimulation();
    announce('环境重力已更新，仿真和累计数据已重置。');
  }
  $('environmentGravity').addEventListener('change',applyEnvironment);
  $('environmentPreset').addEventListener('change',()=>{if($('environmentPreset').value){$('environmentGravity').value=$('environmentPreset').value;applyEnvironment();}});
  function applyTarget(){
    if(!validParameter($('targetGravity'),$('targetGravityError'),0,10))return;
    sim.targetGravity=$('targetGravity').valueAsNumber;$('partialPreset').value='';dirtyChart=true;updateReadouts();
  }
  $('targetGravity').addEventListener('input',applyTarget);$('targetGravity').addEventListener('change',applyTarget);
  $('targetEvaluation').addEventListener('change',()=>{targetEvaluation=$('targetEvaluation').value;dirtyChart=true;updateReadouts();});
  $('targetTolerance').addEventListener('input',()=>{
    if(!validParameter($('targetTolerance'),$('targetToleranceError'),.000001,10))return;
    targetTolerance=$('targetTolerance').valueAsNumber;dirtyChart=true;updateReadouts();
  });
  $('applyPartialGravity').addEventListener('click',()=>{
    if($('partialPreset').value)$('targetGravity').value=$('partialPreset').value;
    if(!validParameter($('targetGravity'),$('targetGravityError'),0,1))return;
    sim.environmentGravity=1;syncEnvironmentPreset();sim.configurePartialGravity($('targetGravity').valueAsNumber);
    targetEvaluation='match';$('targetEvaluation').value='match';partialApplied=true;
    targetTolerance=Math.min(.005,Math.max(.000001,sim.targetGravity*.05));$('targetTolerance').value=targetTolerance;
    syncUniformInputs();resetSimulation();setMode('uniform');announce('已应用地球环境中的倾斜单轴部分重力方案。');
  });
  $('restoreDualAxis').addEventListener('click',()=>{
    sim.environmentGravity=1;sim.targetGravity=.001;sim.initialAngles=[25,-32];sim.uniform=[1,2];sim.setMode('uniform');
    $('targetGravity').value=.001;targetEvaluation='ceiling';$('targetEvaluation').value='ceiling';partialApplied=false;
    $('partialPlanSummary').textContent='已恢复外轴 1 rpm、内轴 2 rpm 的双轴方案；完整 60 秒周期评估方向平均抵消。';
    syncEnvironmentPreset();syncUniformInputs();resetSimulation();setMode('uniform');
  });
  [['outerSpeed',0],['innerSpeed',1]].forEach(([id,i]) => {
    function applySpeed(){
      const input=$(id+'Input');if(!validParameter(input,$(id+'Error'),-10,10))return;
      sim.uniform[i]=input.valueAsNumber;$(id).value=sim.uniform[i];$(id+'Value').textContent=`${format(sim.uniform[i],2)} rpm`;
      if(sim.mode==='uniform'){sim.velocities[i]=sim.targets[i]=sim.uniform[i];record();updateReadouts();}
    }
    $(id+'Input').addEventListener('input',applySpeed);$(id+'Input').addEventListener('change',applySpeed);
    $(id).addEventListener('input',()=>{$(id+'Input').value=$(id).value;applySpeed();});
  });
  function validParameter(input,error,min,max){
    const value=input.valueAsNumber,valid=Number.isFinite(value)&&value>=min&&value<=max;
    error.hidden=valid;
    if(valid)input.removeAttribute('aria-invalid');
    else{input.setAttribute('aria-invalid','true');error.textContent=`请输入 ${min}～${max} 之间的数值。`;}
    return valid;
  }
  [['outerMax',0],['innerMax',1]].forEach(([id,i])=>{
    function applyLimit(){
      const input=$(id+'Input');if(!validParameter(input,$(id+'Error'),.01,10))return;
      sim.limits[i]=input.valueAsNumber;$(id).value=sim.limits[i];
      // Existing speed stays continuous when the permissible target shrinks.
      if(sim.mode==='random') {
        sim.targets[i]=Math.max(-sim.limits[i],Math.min(sim.limits[i],sim.targets[i]));
        if(Math.abs(sim.targets[i]-sim.velocities[i])<=1e-10)sim.chooseTarget(i);
      }
      dirtyChart=true;updateReadouts();
    }
    $(id+'Input').addEventListener('input',applyLimit);$(id+'Input').addEventListener('change',applyLimit);
    $(id).addEventListener('input',()=>{$(id+'Input').value=$(id).value;applyLimit();});
  });
  function applyAcceleration(){
    if(!validParameter($('acceleration'),$('accelerationError'),.000001,10))return;
    sim.acceleration=$('acceleration').valueAsNumber;
    if(sim.mode==='random')$('chartNote').textContent=`变化率 ${sim.acceleration} rpm/s · 达到目标后立即切换下一目标。`;
  }
  $('acceleration').addEventListener('input',applyAcceleration);$('acceleration').addEventListener('change',applyAcceleration);
  $('seed').addEventListener('change',()=>{
    const raw=Number($('seed').value);sim.seed=Number.isFinite(raw)?Math.max(0,Math.min(4294967295,Math.floor(raw))):42;$('seed').value=sim.seed;
    sim.rngState=sim.seed>>>0;if(sim.mode==='random')sim.chooseTargets();updateReadouts();announce('随机种子已更新，重置可从初始状态复现。');
  });
  function applyTimeScale() {
    const input=$('timeScale'),value=input.valueAsNumber;
    if(!Number.isFinite(value)||value<.01||value>1000){
      input.setAttribute('aria-invalid','true');$('timeScaleError').textContent='请输入 0.01～1000 之间的倍率。';$('timeScaleError').hidden=false;
      return;
    }
    timeScale=value;previousFrame=null;
    input.removeAttribute('aria-invalid');$('timeScaleError').hidden=true;$('timeScaleValue').textContent=`${value}×`;
    $('timeScalePreset').value=[...$('timeScalePreset').options].some(option=>option.value===String(value))?String(value):'';
  }
  $('timeScale').addEventListener('input',applyTimeScale);
  $('timeScale').addEventListener('change',applyTimeScale);
  $('timeScalePreset').addEventListener('change',()=>{if($('timeScalePreset').value){$('timeScale').value=$('timeScalePreset').value;applyTimeScale();}});
  function updateRunState(){
    $('runText').textContent=running?'暂停仿真':'开始仿真';$('runIcon').textContent=running?'Ⅱ':'▶';
    $('statusText').textContent=running?'仿真运行中':'仿真已暂停';$('statusDot').classList.toggle('paused',!running);
  }
  $('toggleRun').addEventListener('click',()=>{running=!running;previousFrame=null;updateRunState();updateReadouts();announce(running?'仿真已开始':'仿真已暂停');});
  function resetSimulation(){sim.reset();history=[];trail=[];gravityRecords=[];accelerationRecords=[];lastSample=0;previousFrame=null;$('exportStatus').textContent='';$('componentExportStatus').textContent='';$('coverageSphereReadout').textContent=sphereHint;$('coverageMapReadout').textContent='指向分布图可查看网格的方向范围及采样次数。';record();updateReadouts();announce('已重置角度、仿真时间、累计重力矢量、方向覆盖率和随机序列。');}
  $('reset').addEventListener('click',resetSimulation);
  $('fastForward').addEventListener('click',()=>{
    if(calculating)return;
    calculating=true;previousFrame=null;
    const controls=[...document.querySelectorAll('.controls button,.controls input,.controls select')];
    controls.forEach(el=>el.disabled=true);$('fastForward').disabled=true;
    let remaining=600;
    function computeBatch(){
      for(let n=0;n<100 && remaining>0;n++){
        const step=Math.min(.1,remaining);sim.update(step);remaining=Math.max(0,remaining-step);record();lastSample=sim.time;
      }
      $('fastForward').textContent=`计算中 ${Math.round((600-remaining)/6)}%`;updateReadouts();
      if(remaining>0)requestAnimationFrame(computeBatch);
      else{
        calculating=false;previousFrame=null;controls.forEach(el=>el.disabled=false);$('fastForward').disabled=false;$('fastForward').textContent='快进计算 10 分钟';
        announce('已按当前参数计算额外十分钟，累计平均结果已更新。');
      }
    }
    requestAnimationFrame(computeBatch);
  });
  function setCamera(view){
    camera=view==='front'?{yaw:0,pitch:0,zoom:1}:view==='top'?{yaw:0,pitch:Math.PI/2-.001,zoom:1}:{yaw:.58,pitch:.27,zoom:1};
    document.querySelectorAll('[data-view]').forEach(el=>setPressed(el,el.dataset.view===view));
  }
  document.querySelectorAll('[data-view]').forEach(el=>el.addEventListener('click',()=>setCamera(el.dataset.view)));
  $('resetView').addEventListener('click',()=>setCamera('iso'));
  scene.addEventListener('pointerdown',event=>{if(pointer)return;pointer={id:event.pointerId,x:event.clientX,y:event.clientY};scene.setPointerCapture(event.pointerId);});
  scene.addEventListener('pointermove',event=>{
    if(!pointer||pointer.id!==event.pointerId)return;
    camera.yaw+=(event.clientX-pointer.x)*.008;camera.pitch=Math.max(-Math.PI/2+.02,Math.min(Math.PI/2-.02,camera.pitch+(event.clientY-pointer.y)*.008));
    pointer.x=event.clientX;pointer.y=event.clientY;document.querySelectorAll('[data-view]').forEach(el=>setPressed(el,false));
  });
  scene.addEventListener('pointerup',()=>{pointer=null;});scene.addEventListener('pointercancel',()=>{pointer=null;});scene.addEventListener('lostpointercapture',()=>{pointer=null;});
  scene.addEventListener('wheel',event=>{event.preventDefault();camera.zoom=Math.max(.65,Math.min(1.65,camera.zoom*Math.exp(-event.deltaY*.001)));},{passive:false});
  document.addEventListener('visibilitychange',()=>{previousFrame=null;});
  // Keep background tabs from jumping ahead when they are brought back into view.
  let lastReadout=0;
  function frame(now) {
    const elapsed=previousFrame===null?0:Math.min((now-previousFrame)/1000,.1);previousFrame=now;
    if(running && !calculating && elapsed>0){
      let remaining=elapsed*timeScale;
      while(remaining>1e-8){
        const untilSample=Math.max(0,.1-(sim.time-lastSample));
        if(untilSample<1e-8){record();lastSample=sim.time;continue;}
        const step=Math.min(remaining,untilSample);sim.update(step);remaining-=step;
      }
      if(sim.time-lastSample>=.1-1e-8){record();lastSample=sim.time;}
    }
    drawScene();if(dirtyChart)drawChart();
    if(now-lastReadout>80){updateReadouts();lastReadout=now;}
    requestAnimationFrame(frame);
  }
  record();updateReadouts();requestAnimationFrame(frame);
})();
