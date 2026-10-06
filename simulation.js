(function (root) {
  'use strict';
  const RAD = Math.PI / 180;
  const RPM_TO_DEG_PER_SECOND = 6;
  const EPSILON = 1e-10;
  const GAUSS_NODES = [-.8611363115940526, -.3399810435848563, .3399810435848563, .8611363115940526];
  const GAUSS_WEIGHTS = [.3478548451374538, .6521451548625461, .6521451548625461, .3478548451374538];
  const wrap = degrees => ((degrees % 360) + 360) % 360;
  function reduceCurvePoints(points,start,end,pixels,key='residual') {
    if(!points.length)return [];
    // Keep original endpoints and extrema per horizontal pixel; never
    // replace measurements with averages or repeatedly thin old history.
    let low=0,high=points.length;
    while(low<high){const mid=(low+high)>>>1;if(points[mid].time<start)low=mid+1;else high=mid;}
    const begin=Math.max(0,low-1),columns=Math.max(1,Math.floor(pixels)),span=Math.max(1e-9,end-start);
    const result=[],value=typeof key==='function'?key:point=>point[key];
    let bucket=-1,first=-1,last=-1,min=-1,max=-1;
    function flush(){
      if(first<0)return;
      [...new Set([first,min,max,last])].sort((a,b)=>a-b).forEach(index=>result.push(points[index]));
    }
    for(let i=begin;i<points.length&&points[i].time<=end;i++){
      const column=Math.max(0,Math.min(columns-1,Math.floor((points[i].time-start)/span*columns)));
      if(column!==bucket){flush();bucket=column;first=last=min=max=i;}
      else{last=i;if(value(points[i])<value(points[min]))min=i;if(value(points[i])>value(points[max]))max=i;}
    }
    flush();return result;
  }
  function linearGravityAxis(points,start,end,automatic=false,{ceiling=1,target=.001}={}){
    function axis(upper,step){let decimals=1;while(decimals<9&&Math.abs(step-Number(step.toFixed(decimals)))>step*1e-8)decimals++;return {upper,step,decimals};}
    const fixed=Math.max(ceiling,target,1e-6);
    if(!automatic)return axis(fixed,fixed/5);
    let peak=target;
    for(const point of points)if(point.time>=start&&point.time<=end)peak=Math.max(peak,point.residual);
    if(peak>=fixed*.8&&peak<=fixed)return axis(fixed,fixed/5);
    const required=Math.max(1e-6,target*1.25,peak*1.1),raw=required/5,power=10**Math.floor(Math.log10(raw));
    const step=[1,2,2.5,5,10].find(value=>value>=raw/power)*power;
    let decimals=1;while(decimals<9&&Math.abs(step-Number(step.toFixed(decimals)))>step*1e-8)decimals++;
    return {upper:Math.ceil(required/step)*step,step,decimals};
  }
  class DirectionCoverage {
    constructor(columns = 36, rows = 18) {
      this.columns = columns;
      this.rows = rows;
      this.samplePeriod = .05;
      this.bins = new Float64Array(columns * rows);
      this.visitedCount = 0;
      this.sampleCount = 0;
    }
    index(direction) {
      // Uniform longitude and uniform Y form cells of equal solid angle:
      // dOmega = d(longitude) dY, with Y = sin(latitude).
      const longitude = Math.hypot(direction[0], direction[2]) < 1e-10
        ? 0 : Math.atan2(direction[2], direction[0]);
      const fraction = ((longitude / (2 * Math.PI) + .5) % 1 + 1) % 1;
      const column = Math.min(this.columns - 1, Math.floor(fraction * this.columns));
      const row = Math.max(0, Math.min(this.rows - 1, Math.floor((direction[1] + 1) * .5 * this.rows)));
      return row * this.columns + column;
    }
    observe(direction) {
      const index = this.index(direction);
      if (!this.bins[index]) this.visitedCount++;
      this.bins[index]++;
      this.sampleCount++;
    }
    get percent() { return this.visitedCount / this.bins.length * 100; }
  }
  class GimbalSimulation {
    constructor() {
      this.mode = 'uniform';
      // Speeds and targets are rpm; angles remain degrees.
      this.uniform = [1.33, 2];
      this.limits = [3.33, 4.17];
      this.seed = 42;
      this.acceleration = .01; // rpm per second of simulation time
      this.environmentGravity = 1; // multiples of standard Earth gravity
      this.targetGravity = .001;
      this.initialAngles = [25,-32];
      this.reset();
    }
    random() {
      this.rngState = (this.rngState + 0x6D2B79F5) >>> 0;
      let t = this.rngState;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }
    chooseTarget(i) {
      const limit = this.limits[i];
      const target = (2 * this.random() - 1) * limit;
      // A different target prevents zero-duration cycles.
      this.targets[i] = Math.abs(target - this.velocities[i]) > EPSILON
        ? target : (this.velocities[i] <= 0 ? limit : -limit);
    }
    chooseTargets() {
      for (let i = 0; i < 2; i++) this.chooseTarget(i);
    }
    reset() {
      this.time = 0;
      this.angles = [...this.initialAngles];
      this.gravityIntegral = [0, 0, 0]; // g·s, expressed in the sample frame
      this.coverage = new DirectionCoverage();
      this.rngState = Number(this.seed) >>> 0;
      this.velocities = this.mode === 'uniform' ? [...this.uniform] : [0, 0];
      this.targets = [...this.velocities];
      if (this.mode === 'random') this.chooseTargets();
    }
    setMode(mode) {
      if (mode !== 'uniform' && mode !== 'random') throw new Error('Unknown mode');
      if (this.mode === mode) return;
      this.mode = mode;
      if (mode === 'uniform') { this.velocities = [...this.uniform]; this.targets = [...this.uniform]; }
      else { this.rngState = Number(this.seed) >>> 0; this.chooseTargets(); }
    }
    update(dt) {
      if (!Number.isFinite(dt) || dt < 0) throw new Error('Invalid time step');
      if (this.mode === 'uniform') {
        this.velocities = [...this.uniform];
        this.targets = [...this.uniform];
        this.integrateGravity(dt, [0, 0]);
        for (let i = 0; i < 2; i++) this.angles[i] += this.velocities[i] * RPM_TO_DEG_PER_SECOND * dt;
        this.time += dt;
        return;
      }
      let remaining = dt;
      // Integrate exactly to the earliest arrival, then immediately resample
      // only that axis. Both axes can arrive at different times, with no dwell.
      while (remaining > EPSILON) {
        for (let i = 0; i < 2; i++) {
          if (Math.abs(this.targets[i] - this.velocities[i]) <= EPSILON) this.chooseTarget(i);
        }
        const arrivalTimes = this.targets.map((target, i) => Math.abs(target - this.velocities[i]) / this.acceleration);
        const step = Math.min(remaining, ...arrivalTimes);
        const accelerations = this.targets.map((target, i) => Math.sign(target - this.velocities[i]) * this.acceleration);
        this.integrateGravity(step, accelerations);
        for (let i = 0; i < 2; i++) {
          const before = this.velocities[i];
          const after = arrivalTimes[i] <= step ? this.targets[i]
            : before + Math.sign(this.targets[i] - before) * this.acceleration * step;
          this.angles[i] += (before + after) * .5 * RPM_TO_DEG_PER_SECOND * step;
          this.velocities[i] = after;
        }
        this.time += step;
        remaining -= step;
        for (let i = 0; i < 2; i++) {
          if (arrivalTimes[i] <= step) this.chooseTarget(i);
        }
      }
    }
    setEnvironmentGravity(value){
      if(!Number.isFinite(value)||value<.000001||value>10)throw new Error('Invalid environment gravity');
      this.environmentGravity=value;this.reset();
    }
    configurePartialGravity(target,innerRpm=2){
      if(!Number.isFinite(target)||target<0||target>this.environmentGravity)throw new Error('Partial target must be between zero and environment gravity');
      if(!Number.isFinite(innerRpm)||Math.abs(innerRpm)<.01||Math.abs(innerRpm)>10)throw new Error('Invalid inner-axis speed');
      this.targetGravity=target;this.mode='uniform';this.uniform=[0,innerRpm];
      this.initialAngles=[Math.acos(target/this.environmentGravity)/RAD,0];this.reset();
      return this.initialAngles[0];
    }
    gravityDirectionAt(angles) {
      // Rᵀ · (0, −1, 0), where R = Rx(alpha) Ry(beta).
      const a = angles[0] * RAD, b = angles[1] * RAD;
      return [-Math.sin(a) * Math.sin(b), -Math.cos(a), Math.sin(a) * Math.cos(b)];
    }
    gravityAt(angles){return this.gravityDirectionAt(angles).map(value=>value*this.environmentGravity);}
    worldGravity(){return [0,-this.environmentGravity,0];}
    gravityDirection(){return this.gravityDirectionAt(this.angles);}
    gravity() { return this.gravityAt(this.angles); }
    integrateGravity(dt, accelerations) {
      // Four-point Gauss quadrature on <=0.25 s segments resolves the
      // continuously changing direction; speed is never treated as angle.
      const count = Math.ceil(dt / .25);
      if (!count) return;
      const segment = dt / count, half = segment / 2;
      for (let j = 0; j < count; j++) {
        for (let k = 0; k < 4; k++) {
          const t = (j + .5) * segment + half * GAUSS_NODES[k];
          const angles = this.angles.map((angle, i) => angle + RPM_TO_DEG_PER_SECOND * (this.velocities[i] * t + .5 * accelerations[i] * t * t));
          const gravity = this.gravityAt(angles), weight = half * GAUSS_WEIGHTS[k];
          for (let i = 0; i < 3; i++) this.gravityIntegral[i] += gravity[i] * weight;
        }
      }
      // Direction samples have a fixed simulation-time schedule, independent
      // of rendering, playback factor, integration steps and fast-forward.
      const period = this.coverage.samplePeriod, end = this.time + dt;
      let sampleTime = (this.coverage.sampleCount + .5) * period;
      while (sampleTime <= end + EPSILON) {
        const t = Math.max(0, sampleTime - this.time);
        const angles = this.angles.map((angle, i) => angle + RPM_TO_DEG_PER_SECOND * (this.velocities[i] * t + .5 * accelerations[i] * t * t));
        this.coverage.observe(this.gravityDirectionAt(angles));
        sampleTime = (this.coverage.sampleCount + .5) * period;
      }
    }
    meanGravity() {
      return this.time > 0 ? this.gravityIntegral.map(component => component / this.time) : null;
    }
    residualGravity() {
      const mean = this.meanGravity();
      return mean ? Math.hypot(...mean) : null;
    }
    direction() {
      const a = this.angles[0] * RAD, b = this.angles[1] * RAD;
      return [Math.sin(b), -Math.sin(a) * Math.cos(b), Math.cos(a) * Math.cos(b)];
    }
  }
  const api = { GimbalSimulation, DirectionCoverage, reduceCurvePoints, linearGravityAxis, RAD, wrap, RPM_TO_DEG_PER_SECOND };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.GimbalPhysics = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
