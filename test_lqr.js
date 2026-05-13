// Test lqr_explorer.html core functions
const fs = require('fs');
const src = fs.readFileSync('lqr_explorer.html', 'utf8');

// Find the inline script (after the CDN script tag)
const cdnEnd = src.indexOf('</script>');
const inlineStart = src.indexOf('<script>', cdnEnd + 9);
const inlineEnd = src.lastIndexOf('</script>');

let js = src.substring(inlineStart + 8, inlineEnd);

// Strip IIFE wrapper
const iifeStart = js.indexOf('{'); // after (function() {
js = js.substring(iifeStart + 1);
const iifeEnd = js.lastIndexOf('})();');
js = js.substring(0, iifeEnd);

// Strip event wiring at the end - find the event wiring comment block
const ewStart = js.lastIndexOf("/* ═════");
js = js.substring(0, ewStart);

// Mock browser
class MockElement {
  constructor(id) {
    this.id = id; this._value = ''; this._checked = false;
    this.style = { display: '' };
    this.classList = {
      _list: [],
      add(c) { if (!this._list.includes(c)) this._list.push(c); },
      remove(c) { this._list = this._list.filter(x => x !== c); },
      toggle(c, v) { if (v) this.add(c); else this.remove(c); },
      contains(c) { return this._list.includes(c); }
    };
    this._events = {};
    this.dataset = {};
  }
  get value() { return this._value; }
  set value(v) { this._value = v; }
  get valueAsNumber() { return parseFloat(this._value) || 0; }
  get checked() { return this._checked; }
  set checked(v) { this._checked = v; }
  addEventListener(ev, fn) {
    if (!this._events[ev]) this._events[ev] = [];
    this._events[ev].push(fn);
  }
  getBoundingClientRect() { return { width: 600, height: 400 }; }
  getContext() { return { save:()=>{}, restore:()=>{}, scale:()=>{}, clearRect:()=>{}, fillRect:()=>{}, fill:()=>{}, stroke:()=>{}, beginPath:()=>{}, moveTo:()=>{}, lineTo:()=>{}, arc:()=>{}, setLineDash:()=>{}, fillText:()=>{}, strokeText:()=>{}, translate:()=>{}, rotate:()=>{}, measureText:()=>({width:50}) }; }
  querySelectorAll(sel) { return []; }
}
const els = {};
global.document = {
  getElementById: (id) => { if (!els[id]) els[id] = new MockElement(id); return els[id]; },
  querySelectorAll: (sel) => [],
  body: { style: {} }
};
global.window = { devicePixelRatio: 2, addEventListener: () => {} };
global.Int8Array = Int8Array;
function matInv(M) {
  const n = M.length;
  const aug = M.map((row, i) => [...row, ...Array.from({length:n}, (_,j)=>i===j?1:0)]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(aug[r][c]) > Math.abs(aug[p][c])) p = r;
    [aug[c], aug[p]] = [aug[p], aug[c]];
    if (Math.abs(aug[c][c]) < 1e-12) return null;
    const d = aug[c][c];
    for (let j = c; j < 2*n; j++) aug[c][j] /= d;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = aug[r][c];
      for (let j = c; j < 2*n; j++) aug[r][j] -= f * aug[c][j];
    }
  }
  return aug.map(row => row.slice(n));
}
global.numeric = {
  rep: ([r,c], v) => Array.from({length:r}, () => Array(c).fill(v)),
  dot: (A, B) => { const ra=A.length,ca=A[0].length,cb=B[0].length; const C=Array.from({length:ra},()=>Array(cb).fill(0)); for(let i=0;i<ra;i++)for(let j=0;j<cb;j++)for(let k=0;k<ca;k++)C[i][j]+=A[i][k]*B[k][j]; return C; },
  inv: matInv,
  solve: (A, b) => {
    const Ainv = matInv(A);
    if (!Ainv) return null;
    const n = A.length, x = new Array(n);
    for (let i = 0; i < n; i++) {
      x[i] = 0;
      for (let j = 0; j < n; j++) x[i] += Ainv[i][j] * b[j];
    }
    return x;
  },
  eig: M => { const n=M.length;const lambda={x:[],y:[]};const E={x:Array.from({length:n},()=>[]),y:Array.from({length:n},()=>[])};for(let i=0;i<n;i++){lambda.x.push(-(i+1+Math.random()*0.1));lambda.y.push(i%3===0?0:1+i*0.1);for(let j=0;j<n;j++){E.x[j].push(i===j?1:0);E.y[j].push(0)}}return{lambda,E}; }
};

// Remove \"use strict\" from eval'd code (strict eval doesn't leak to enclosing scope)
js = js.replace('"use strict";', '');
eval(js);

function t(name, fn) { try { fn(); console.log('PASS: ' + name); } catch(e) { console.log('FAIL: ' + name + ' - ' + e.message); } }

t('computeMotorPoles', () => {
  const r = computeMotorPoles(4, 0.02, 0.06, 0.002, 2e-4);
  if (r.olPoles.length !== 3) throw new Error('len=' + r.olPoles.length);
});
t('simulateLQR full', () => {
  const s = simulateLQR({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},{kTheta:10,kOmega:0.5,kCur:0.05,kInt:6},{Vmax:12,tauDist:0,tDistOn:999},2.0,200);
  if (s.theta.length !== 200) throw new Error('len');
  console.log('  theta_end=' + s.theta[199].toFixed(4));
});
t('simulateLQR reduced', () => {
  const s = simulateLQR({R:4,L:0.001,Kt:0.06,J:0.002,B:2e-4},{kTheta:10,kOmega:0.5,kCur:0.05,kInt:6},{Vmax:12,tauDist:0,tDistOn:999},2.0,200);
  if (s.theta.length !== 200) throw new Error('len');
  console.log('  theta_end=' + s.theta[199].toFixed(4));
});
t('simulateServoPID', () => {
  const s = simulateServoPID({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},{Kp:10,Kd:1,Ki:6},{Vmax:12,tauDist:0,tDistOn:999},2.0,200);
  if (s.theta.length !== 200) throw new Error('len');
});
t('computeLQRGains', () => {
  const g = computeLQRGains({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},100,1,0.1,10,0.1);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
});
t('computeLQRPoles', () => {
  const p = computeLQRPoles({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},[10,0.5,0.05,6]);
  if (p.length !== 4) throw new Error('len=' + p.length);
});
t('LQR sign convention — negative kInt drives toward ref', () => {
  // LQR CARE solver gives kInt < 0 for motor plants (see comment in source).
  // With kInt negative, θ should go positive toward the reference.
  const s = simulateLQR({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},{kTheta:10,kOmega:0.5,kCur:0.05,kInt:-1},{Vmax:12,tauDist:0,tDistOn:999},2.0,200);
  const th = s.theta[199];
  if (th <= 0) throw new Error('theta_end=' + th.toFixed(4) + ' — should be positive when kInt < 0');
  console.log('  theta_end=' + th.toFixed(4) + ' (correct direction with neg kInt)');
});
t('analyzeDominantPoles with 4-pole LQR system', () => {
  const p = computeLQRPoles({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4},[10,0.5,0.05,6]);
  const info = analyzeDominantPoles(p);
  if (!info) throw new Error('null result');
  console.log('  poles=' + p.length + ' zeta=' + (info.zetaEff||'null') + ' wn=' + (info.wnEff||'null'));
});
t('LQR sim with CARE gains (qInt=100, tMax=3)', () => {
  const g = computeLQRGains({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4}, 100, 1, 0.1, 100, 0.1);
  if (!g) throw new Error('null');
  console.log('  CARE K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4}, g, {Vmax:12,tauDist:0,tDistOn:999}, 3, 300);
  const th = s.theta[299];
  console.log('  theta_end=' + th.toFixed(4) + ' (t=3s)');
  if (th < 0.85 || th > 1.5) throw new Error('theta_end=' + th.toFixed(4) + ' out of range');
});
t('fallbackGains stabilize motor', () => {
  const g = fallbackGains({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4}, 100, 1, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  fallback K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR({R:4,L:0.02,Kt:0.06,J:0.002,B:2e-4}, g, {Vmax:12,tauDist:0,tDistOn:999}, 2, 200);
  const th = s.theta[199];
  if (th < 0.5 || th > 1.5) throw new Error('theta_end=' + th.toFixed(4) + ' out of range [0.5, 1.5]');
  console.log('  theta_end=' + th.toFixed(4) + ' (fallback gains work)');
});
console.log('Done.');
