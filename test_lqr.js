// Test lqr_explorer.html core functions with real numeric.js
const numeric = require('numeric');
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

// Strip event wiring at the end
const ewStart = js.lastIndexOf("/* ═════");
js = js.substring(0, ewStart);

// Mock browser DOM
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

// Use REAL numeric from npm
global.numeric = numeric;
console.log('numeric.js check: object ' + typeof numeric + ' eig: ' + typeof numeric.eig);

// Suppress CARE debug output in tests
js = js.replace('let _careDbg = true;', 'let _careDbg = false;');

// Remove "use strict" from eval'd code
js = js.replace('"use strict";', '');
eval(js);

function t(name, fn) {
  try {
    fn();
    console.log('PASS: ' + name);
  } catch(e) {
    console.log('FAIL: ' + name + ' - ' + e.message);
    console.log('  Stack: ' + e.stack.split('\n').slice(0,3).join(' | '));
  }
}

const motor = {R:4, L:0.02, Kt:0.06, J:0.002, B:2e-4};
const amp = {Vmax:12, tauDist:0, tDistOn:999};

t('computeMotorPoles', () => {
  const r = computeMotorPoles(4, 0.02, 0.06, 0.002, 2e-4);
  if (r.olPoles.length !== 3) throw new Error('len=' + r.olPoles.length);
});

t('simulateLQR full', () => {
  const s = simulateLQR(motor, {kTheta:10,kOmega:0.5,kCur:0.05,kInt:6}, amp, 2.0, 200);
  if (s.theta.length !== 200) throw new Error('len');
  console.log('  theta_end=' + s.theta[199].toFixed(4));
});

t('simulateLQR reduced', () => {
  const s = simulateLQR({R:4,L:0.001,Kt:0.06,J:0.002,B:2e-4}, {kTheta:10,kOmega:0.5,kCur:0.05,kInt:6}, amp, 2.0, 200);
  if (s.theta.length !== 200) throw new Error('len');
  console.log('  theta_end=' + s.theta[199].toFixed(4));
});

t('simulateServoPID', () => {
  const s = simulateServoPID(motor, {Kp:10,Kd:1,Ki:6}, amp, 2.0, 200);
  if (s.theta.length !== 200) throw new Error('len');
});

t('computeLQRGains (balanced)', () => {
  const g = computeLQRGains(motor, 100, 1, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
});

t('computeLQRPoles', () => {
  const p = computeLQRPoles(motor, [10, 0.5, 0.05, 6]);
  if (p.length !== 4) throw new Error('len=' + p.length);
});

t('LQR sign convention — negative kInt drives toward ref', () => {
  const s = simulateLQR(motor, {kTheta:10,kOmega:0.5,kCur:0.05,kInt:-1}, amp, 2.0, 200);
  const th = s.theta[199];
  if (th <= 0) throw new Error('theta_end=' + th.toFixed(4) + ' — should be positive when kInt < 0');
  console.log('  theta_end=' + th.toFixed(4) + ' (correct direction with neg kInt)');
});

t('analyzeDominantPoles with 4-pole LQR system', () => {
  const p = computeLQRPoles(motor, [10, 0.5, 0.05, 6]);
  const info = analyzeDominantPoles(p);
  if (!info) throw new Error('null result');
  console.log('  poles=' + p.length + ' zeta=' + (info.zetaEff||'null') + ' wn=' + (info.wnEff||'null'));
});

t('LQR sim with CARE gains (qInt=100, tMax=3)', () => {
  const g = computeLQRGains(motor, 100, 1, 0.1, 100, 0.1);
  if (!g) throw new Error('null');
  console.log('  CARE K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 3, 300);
  const th = s.theta[299];
  console.log('  theta_end=' + th.toFixed(4) + ' (t=3s)');
  if (th < 0.85 || th > 1.5) throw new Error('theta_end=' + th.toFixed(4) + ' out of range');
});

t('fallbackGains stabilize motor', () => {
  const g = fallbackGains(motor, 100, 1, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  fallback K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 2, 200);
  const th = s.theta[199];
  if (th < 0.5 || th > 1.5) throw new Error('theta_end=' + th.toFixed(4) + ' out of range [0.5, 1.5]');
  console.log('  theta_end=' + th.toFixed(4));
});

t('extreme: low qTheta (0.1), high R (10)', () => {
  const g = computeLQRGains(motor, 0.1, 1, 0.1, 10, 10);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 4, 400);
  const th = s.theta[399];
  if (th < 0 || th > 2) throw new Error('theta_end=' + th.toFixed(4) + ' out of range');
  console.log('  theta_end=' + th.toFixed(4));
});

t('extreme: low qOmega (0.01)', () => {
  const g = computeLQRGains(motor, 100, 0.01, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 4, 400);
  const th = s.theta[399];
  if (th < 0 || th > 2) throw new Error('theta_end=' + th.toFixed(4) + ' out of range');
  console.log('  theta_end=' + th.toFixed(4));
});

t('extreme: high qTheta (1000), low R (0.001)', () => {
  const g = computeLQRGains(motor, 1000, 1, 0.1, 10, 0.001);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 4, 400);
  const th = s.theta[399];
  if (th < 0 || th > 2) throw new Error('theta_end=' + th.toFixed(4) + ' out of range');
  console.log('  theta_end=' + th.toFixed(4));
});

t('low damping: qTheta=1, qOmega=0.5', () => {
  const g = computeLQRGains(motor, 1, 0.5, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 5, 500);
  var signChanges = 0, lastSign = 0;
  for (var i = 300; i < 500; i++) {
    var diff = s.theta[i] - 1.0;
    if (Math.abs(diff) > 0.15 && diff * lastSign < 0) signChanges++;
    if (Math.abs(diff) > 0.01) lastSign = diff > 0 ? 1 : -1;
  }
  console.log('  theta_end=' + s.theta[499].toFixed(4) + ' oscillations=' + signChanges +
    ' max=' + Math.max.apply(null, s.theta.slice(300)).toFixed(3) +
    ' min=' + Math.min.apply(null, s.theta.slice(300)).toFixed(3));
  if (signChanges > 10) throw new Error('Too many oscillations: ' + signChanges);
});

t('very low damping: qTheta=1, qOmega=0.1', () => {
  const g = computeLQRGains(motor, 1, 0.1, 0.1, 10, 0.1);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  const s = simulateLQR(motor, g, amp, 5, 500);
  var signChanges = 0, lastSign = 0;
  for (var i = 300; i < 500; i++) {
    var diff = s.theta[i] - 1.0;
    if (Math.abs(diff) > 0.15 && diff * lastSign < 0) signChanges++;
    if (Math.abs(diff) > 0.01) lastSign = diff > 0 ? 1 : -1;
  }
  console.log('  theta_end=' + s.theta[499].toFixed(4) + ' oscillations=' + signChanges +
    ' max=' + Math.max.apply(null, s.theta.slice(300)).toFixed(3) +
    ' min=' + Math.min.apply(null, s.theta.slice(300)).toFixed(3));
  if (signChanges > 20) throw new Error('Too many oscillations: ' + signChanges);
});

// User-reported oscillation case
t('oscillation case: qTheta=0.1, qOmega=0.03, qCur=0.9, qInt=3, R=0.001', () => {
  const g = computeLQRGains(motor, 0.1, 0.03, 0.9, 3, 0.001);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  console.log('  fallback=' + (g.fallback ? 'TRUE' : 'false'));
  const s = simulateLQR(motor, g, amp, 5, 500);
  // Check for oscillations
  var signChanges = 0, lastSign = 0;
  for (var i = 300; i < 500; i++) {
    var diff = s.theta[i] - 1.0;
    if (Math.abs(diff) > 0.15 && diff * lastSign < 0) signChanges++;
    if (Math.abs(diff) > 0.01) lastSign = diff > 0 ? 1 : -1;
  }
  var mx = Math.max.apply(null, s.theta.slice(200));
  var mn = Math.min.apply(null, s.theta.slice(200));
  console.log('  theta_end=' + s.theta[499].toFixed(4) + ' oscillations=' + signChanges +
    ' max=' + mx.toFixed(3) + ' min=' + mn.toFixed(3));
  if (signChanges > 25) throw new Error('Too many oscillations: ' + signChanges);
  // Verify stable: theta settles near 1.0
  if (s.theta[499] < 0.7 || s.theta[499] > 1.5) throw new Error('theta_end=' + s.theta[499].toFixed(4) + ' not settled');
});

// Additional edge case: very low qTheta + qOmega with low R
t('edge: qTheta=1, qOmega=0.03, qCur=0.1, qInt=10, R=0.001', () => {
  const g = computeLQRGains(motor, 1, 0.03, 0.1, 10, 0.001);
  if (!g) throw new Error('null');
  console.log('  K=[' + g.kTheta.toFixed(2) + ' ' + g.kOmega.toFixed(4) + ' ' + g.kCur.toFixed(4) + ' ' + g.kInt.toFixed(2) + ']');
  console.log('  fallback=' + (g.fallback ? 'TRUE' : 'false'));
  const s = simulateLQR(motor, g, amp, 5, 500);
  var signChanges = 0, lastSign = 0;
  for (var i = 300; i < 500; i++) {
    var diff = s.theta[i] - 1.0;
    if (Math.abs(diff) > 0.15 && diff * lastSign < 0) signChanges++;
    if (Math.abs(diff) > 0.01) lastSign = diff > 0 ? 1 : -1;
  }
  console.log('  theta_end=' + s.theta[499].toFixed(4) + ' oscillations=' + signChanges +
    ' max=' + Math.max.apply(null, s.theta.slice(200)).toFixed(3) +
    ' min=' + Math.min.apply(null, s.theta.slice(200)).toFixed(3));
  if (signChanges > 25) throw new Error('Too many oscillations: ' + signChanges);
  if (s.theta[499] < 0.7 || s.theta[499] > 1.5) throw new Error('theta_end=' + s.theta[499].toFixed(4) + ' not settled');
});

console.log('Done.');
