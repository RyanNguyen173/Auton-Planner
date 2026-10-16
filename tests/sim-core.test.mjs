/* Simulator core: the controllers converge with default gains, paths track, early exit behaves, old saves migrate.
   Run with `npm test` (node --test). */
import test from 'node:test';
import assert from 'node:assert/strict';
import { simulate, exampleProject, defCfg, plantOf, ffFromPlant, CMDS, normalizeProject, genTraj, slewVoltage } from '../js/sim/core.js';

const cmd = (type, o = {}) => ({ id: type + Math.random(), type, ...structuredClone(CMDS[type].def), ...o });
const project = (start, cmds, tweak) => { const p = exampleProject(); p.start = start; p.cmds = cmds; if (tweak) tweak(p.cfg); p.cfg.ff = ffFromPlant(plantOf(p.cfg)); return p; };
const small = cfg => { Object.assign(cfg.drive, { motors: 6, mass: 12, track: 11.5 }); cfg.robot = { w: 15, l: 15 }; };
const finite = R => R.log.every(l => Number.isFinite(l.x) && Number.isFinite(l.y) && Number.isFinite(l.uL) && Number.isFinite(l.uR));

test('example routine runs with no timeouts and no NaN', () => {
  const R = simulate(exampleProject());
  assert.ok(finite(R));
  for (const r of R.res) assert.notEqual(r.reason, 'Timeout');
});

for (const [label, tweak] of [['24 in robot', null], ['15 in robot', small]]) {
  test(`turns settle within 1.5° (${label})`, () => {
    for (const [from, to] of [[0, 90], [0, 180], [90, 135], [0, 20]]) {
      const R = simulate(project({ x: 0, y: 0, h: from }, [cmd('turnHeading', { h: to, timeout: 3000 })], tweak));
      assert.equal(R.res[0].reason, 'Settled', `${from}→${to}`);
      assert.ok(Math.abs(R.res[0].headErr) < 1.5, `${from}→${to} error ${R.res[0].headErr}`);
    }
  });
  test(`move to point settles within 1.5 in (${label})`, () => {
    for (const d of [12, 24, 48, 72]) {
      const R = simulate(project({ x: 0, y: -40, h: 0 }, [cmd('moveToPoint', { x: 0, y: -40 + d, timeout: 4000 })], tweak));
      assert.equal(R.res[0].reason, 'Settled', `${d} in`);
      assert.ok(R.res[0].posErr < 1.5, `${d} in error ${R.res[0].posErr}`);
    }
  });
}

test('move to point does not "settle" while facing sideways', () => {
  const R = simulate(project({ x: 0, y: 0, h: 0 }, [cmd('moveToPoint', { x: 4, y: 0, timeout: 3000 })]));
  assert.ok(R.res[0].reason === 'Timeout' || R.res[0].posErr < 2.5, `ended ${R.res[0].reason} with ${R.res[0].posErr} in error`);
});

test('RAMSETE and pure pursuit track forward and reversed paths', () => {
  const ex = exampleProject();
  for (const ctrl of ['ramsete', 'purepursuit']) for (const rev of [false, true]) {
    const path = structuredClone(ex.cmds[rev ? 6 : 0]); path.ctrl = ctrl;
    const start = rev ? { x: -24, y: -48, h: 180 } : ex.start;
    const R = simulate(project(start, [path]));
    assert.equal(R.res[0].reason, 'Settled', `${ctrl} rev=${rev}`);
    assert.ok(R.res[0].posErr < 2, `${ctrl} rev=${rev} end error ${R.res[0].posErr}`);
  }
});

test('trajectory respects wheel speed limit on curves', () => {
  const p = exampleProject(), tr = genTraj(p.cmds[0], p.cfg), W = p.cfg.drive.track * 0.0254, vmax = p.cfg.traj.v * 0.0254;
  for (let i = 0; i < tr.pts.length; i++) {
    const outer = Math.abs(tr.vprof[i]) * (1 + Math.abs(tr.k[i]) * W / 2);
    assert.ok(outer <= vmax * 1.001, `outer wheel ${outer} > ${vmax} at ${i}`);
  }
});

test('early exit: guide lands closer than chain, coast overshoots more than brake', () => {
  const run = mode => simulate(project({ x: -60, y: -40, h: 0 }, [cmd('moveToPoint', { x: -60, y: 30, exit: 8, exitMode: mode, exitMs: 1500 })])).res[0];
  const chain = run('chain'), coast = run('coast'), brake = run('brake'), guide = run('guide');
  for (const r of [chain, coast, brake, guide]) assert.equal(r.reason, 'Early exit');
  assert.ok(guide.posErr < 1, `guide ${guide.posErr}`);
  assert.ok(guide.posErr < chain.posErr);
  assert.ok(coast.posErr > brake.posErr);
  assert.equal(chain.tExit, null);
  assert.ok(guide.tExit != null && guide.exitMode === 'guide');
});

test('early exit works on turns and paths', () => {
  const t = simulate(project({ x: 0, y: -40, h: 0 }, [cmd('turnHeading', { h: 180, exit: 20, exitMode: 'guide', exitMs: 1500 })])).res[0];
  assert.equal(t.reason, 'Early exit'); assert.ok(Math.abs(t.headErr) < 1.5);
  const ex = exampleProject(), path = { ...structuredClone(ex.cmds[0]), exit: 8, exitMode: 'guide', exitMs: 1000 };
  const r = simulate(project(ex.start, [path])).res[0];
  assert.equal(r.reason, 'Early exit'); assert.ok(r.posErr < 2);
});

test('stop modes: hold stops shortest, coast rolls furthest', () => {
  const endY = stop => { const R = simulate(project({ x: -60, y: -40, h: 0 }, [cmd('moveToPoint', { x: -60, y: 30, minSpd: 60, exit: 2 }), cmd('wait', { ms: 800 })], c => { c.stop = stop; })); return R.log[R.log.length - 1].y; };
  const coast = endY('coast'), brake = endY('brake'), hold = endY('hold');
  assert.ok(coast > brake && brake > hold, `coast ${coast} brake ${brake} hold ${hold}`);
});

test('model mismatch shows up as real error (odometry drift)', () => {
  const p = exampleProject(); Object.assign(p.cfg.real, { odoScale: 3 });
  const R = simulate(p), worst = Math.max(...R.log.map(l => Math.hypot(l.x - l.ox, l.y - l.oy)));
  assert.ok(worst > 1.5, `odometry diverged only ${worst} in`);
  const clean = simulate(exampleProject());
  assert.ok(Math.max(...clean.log.map(l => Math.hypot(l.x - l.ox, l.y - l.oy))) < 0.01);
});

test('older saved projects migrate', () => {
  const p = exampleProject(); p.cfg.brake = false; delete p.cfg.ex; delete p.cfg.stop; p.cmds.forEach(c => { delete c.exitMode; delete c.exitMs; delete c.exit; });
  normalizeProject(p);
  assert.equal(p.cfg.stop, 'coast'); assert.ok(p.cfg.ex.glideA > 0);
  for (const c of p.cmds) if (c.type !== 'wait') { assert.equal(c.exit, 0); assert.equal(c.exitMode, 'chain'); }
  assert.ok(simulate(p).res.every(r => r.reason !== 'Timeout'));
});

test('voltage ramp bounds how fast commanded voltage can change', () => {
  const S0 = { uL: -12, uR: -12 };
  const P = plantOf(Object.assign(defCfg(), { real: { ...defCfg().real, rampMs: 200 } }));
  assert.ok(isFinite(P.uRate) && Math.abs(P.uRate - 60) < 1e-9, `uRate ${P.uRate}`);
  const u = slewVoltage(S0, P, 12, 12, 0.01);
  assert.ok(Math.abs(u.uL - (-12)) <= 60 * 0.01 + 1e-9, `jumped to ${u.uL} in one 10 ms tick`);
  assert.ok(u.uL > -12, 'did not move at all');
  const off = plantOf(defCfg());
  assert.equal(off.uRate, Infinity);
  const instant = slewVoltage(S0, off, 12, 12, 0.01);
  assert.equal(instant.uL, 12, 'rampMs=0 should behave exactly like before: no limiting');
});

test('voltage ramp smooths reversals and lowers peak saturation', () => {
  const cmds = [cmd('moveToPoint', { x: 0, y: 24, timeout: 3000 }), cmd('moveToPoint', { x: 0, y: -24, timeout: 3000 })];
  const sharp = simulate(project({ x: 0, y: 0, h: 0 }, cmds, c => { c.real.rampMs = 0; }));
  const smooth = simulate(project({ x: 0, y: 0, h: 0 }, cmds, c => { c.real.rampMs = 250; }));
  const maxStep = R => { let m = 0; for (let i = 1; i < R.log.length; i++) m = Math.max(m, Math.abs(R.log[i].uL - R.log[i - 1].uL), Math.abs(R.log[i].uR - R.log[i - 1].uR)); return m; };
  assert.ok(maxStep(smooth) < maxStep(sharp), `ramped step ${maxStep(smooth)} vs instant ${maxStep(sharp)}`);
  const satPeak = R => Math.max(0, ...R.res.filter(Boolean).map(r => r.sat));
  assert.ok(satPeak(smooth) <= satPeak(sharp) + 1e-9, `ramped saturation ${satPeak(smooth)} vs instant ${satPeak(sharp)}`);
});

test('older saved projects without a ramp field still migrate (default to instant)', () => {
  const p = exampleProject(); delete p.cfg.real.rampMs;
  normalizeProject(p);
  assert.equal(p.cfg.real.rampMs, 0);
  assert.ok(simulate(p).res.every(r => r.reason !== 'Timeout'));
});

test('defaults derive sensible plant constants', () => {
  const P = plantOf(defCfg());
  assert.ok(P.vMax / 0.0254 > 55 && P.vMax / 0.0254 < 65);
  assert.ok(P.kV > 7 && P.kV < 8.5);
});
