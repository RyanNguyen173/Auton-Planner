/* Planner → simulator conversion (js/bridge.js). */
import test from 'node:test';
import assert from 'node:assert/strict';
import { routeToSimProject } from '../js/bridge.js';
import { simulate } from '../js/sim/core.js';

const params = { wheel: 3.25, cart: 600, gIn: 36, gOut: 48, motors: 6, mass: 12, eff: 80, track: 11.5, vMode: 'derived', vMan: 55, aMode: 'derived', aMan: 120, turnPct: 70, curv: true, aLat: 180 };
const L = (x, y, o = {}) => ({ type: 'line', x, y, c1: null, c2: null, rev: false, lock: null, spd: 100, ...o });
const B = (x, y, c1, c2, o = {}) => ({ type: 'bezier', x, y, c1: { x: c1[0], y: c1[1] }, c2: { x: c2[0], y: c2[1] }, rev: false, lock: null, spd: 100, ...o });
const robot = (segs, actions = [], start = { x: -60, y: -40, h: 90 }) => ({ name: 'Robot 1', w: 15, l: 15, params, start, segs, actions });

test('lines become turn + move to point; reversed lines drive backwards', () => {
  const p = routeToSimProject({ robot: robot([L(-36, -40), L(-36, -20), L(-36, -40, { rev: true })]) });
  assert.deepEqual(p.cmds.map(c => c.type), ['moveToPoint', 'turnHeading', 'moveToPoint', 'moveToPoint']);
  assert.equal(p.cmds[1].h, 0);
  assert.equal(p.cmds[3].back, true);
});

test('smooth Bezier runs merge into one path; corners split it', () => {
  const smooth = routeToSimProject({ robot: robot([B(-36, -24, [-50, -40], [-40, -30]), B(-30, 0, [-32, -18], [-30, -10])]) });
  const paths = smooth.cmds.filter(c => c.type === 'followPath');
  assert.equal(paths.length, 1); assert.equal(paths[0].anchors.length, 3);
  const corner = routeToSimProject({ robot: robot([B(-36, -24, [-50, -40], [-40, -30]), B(-20, -24, [-30, -24], [-24, -24])]) });
  assert.equal(corner.cmds.filter(c => c.type === 'followPath').length, 2);
});

test('waiting actions become waits, moving actions are counted', () => {
  const p = routeToSimProject({ robot: robot([L(-36, -40)], [{ wp: 1, kind: 'scorePin', dur: 0.7, mode: 'wait' }, { wp: 1, kind: 'intakePin', dur: 0.4, mode: 'moving' }]) });
  const w = p.cmds.find(c => c.type === 'wait');
  assert.equal(w.ms, 700); assert.equal(p.source.skipped, 1);
});

test('drivetrain and manual limits carry over', () => {
  const r = robot([L(-36, -40)]); r.params = { ...params, vMode: 'manual', vMan: 42, aMode: 'manual', aMan: 80 };
  const p = routeToSimProject({ robot: r, planned: 3.2 });
  assert.equal(p.cfg.drive.motors, 6); assert.equal(p.cfg.traj.v, 42); assert.equal(p.cfg.traj.a, 80);
  assert.deepEqual(p.cfg.robot, { w: 15, l: 15 }); assert.equal(p.source.planned, 3.2);
});

test('a converted route simulates without timeouts', () => {
  const p = routeToSimProject({ robot: robot([L(-36, -40), B(-24, -12, [-30, -40], [-24, -24], { lock: 0 }), L(-24, -2), L(-30, -2)], [{ wp: 2, kind: 'scorePin', dur: 0.5, mode: 'wait' }]) });
  const R = simulate(p);
  for (const r of R.res) assert.notEqual(r.reason, 'Timeout');
});
