/* Planner → simulator handoff.
   Converts one planner robot (start pose, line / Bezier segments, heading locks, actions, drivetrain settings)
   into a simulator project. Pure function: no DOM, covered by tests/bridge.test.mjs.

   Mapping
   - line segment            → Turn to heading (if not already facing it) + Move to point (backwards if reversed,
                                speed cap carried over)
   - run of Bezier segments   → one Follow path (RAMSETE). A run breaks at a corner, a heading lock,
                                a direction change, or a waypoint with a waiting action.
   - start of a path whose first tangent differs from the robot heading → Turn to heading first
                                (the planner does the same point turn)
   - heading lock             → Turn to heading after the segment (only when it differs from the arrival heading)
   - "waits" action           → Wait for its duration. "While moving" actions have no simulator equivalent and are counted.
   - drivetrain params        → simulator drivetrain; manual top speed / acceleration become the path speed limits */
import { CMDS, defCfg, plantOf, ffFromPlant, nid, wrap180, norm360, bearing } from './sim/core.js';

const clone = o => JSON.parse(JSON.stringify(o));
const r2 = v => Math.round(v * 100) / 100;
const CORNER_DEG = 4, TURN_DEG = 3;

/* Compass heading of the curve as it leaves p0 / arrives at its end point (handles may be zero length). */
function dirOut(p0, s) {
  if (s.type === 'bezier') for (const q of [s.c1, s.c2, s]) if (q && Math.hypot(q.x - p0.x, q.y - p0.y) > 1e-3) return bearing(p0.x, p0.y, q.x, q.y);
  return bearing(p0.x, p0.y, s.x, s.y);
}
function dirIn(p0, s) {
  if (s.type === 'bezier') for (const q of [s.c2, s.c1, p0]) if (q && Math.hypot(s.x - q.x, s.y - q.y) > 1e-3) return bearing(q.x, q.y, s.x, s.y);
  return bearing(p0.x, p0.y, s.x, s.y);
}

/**
 * @param {{robot:object, alliance?:string, planned?:number|null}} payload  as produced by the planner's "Simulate this route"
 * @returns {object} simulator project ({v,start,cfg,cmds,source})
 */
export function routeToSimProject({ robot: R, planned = null }) {
  const cfg = defCfg(), p = R.params || {};
  for (const k of ['wheel', 'cart', 'gIn', 'gOut', 'motors', 'mass', 'eff', 'track']) if (p[k] != null && isFinite(+p[k])) cfg.drive[k] = +p[k];
  cfg.robot = { w: R.w || 18, l: R.l || 18 };
  if (p.vMode === 'manual' && p.vMan > 0) cfg.traj.v = +p.vMan;
  if (p.aMode === 'manual' && p.aMan > 0) { cfg.traj.a = +p.aMan; cfg.traj.d = +p.aMan; }
  if (p.aLat > 0) cfg.traj.lat = +p.aLat;
  cfg.ffAuto = true; cfg.ff = ffFromPlant(plantOf(cfg));

  const mk = (type, o) => ({ id: nid(), type, ...clone(CMDS[type].def), ...o });
  const actions = R.actions || [], segs = R.segs || [], cmds = [];
  let skipped = 0;
  const waitsAt = wi => {
    for (const a of actions.filter(a => a.wp === wi)) {
      if (a.mode === 'wait') { if (a.dur > 0) cmds.push(mk('wait', { ms: Math.round(a.dur * 1000), note: a.label || a.kind })); }
      else skipped++;
    }
  };
  const wp = k => k === 0 ? R.start : segs[k - 1];
  const hasWait = wi => actions.some(a => a.wp === wi && a.mode === 'wait');
  const turnIfNeeded = (target, h) => { if (Math.abs(wrap180(target - h)) > TURN_DEG) cmds.push(mk('turnHeading', { h: r2(norm360(target)) })); return norm360(target); };

  let h = R.start.h;
  waitsAt(0);
  for (let i = 0; i < segs.length;) {
    const s = segs[i], from = wp(i), back = !!s.rev;
    if (s.type !== 'bezier') {
      /* The planner point-turns before each straight drive; Move to point stops steering inside 7.5 in, so add the turn */
      h = turnIfNeeded(bearing(from.x, from.y, s.x, s.y) + (back ? 180 : 0), h);
      cmds.push(mk('moveToPoint', { x: r2(s.x), y: r2(s.y), back, maxSpd: Math.min(100, Math.max(5, s.spd || 100)) }));
      h = norm360(bearing(from.x, from.y, s.x, s.y) + (back ? 180 : 0));
      if (s.lock != null) h = turnIfNeeded(s.lock, h);
      waitsAt(i + 1); i++; continue;
    }
    let j = i;
    while (j + 1 < segs.length) {
      const nx = segs[j + 1];
      if (nx.type !== 'bezier' || !!nx.rev !== back || segs[j].lock != null || hasWait(j + 1)) break;
      if (Math.abs(wrap180(dirIn(wp(j), segs[j]) - dirOut(segs[j], nx))) > CORNER_DEG) break;
      j++;
    }
    h = turnIfNeeded(dirOut(from, s) + (back ? 180 : 0), h);
    const anchors = [{ x: r2(from.x), y: r2(from.y), ix: 0, iy: 0, ox: r2(s.c1.x - from.x), oy: r2(s.c1.y - from.y) }];
    let spd = 100;
    for (let m = i; m <= j; m++) {
      const sg = segs[m], nx = m < j ? segs[m + 1] : null;
      spd = Math.min(spd, sg.spd || 100);
      anchors.push({ x: r2(sg.x), y: r2(sg.y), ix: r2(sg.c2.x - sg.x), iy: r2(sg.c2.y - sg.y), ox: nx ? r2(nx.c1.x - sg.x) : 0, oy: nx ? r2(nx.c1.y - sg.y) : 0 });
    }
    cmds.push(mk('followPath', { ctrl: 'ramsete', rev: back, spd: Math.max(5, spd), anchors }));
    h = norm360(dirIn(wp(j), segs[j]) + (back ? 180 : 0));
    if (segs[j].lock != null) h = turnIfNeeded(segs[j].lock, h);
    waitsAt(j + 1); i = j + 1;
  }
  return { v: 1, start: { x: r2(R.start.x), y: r2(R.start.y), h: r2(norm360(R.start.h)) }, cfg, cmds,
    source: { from: 'planner', name: R.name || 'Robot', planned: planned != null && isFinite(planned) ? planned : null, skipped } };
}
