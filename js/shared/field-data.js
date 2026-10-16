/* VEX V5RC / VEX U "Override" (2026–27) field geometry, shared by the planner and the simulator.
   Coordinates are inches from the field center, +X toward the blue alliance station, +Y toward the top wall.
   Element positions come from the VEXcode VR Override GPS coordinates (mm, converted here).
   Tape geometry (Autonomous Line spacing, Midfield size, Load Zones) was measured from the Game Manual v2.1
   field figures because the manual has no dimension table for it; treat those numbers as estimates. */

export const HALF = 72;              /* half of the 12 ft field */
export const MM = 25.4;
export const GOAL_R = 3.5, TALL_R = 5; /* collision radii used for "clips a goal" warnings */

/* Midfield: diamond of white tape with corners at (±24,0) and (0,±24), bounded by the tape's inner edges. */
export const MID_IN = 23;
export const MIDPOLY = [{ x: MID_IN, y: 0 }, { x: 0, y: MID_IN }, { x: -MID_IN, y: 0 }, { x: 0, y: -MID_IN }];
/* Autonomous Line: double tape on the top-left → bottom-right diagonal. Red owns the left and bottom Quadrants. */
export const LINE_HALF = 1.5;
/** Signed distance from the Autonomous Line: < 0 red side, > 0 blue side. */
export const sideOf = (x, y) => (x + y) / Math.SQRT2;
export const LZ = { w: 13.3, h: 26 };
export const LOADZONES = [
  { a: 'red', x0: -HALF, x1: -HALF + LZ.w, y0: HALF - LZ.h, y1: HALF },
  { a: 'red', x0: -HALF, x1: -HALF + LZ.w, y0: -HALF, y1: -HALF + LZ.h },
  { a: 'blue', x0: HALF - LZ.w, x1: HALF, y0: HALF - LZ.h, y1: HALF },
  { a: 'blue', x0: HALF - LZ.w, x1: HALF, y0: -HALF, y1: -HALF + LZ.h },
];

const g = (x, y) => ({ x: x / MM, y: y / MM });
export const GOALS = [
  { id: 'C', name: 'Center tall goal', type: 'neutral', tall: true, ...g(0, 0) },
  { id: 'NT', name: 'Neutral goal · top', type: 'neutral', ...g(-600, 1200) },
  { id: 'NL', name: 'Neutral goal · left', type: 'neutral', ...g(-1200, 600) },
  { id: 'NR', name: 'Neutral goal · right', type: 'neutral', ...g(1200, -600) },
  { id: 'NB', name: 'Neutral goal · bottom', type: 'neutral', ...g(600, -1200) },
  { id: 'RL', name: 'Red goal · left', type: 'red', ...g(-1200, -600) },
  { id: 'RB', name: 'Red goal · bottom', type: 'red', ...g(-600, -1200) },
  { id: 'BT', name: 'Blue goal · top', type: 'blue', ...g(600, 1200) },
  { id: 'BR', name: 'Blue goal · right', type: 'blue', ...g(1200, 600) },
];
export const TOGGLES = [
  { id: 'T', name: 'Top wall toggle', ...g(0, 1780), horiz: true },
  { id: 'R', name: 'Right wall toggle', ...g(1780, 0), horiz: false },
  { id: 'B', name: 'Bottom wall toggle', ...g(0, -1780), horiz: true },
  { id: 'L', name: 'Left wall toggle', ...g(-1780, 0), horiz: false },
];
export const TOGGLE_SIZE = { len: 25.8, face: 2.05 };
export const LOADERS = [{ a: 'red', ...g(-1740, 1490) }, { a: 'red', ...g(-1740, -1490) }, { a: 'blue', ...g(1740, 1490) }, { a: 'blue', ...g(1740, -1490) }];
export const OBJECTS = [[-600, 1745], [600, 1745], [-1200, 1200], [1200, 1200], [-1745, 600], [-600, 600], [0, 600], [600, 600], [1745, 600], [-600, 0], [600, 0], [-1745, -600], [-600, -600], [0, -600], [600, -600], [1745, -600], [-1200, -1200], [1200, -1200], [-600, -1745], [600, -1745]].map(p => g(p[0], p[1]));

/** Quadrant a point belongs to: L / R / T / B (each is a triangle with one wall Toggle). */
export const quadOf = (x, y) => Math.abs(x) >= Math.abs(y) ? (x < 0 ? 'L' : 'R') : (y > 0 ? 'T' : 'B');
export const QUAD_ALLY = { L: 'red', B: 'red', T: 'blue', R: 'blue' };
export const goalQuad = gl => gl.id === 'C' ? 'M' : quadOf(gl.x, gl.y);
