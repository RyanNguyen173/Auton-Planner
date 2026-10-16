/* App entry: wires the shell (view switch, theme, hover help, dialogs) to the two views. */
import { initTheme, initTooltips, initModal, registerTips } from './shared/ui.js';
import { initPlanner } from './planner/planner.js';
import { initSim } from './sim/sim-ui.js';
import { routeToSimProject } from './bridge.js';

const panes = { plan: document.querySelector('[data-pane="plan"]'), sim: document.querySelector('[data-pane="sim"]') };
const nav = document.getElementById('viewSeg');
let active = location.hash === '#sim' ? 'sim' : 'plan';

initTheme(document.getElementById('themeBtn'));
initModal();
initTooltips();
registerTips('shell', {
  theme: 'Switch between light and dark mode. Your choice is remembered in this browser.',
  'view-plan': 'Draw both robots\' routes on the field, estimate timing and check scoring and rules.',
  'view-sim': 'Run motion commands on a modeled tank drive and see how the controllers really behave.',
});

/* Views must not draw while hidden (their canvases have no size), so mark the start view first. */
for (const k in panes) panes[k].hidden = k !== active;
const sim = initSim(panes.sim, { isActive: () => active === 'sim' });
const planner = initPlanner(panes.plan, {
  isActive: () => active === 'plan',
  simulate: payload => { sim.loadProject(routeToSimProject(payload)); show('sim'); },
});
const views = { plan: planner, sim };

function show(v) {
  active = v;
  for (const k in panes) panes[k].hidden = k !== v;
  nav.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.view === v)));
  history.replaceState(null, '', '#' + v);
  views[v].show();
}
nav.addEventListener('click', e => { const b = e.target.closest('button[data-view]'); if (b && b.dataset.view !== active) show(b.dataset.view); });
window.addEventListener('hashchange', () => { const v = location.hash === '#sim' ? 'sim' : 'plan'; if (v !== active) show(v); });
show(active);
