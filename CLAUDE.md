# Override Auton Planner — notes for Claude Code

A browser tool for a VEX U team (2026–27 game **Override**) with two views:

- **Route planner** (`js/planner/planner.js`): draw routes for both robots (24 in and 15 in) plus opponent ghosts, estimate timing with a tank-drive motion profile, score the autonomous period, check rules, and export a PNG and JSON.
- **Motion sim** (`js/sim/`): build a routine from motion commands (Move to point, Move to pose, Turn to heading, Turn to point, Follow Bezier path, Wait) and run it on a physics model of a tank drive with 10 ms control steps. The page shows telemetry.

The planner's **Simulate this route** button converts the selected robot's route into a sim routine (`js/bridge.js`).

## Run, test, deploy

- `npm run dev` serves the folder at http://localhost:5173. ES modules do not load from `file://`, so always use a server.
- `npm test` runs `node --test tests/*.test.mjs`. There are no dependencies.
- Deploy: static site. On Vercel, set the framework preset to **Other** and leave the build and output settings empty.
- There is no bundler or transpiler. Write plain modern JS modules that run in current browsers and Node 20+.

## Layout

```
index.html              shell: app bar (view switch, theme toggle) + both views' markup + #tip / #modal
css/tokens.css          Vesper design tokens, light and black-dark themes
css/app.css             layout and components
js/main.js              wires the shell to the views, hash routing (#plan / #sim)
js/shared/ui.js         theme, hover help (registerTips), modal + two-step confirmDelete, downloadBlob
js/shared/field-data.js Override field geometry (goals, toggles, loaders, tape, quadrants)
js/planner/planner.js   planner view: motion profile, scoring, rule checks, canvas, panels (one module)
js/sim/core.js          simulator engine, no DOM: plant model, trajectory generation, controllers, runner
js/sim/sim-ui.js        simulator view
js/bridge.js            planner route → sim project (pure)
tests/                  node:test suites for core.js and bridge.js
```

Each view is `init*(root, hooks)` and returns `{ show() }` (the sim also returns `loadProject`). Views query only inside their `root` (the `[data-pane]` section). Simulator DOM ids are prefixed `s-` so they never collide with planner ids. Keep it that way.

## Conventions that must not drift

- **Units in the UI:** inches from the field center, +X toward the blue station, +Y toward the top wall. Headings are compass degrees: 0° faces the top wall and angles increase clockwise, like the V5 inertial sensor.
- **Units inside `sim/core.js`:** SI and math heading (radians, 0 = +X, counter-clockwise). Convert only through `h2t` / `t2h` / `IN`.
- **Tank drive kinematics:** outer wheel speed = v·(1 + |κ|·W/2), so curve speed caps come from the outer wheel. In a point turn the sides run at ±ω·W/2.
- **Reversed paths:** v is negated, but ω = κ·(path speed) keeps its sign. The original spec said to negate both, which is wrong; the reversed-path tests catch it.
- **Pure pursuit** extends the path past its end along the final tangent so the lookahead distance never collapses.
- **Early exit** applies to every motion command (`exit`, `exitMode`: chain / coast / brake / guide, `exitMs`). Stop modes for waits are coast (open circuit), brake (shorted motor, which is 0 V in the plant model) and hold (position hold).
- **Saved data:** `normalizeProject()` in core.js fills in fields added later. Add new command or config fields there so older saves keep loading. Planner saves use `projectJSON()` and `loadProject()`.
- **Design system:** Vesper. Use the tokens in `css/tokens.css`; no literal colors in components. Text is sentence case. Uppercase is only for mono labels and tags. Line icons are 1.5 px. Alliance red and blue, the speed ramp and chart series are data colors kept separate from the sky accent. Canvas drawing reads CSS variables, so redraw on `themechange`.
- Every delete goes through `confirmDelete` (two steps). Every setting gets hover help: add its text to the view's `TIPS` and tag the element with `data-tip` or a `data-bind` the tagger knows.

## Game rules source

The VEX V5RC Override Game Manual **v2.1**, Section 6 (VEX U):
- 30 s autonomous; two robots, one 24 in and one 15 in.
- Auton bonus counts Midfield robots and Midfield yellow pins (VUG5).
- Win point (VUG6): 12 pin halves, 4 goals each holding 2 halves, no perimeter contact, and a robot in the Midfield.
- The Midfield goal starts with a yellow/yellow pin.

Field element positions come from the VEXcode VR GPS table. The tape geometry is measured from figures, so treat it as an estimate. When the manual updates, re-check `field-data.js` and the scoring in `planner.js`.

## Important constraint

VEX rule **<G4>** forbids competition robot code written or improved by LLMs. This project is a planning and analysis tool. Do not generate code meant to run on the team's competition robot, such as PROS, LemLib or VEXcode exports of these controllers. If asked, explain the rule and offer analysis instead.

## Known gaps and good next steps

- `planner.js` is one large module that mixes the motion profile, scoring and DOM. Splitting out a pure `planner/core.js` would let it be unit tested the way the simulator is.
- Skills mode in the planner reuses match scoring.
- Ideas: an odometry log overlay (paste real robot runs onto the field), drawing the simulator's path back onto the planner, opponent route presets.
