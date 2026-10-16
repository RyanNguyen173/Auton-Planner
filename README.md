# Override Auton Planner

Plan and test autonomous routines for **VEX U Override (2026–27)**. One app, two views:

- **Route planner.** Draw both robots' routes with straight lines or Bezier curves, add actions (score, intake, flip toggle), and see a tank-drive timing estimate, the auton score, bonus and win-point checks, rule warnings and collisions. Export a route image with an action key, or the project JSON.
- **Motion sim.** Chain Move to point, Move to pose (boomerang), Turn to heading, Turn to point, Follow Bezier path (RAMSETE or pure pursuit) and Wait. A modeled tank drive runs the routine, and the view charts wheel speed, voltage and error.
  - Every motion supports **early exit**. After exiting, the robot can chain into the next command, coast, brake, or guide itself to the target.
  - **Real-world effects** let the robot behave differently from what the controller assumes: wrong constants, odometry drift or gyro drift.

In the planner's Path tab, **Simulate this route** sends the selected robot's route to the simulator. Lines become Move to point, smooth Bezier runs become one Follow path, heading locks become turns, and waiting actions become waits.

## Run locally

```bash
npm run dev      # http://localhost:5173
npm test         # node:test suites, no install needed
```

The app uses ES modules, so open it through a server, not by double-clicking `index.html`.

## Deploy on Vercel

Import the repo, set the framework preset to **Other**, and leave the build command and output directory empty. Vercel serves `index.html` at the root. `/#sim` opens the simulator directly.

## Data

- Your current planner project, saved route library, field image, simulator routine and theme are kept in each browser's local storage.
- Use **Download .json** in either view's Saves tab to share a routine or commit it to the repo.

## Working on it with Claude Code

`CLAUDE.md` describes the architecture, unit and heading conventions, the design system rules, and the game manual sources. Read it first.

Competition robot code must be written by students (VEX rule G4). This repo is for planning and analysis only.
