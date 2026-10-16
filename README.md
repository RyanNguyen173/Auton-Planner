# Override Auton Planner

A top-down autonomous route planner for **VEX U Override (2026–27)**. Plan both robots' paths, see how long each route takes, check them against the game rules, and export a picture of the route for the team.

It is a single static page (`index.html`) with no build step and no dependencies. It uses the Vesper design system: Public Sans and Geist Mono, paper and sky surfaces, hairline cards, and a black dark mode. The sun/moon button in the toolbar switches themes; the choice is remembered per browser.

## What it does

- **Two-robot routes** drawn with straight lines or Bezier curves, plus opponent "ghost" robots to plan around.
- **Tank-drive motion profiles.** Each side's speed is limited separately, so curves slow the robot down and point turns spin one side forward and one back. Top speed and acceleration come from wheel size, cartridge, gearing, motor count, mass and efficiency, or you can enter measured values.
- **Actions at waypoints**: score pin, score cup, intake, flip toggle, or a custom step, each with a duration and either "waits" or "while moving".
- **Scoring and rule checks** from Game Manual v2.1:
  - pin-half visibility
  - toggle ownership
  - the yellow pin that starts in the Midfield goal
  - the VEX U auton bonus (VUG5) and win point (VUG6)
  - Autonomous Line crossings
  - starting-position rules
  - robot-to-robot and robot-to-field collisions
- **Exports**: route image (PNG) with a numbered action key, and project JSON you can commit to this repo.
- **Field image overlay**: paste or load a screenshot of the field to trace over.

## Run it locally

Open `index.html` in a browser, or serve the folder:

```bash
npx serve .
# or
python3 -m http.server 8000
```

## Deploy on Vercel

1. Push this folder to a GitHub repo.
2. On vercel.com, choose **Add New → Project** and import the repo.
3. Set **Framework Preset** to **Other**. Leave the build command and output directory empty.
4. Deploy. Vercel serves `index.html` at the root URL.

Or use the CLI from this folder: `npx vercel` (preview) and `npx vercel --prod`.

## Data and storage

- Saved routes and the field image are kept in each browser's local storage, so each teammate's library is their own.
- Use **Saves → Download .json** to share a routine, or to keep it in git.

## Notes and assumptions

- Field coordinates come from the VEX GPS layout, in inches from field center. Heading 0° faces the top wall, and angles increase clockwise, the same as a V5 inertial sensor.
- I estimated the tape spacing for the Autonomous Line and Midfield, and the Load Zone size, from the manual's field figures.
- All four toggles start yellow.
- Skills mode currently reuses the match scoring table.
- Re-check the rules whenever a new Game Manual version is released.
