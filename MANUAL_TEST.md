# BetterReplay: in-sim test checklist

Work through these in order. Each step says what to do, what you should see, and the console lines to look for. Every module log line starts with `[BetterReplay]`.

**If anything differs, note the step and copy the surrounding console lines.** Most of the unknowns flagged in the README show up here as a specific warning.

## 0. Setup

- [ ] Close the sim. Run `.\build.ps1 -Configuration Debug -Package -Install`.
  - Debug builds log every command and freeze check.
  - `-Package` runs `fspackagetool` once to compile the panel `.spb`. It briefly starts the sim in builder mode.
  - **Expect:** `cached InGamePanel_BetterReplay.spb in package\spb\` and no `No compiled panel` warning.
  - **If the `.spb` step fails:** read `%APPDATA%\Microsoft Flight Simulator 2024\BuilderLogError.txt` and note the error. The package still installs without the panel. Use the console log for everything below, and check steps 2–9 via the files in the work folder.
- [ ] Check that `…\Community\betterreplay-recorder\` contains `layout.json`, `manifest.json`, `modules\BetterReplay.wasm`, `html_ui\…` and `InGamePanels\InGamePanel_BetterReplay.spb`.
- [ ] Enable Developer Mode (Options → General → Developers). Open the Console from the DevMode toolbar (Tools → Console) and filter on `BetterReplay`.

## 1. Module loads (main menu)

- [ ] Start the sim and wait for the main menu. **Expect these lines:**
  ```
  [BetterReplay] BetterReplay 0.1.0 starting
  [BetterReplay] work folder: '\work\'
  [BetterReplay] no index found; starting a new library        (first run only)
  [BetterReplay] library: 0 clip(s)
  [BetterReplay] SimConnect ready: 37 channels + 6 extras per frame
  [BetterReplay] CommBus: listening on BetterReplay.Cmd
  [BetterReplay] connected to … (SimConnect …)
  ```
- [ ] **No** `SimConnect exception` warnings. If there are any, copy them. They name the call, e.g. `NAME_UNRECOGNIZED (7) from "LIGHT NAV"`.
- [ ] `ERROR no writable work folder`: note it. Clips can't be saved.
- [ ] `no SimConnect messages 2 s after init; polling …`: note it. The module switches to polling and should still work. This tells us how 2024 dispatch behaves.
- [ ] The work folder exists: `%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\`.

## 2. Flight start and recording

- [ ] Start a Free Flight on a runway, engines running (e.g. a C172). **Expect:**
  ```
  FlightLoaded: …            /  flow event FLT_LOADED …
  flight loaded (…): buffer cleared
  detector: detector disarmed (…); arming in 10 s
  sim running
  first data frame: lat … lon … alt … ft, AGL … ft, on ground 1, sim time …
  recording resumed (was: …)
  aircraft: <aircraft title>
  detector: detector armed (on ground, AGL 0 ft, GS 0 kt)     about 10 s later
  ```
- [ ] Sanity-check the first data frame:
  - lat/lon are your real position in **degrees**
  - AGL is about 0
  - on ground = 1
- [ ] **No clip is created by the spawn**: no `saved` line.
- [ ] After a minute: `status: recording, phase ground, buffer 60 s, …` (the heartbeat repeats every 60 s).

## 3. Toolbar panel

- [ ] The BetterReplay icon (circular arrow with a play triangle) is in the toolbar. If not, look in the toolbar settings list.
- [ ] Open it:
  - The status dot is green, with **Recording · on ground · buffer m:ss · <aircraft>**.
  - The buffer counts up, and the list says "No clips yet".
- [ ] Close and reopen the panel. The state is the same; the panel holds no state.
- [ ] If the panel says *"No response from the recorder"*: note whether section 1's lines appeared. This means CommBus isn't reaching the module.

## 4. Takeoff

- [ ] Taxi to the runway, stop briefly, then take off normally and climb past 1,000 ft AGL. **Expect:**
  ```
  detector: liftoff: takeoff candidate (roll began N s earlier)
  detector: takeoff confirmed (5.0 s airborne, AGL N ft)
  detector: takeoff finished: N s clip (liftoff at +N s)
  saved YYYY-MM-DD_HHMM_takeoff.rpl: N s, N frames, N KB
  ```
- [ ] The panel shows **TO · Takeoff · <local date/time>** with the aircraft and duration.
- [ ] Check the clip: `node tools\rpl-dump.mjs "<work folder>\<file>.rpl"`.
  - It starts with ground speed under 10 kt.
  - `ongnd` goes 1 → 0 at the liftoff offset.
  - It ends near 1,000 ft AGL or 60 s after liftoff.

## 5. Landing

- [ ] Fly a pattern and make a normal landing, then roll out below 30 kt. **Expect:**
  ```
  detector: touchdown: landing candidate (VS -N fpm, GS N kt)
  detector: landing confirmed (3.0 s on the ground, 0 bounce(s))
  detector: landing finished: N s clip, VS -N fpm, peak N.NN G, 0 bounce(s)
  saved YYYY-MM-DD_HHMM_landing.rpl: …
  ```
- [ ] The panel shows **LDG · Landing**, with VS, G and duration (about 120 s plus the rollout).
- [ ] **Compare the VS with another source** (a landing-rate tool, or the sim's own). The clip uses `PLANE TOUCHDOWN NORMAL VELOCITY`; note its sign and magnitude.

## 6. Bounce

- [ ] Make a firm landing that bounces, then settle and roll out.
  - **Expect** `landing: bounced, airborne N s after touchdown` and `touchdown again (bounce 1)`.
  - Exactly **one** landing clip, with `1 bounce`.
  - The VS is from the first touchdown.

## 7. Touch-and-go

- [ ] Land, stay on the ground for more than 3 s above 30 kt, then take off again.
  - **Expect** `landing: airborne again`, then a landing clip.
  - Then a takeoff clip that starts at the touchdown.

## 8. Pause mid-landing

- [ ] On short final (below 200 ft), pause (Esc or P). Wait 30 s, unpause and land. **Expect:**
  - `pause state 0x…`, `recording suspended: paused`, then `recording resumed (was: paused)`
  - The landing is still saved.
- [ ] Repeat, pausing right after touchdown. The landing is saved, and its duration doesn't include the pause.
- [ ] *(Optional)* Pause and move the aircraft from the pause/map menu if possible, then unpause.
  - **Expect** `detector: aircraft moved N m / N ft while paused` and `detector disarmed (relocated during pause)`.
  - No false clip.
- [ ] Note which pause flags appeared. `0x1` = full pause, `0x4` = active pause, `0x8` = sim pause.

## 9. Slew, spawns, menus

- [ ] In flight, enable slew (Y), move the aircraft, then disable it. **Expect:**
  - `recording suspended: slew`, then `recording resumed (was: slew)` and `detector disarmed (slew); arming in 10 s`
  - No clip.
- [ ] Start a new flight **in the air**: no clip.
- [ ] Start a new flight **on a runway**: no clip.
- [ ] Return to the main menu: `flow event BACK_TO_MAIN_MENU`, `back at the main menu`, and recording suspended.
- [ ] Use the sim's own replay, if available: `sim replay/cinematic started` and recording suspended.

## 10. Manual save

- [ ] After at least 5 minutes of flight, click **Save last 5 min**.
  - **Expect** `saved …_manual.rpl` and the toast "Saved the last few minutes".
  - The clip is about 5 minutes long.
- [ ] The button is disabled while a clip is playing.

## 11. Playback

- [ ] In flight (any phase), click **Play** on a landing clip. **Expect:**
  ```
  playback of …_landing.rpl (N s, N frames, 37/37 channels); aircraft snapshot taken
  playback: writing 18 channels per frame; freezing position/altitude/attitude
  DEBUG lat/lon freeze on confirmed        (Debug build; x3)
  ```
  - If you see `freeze still off after SET; sending TOGGLE`, note it. Note whether it then worked.
  - If you see `could not turn … freeze on`, playback will fight the sim.
- [ ] The aircraft jumps to the clip start and flies the recorded path smoothly. Note any jitter, drift or stutter.
- [ ] The panel shows **Replaying** (blue) and the player bar, with the amber touchdown mark at 2:00.
- [ ] Check visually, and note what doesn't follow the clip:
  - gear position and gear handle
  - flaps
  - spoilers
  - ailerons, elevator and rudder
  - landing, taxi, nav, beacon and strobe lights (`DEBUG playback: LANDING_LIGHTS_SET 1` …)
- [ ] **Speed:** 0.25x, 0.5x, 2x and 4x all change the playback rate.
- [ ] **Seek:** click and drag on the bar. The aircraft jumps to the new point and the time label matches.
- [ ] **Pause/Play:** the aircraft holds position, then continues. At the end it pauses, and the button says "Replay".
- [ ] **Stop. Expect:**
  ```
  playback: restored aircraft to lat … lon … alt … ft
  playback ended (stopped); aircraft restored
  recording resumed (was: restoring aircraft)
  detector: detector disarmed (restoring aircraft); arming in 10 s
  ```
  - The aircraft is back where it was, at the same attitude and speed. Lights are back as they were.
  - The aircraft is unfrozen and flies normally.
- [ ] Play a clip, then play a **different** clip without stopping, then Stop. You're returned to the position from *before the first* clip.
- [ ] While a clip plays, pause the sim. Playback holds, then continues when you unpause.
- [ ] Note whether crash detection ever triggered during playback (e.g. near the ground).

## 12. Library management

- [ ] **Keep** a clip (it turns amber, "Kept"). **Delete** another: the first click shows "Sure?", the second deletes it, and the file disappears from the work folder.
- [ ] Restart the sim. The library reloads (`loaded index.json (N clips)`) and keep flags are preserved.
- [ ] *(Optional)* Delete `index.json` while the sim is closed, then restart.
  - **Expect** `adopted … from disk` for each clip. Keep flags are reset.

## 13. Report back

Send:

- the console lines around anything that didn't match
- `BuilderLogError.txt`, if packaging failed
- the aircraft used
- for a suspicious clip, `node tools\rpl-dump.mjs <clip> --csv clip.csv`
