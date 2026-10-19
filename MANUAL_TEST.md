# BetterReplay: in-sim test checklist

Work through these in order. Each step says what to do, what you should see, and the console lines to look for. Every module log line starts with `[BetterReplay]`.

The same lines are written to `betterreplay.log` in the work folder: `%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\`. The previous sim session's log is `betterreplay.prev.log`. So after a test you can send that file, or have it read from there, instead of copying lines from the console.

**If anything differs, note the step and copy the surrounding console lines.** Most of the unknowns in [Known gaps](DEVELOPMENT.md#known-gaps) show up here as a specific warning.

## 0. Setup

- [ ] Close the sim. Run `.\build.ps1 -Configuration Debug -Package -Install`.
  - Debug builds log every command and freeze check.
  - `-Package` runs `fspackagetool` once to compile the panel `.spb`. It briefly starts the sim in builder mode.
  - **Expect:** `cached InGamePanel_BetterReplay.spb in package\spb\` and no `No compiled panel` warning.
  - **If it stops with "Steam still thinks MSFS 2024 is running":** click Stop on the game in Steam, or exit and restart Steam, then run it again.
  - **If the `.spb` step fails:** read `%APPDATA%\Microsoft Flight Simulator 2024\BuilderLogError.txt` and note the error. The package still installs without the panel. Use the console log for everything below, and check steps 2–9 via the files in the work folder.
- [ ] Check that `…\Community\betterreplay-recorder\` contains `layout.json`, `manifest.json`, `modules\BetterReplay.wasm`, `html_ui\…` and `InGamePanels\InGamePanel_BetterReplay.spb`.
- [ ] Enable Developer Mode (Options → General → Developers). Open the Console from the DevMode toolbar (Tools → Console) and filter on `BetterReplay`.

## 1. Module loads (main menu)

- [ ] Start the sim and wait for the main menu. **Expect these lines:**
  ```
  [BetterReplay] BetterReplay 0.1.0 starting
  [BetterReplay] work folder: '\work\'
  [BetterReplay] log file started (UTC times; …)
  [BetterReplay] BetterReplay 0.1.0, debug logging on
  [BetterReplay] no index found; starting a new library        (first run only)
  [BetterReplay] library: 0 clip(s)
  [BetterReplay] SimConnect ready: 58 simvar channels + 18 extras per frame, 14 input event / L:var channels
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
- [ ] About 2 s after the flight loads: `input event channels: recording ie_thr_1=ENGINE_THROTTLE_1, ie_gear=LANDING_GEAR_GEAR…; not on this aircraft: …`. That says which of the aircraft's own controls get recorded, and from which input event.
- [ ] `aircraft path: …`, then `aircraft key …: no control profile yet` (or the profile's summary). Note the path and the key: control profiles are stored under the key, which variants share (`…/pmdg 737` for every PMDG 737, `…/a350` for the A350-900, -900ULR and -1000).
- [ ] Within 15 s: `airports: N around the aircraft`, with N above 0. Clips name their airport from these.
- [ ] **No clip is created by the spawn**: no `saved` line.
- [ ] After a minute: `status: recording, phase ground, buffer 60 s, …` (the heartbeat repeats every 60 s).

## 3. Toolbar panel

- [ ] The BetterReplay icon (circular arrow with a play triangle) is in the toolbar. If not, look in the toolbar settings list.
- [ ] Open it:
  - The status dot is green, with **Recording** and *On ground* under it, and the aircraft's name on the card below.
  - The buffer counts up and its bar fills towards 10:00. **Saved clips** at the bottom opens the library ("No clips yet" at first); the back arrow returns.
- [ ] Close and reopen the panel. The state is the same; the panel holds no state.
- [ ] If the panel says *"No response from the recorder"*: note whether section 1's lines appeared. This means CommBus isn't reaching the module.

## 4. Takeoff

- [ ] Taxi to the runway, line up, then take off normally with flaps out and climb until they're up. **Expect:**
  ```
  detector: liftoff: takeoff candidate (onto the runway N s earlier, flaps out)
  detector: takeoff confirmed (5.0 s airborne, AGL N ft)
  detector: takeoff: flaps up
  detector: takeoff finished: N s clip (liftoff at +N s)
  saved YYYY-MM-DD_HHMM_takeoff.rpl: N s, N frames, N KB, at XXXX
  ```
- [ ] Under **Saved clips**, a green **Takeoff** tag, the aircraft and the departure airport (*A350-900 | YMML*), and underneath the local date/time and duration.
  - If the `saved` line has no `, at …` (and the row no airport), note the airport and runway: it's further than 8 km from the liftoff, or the sim didn't list it.
- [ ] Check the clip: `node tools\rpl-dump.mjs "<work folder>\<file>.rpl"`.
  - It starts 10 s before the aircraft rolls onto the runway.
  - `ongnd` goes 1 → 0 at the liftoff offset.
  - It ends 5 s after the flaps are fully up (`flap_idx` 0, `flap_l` near 0).
- [ ] Line up and wait a couple of minutes before taking off: the clip starts about 30 s before the engines spool up, not at the line-up.
- [ ] Take off with the flaps up: the clip ends 2 minutes after liftoff (`takeoff: flaps were up at liftoff`).

## 5. Landing

- [ ] Fly a pattern, make a normal landing and vacate the runway without stopping. **Expect:**
  ```
  detector: touchdown: landing candidate (VS -N fpm, GS N kt)
  detector: landing confirmed (3.0 s on the ground, 0 bounce(s))
  ```
  then, 10 s after leaving the runway:
  ```
  detector: landing: off the runway
  detector: landing finished: N s clip, VS -N fpm, peak N.NN G, 0 bounce(s)
  saved YYYY-MM-DD_HHMM_landing.rpl: …
  ```
- [ ] Under **Saved clips**, a blue **Landing** tag, the aircraft and the airport you landed at, and underneath the date/time, duration (120 s plus the rollout and 10 s past the turn-off), VS and G.
- [ ] Land again and come to a stop on the runway: `landing: stopped`.
- [ ] Land, and before the landing clip is saved, go back to the main menu: the landing is saved up to that moment (`landing ends here: the recording stops`).
- [ ] **Compare the VS with another source** (a landing-rate tool, or the sim's own). The clip uses `PLANE TOUCHDOWN NORMAL VELOCITY`; note its sign and magnitude.

## 6. Bounce

- [ ] Make a firm landing that bounces, then settle and roll out.
  - **Expect** `landing: airborne again N s after touchdown` and `touchdown again (bounce 1)`.
  - Exactly **one** landing clip, with `1 bounce`.
  - The VS is from the first touchdown.

## 7. Touch-and-go

- [ ] Land, stay on the ground for a few seconds without stopping, then take off again.
  - **Expect** `landing: airborne again N s after touchdown`, then 15 s later `landing: airborne again for 15 s` and a landing clip that ends there.
  - Then, once the flaps are up again (or 2 minutes after the liftoff if they stayed up), a takeoff clip that starts 5 s before the engines spooled up again on the runway (it never rolled onto it).

## 8. Pause mid-landing

- [ ] On short final (below 200 ft), pause (Esc or P). Wait 30 s, unpause and land. **Expect:**
  - `pause state 0x…`, `recording suspended: paused`, then `recording resumed (was: paused)`
  - The landing is still saved.
- [ ] Repeat, pausing right after touchdown. The landing is saved, and its duration doesn't include the pause.
- [ ] *(Optional)* Pause and move the aircraft from the pause/map menu if possible, then unpause.
  - **Expect** `detector: aircraft moved N m / N ft while paused` and `detector disarmed (relocated during pause)`.
  - No false clip. If you'd touched down before pausing, the landing is saved up to the pause.
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

- [ ] After at least 10 minutes of flight, click **Save last 10 minutes**.
  - **Expect** `saved …_manual.rpl` (no airport), and the button turning green with a tick and *Saved* for a couple of seconds before going back to **Save last 10 minutes**.
  - The clip is about 10 minutes long (about 5.5 MB), and the sim doesn't stutter noticeably while it saves.
- [ ] The button is disabled while a clip is playing.

## 11. Playback

- [ ] In flight (any phase), open **Saved clips** and click the play button on a landing clip. **Expect:**
  ```
  playback of …_landing.rpl (N s, N frames, 41/41 channels); aircraft snapshot taken
  playback: writing 22 channels per frame (2 engine(s)); freezing position/altitude/attitude; throttle via simvars, hardware throttle input blocked
  DEBUG lat/lon freeze on confirmed        (Debug build; x3)
  DEBUG playback: setting gear_c 1.00, gear_l 1.00, gear_r 1.00     (Debug build)
  ```
  - The second line also says how many of the aircraft's input events are replayed, and how the throttle is driven:
    - `throttle via the aircraft's input events`: the aircraft has `ENGINE_THROTTLE_n` and the clip recorded them. At the end, expect `input events followed the clip: ENGINE_THROTTLE_1, …`.
    - `throttle via simvars, hardware throttle input blocked`: the default for other aircraft.
    - `throttle via axis events (this aircraft overrides the lever simvar)`: this aircraft already showed it handles its levers itself (see below).
    - `no throttle data in this clip …`: the clip is older than throttle support. The levers stay with your hardware; record a new clip.
  - If you see `freeze still off after SET; sending TOGGLE`, note it. Note whether it then worked.
  - If you see `could not turn … freeze on`, playback will fight the sim.
- [ ] The aircraft jumps to the clip start and flies the recorded path smoothly. Note any jitter, drift or stutter.
- [ ] The log has `playback: position, attitude and velocities set directly every frame (N of 60 frames off)` with N near 0, and `playback: the rotation rates follow the clip (written x0.305)` a few seconds in.
- [ ] The engine sound stays steady at speed, in the cabin near the engines too: no flutter, and no squeaks when the sim stutters for a frame. If you hear one, note roughly when in the clip, and replay it with the trace on (`kPlaybackTrace`): its `wall_dt_ms` and `step_ms` columns show the frame times and the step the replay took for each. On the A350, small pitch jitters in the fan's whine on FLX takeoffs from about 80 kt are a known open issue (see *A350 engine whine in replays* in [Known gaps](DEVELOPMENT.md#known-gaps)).
- [ ] The panel switches to the player: the clip's name, a large seek bar with the amber touchdown mark at 2:00, round play/pause and stop buttons, and the speeds.
- [ ] The PFD airspeed and vertical speed follow the clip (body velocities are written every frame), not the speed you had before pressing Play.
- [ ] **Throttle** (clips recorded with throttle support): the thrust levers follow the clip.
  - Simvar mode: moving your hardware throttle has no effect while it plays. On Stop you'll see `playback: hardware throttle input released (N input event(s) were blocked)`.
  - If the aircraft overrides the lever simvar, within about half a second of the clip's lever differing from yours you'll see `playback: the aircraft overrode the throttle lever simvar in N% of the last 30 frames; driving the levers with THROTTLEn_AXIS_SET_EX1 events …`. From then on the levers should follow the clip for that aircraft. Your hardware lever isn't blocked in this mode, so don't touch it.
    - At the end you'll see either `throttle levers followed the clip via axis events` or `… were still more than 3% away from the clip …`. The second means the aircraft needs its own variables (LVars, future work). Copy the `aircraft input event: …THROTTLE…` lines.
  - Note the aircraft, the mode line, N and the follow/away line.
- [ ] **Reversers** (clips recorded with reverser support): play a landing where you used reverse.
  - **Expect** `playback: rev_1 1 + SET_THROTTLE1_REVERSE_THRUST_ON` (and the other engines) at the point you engaged reverse, and `… 0 + SET_THROTTLE1_REVERSE_THRUST_OFF` when you stowed it.
  - The reverser doors deploy on the aircraft, and the engines spool up in reverse.
  - At the end: `all N checked light/reverser channels followed the clip`, or `rev_1 (…) didn't follow the clip in N% of frames`. The second means this aircraft ignores the reverse thrust events.
- [ ] At the end of each playback, look for `playback: <channel> (...) was overridden by the aircraft in N% of frames`. That names any channel (throttle, flaps, spoilers…) the aircraft's own systems keep overwriting. `all N written channels held their values` means nothing fought the replay.
- [ ] Check visually, and note what doesn't follow the clip:
  - gear: playback commands the gear when the *recorded gear* starts moving and lets the sim animate it, so expect no flicker. It can lag the recording by the aircraft's own door/sequence time, and more after a seek. Look for `playback: GEAR_SET 0 + GEAR_UP` (or `GEAR_SET 1 + GEAR_DOWN`).
    - If you then see `the aircraft's gear did not move 3 s after GEAR_SET/GEAR_UP`, this aircraft ignores the legacy gear events. Copy the `aircraft input event: …` lines logged about 2 s after the flight loaded. They name the aircraft's own gear control, which the next version can drive instead.
  - flaps
  - spoilers
  - ailerons, elevator and rudder
  - landing, taxi, nav, beacon and strobe lights (`DEBUG playback: LANDING_LIGHTS_SET 1` …)
- [ ] **Speed:** 0.25x, 0.5x, 2x and 4x all change the playback rate.
- [ ] **Seek:** click and drag on the bar. The aircraft jumps to the new point and the time label matches.
  - The gear jumps with it: seek from gear-up (after the takeoff) back to the takeoff roll and the gear is down straight away, not lowering.
  - If you see `playback: gear_c is 0.00 right after being set to 1.00; the aircraft moves it back itself`, this aircraft won't take gear positions. It animates the gear from the command instead.
- [ ] **Pause/Play:** the aircraft holds position, then continues. At the end it pauses, and the big button shows the replay icon. Replay starts over with the gear and lights as they are at the clip's start.
- [ ] **Stop. Expect:**
  ```
  playback: restored aircraft to lat … lon … alt … ft
  playback ended (stopped); aircraft restored
  recording resumed (was: restoring aircraft)
  detector: detector disarmed (restoring aircraft); arming in 10 s
  ```
  - The aircraft is back where it was, at the same attitude and speed. Lights and gear handle are back as they were.
  - The aircraft is unfrozen and flies normally.
- [ ] **Stop on the ground after a clip that ends gear-up** (e.g. a takeoff clip, played while parked). The gear snaps back down at once.
  - **Expect** `playback: waiting for the gear before unfreezing`, then `playback: gear down; unfreezing` within a second or so.
  - If the aircraft won't take the snap, the wait lasts until its gear has lowered by itself, 10 s at most. It stays frozen meanwhile, so it doesn't drop onto its belly. The main screen shows **Unfreezing in N s** (*Waiting for the gear to come down*), counting down, with the blue dot draining. At 10 s: `playback: the gear isn't back down yet (at …); unfreezing anyway`.
  - Then **Recording** again, and the buffer counts up.
- [ ] **Several replays in a row on the A350, parked** (a takeoff clip, Stop, another clip, Stop, and so on). Each Stop unfreezes within 10 s, and recording resumes in between (`recording resumed (was: restoring aircraft)`). Before, from the second Stop on, the aircraft stayed frozen for half a minute: the A350 reports its nose gear up for a while after a replay, and the next Stop waited for the gear to match that.
- [ ] **Stop mid-air on the A350, parked** (a takeoff clip, Stop after the gear is up). **Expect** `playback: lowering the gear before putting the aircraft back on the ground (gear at …)`, then the aircraft back at the gate with its gear down within the 10 s. With a 5 s limit it was put back with its gear 55% down (`the gear is still coming down (at 0.55)`): the A350's gear takes longer than 5 s from part way. If you still see that warning, **note the gear value** and whether the aircraft sank.
- [ ] Play a clip, then play a **different** clip without stopping, then Stop. You're returned to the position from *before the first* clip.
- [ ] While a clip plays, pause the sim. Playback holds, then continues when you unpause.
- [ ] Note whether crash detection ever triggered during playback (e.g. near the ground).
- [ ] **Another variant:** in the A350-900ULR, play a clip recorded in the -900 (or the other way round). **Expect** it to be refused (*Recorded in the A350-900 (Default Cabin): switch to that aircraft to replay it*, and `playback of … refused` in the log), while the control profile stays shared: the log shows the same `aircraft key simobjects/airplanes/a350` for both.

## 12. Control setup

Use an aircraft without a profile; the A320neo V2 is the interesting one. Start at the gate.

- [ ] Open the panel. **Expect** the whole panel to show *Get ready: Set up replay controls for this aircraft*, with a checklist: *Parking brake set*, *On a runway*, *Engines running*. Start the engines, taxi onto the runway and set the parking brake; each item should tick as it becomes true.
  - If *Parking brake set* doesn't tick with the brake set, the aircraft's brake isn't the `BRAKE PARKING POSITION` simvar: note the aircraft. **Later**, then **Set up** on the aircraft card, starts the setup anyway.
  - If *On a runway* never ticks on a runway, note the airport and runway: `ON ANY RUNWAY` isn't set there. **Later**, then **Set up** on the aircraft card, starts the setup anyway.
- [ ] Once all three are ticked, **expect** the page to change to *Set up replay controls for this aircraft?* with **Start setup**, **Later** and **Never for this aircraft**.
  - **Later** goes back to the normal panel until the next flight. The aircraft card under the status then says *Not configured* in red, and **Set up** there still starts it.
- [ ] Click **Start setup**. The steps show across the top, the current one highlighted. **Expect** `control setup started on the ground: spoilers, flaps, throttle, gear (…)` and `recording suspended: control setup`. For each step, expect `setup scan: N input event(s), K of them readable, and M L:var(s)`.
  - **Note K.** If it's 0, input events can't be read this way, and only L:vars can be found by their movement.
- [ ] **Spoilers:** the panel says *Deploy the spoilers*; deploy them fully and hold. A green tick appears, then *Retract the spoilers*; retract them and wait. Note whether each tick comes at the right moment. Then *Deploy the spoilers* once more: deploy them and leave them out. The setup retracts them itself, trying each method in turn (`control setup: … brought the spoilers back from where you left it`). **Expect:**
  ```
  control setup: the spoilers started moving (…)
  control setup:   input event … moved with it (score …, peak …)      (and/or L:var …)
  control setup: you moved the spoilers over N s (…); trying N method(s)
  control setup: replaying the spoilers with … (method 1 of N)
  ```
  - Keep your hands off the controls while it replays.
  - Each method that doesn't move the spoilers is logged as `… moved the spoilers N% as far as you did`, and the next one starts by itself.
  - When one passes, the panel asks *Did the spoilers come up and go back down?* Watch the spoilers (the external view helps), then answer. **Watch again** replays it once more.
  - **Copy the `moved with it` lines, and note which method you said yes to.** On the A320neo V2 this step is the most likely to find something new, since it has no speed brake input event.
- [ ] **Flaps:** one notch, wait for them to extend, retract them, and wait; then one notch once more, and leave them there. Expect the same pattern. Going by your earlier test, `HANDLING_FLAPS` with its value as read should pass.
- [ ] **Throttle:** set the parking brake first (the panel says so if it isn't set). Follow the instructions: about halfway and hold it until the ring fills and ticks, then full and hold it, then back to idle, and wait for the engines to spool down. Each method can take up to half a minute while the engines spool.
  - **Expect** `control setup: throttle held at X% (hold 1 of 2)` and `(hold 2 of 2)`. If a hold never ticks while you're holding still, note the aircraft: its throttle simvar keeps moving faster than 2% a second while the lever is still.
  - If the aircraft starts to roll, the replay stops (`the aircraft started to roll …`), the throttle goes back to idle, and it asks anyway.
  - If the panel keeps saying *Set the parking brake* although the brake is set, the aircraft's brake isn't the simvar. Click **Continue anyway** and note it.
  - The log says what the throttle simvar did while you held the throttle: `it's the lever` (a normal aircraft: after the method's **Yes** the step ends there), `… it isn't the lever here` (the A320neo V2, CJ4, PMDG 737), or `held still, but read N% at most` / `couldn't tell` (after **Yes** the panel asks *Do the levers stay put under autothrust?*).
  - **A320neo V2 (and other aircraft whose simvar isn't the lever):** on the A320 hold in CL for halfway and TOGA for full. After the method's **Yes** expect `control setup: the throttle lever is L:var …` after lines listing each candidate's positions held, and straight on to *Hands off the throttle* (no second pass of the throttle). If the holds didn't show the lever's variable, the panel asks for them once more (*Couldn't find the lever's own position in that movement*). Then *Hands off the throttle*: click **Start**. The throttle moves by itself from idle to full and back to idle, then tries reverse, 20 to 40 s, the engines spooling against the parking brake. **Expect** `control setup: reverse thrust: …`:
    - A350: `the reverse thrust events didn't deploy the reversers`, `toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE) in`, `toggle reverse deployed the reversers (axis up to …; throttle …% at reverse idle, …% at most)`, `toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE) back out`, `out of reverse again (throttle …% at axis 0.15)`, then `reverse thrust: with toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE), the axis setting the amount`.
    - PMDG 737: `the reverse thrust events didn't deploy the reversers`, `toggle reverse didn't deploy the reversers` (it's toggled back out anyway), then `holding THROTTLE_DECR at idle deployed the reversers`, then `by holding THROTTLE_DECR`.
    - Then `control setup: throttle lever calibrated: N forward point(s), …, detents …` and the result `key events THROTTLEn_AXIS_SET_EX1, lever from L:var …, reverse …`. **Copy those lines.**
    - Afterwards the aircraft must be out of reverse: release the parking brake, push the throttle up a little and check it moves forward. `still in reverse after toggling back … toggling again` is fine once; **note it**. If the panel says *Couldn't get out of reverse thrust*, press your "toggle reverse" binding yourself and note it. `didn't come back to idle (…%); toggling anyway` means the throttle simvar was slower than 6 s to come back: note it too.
  - **Aircraft whose simvar is the lever, with reversers (PMDG 737):** after **Yes** (or **No** to *Do the levers stay put under autothrust?*) *Hands off the throttle* comes up anyway, for reverse only: **Start**. **Expect** `control setup: the throttle needs no lever calibration; trying reverse thrust`, then the reverse lines above and `throttle ok via key events THROTTLEn_AXIS_SET_EX1, reverse by holding THROTTLE_DECR`.
  - **Reverse in a replay:** on the ground with the parking brake off, record some reverse both ways the aircraft takes it ("decrease throttle" held at idle; "toggle reverse" then the throttle forward), save, play. **Expect** on the PMDG `controls: holding THROTTLE_DECR (the clip's reversers are out)` and `let go of THROTTLE_DECR`. The reversers should come out when they did in the clip.
  - **Partial reverse on the A350** (profile `reverse with toggle reverse`): record a landing or a taxi with idle reverse or a little more ("toggle reverse", the throttle just above idle), and another with more, save, play. **Expect** `controls: reverse thrust with toggle reverse while the clip's reversers are out` at the start; around each reverse `controls: toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE) in: the clip's reversers are out` and `… back out: the clip's reversers are in`; at the end `controls: toggle reverse sent N time(s) (0 correction(s)); a reverse amount in … engine frame(s)` with N even. The `engine N N1 was X% off the clip` lines should be small: before, every reverse replayed at full (N1 about 45% where the clip had about 21%). `the aircraft is in reverse after all` / `isn't in reverse after all` means a toggle didn't take and it toggled again: **note it**. Also try a seek out of the middle of a reverse, and **Stop** during one: both should leave the aircraft out of reverse (`playback: taking the throttle out of reverse before letting go of it` on Stop). Check it moves forward with the throttle up afterwards.
  - **Autothrust on the A350:** replay a landing flown on autothrust (levers in CL down the approach), and a takeoff clip that goes past the thrust reduction to CL. **Expect** `controls: autothrust in 1 stretch(es) of the clip (… s): the throttle follows its thrust demand there, the levers no further forward than the clip's` at the start, and at the end `controls: N engine frame(s) followed the clip's thrust demand (autothrust)`. The `engine N N1 was X% off the clip` lines should be a few percent at most: before, 32% on a landing and 45% on a save of the approach, at climb power all the way down.
    - Listen on the approach: approach power, rising and falling as it did, not climb power. Outside the stretches nothing changes: on the takeoff roll the levers stay in FLX, at the retard they go to idle, and reverse works as before.
    - In the cockpit the levers sit where the thrust needs them during the approach (just above idle), not in CL. That's expected.
    - If the engines hunt (the `changed direction` count well above the clip's) or N1 is off by more than about 5%, **copy the engine lines**: the follower's gains may need tuning for the real aircraft.
    - **The engine sound isn't distorted.** The first test (N1 1.1% off) sounded distorted, with `controls: 3404 value(s) sent to the throttle` in a 160 s replay, about 20 a second. Now `value(s) sent to the throttle` should be roughly 1 to 3 a second of replay, and N1 still within a few percent. If it's still distorted at that rate, the events aren't the cause: **note it**.
  - To redo only the throttle on an aircraft that's already set up: **Redo**, **Skip this step** on the spoilers and flaps (they keep their earlier setup), do the throttle, then **Finish later** at the gear.
  - Then record a takeoff and play it: **expect** `controls: throttle levers from L:var …` at the start and `controls: N engine frame(s) placed from L:var …` at the end; the levers should sit in the detents you used. Compare the `engine N N1 was X% off the clip …` lines with earlier playbacks.
- [ ] **Gear, on the runway:** stopped, the panel says *Lifting the aircraft off the runway* and the aircraft goes straight up, frozen, about 30 ft plus its height above the ground. **Expect** `control setup: holding the aircraft N ft above where it stood (… ft above the ground there), frozen, for the gear`, then within a second `the aircraft counts as airborne above the runway`. Follow *Raise the gear* and *Lower the gear*, then *Raise the gear* once more and leave it up: the setup lowers it itself, each way in turn. The replay then raises and lowers it once more. After **Yes** (or **Skip**, or **Cancel setup**), **expect** `putting the aircraft back where it stood on the runway (gear at 1.00)` and `put the aircraft back where it stood, lat … ; unfreezing`: the aircraft is back on its wheels where it was, not dropped, and not frozen.
  - **Note whether the gear moved at all while frozen**, both ways, and copy the lines: frozen in the air before a jump, the PMDG 777's and the A380's gear didn't come down, and that may have been the freeze. If the gear stays up, *Waiting for the gear to come down* turns into *Lower the gear* after 30 s; lower it yourself. If it still won't come down, **Cancel setup** puts the aircraft back as it is, gear up: try that only on an aircraft you don't mind belly-landing.
  - If the aircraft still counted as on the ground up there (`can't set up the gear above the runway: the aircraft still counted as on the ground there`), or the gear didn't move with its lever up, it goes back down and the panel says *Take off*, with a note saying why. Then take off, and follow the same instructions in the air, staying slow.
  - **PMDG 777:** **expect** `… #69927 didn't bring the gear back from where you left it (it didn't move)` and `… #69927 (536870912, then 131072) as a toggle brought the gear back from where you left it` (a left click on the lever), then the replay raising it with `#69927` (`… but didn't bring it back`) and from there `key events GEAR_SET/GEAR_UP/GEAR_DOWN brought the gear back from where the replay left it`. After **Yes**: `gear ok via down: key events GEAR_SET/GEAR_UP/GEAR_DOWN, then … as a toggle; up: the aircraft's own event #69927`. In replays, a way that doesn't lower the gear gives way to the next after about 6 s (`controls: the gear didn't go down with …; now …`), and `control profile: the gear down with … first from now on` when the second one did. Replay a clip that lowers the gear once after you've raised it yourself and once after the replay raised it. **Copy those lines.**
  - **Nothing is recorded meanwhile:** no clip from the hover, and after a takeoff for the gear, no `saved …_takeoff.rpl`.
- [ ] **Done:** the panel lists each control's result and method, under *Recording starts again now, with these controls*. **Expect** `recording resumed (was: control setup)`, `recording starts afresh after the control setup` and the buffer on the main screen starting again from 0:00. Click **Done**, then check:
  - the aircraft card under the status says *Configured* in green, or *Partially configured* in yellow if a step was skipped or didn't pass (hover it to see each control), and **Redo** replaces **Set up**
  - `aircraft.json` in the work folder
- [ ] Fly a pattern and land: the landing is saved as usual (it's after the setup). Play the landing clip. **Expect** `playback: spoilers via … (control profile)`, and the same for each set-up control. Note whether each control follows the clip now.
  - Clips from before the setup may log `… profile method … doesn't fit this clip …; using the defaults` instead. That's expected.
- [ ] Restart the flight: no setup screen, since the profile is known. **Redo** runs the setup again.

## 13. Library management

- [ ] **Delete** a clip: the bin turns into a red "Delete?"; clicking that deletes it, and the file disappears from the work folder.
- [ ] Restart the sim. The library reloads (`loaded index.json (N clips)`).
- [ ] *(Optional)* Delete `index.json` while the sim is closed, then restart.
  - **Expect** `adopted … from disk` for each clip.

## 14. Report back

Send:

- `betterreplay.log` from the work folder (or `betterreplay.prev.log` if the sim has been restarted since), or the console lines around anything that didn't match
- `BuilderLogError.txt`, if packaging failed
- the aircraft used
- for a suspicious clip, `node tools\rpl-dump.mjs <clip> --csv clip.csv`
