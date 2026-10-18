# BetterReplay

An in-sim auto replay recorder for Microsoft Flight Simulator 2024. It records your flight continuously, saves every takeoff and landing as a clip automatically, and lets you browse and replay clips from a toolbar panel. Everything runs inside the sim, with no external app.

- **Recorder:** a standalone WASM module. It keeps a 5-minute ring buffer at 30 Hz and runs whether or not the panel is open.
- **Auto clips:** takeoffs and landings, including touchdown vertical speed, peak G and bounce count. They're written to disk as soon as each event finishes.
- **Toolbar panel:** shows recorder status and the clip list, and has play/pause, seek, speed (0.25x–4x), stop, delete and "save last 5 min".
- **Control setup:** a guided setup per aircraft that finds out how replays can move its spoilers, flaps, throttle and gear, by watching you move each one and testing ways to replay it. See [Control setup](#control-setup).

> **Status: MVP, early in-sim testing.** The platform-independent core has 118 unit tests and the module builds and links against the MSFS 2024 SDK. A few in-sim test runs so far have led to the playback fixes for airspeed, gear and throttle. The control setup hasn't run in the sim yet. Work through [MANUAL_TEST.md](MANUAL_TEST.md) and see [Known gaps](#known-gaps).

## Requirements

| | |
|---|---|
| MSFS 2024 | Steam or Microsoft Store |
| MSFS 2024 SDK | Core 1.7.3 or later, with `MSFS2024_SDK` set (the installer does this) |
| Visual Studio 2022 or Build Tools 2022 | C++ workload (MSBuild, v143), for the native unit tests |
| Node.js 22.13+ or 24 | Builds the panel (built-in TypeScript type stripping, no npm packages) |
| *Optional:* the MSFS2024 platform toolset | Builds the module from the VS solution. Without it, `build.ps1` calls the SDK's clang directly. |

## Build and install

```powershell
# First build: also compiles the toolbar panel definition (.spb).
# Close the sim first: fspackagetool starts FlightSimulator2024.exe in package-builder mode.
.\build.ps1 -Package

# Later builds reuse the cached .spb (package\spb\), and -Install copies into your Community folder.
.\build.ps1 -Install
```

| Option | |
|---|---|
| `-Configuration Debug` | Unoptimized module with debug info and verbose logging |
| `-SkipTests` | Skip the native core tests |
| `-Package` | Run `fspackagetool` to (re)build the panel `.spb`. Needed once, and again whenever `package\PackageSources\InGamePanels\InGamePanel_BetterReplay.xml` changes. |
| `-Install` | Copy `dist\Community\betterreplay-recorder` into the Community folder (found via `UserCfg.opt`) |
| `-CommunityPath <dir>` | Install somewhere else |

The output is `dist\Community\betterreplay-recorder`. To install by hand, copy that folder into your Community folder. On Steam it's `%APPDATA%\Microsoft Flight Simulator 2024\Packages\Community` unless you moved it.

If the `.spb` hasn't been built, the recorder still runs and saves clips, but the toolbar panel won't appear. `build.ps1` warns about this.

`-Package` needs Steam to agree that MSFS 2024 isn't running. If a previous sim instance ended abnormally (crash, or killed in Task Manager), Steam can keep its "running" flag, and then it refuses with *"game is already running"*. `build.ps1` checks for this first. To clear it, click **Stop** on MSFS 2024 in your Steam library, or exit Steam completely and restart it.

You can also build the package in DevMode: **File → Open project…** → `build\package\BetterReplay.xml` (created by `build.ps1`) → **Build**.

## Using it

1. Start a flight. Recording starts once the sim is running. Event detection arms 10 s after the flight loads, so spawning on a runway or in the air never creates a clip.
2. Take off and land. Clips appear in the panel within a few seconds of the end of each event:
   - **Takeoff:** starts at the last moment you were below 10 kt on the ground. Ends at 1,000 ft AGL or 60 s after liftoff, whichever comes first.
   - **Landing:** starts 120 s before the first touchdown. Ends below 30 kt or when airborne again. Bounces count as one landing.
3. Open **BetterReplay** from the toolbar. If the icon is hidden, enable it in the toolbar's settings. The panel shows one screen at a time:
   - **Main:** the recorder's status, large. The dot shows the state: green = recording and armed, amber = arming, grey = paused (the reason is shown), blue = replaying. Below it a bar shows the buffer filling up to its 5 minutes, and **Save last 5 minutes** saves the whole buffer as a manual clip; the button itself shows *Saved* (or *Couldn't save*) for a moment. Under that is the aircraft, with its replay controls *Configured* (green), *Partially configured* (yellow: some controls were skipped, didn't pass, or the gear is still to do) or *Not configured* (red), and **Set up** / **Redo** for the [control setup](#control-setup). **Saved clips** at the bottom, with the clip count, opens the library.
   - **Saved clips:** the library, newest first by when each clip ends (the time each row shows), with a back arrow. So a "Save last 5 minutes" right after a takeoff lists above the takeoff clip, though it starts earlier. Each clip has a play button and a bin; the bin asks once more ("Delete?").
   - **Player:** play freezes your aircraft and replays the clip on it, with the player filling the panel: a large seek bar (the amber mark is the touchdown), round play/pause and stop buttons, and the speeds. **Stop** puts the aircraft back exactly where it was, with its velocities, resumes recording and returns to the main screen.

Recording pauses while the sim is paused, in slew, in the menus, during the sim's own replay, and during BetterReplay playback.

### Control setup

Aircraft differ in how their controls can be driven. Many 2024 aircraft keep their levers in their own input events or L:vars and overwrite the simvars playback would otherwise write. The setup finds out, once per aircraft, what actually works:

1. **The offer.** For an aircraft without a profile the panel always shows the setup. Until you're on a runway with the engines running it says so, with a checklist (parking brake set, on a runway, engines running) that ticks off as you get there; then it offers **Start setup**. Either way you can pick **Later** (until the next flight) or **Never for this aircraft**. While the setup is offered or running it fills the whole panel, with the steps across the top. **Set up** / **Redo** on the main screen's aircraft card starts it any time.
2. **Each control, in turn:** spoilers, flaps and throttle on the ground (throttle needs the parking brake set), then the gear after takeoff. Controls the aircraft doesn't have are skipped.
   - **You move it,** one instruction at a time: "Deploy the spoilers", then "Retract the spoilers"; for the throttle "Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle", "Wait for the engines to spool down". Each gets a green tick once it's done. A hold shows a ring that fills while the throttle is held still (its simvar moving less than 2% a second, for 2 s; on a lever with detents, the detent nearest halfway will do), and dots beside *Your turn* show how far through the step's instructions you are. Meanwhile every input event and L:var the aircraft has is watched, to find the ones that moved with the control, whatever they're called.
   - **It replays your movement**, with each candidate method in turn: the aircraft's own input event preset (the cockpit's path), the legacy key events (`FLAPS_SET`, `SPOILERS_SET`…), then what hardware bindings send for aircraft that ignore those (the PMDG 737): `FLAPS_INCR`/`FLAPS_DECR` a notch at a time toward the recorded handle position, `AXIS_FLAPS_SET`, `AXIS_SPOILER_SET`; then the simvars, then the L:vars it found. Hands off the controls meanwhile. An L:var it found may only animate one panel (the PMDG's `NGXLeftInbdTrailFlaps` moved one flap section), so after trying one the setup puts it back as it was.
   - **It checks the result automatically:** did the spoilers, flap surfaces, gear or N1 (RPM on pistons) move at least 60% as far as when you did it, and come back? Methods that fail move straight on to the next one.
   - **It asks you:** "Did the engines spool up and back down?" **Yes** keeps that method, **No** tries the next, **Watch again** replays it once more, **Skip** leaves the control as it is without a profile (on a later run, a control that was already set up keeps its earlier setup).
   - If nothing works, the control is marked *not replayable*, and replays leave it to the aircraft instead of fighting it.
   - If the aircraft starts to roll during the throttle replay, it stops and goes back to idle.
   - **The throttle lever,** only where it's needed. On some aircraft the lever simvar isn't the lever: on the A320neo V2 it's the autothrust's thrust demand, about 85 at TOGA and creeping with speed while the lever sits in its detent. That's what the throttle's two holds are for: if the simvar kept moving while a lever-like variable held still (your hand was still), and the throttle replays with the key events, the step goes on. If the holds didn't tell (say the simvar held still but read short of 100% at full, like a thrust demand that settles at once), it asks "Do the levers stay put under autothrust?". Aircraft whose simvar is the lever skip all of this. In tests it also ran on the CJ4 (its FADEC sets the simvar) and the PMDG 737 (the simvar trailed its lever by a few percent), and replays from their levers' own variables work well.
     - **The lever's own variable** comes from the same holds: the L:var that moved with the throttle and held still at idle, halfway and full for the longest (the thrust demand keeps creeping; flags and detent indexes take too few values). So there's no second pass of the throttle. Only if no variable did, the setup asks for the holds once more: "Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle".
     - **"Hands off the throttle":** after **Start** the setup steps the throttle axis (`THROTTLEn_AXIS_SET_EX1`) from the bottom to full in 2.5% steps and notes where the lever went, then, with reversers that aren't on the axis, engages them and steps the reverse amount too, and puts the throttle back to idle. The engines spool up against the parking brake for about 15 seconds; if the aircraft starts to roll it stops.
     - The lever's L:var and that table go into the profile. Clips then record the lever (`lv_thr_N`), and replays turn it back into the axis value that put it there: the middle of a detent when it was in one, so it lands in the detent. Clips recorded before this replay from the simvar as before.
3. **Replays** then drive each set-up control with its method, through the same code the setup tested. Clips recorded before the setup (or from different variables) fall back to the defaults for that control, and the log says so.

If you skip the gear step or quit before it, the panel offers just the gear once you're airborne again. Profiles are kept per aircraft (by its `aircraft.cfg` path) in `aircraft.json` in the work folder.

### Where clips live

`%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\` (Steam). That folder holds:

- `YYYY-MM-DD_HHMM_<takeoff|landing|manual>.rpl`: one file per clip, timestamped in **UTC**.
- `index.json`: the library (event, date, aircraft, duration, touchdown VS, peak G, bounces). If it's lost, it's rebuilt from the clip headers.
- `aircraft.json`: the control profiles, one per aircraft: for each control its state (`ok`, `failed`, `skipped`, `absent`, `pending`), method and variable.
- `betterreplay.log` and `betterreplay.prev.log`: the module's log for this and the previous sim session.

Only the newest 20 auto clips (takeoffs and landings) are retained; older ones are deleted when a new one is saved, and at startup. "Save last 5 min" clips are never deleted automatically.

To inspect a clip outside the sim:

```powershell
node tools\rpl-dump.mjs <clip.rpl>                  # header + a sample row per second
node tools\rpl-dump.mjs <clip.rpl> --csv clip.csv   # every frame, every channel
node tools\rpl-dump.mjs <clip.rpl> --cols gs,thr_1,rev_1,rnoz_1 --every 15   # chosen channels, every 0.5 s
```

## Configuration

All detection thresholds are in `DetectorConfig` in [src/core/EventDetector.h](src/core/EventDetector.h). The sample rate, buffer length and retention count are in `EngineConfig` in [src/core/Engine.h](src/core/Engine.h). Rebuild after changing them.

Two detector rules go slightly beyond the original spec:

- `landingMinAirborneSec = 5`: a touchdown only counts as a landing after at least 5 s airborne, so skips on the takeoff roll aren't "landings".
- `holdOffNewEventsAfterPause = false`: after a pause, detection only re-arms if the aircraft moved while paused. Otherwise a pause on short final would lose the landing. Set it to `true` for the strict "re-arm after every pause" behaviour.

## How it works

```
 Toolbar panel (HTML/TS)            WASM module (C++)
 ┌──────────────────────┐ CommBus  ┌───────────────────────────────────────────────┐
 │ stateless remote     │ ───────► │ CommBridge   JSON commands / replies           │
 │ polls status at 4 Hz │ ◄─────── │ Engine       ring buffer, detector, library,  │
 └──────────────────────┘          │              playback   (src/core, no SDK)    │
                                   │ SimBridge    SimConnect: per-frame data,      │
                                   │              events, freeze, SetData          │
                                   │ FileStore    \work stdio I/O                   │
                                   └───────────────────────────────────────────────┘
```

- **`src/core/`** is plain C++17 with no SDK dependencies. It holds:
  - the data-driven channel table (`Channels.cpp`)
  - the ring buffer, event detector, clip format and clip index (`index.json`)
  - interpolation, the playback controller, and the `Engine` that ties them together

  It builds natively for the unit tests and into the WASM module.
- **`src/module/`** is the thin SDK adapter: SimConnect (`SimBridge`), CommBus (`CommBridge`), the work folder (`FileStore`), and the entry points (`Module.cpp`: `module_init`, `module_post_sim_systems_update`, `module_deinit`).
- **`src/panel/`** is the toolbar panel. The TypeScript is stripped to JavaScript at build time (`tools/strip-types.mjs`).
- **`package/`** holds the package definition, the panel definition and the toolbar icon.

**Channels.** Each channel is one row in `kChannels`: simvar, units, storage type, interpolation and playback mode. That row drives the SimConnect definitions, the file format and playback. Add a row to record more.

**CommBus protocol.**
- Commands go to `BetterReplay.Cmd` as JSON: `{"op": …, "id": n}`.
  - `status`
  - `clips{offset}` (20 per page)
  - `play{file}`, `pause`, `resume`, `seek{t}`, `speed{x}`, `stop`
  - `delete{file}`, `saveLast`
  - `setupStart`, `setupAnswer{a}` (`yes`, `no`, `again`, `skip`, `retry`, `ready`, `done`, `begin`, `later`, `cancel`, `close`), `setupLater`, `setupNever`, `setupReset`
  - `setDebug{on}`
- The status message's `setup` object has the offer, the aircraft's profile summary and, while the setup runs, its step, prompt and the answers that fit.
- Replies come back on `BetterReplay.Status`, `BetterReplay.Clips` and `BetterReplay.Reply`.

**Clip format.** The `.rpl` format is chunked and little-endian:

- `BRPL` + version, then `META` (event, UTC start, sample rate, stats, aircraft), `CHAN` (channel keys and storage) and `SAMP` (frames).
- `VARS` (optional) lists which aircraft variable each input event and L:var channel recorded, since that depends on the aircraft and its profile.
- Unknown chunks and channels are skipped, and channels a clip doesn't have are left alone during playback. That leaves room for new channels and a future `LVAR` chunk without breaking old files.
- See [src/core/ClipFormat.h](src/core/ClipFormat.h).

## Development

- **Tests:** `.\build.ps1` builds and runs them. To run a subset: `build\tests\Release\CoreTests.exe Detector_`. The detector tests replay synthetic traces from `tests/TraceGen.h`: normal takeoff, landing, bounce, touch-and-go, spawn on runway, spawn airborne, and a pause mid-landing.
- **Panel without the sim:** run `.\build.ps1 -SkipTests`, then `node tools\panel-harness.mjs` and open http://localhost:8077/. It serves the built panel with the MSFS globals mocked and a fake recorder behind CommBus.
- **Module check:** `tools/check-wasm.mjs` runs after every module build. It fails if a required MSFS 2024 export is missing or the module imports something unexpected.
- **VS solution:** `BetterReplay.sln` has `CoreTests` (x64) and `BetterReplayModule` (MSFS|MSFS2024). The module project only loads once the MSFS2024 platform toolset is installed.

## Logging

Every line is prefixed `[BetterReplay]`. In DevMode, open the Console and filter on it. Debug builds (`-Configuration Debug`) also log commands, freeze confirmations and light events. A status heartbeat is logged every 60 s. [MANUAL_TEST.md](MANUAL_TEST.md) lists the lines to expect at each step.

The module also writes every line, with a UTC timestamp, to `betterreplay.log` in the work folder (see [Where clips live](#where-clips-live)). You don't need DevMode or the console for it. Each sim start begins a new file and keeps the previous session's as `betterreplay.prev.log`.

## Known gaps

None of these could be verified without running the sim. Each is covered by a step in MANUAL_TEST.md.

- **SDK samples not installed.** The InGamePanels XML and package definition follow a dev-forum example plus the sim executable's own property names, not an SDK sample. If the panel doesn't appear, compare them against the SDK's InGamePanel sample.
- **No MSFS2024 VS toolset.** Builds use the SDK's clang directly, with the same exports and version library as a working 2024 module. The VS project has never been built.
- **SimConnect dispatch in WASM.** `SimConnect_CallDispatch` is called once at init. If no messages arrive within 2 s, the module falls back to calling it every frame and logs that.
- **Freeze event parameter is undocumented.** `FREEZE_*_SET` is sent with 1/0 and verified through `IS … FREEZE ON`. If it didn't take, the module sends `FREEZE_*_TOGGLE` and logs it.
- **Flaps, spoilers, surfaces and throttle levers** are written every frame via simvars. Aircraft systems, especially complex add-ons that animate from their own LVars, may override them during playback. That's what the future LVar recording is for. At the end of each playback, the module logs which channels the aircraft overrode.
- **Aircraft input events.** Many 2024 aircraft built on the Asobo templates keep their cockpit controls in input events (`B:` vars) and overwrite the simvars from them. The A320neo V2 ignores the throttle, gear, flap and spoiler simvars and the gear key events. So the module also records these input events where the aircraft has them, by name (`ENGINE_THROTTLE_1..4`, `LANDING_GEAR_GEAR`, `HANDLING_FLAPS`, `HANDLING_SPEED_BRAKES`), and replays them through each one's `<NAME>_SET` preset (`fsVarsBVarCall`), the same path a cockpit click or hardware binding takes. Writing the value directly (`SimConnect_SetInputEvent`) doesn't run the aircraft's code, and the A320neo V2 simply recomputed it. Values are recorded as reported (the A320's throttle in percent: 0 full reverse, 16.7 idle, about 86 at TOGA) and scaled to the 0..1 fraction the preset takes. While one is replayed, the simvars it drives aren't written. After each flight load the log lists which of them the aircraft has, and every input event name at debug level.
- **Overridden controls.** If the aircraft keeps overriding flaps, spoilers or a control surface for half a second, playback stops writing that channel for the rest of the clip, so the two don't fight (flicker), and logs it. Likewise, if the gear snap is undone by the aircraft, the gear isn't snapped again on that aircraft.
- **Throttle** (aircraft without throttle input events) is written as the lever simvars, and the hardware throttle is blocked by masking the standard throttle events. Many 2024 aircraft handle the lever themselves: their input events intercept the hardware before SimConnect masking sees it, and they overwrite the simvar. When playback sees the aircraft overriding the throttle simvar, it switches to sending `THROTTLEn_AXIS_SET_EX1`, the same events a hardware axis sends, and logs it. That mode can't block the hardware lever, so leave it alone while a clip plays.
  - Some aircraft don't put the lever position in that simvar. The A320neo V2's value climbs slowly during the takeoff roll, which looks like its thrust command rather than the lever. Replaying it still approximates the lever, but exact detents need the aircraft's own variables (the LVar work).
- **Reversers** are recorded per engine (`GENERAL ENG REVERSE THRUST ENGAGED`, plus `TURB ENG REVERSE NOZZLE PERCENT` for analysis). The engaged flag isn't settable, so playback sends `SET_THROTTLEn_REVERSE_THRUST_ON/OFF`, and the reverse amount is the negative throttle lever. In axis-event mode, the axis carries the reverse amount once the aircraft reports its reverser engaged, scaled to its `THROTTLE LOWER LIMIT`. At the end of each playback, the log says whether the lights and reversers followed the clip.
- **Gear** is replayed as commands, not positions, because writing gear positions every frame made the gear flicker. The command comes from when the *recorded gear positions* start moving, since some aircraft (e.g. the A320neo V2) only update `GEAR HANDLE POSITION` once the gear has finished moving. Playback sends `GEAR_SET` plus `GEAR_UP`/`GEAR_DOWN` and lets the sim animate the gear. After play, seek, replay and stop, the gear (and lights) are resent and the gear positions are snapped for a moment, so a jump starts with the gear where the clip has it. If the aircraft moves the gear back itself, the log says so; when stopping on the ground, the aircraft then stays frozen until the gear is back down.
  - Aircraft that ignore all of these events need their own gear input event. The module logs a warning if the gear doesn't respond, and lists the aircraft's gear- and throttle-related input events after each flight load.
- **Gear before a jump onto the ground.** On aircraft where the gear can't be snapped into place (the A320neo V2 animates its own), lowering it after the jump puts the extending gear into the runway and it stops half way. So when Play, a seek or Stop goes onto the ground with the gear down while the aircraft's gear is up, the aircraft stays frozen where it is while the gear comes down, then jumps; 30 s at most. Meanwhile the panel shows "Lowering the gear" in place of the replay controls, with a Cancel button. Look for `playback: lowering the gear before going to the runway`.
- **Flaps after a jump.** Some aircraft pick the flap setting from the lever *and* their speed: the A320neo V2 gives 1+F (slats and flaps) for lever 1 on the ground, but CONF 1 (slats only) when the lever moves while fast. A jump moves the aircraft and the lever on the same frame, so it still counted as fast. Now, after a jump, a flap lever driven by the control profile or an input event waits until the aircraft is where the clip has it, on the ground or not and within 20 kt of its ground speed, for 1.5 s (5 s at most). Stop puts the profile's flaps back the same way. A lever already in the right place with the wrong setting (e.g. CONF 1 at lever 1 from a previous approach) isn't corrected.
- **Engine power under autothrust.** Replaying the lever reproduces the engines where the lever sets the thrust. On the CJ4, N1 stayed within 0.9% of the clip on average. On the PMDG 737, whose autothrottle moves the levers, an autothrottle TOGA takeoff with a later MCP speed reduction replayed as flown (7.1% on average, mostly at the jump). Where the aircraft's own logic sets the thrust with the lever in a detent, it doesn't: under autothrust in CL the A350's recorded thrust went from 74 to 8 and back to 77 with the lever still, and in the replay N1 was 13.5% off the clip on average. Writing the recorded `TURB ENG N1`/`N2` every frame was tried and doesn't work: the A350, the CJ4 and the PMDG 737 all kept their own N1 (0 of 60 frames held). N2 is still recorded (`n2_N`).
- **Engine diagnostics.** At the end of each playback the log has a line per engine: `playback: engine N N1 was X% off the clip on average (at most Y%) and changed direction Z times a second (the clip: W)`. If the engine hunts (sound "distorted") its direction changes are well above the clip's.
- **Light key events** are sent with circuit index 0, assumed to mean "all circuits".
- **Control setup** (untested in the sim). Each of these logs what it found:
  - **Aircraft identity** is the path from `SimConnect_RequestSystemState("AircraftLoaded")`, falling back to the title. If 2024 gives each livery its own path, each livery needs its own setup. Look for `aircraft path: …`.
  - **Watching input events** reads them with `fsVarsBVarGet`. If that can't read them, only L:vars are found by their movement. Look for `setup scan: N input event(s), M of them readable`.
  - **Key events** are sent from WASM with `fsEventsTriggerKeyEvent` (not SimConnect, where the throttle axis events failed on the A320neo V2). They act like moving the hardware, so they're only sent when the value really changes (no periodic resend).
  - **Throttle with key events** replays the pilot's hardware axis (`thr_axis_N`), recorded from the throttle key events the module sees (`fsEventsRegisterKeyEventHandler`), so the lever lands where it was, detents included. The lever simvar isn't the lever on every aircraft: on the A320neo V2 under autothrust it's the thrust demand, so CLB replayed as TOGA. Clips without the axis (older ones, or before the hardware first moved) fall back to the simvar. Look for `throttle hardware: key event …` after you first move the throttle; if it never appears, the aircraft takes the hardware another way.
    - On the A320neo V2 it never appears: its hardware throttle doesn't come through as key events the module can see, so its clips have no `thr_axis_N`. With the throttle lever set up (see [Control setup](#control-setup)) they replay the lever's own L:var through its calibration; look for `control setup: throttle lever calibrated: … detents …` and, on playback, `controls: throttle levers from L:var …`. Without it they replay from the simvar: since each key event makes the aircraft re-evaluate its lever (and the engine sound follows), a change in the simvar of 2% or more goes at once, and a smaller one only once it has lasted 30 frames.
  - **Simvars** are written with `fsVarsAVarSet`, with the index (`:1`) passed as a parameter.
  - **L:var channels** (`lv_*`) are only recorded on aircraft whose profile chose an L:var, or (`lv_thr_N`) found the throttle lever's own.
  - **Readings the checks use:** `TURB ENG N1`, `GENERAL ENG RPM`, `SPOILERS LEFT POSITION`, `ENGINE TYPE`, `BRAKE PARKING POSITION`, `ON ANY RUNWAY`, `GENERAL ENG COMBUSTION`, `IS GEAR RETRACTABLE`, `SPOILER AVAILABLE` and `FLAPS NUM HANDLE POSITIONS`. N1, RPM and spoiler position are also recorded in clips (about 0.6 MB more per 5 minutes).
  - **Parking brake:** if the aircraft's doesn't set the simvar, the throttle step waits on "Set the parking brake". **Continue anyway** carries on.

## Future work (designed for, not built)

- **LVar recording:** a change-only `LVAR` chunk with per-aircraft auto-discovery. The control setup's profiles already record the L:vars it chose for the four controls.
- **"Take control"** from any point in a replay.
- Nearest-airport file names, go-around detection, and merging touch-and-goes into one clip.
