# BetterReplay

An in-sim auto replay recorder for Microsoft Flight Simulator 2024. It records your flight continuously, saves every takeoff and landing as a clip automatically, and lets you browse and replay clips from a toolbar panel. Everything runs inside the sim, with no external app.

- **Recorder:** a standalone WASM module. It keeps a 5-minute ring buffer at 30 Hz and runs whether or not the panel is open.
- **Auto clips:** takeoffs and landings, including touchdown vertical speed, peak G and bounce count. They're written to disk as soon as each event finishes.
- **Toolbar panel:** shows recorder status and the clip list, and has play/pause, seek, speed (0.25x–4x), stop, keep, delete and "save last 5 min".

> **Status: MVP, not yet flown.** The platform-independent core has 74 unit tests and the module builds and links against the MSFS 2024 SDK. Nothing has run inside the sim yet. Work through [MANUAL_TEST.md](MANUAL_TEST.md) and see [Known gaps](#known-gaps).

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
3. Open **BetterReplay** from the toolbar. If the icon is hidden, enable it in the toolbar's settings.
   - The dot shows the recorder state: green = recording and armed, amber = arming, grey = paused (the reason is shown), blue = replaying.
   - **Save last 5 min** saves the whole buffer as a manual clip.
   - **Play** freezes your aircraft and replays the clip on it. Use pause, seek (the amber mark is the touchdown) and speed while it plays. **Stop** puts the aircraft back exactly where it was, with its velocities, and recording resumes.
   - **Keep** protects a clip from automatic cleanup, and **Delete** asks for a second click.

Recording pauses while the sim is paused, in slew, in the menus, during the sim's own replay, and during BetterReplay playback.

### Where clips live

`%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\` (Steam). That folder holds:

- `YYYY-MM-DD_HHMM_<takeoff|landing|manual>.rpl`: one file per clip, timestamped in **UTC**.
- `index.json`: the library (event, date, aircraft, duration, touchdown VS, peak G, bounces, keep flag). If it's lost, it's rebuilt from the clip headers, but keep flags are lost.

Only the newest 50 auto clips that aren't marked keep are retained. Manual and kept clips are never deleted automatically.

To inspect a clip outside the sim:

```powershell
node tools\rpl-dump.mjs <clip.rpl>                  # header + a sample row per second
node tools\rpl-dump.mjs <clip.rpl> --csv clip.csv   # every frame, every channel
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
  - `keep{file,keep}`, `delete{file}`, `saveLast`
  - `setDebug{on}`
- Replies come back on `BetterReplay.Status`, `BetterReplay.Clips` and `BetterReplay.Reply`.

**Clip format.** The `.rpl` format is chunked and little-endian:

- `BRPL` + version, then `META` (event, UTC start, sample rate, stats, aircraft), `CHAN` (channel keys and storage) and `SAMP` (frames).
- Unknown chunks and channels are skipped, and channels a clip doesn't have are left alone during playback. That leaves room for new channels and a future `LVAR` chunk without breaking old files.
- See [src/core/ClipFormat.h](src/core/ClipFormat.h).

## Development

- **Tests:** `.\build.ps1` builds and runs them. To run a subset: `build\tests\Release\CoreTests.exe Detector_`. The detector tests replay synthetic traces from `tests/TraceGen.h`: normal takeoff, landing, bounce, touch-and-go, spawn on runway, spawn airborne, and a pause mid-landing.
- **Panel without the sim:** run `.\build.ps1 -SkipTests`, then `node tools\panel-harness.mjs` and open http://localhost:8077/. It serves the built panel with the MSFS globals mocked and a fake recorder behind CommBus.
- **Module check:** `tools/check-wasm.mjs` runs after every module build. It fails if a required MSFS 2024 export is missing or the module imports something unexpected.
- **VS solution:** `BetterReplay.sln` has `CoreTests` (x64) and `BetterReplayModule` (MSFS|MSFS2024). The module project only loads once the MSFS2024 platform toolset is installed.

## Logging

Every line is prefixed `[BetterReplay]`. In DevMode, open the Console and filter on it. Debug builds (`-Configuration Debug`) also log commands, freeze confirmations and light events. A status heartbeat is logged every 60 s. [MANUAL_TEST.md](MANUAL_TEST.md) lists the lines to expect at each step.

## Known gaps

None of these could be verified without running the sim. Each is covered by a step in MANUAL_TEST.md.

- **SDK samples not installed.** The InGamePanels XML and package definition follow a dev-forum example plus the sim executable's own property names, not an SDK sample. If the panel doesn't appear, compare them against the SDK's InGamePanel sample.
- **No MSFS2024 VS toolset.** Builds use the SDK's clang directly, with the same exports and version library as a working 2024 module. The VS project has never been built.
- **SimConnect dispatch in WASM.** `SimConnect_CallDispatch` is called once at init. If no messages arrive within 2 s, the module falls back to calling it every frame and logs that.
- **Freeze event parameter is undocumented.** `FREEZE_*_SET` is sent with 1/0 and verified through `IS … FREEZE ON`. If it didn't take, the module sends `FREEZE_*_TOGGLE` and logs it.
- **Gear, flaps, surfaces and lights** are written via simvars and key events. Aircraft systems, especially complex add-ons that animate from their own LVars, may override them during playback. That's what the future LVar recording is for.
- **Light key events** are sent with circuit index 0, assumed to mean "all circuits".

## Future work (designed for, not built)

- **LVar recording:** a change-only `LVAR` chunk with per-aircraft auto-discovery and profiles.
- **"Take control"** from any point in a replay.
- Nearest-airport file names, go-around detection, and merging touch-and-goes into one clip.
