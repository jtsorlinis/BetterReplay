# BetterReplay — MVP plan

In-sim auto replay recorder for MSFS 2024. It's a standalone WASM module (all logic and state) plus a stateless toolbar panel, talking over CommBus.

## 1. What was verified, and against what

| Topic | Source | Status |
|---|---|---|
| SDK location | `MSFS2024_SDK=C:\MSFS 2024 SDK\` (Core 1.7.3) | ✅ |
| CommBus C API | `WASM/include/MSFS/MSFS_CommBus.h`: `fsCommBusRegister(name, cb(buf,size,ctx), ctx)`, `fsCommBusCall(name, buf, size, FsCommBusBroadcastFlags)`, `fsCommBusUnregister*` | ✅ header |
| CommBus JS API | sim's `fs-base-ui/html_ui/JS/Services/CommBus.js`: `RegisterCommBusListener(cb)` → `.on(name, fn)`, `.callWasm(name, str)` (Promise) | ✅ shipped source |
| SimConnect API | `SimConnect SDK/include/SimConnect.h` (incl. `SubscribeToFlowEvent`, CommBus, `_USE_WEBASSEMBLY_WASM_RT` dispatch variant) | ✅ header |
| System events | 2024 docs: `FlightLoaded`, `Sim`, `SimStart`, `SimStop`, `Pause_EX1` (flags 1 pause / 4 active pause / 8 sim pause), `Crashed` | ✅ docs |
| Flow events | `SIMCONNECT_FLOW_EVENT_*` in header (FLT_LOADED, TELEPORT_START/DONE, BACK_TO_MAIN_MENU, REPLAY_START/END…) | ✅ header |
| Module entry points | 2024 docs: `module_init`, `module_deinit`, `module_pre_sim_physics_update(float dTime)` … `module_post_sim_systems_update(float dTime)`, `module_late_update` | ✅ docs |
| Toolset project settings | SDK `WASM/projects/gdiplus/*.vcxproj`: Platform `MSFS`, PlatformToolset `MSFS2024`, `$(MSFS_IncludePath)`, defines `__wasi__;_STRING_H_CPLUSPLUS_98_CONFORMANCE_;_WCHAR_H_CPLUSPLUS_98_CONFORMANCE_;_LIBCPP_HAS_NO_THREADS;_WINDLL` | ✅ SDK project |
| 2024 module ABI | Must link `WASM/WasmVersions/MSFS_WasmVersions.a` and export `GetSimConnectVersion`, `GetExtensionVersion(Buffer)`, plus `malloc/free/mallinfo/mchunkit_*/get_pages_state/mark_decommit_pages`. Compared against a working 2024 module (`cmbaviator-landing-monitor.wasm`). Test link with SDK clang 15.0.1 + wasm-ld reproduces the same export set. | ✅ built + compared |
| `\work` folder | docs: `\work` = read/write persistent storage. On disk: `%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\<package>\work\` | ✅ docs + disk |
| WASI file/clock | Shipped 2024 modules import `clock_time_get`, `path_rename`, `path_unlink_file`, `fd_readdir` → `time()`, `rename()`, `remove()`, `opendir()` are usable. No module uses `path_create_directory`, so I keep a **flat** `\work` layout. | ✅ inferred from shipped modules |
| Toolbar panel package layout | Installed 2024 add-ons: `InGamePanels/*.spb` (compiled), `html_ui/InGamePanels/<Name>/*.html|js|css`, `html_ui/icons/toolbar/*.svg`, `modules/*.wasm`, `layout.json`, `manifest.json`. Panel HTML uses `<ingame-ui panel-id=…>` + `TemplateElement`, `panelActive`/`panelInactive` events. | ✅ shipped packages |
| InGamePanels XML source | `SimBase.Document Type="InGamePanels"` → `InGamePanels.InGamePanelDefinition id=… Name=… url=… icon=…`. The sim exe confirms the element name and that `id` and `Name` are required (`<Url>` is "deprecated, prefer UICOMPONENT" but still accepted). Full attribute list is from a dev-forum post. | ⚠️ partial |
| Simvar / key-event names | All §4 names, units and settable flags were checked against the 2024 SimVar, Environment Var and Key Event pages. Freeze `*_SET` events have no documented parameter meaning, so I'll send 1/0 and read back `IS … FREEZE ON`, falling back to `*_TOGGLE`. | ✅ docs (one caveat) |

### Gaps

1. **SDK samples aren't installed** (no `Samples\`; they're a separate download). I couldn't read the StandaloneModule or InGamePanel samples. Get them from DevMode → Help → SDK samples, or the official installer at `https://sdk.flightsimulator.com/msfs2024/files/installers/1.7.3/MSFS2024_SDK_Samples_Installer_1.7.3.zip` (3.4 GB). I haven't downloaded it. If you install them, I'll diff my package/panel XML and dispatch pattern against them.
2. **The MSFS2024 VS platform toolset isn't installed.** Only VS Build Tools 2022 is present, with no `MSFS` platform in MSBuild. I'll still ship the `.sln`/`.vcxproj` (MSFS|MSFS2024, modelled on the SDK's own project), but I can't build it here. `build.ps1` will use MSBuild when the toolset exists, and otherwise call the SDK's clang/wasm-ld directly with flags that match the toolset's output ABI (verified above).
3. **WASM dispatch pattern.** The working 2024 module calls `SimConnect_CallDispatch` and exports no update callbacks. That suggests the classic WASM pattern: call `CallDispatch` once in `module_init`, and the sim invokes the proc afterwards. I'll implement that, log the first N dispatches, and add a fallback (call it from `module_pre_sim_systems_update` if no data arrives within 2 s).
4. **`.spb` compilation** needs `fspackagetool.exe`, which launches the sim in build mode. The script will run it once to produce the `.spb`, then cache it. Every other package file (layout.json, manifest.json, copies) the script writes itself, so the inner dev loop doesn't need the packager.

## 2. File layout

```
BetterReplay.sln
build.ps1                       # tests → wasm → panel → dist\Community\betterreplay-recorder
src/core/                       # C++17, no SDK headers, compiles natively and in WASM
  Channels.h/.cpp               # data-driven channel table (key, simvar, units, storage, interp, playback?)
  RingBuffer.h                  # fixed-capacity, preallocated frame ring
  EventDetector.h/.cpp          # takeoff/landing state machine + DetectorConfig (all thresholds)
  ClipFormat.h/.cpp             # .rpl chunked binary read/write
  ClipIndex.h/.cpp              # index model, retention policy, file naming
  Interp.h/.cpp                 # lerp, shortest-angle, step
  PlaybackController.h/.cpp     # playhead/speed/seek → interpolated frame
  Json.h/.cpp                   # tiny JSON writer + parser (commands, index.json)
  Engine.h/.cpp                 # orchestration: suspend/resume, 30 Hz decimation, detection → save,
                                #   index persistence/rebuild, retention, commands, status JSON
                                #   (IStorage interface; unit-tested end to end with in-memory storage)
src/module/                     # WASM adapter (SDK-dependent, thin)
  Module.cpp                    # module_init/deinit/update callbacks
  SimBridge.h/.cpp              # SimConnect defs built from channel table, events, freeze, set data
  CommBridge.h/.cpp             # CommBus command routing + JSON replies
  FileStore.h/.cpp              # \work stdio I/O (IStorage)
  Log.h/.cpp                    # "[BetterReplay]" console logging with levels
  BetterReplayModule.vcxproj    # MSFS|MSFS2024
tools/check-wasm.mjs            # post-build check: required exports present, only known host imports
src/panel/                      # TypeScript, erasable syntax only → JS via Node 24 stripTypeScriptTypes
  BetterReplayPanel.ts/.html/.css
tests/                          # native x64 vcxproj, built with MSBuild (Build Tools 2022)
  CoreTests.vcxproj, test_*.cpp, TraceGen.h (synthetic flight traces), mini test harness
package/
  BetterReplay.xml              # fspackagetool project
  PackageDefinitions/betterreplay-recorder.xml
  PackageSources/InGamePanels/InGamePanel_BetterReplay.xml
  PackageSources/html_ui/icons/toolbar/ICON_TOOLBAR_BETTERREPLAY.svg
README.md, MANUAL_TEST.md
```

## 3. Runtime design

- **Clock.** "Recording time" advances only while recording is active. Each frame adds Δ`SIMULATION TIME`, clamped to 0–0.5 s. That respects sim rate and stops while paused. Fallback: the update callback's `dTime`. Every timer (arming, 3 s ground, 5 s airborne) runs on this clock, so a pause squeezes out of clips and detector timers.
- **Suspend recording** when any of these hold: `Pause_EX1 != 0`, `IS SLEW ACTIVE`, `Sim == 0` / `SimStop` / flow `BACK_TO_MAIN_MENU`, no flight loaded, or our own playback is active.
- **Arming.** Detection is disarmed for 10 s after `FlightLoaded`, flow `TELEPORT_DONE`, leaving slew, or leaving pause. When it arms, the detector seeds its state from the current sample, so no transition is synthesized. A spawn on a runway or in the air therefore can't trigger an event.
- **Pause mid-event.** A candidate already in progress survives a pause if the aircraft didn't move: less than 200 m and 100 ft between the last pre-pause and first post-pause sample. If it moved, the candidate is dropped and detection re-arms after 10 s. A slew or teleport always aborts the candidate. *Changed during implementation:* the approved plan also held off *new* events for 10 s after every pause. That would lose the landing after a pause on short final, so by default the hold-off now applies only when the aircraft moved. `DetectorConfig::holdOffNewEventsAfterPause = true` restores the strict behaviour.
- **Takeoff.** On ground→air: liftoff = t. Confirm once airborne continuously for at least 5 s and AGL > 50 ft. Clip start = max(last t on ground with GS < 10 kt, start of this ground segment), which keeps touch-and-go takeoffs sane. Clip end = AGL ≥ 1000 ft, or 60 s after liftoff, or touchdown.
- **Landing.** On air→ground after at least 5 s airborne (`landingMinAirborneSec`, added so takeoff-roll skips aren't "landings"): first touchdown = t, recording touchdown VS (`PLANE TOUCHDOWN NORMAL VELOCITY` if available, else VS at contact). Confirm after 3 s of continuous ground contact. A re-liftoff that doesn't meet the takeoff criteria (5 s and 50 ft) counts as a **bounce** and stays in the same landing (`bounces++`). One that does meet them cancels the landing candidate (an aborted landing). Clip = [first touchdown − 120 s, GS < 30 kt or airborne again]. Peak G is measured over [touchdown − 1 s, end].
- **Save.** On finalize, the frame range is streamed from the ring buffer straight to `\work\YYYY-MM-DD_HHMM_<event>.rpl`, with `_2`, `_3`… on collision. Times are **UTC**, since WASM has no time zone. Then `index.json` is rewritten (write `.tmp` → rename) and retention runs: the newest 50 non-kept auto clips survive. Manual and kept clips are never auto-deleted. If `index.json` is missing or corrupt, it's rebuilt by scanning `*.rpl` headers.
- **No per-frame allocation.** The ring buffer (9000 frames × up to 48 channels × f64 ≈ 3.5 MB) and SimConnect receive slots are preallocated. JSON is built into a fixed scratch buffer, and only when the panel asks.
- **Playback.** Load the clip, then snapshot the playback channels plus body velocities. Send `FREEZE_LATITUDE_LONGITUDE_SET`/`FREEZE_ALTITUDE_SET`/`FREEZE_ATTITUDE_SET` = 1. Each frame: playhead += dt × speed (0.25–4×), binary-search the frame pair, interpolate, then `SetDataOnSimObject`. lat/alt lerp; lon, pitch, bank and heading use shortest-angle; gear, flaps and surfaces lerp. Lights step, and only changes are sent, as key events. At the end, playback auto-pauses. Stop: write the snapshot, unfreeze, resume recording, then a 10 s arming hold-off. Leaving the flight during playback just stops.

## 4. Channels (data-driven, verified against the 2024 docs)

Each row has a stable key, simvar, units, storage (`f64|f32|u8`), interp (`linear|angle|step`) and a playback mode (`set` = SetDataOnSimObject every frame, `event` = key event on change, `snap` = SetDataOnSimObject only after a jump, `none`). Adding a row extends the recording, file and playback with no format change.

| Key | SimVar (units requested) | Play |
|---|---|---|
| lat, lon | PLANE LATITUDE / LONGITUDE (`degrees`; the docs default is radians) f64 | set (lon uses shortest-angle) |
| alt | PLANE ALTITUDE (`feet`) f64 | set |
| agl | PLANE ALT ABOVE GROUND (`feet`) | none (detection) |
| pitch, bank, hdg | PLANE PITCH DEGREES / BANK DEGREES / HEADING DEGREES TRUE (`degrees`; the docs default is radians) | set, angle |
| vbx, vby, vbz | VELOCITY BODY X/Y/Z (`feet per second`) | set (so airspeed/VS indications follow the clip; added after the first in-sim test) |
| rvx, rvy, rvz | ROTATION VELOCITY BODY X/Y/Z (`radians per second`) | set |
| gs | GROUND VELOCITY (`knots`; not settable) | none |
| vs | VERTICAL SPEED (`feet/minute`; the docs default is ft/s) | none |
| ongnd | SIM ON GROUND (`bool`) u8 | none |
| g | G FORCE (`gforce`) | none |
| gear_c/l/r | GEAR CENTER/LEFT/RIGHT POSITION (`percent over 100`) | snap: written for ~20 frames after play, seek and stop only, so a jump starts with the gear where the clip has it. Writing them every frame made the gear flicker in the sim. |
| gear_h | GEAR HANDLE POSITION (`percent over 100`) | event: `GEAR_SET` + `GEAR_UP`/`GEAR_DOWN` on change, derived from the gear positions; the sim animates the gear |
| flap_l/r | TRAILING EDGE FLAPS LEFT/RIGHT PERCENT (`percent over 100`) | set |
| flap_idx | FLAPS HANDLE INDEX (`number`) | set, step |
| spoil_h | SPOILERS HANDLE POSITION (`percent over 100`) | set |
| ail_l/r, elev, rud | AILERON LEFT/RIGHT DEFLECTION, ELEVATOR DEFLECTION, RUDDER DEFLECTION (`radians`). The settable surface angles. The `*_PCT` variants are read-only. | set |
| ail_pos, elev_pos, rud_pos | AILERON / ELEVATOR / RUDDER POSITION (`position`): the pilot input, recorded for analysis | none |
| thr_1..thr_4 | GENERAL ENG THROTTLE LEVER POSITION:1..4 (`percent`), added after the first in-sim test | set, only for engines the aircraft has; hardware throttle events (`THROTTLE_SET`, `AXIS_THROTTLE*_SET`, `THROTTLE*_AXIS_SET_EX1`…) are masked while playing. If the aircraft overrides the simvar in half of a 30-frame window (its own throttle handling, e.g. 2024 input events), playback switches to sending `THROTTLEn_AXIS_SET_EX1` for that aircraft, with the mask off. |
| l_nav/beacon/strobe/land/taxi | LIGHT NAV / BEACON / STROBE / LANDING / TAXI (`bool`) u8 | event: `NAV_LIGHTS_SET`, `BEACON_LIGHTS_SET`, `STROBES_SET`, `LANDING_LIGHTS_SET`, `TAXI_LIGHTS_SET` on change |

Event channels are also resent after every play, seek, replay-from-the-end and stop, so the aircraft can't stay out of step with the clip (e.g. gear left up by the end of the previous run).

Not channels: `TITLE` (string, read on flight load), `PLANE TOUCHDOWN NORMAL VELOCITY` (ft/s, read at touchdown → stored as fpm), `IS SLEW ACTIVE`, `SIMULATION TIME`, `IS LATITUDE LONGITUDE/ALTITUDE/ATTITUDE FREEZE ON`.

Known risk: complex add-ons (e.g. PMDG, iniBuilds) drive gear, surface and light animations from their own LVars and systems, so a replay may show positions correct but animations wrong. That's what the future LVar change-log is for.

## 5. Clip file format (`.rpl`, little-endian, chunked)

```
"BRPL" u16 formatVersion=1 u16 flags
chunk*: fourcc type, u32 size, payload        (unknown chunks are skipped → forward compatible)
  META: u8 event(0 manual,1 takeoff,2 landing), i64 utcStartMs, f32 sampleRateHz, u32 frames,
        f32 eventOffsetSec, f32 touchdownVsFpm, f32 peakG, u16 bounces, str aircraftTitle
        (new fields are appended; readers use the size)
  CHAN: u16 n; n × { u8 storage, u8 interp, str key }
  SAMP: u32 frames, u32 stride; frames × { f32 t, values in CHAN order/storage }
  (future) LVAR: change-only log {f32 t, u16 id, f64 value} + id→name table
```
When playing, channels are matched by **key**. Unknown keys are ignored, and channels a file doesn't have are left untouched on the aircraft.

`index.json`: `{version, clips:[{file, event, utc, aircraft, durationSec, touchdownVsFpm, peakG, bounces, keep, auto}]}`.

## 6. CommBus protocol (panel is stateless)

- JS → WASM, event `BetterReplay.Cmd`, payload `{"op":…, "id":n, …}`. Ops: `status`, `clips`, `play{file}`, `pause`, `resume`, `seek{t}`, `speed{x}`, `stop`, `keep{file,keep}`, `delete{file}`, `saveLast`.
- WASM → JS: `BetterReplay.Status` (recorder state + reason, armed/arm-in, buffer seconds, detector phase, playback {file, t, dur, speed, paused}, `clipsRev`), `BetterReplay.Clips` (the list), and `BetterReplay.Reply` (`{id, ok, error}`).
- While active, the panel polls `status` at 4 Hz and refetches `clips` when `clipsRev` changes. Closing the panel stops polling. The module doesn't care whether a panel exists.

## 7. Milestones (one commit each)

1. Scaffold: git init, sln, test harness, build.ps1 skeleton.
2. Core data: channels, ring buffer, clip format, JSON, index/retention, interp, with tests.
3. Event detector and synthetic traces: normal takeoff, normal landing, bounce, touch-and-go, spawn on runway, spawn airborne, and pause mid-landing.
4. WASM module: SimConnect recording, suspend/arm, detection → save, index, logging. Builds to `.wasm`.
5. Playback: controller tests, freeze, set data, snapshot/restore.
6. Panel, package definition, `.spb`, and a full Community package from `build.ps1`.
7. README and MANUAL_TEST.md.

## Status (2026-10-17)

Milestones 1–7 are implemented and committed. The core has 81 passing unit tests, the module builds with the SDK clang, and the panel has been exercised in a browser harness against a fake module. Still pending: in-sim verification ([MANUAL_TEST.md](MANUAL_TEST.md)) and compiling the panel `.spb` (`build.ps1 -Package`, which needs the sim's package builder).

## 8. Decisions (answered: defaults approved)

1. Package name `betterreplay-recorder` and toolbar title "BetterReplay". OK?
2. The pause-mid-event interpretation in §3. OK?
3. UTC in file names. OK?
4. TypeScript: I'll build with Node 24's built-in type stripping (no download). Should I also add `typescript` as an npm devDependency (~23 MB download) so `build.ps1` type-checks?
5. Do you want to install the SDK samples and toolset before I start, or go ahead with the sources above and diff against the samples later?
