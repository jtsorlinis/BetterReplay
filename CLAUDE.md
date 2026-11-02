# BetterReplay

An auto replay recorder for MSFS 2024: a WASM module (C++) that records the flight in the background, saves takeoffs and landings as clips and replays them, and a toolbar panel that drives it. [README.md](README.md) is the user-facing doc; [KNOWN_ISSUES.md](KNOWN_ISSUES.md) lists what doesn't replay right. There's no design doc: the reasoning and the sim evidence for each change are in its commit message.

## Layout

- `src/core/`: plain C++17, no SDK. The channel table (`Channels.cpp`, one row per recorded simvar), ring buffer, event detector, clip format and index, playback controller, control setup and aircraft profiles; `Engine` ties them together. Built natively for the tests and into the module.
- `src/module/`: the SDK adapter. `SimBridge` (SimConnect, per-frame playback), `ControlDriver` (gear, flaps, spoilers, throttle by the profile's methods), `CommBridge` (the panel's JSON commands over CommBus), `FileStore`, `Log`, and the entry points in `Module.cpp`.
- `src/panel/`: the toolbar panel. The TypeScript is stripped to JavaScript at build time.
- `package/`: the package and panel definitions; `package/spb/` caches the compiled panel `.spb`.
- `profiles/`: the included control profiles, one JSON per aircraft.
- Tunables: `DetectorConfig` in `src/core/EventDetector.h`, `EngineConfig` in `src/core/Engine.h`.

## Build and test

- `.\build.ps1` runs the core tests, then builds the WASM module, the panel and `dist\Community\betterreplay-recorder`. `-SkipTests`, `-Install` (copies it into the Community folder), `-Configuration Debug` (verbose logging), `-Zip` (release zip).
- `-Package` is only for when `package\PackageSources\InGamePanels\InGamePanel_BetterReplay.xml` changes: it runs fspackagetool, which starts the sim in package-builder mode (the sim must be closed). Otherwise the cached `.spb` is used. Run from the PowerShell tool, fspackagetool's stderr stops the script (PowerShell 5.1 turns it into an error), so leave `-Package` to the user.
- A subset of the tests: `build\tests\Release\CoreTests.exe Detector_`.
- The panel without the sim: build, then the `panel-harness` launch config (`node tools\panel-harness.mjs`, http://localhost:8077/), with the MSFS globals mocked and a fake recorder.
- Needs the MSFS 2024 SDK (`MSFS2024_SDK`), VS 2022 C++ build tools and Node 22.13+.

## The sim's work folder

`%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\`

- `betterreplay.log` is this sim session's log (UTC times), `betterreplay.prev.log` the previous session's. Read them when the user has flown, set up or replayed something.
- Clips are `YYYY-MM-DD_HHMM_<takeoff|landing|manual>.rpl` (UTC), listed in `index.json`. `node tools\rpl-dump.mjs <clip.rpl>` prints one; `--csv <file>` writes every frame, `--cols gs,thr_1 --every 15` picks channels.
- `aircraft.json` holds the control profiles set up in the sim.
- `trace_<clip>.csv` is a per-frame replay trace, written only with `kPlaybackTrace = true` in `src/module/SimBridge.h` (rebuild, and set it back after).

## Including an aircraft's profile

1. `node tools\export-profiles.mjs` copies the work folder's profiles into `profiles\`, one file per aircraft, matched by the `key` inside it; a file is only replaced by a newer profile.
2. `build.ps1` packs them into the package's `profiles\aircraft.json` and prints `N included control profile(s)`.
3. Mark the aircraft ✅ in the README's table, and add anything that doesn't replay right to KNOWN_ISSUES.md.

## Releasing

Set the version in `package\PackageDefinitions\betterreplay-recorder.xml` (`Version`) and `src\module\Module.cpp` (`kVersion`), commit and push, then `.\build.ps1 -Zip`. It writes `dist\betterreplay-recorder-<version>.zip` and prints GitHub's new-release link; the user publishes it in the browser.
