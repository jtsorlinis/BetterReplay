# BetterReplay

An auto replay recorder for Microsoft Flight Simulator 2024. It records your flight in the background, saves every takeoff and landing as a clip, and lets you replay them from a toolbar panel. Everything runs inside the sim, with no external app.

> **Early testing.** BetterReplay works in the sim but is still being tested, and replays may not get every control right on every aircraft. See [Known gaps](DEVELOPMENT.md#known-gaps).

## Features

- **Always recording** the last 10 minutes of your flight, whether or not the panel is open.
- **Automatic clips** of every takeoff and landing. Landings show touchdown vertical speed, peak G and bounces.
- **Replays on your own aircraft,** with play/pause, seek, 0.25x–4x speed, and Stop, which puts you back exactly where you were.
- **Save the last 10 minutes** as a clip any time.
- **Control setup:** a short guided setup per aircraft, so replays move its spoilers, flaps, throttle and gear properly.

## Install

1. Get the `betterreplay-recorder` package folder. To build it yourself, see [Build and install](DEVELOPMENT.md#build-and-install).
2. Copy it into your Community folder. On Steam that's `%APPDATA%\Microsoft Flight Simulator 2024\Packages\Community` unless you moved it.
3. Start the sim. If the BetterReplay icon isn't on the toolbar, enable it in the toolbar's settings.

## Using it

### Recording

Recording starts once you're in a flight. Clips appear in the panel a few seconds after each takeoff or landing ends:

- **Takeoff:** from just before you roll onto the runway until the flaps are up (or 2 minutes after liftoff if you took off without flaps).
- **Landing:** from 2 minutes before touchdown until you stop or leave the runway. Bounces stay part of the same landing; a touch-and-go or go-around ends it.

Nothing is detected for the first 10 seconds of a flight, so spawning on a runway or in the air never makes a clip. Recording pauses while the sim is paused, in slew, in the menus, during the sim's own replay, and while a clip plays.

### The panel

Open **BetterReplay** from the toolbar.

- **Main screen:** the recorder's status. The dot is green when recording, amber while getting ready, grey when paused (with the reason) and blue while replaying. **Save last 10 minutes** saves the last 10 minutes as a clip. Below that is your aircraft and whether its controls are set up.
- **Saved clips:** your clips, newest first, named after the aircraft and airport (*A350-900 | YMML*), with landing stats. Each has a play button and a delete button.
- **Player:** Play freezes your aircraft and replays the clip on it. Drag the seek bar (the amber mark is the touchdown), change the speed, or press **Stop** to put the aircraft back where it was and carry on flying. On the ground it may stay frozen for up to 10 seconds while the gear comes back down.

A clip only replays on the aircraft it was recorded on, down to the variant (an A350-900 clip won't play on the -1000). From another aircraft, the clip says which one to switch to.

### Control setup

Many aircraft drive their own levers in ways a replay can't simply copy. The control setup finds out, once per aircraft, how to replay its spoilers, flaps, throttle and gear.

1. For an aircraft that hasn't been set up, the panel offers the setup, with a checklist: parking brake set, on a runway, engines running. Then pick **Start setup**. You can also pick **Later** or **Never for this aircraft**, and start it any time with **Set up** / **Redo** on the main screen.
2. For each control, the panel asks you to move it ("Deploy the spoilers", "Move the throttle to about halfway and hold it"…). Then it replays your movement and asks whether it worked: **Yes**, **No**, **Watch again** or **Skip**. Keep your hands off the controls while it replays.
3. On some aircraft the throttle step also asks you to keep your hands off while it moves the throttle and tests reverse thrust itself. The engines spool up against the parking brake for up to 40 seconds, and it stops if the aircraft starts to roll.
4. The gear needs no takeoff: BetterReplay lifts the aircraft about 30 ft above the runway, holds it there frozen, and asks you to raise and lower the gear. When the step is done (or you skip or cancel it) it waits for the gear to be down, asks you to lower it if it isn't within 30 seconds, then puts the aircraft back where it was. If the aircraft still counts as on the ground up there, or its gear won't move there, the gear is done after takeoff instead. If you skip it, the panel offers it again next time you're airborne.

Recording stops while the setup runs and starts again when it finishes, so the setup never ends up in a clip. Variants of an aircraft (like the PMDG 737-700 and -800) share one setup.

## Where clips are kept

`%APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\betterreplay-recorder\work\` (Steam)

The newest 20 takeoff and landing clips are kept, and older ones are deleted automatically. "Save last 10 minutes" clips (about 5.5 MB each) are never deleted automatically.

## Troubleshooting

- **No toolbar icon:** enable it in the toolbar's settings. If it's still missing, the package was built without its panel (see [Build and install](DEVELOPMENT.md#build-and-install)); clips are still saved.
- **Something went wrong:** the log is `betterreplay.log` in the folder above, and `betterreplay.prev.log` is the previous sim session's. Include it when you report a problem.

## Development

Building from source, how it works, configuration and known gaps are in [DEVELOPMENT.md](DEVELOPMENT.md). [MANUAL_TEST.md](MANUAL_TEST.md) is the in-sim test checklist.
