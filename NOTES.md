# BetterReplay: notes

Open bugs, things I'd like to add, and which aircraft have been tested and have a control profile included. How it works and the known gaps are in [DEVELOPMENT.md](DEVELOPMENT.md).

## Bugs

- **PMDG 777: the autothrottle can come on by itself at acceleration altitude** in a takeoff replay. The replay disconnects it again once the levers have been off where it put them for half a second, so it works, but that delay makes the move from MCT to CLB uneven instead of smooth. Possible fix: disarm the A/T completely for the replay.
- **iniBuilds A350: the engine whine sounds distorted in the cabin.** In the flight itself the whine went away when the thrust came back from takeoff to climb. In the replay it kept playing, distorted, though at the lower N1 it shouldn't be playing at all. See _A350 engine whine in replays_ under [Known gaps](DEVELOPMENT.md#known-gaps).
- **iniBuilds A380 (and probably other fly-by-wire aircraft): the control surfaces don't replay.** Writing them doesn't work: the A380 rewrites the surface simvars and its own animation L:vars every other frame. While it's frozen, the g its fly-by-wire reads sinks from about 1.0 to 0.55, so it commands full nose-up.

## Wishlist

- **Lights on all aircraft:** nav, beacon, strobe, landing and taxi lights are replayed with the sim's own light events, which not every aircraft takes
- **Manual recording:** start and stop a clip yourself, so a whole flight can be recorded (_Save last 10 minutes_ only keeps the last 10 minutes)
- **Weather and time of day** from the clip

## Aircraft

**Included** means the package ships the aircraft's control profile (`profiles\`), so it replays without running the control setup first.

| Aircraft                       | Tested | Included |
| ------------------------------ | ------ | -------- |
| iniBuilds A320neo V2 (default) | ❌     | ❌       |
| iniBuilds A330 (default)       | ❌     | ❌       |
| Working Title 787-10 (default) | ❌     | ❌       |
| Working Title 747-8 (default)  | ❌     | ❌       |
| iniBuilds A340                 | ❌     | ❌       |
| iniBuilds A350                 | ✅     | ✅       |
| iniBuilds A380                 | ✅     | ✅       |
| Fenix A320                     | ❌     | ❌       |
| Synaptic A220                  | ✅     | ✅       |
| FlightSim Studio E-Jets        | ❌     | ❌       |
| PMDG 737                       | ✅     | ✅       |
| PMDG 777                       | ✅     | ✅       |
