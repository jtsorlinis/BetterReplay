# BetterReplay: notes

Open bugs, things I'd like to add, and which aircraft have been tested. How it works and the known gaps are in [DEVELOPMENT.md](DEVELOPMENT.md).

## Bugs

- **PMDG 777: the autothrottle can come on by itself at acceleration altitude** in a takeoff replay. Possible fix: disarm the A/T completely for the replay.
- **iniBuilds A350: the engine whine sounds distorted in the cabin.** In the flight itself the whine went away when the thrust came back from takeoff to climb. In the replay it kept playing, distorted, though at the lower N1 it shouldn't be playing at all. See *A350 engine whine in replays* under [Known gaps](DEVELOPMENT.md#known-gaps).
- **iniBuilds A380 (and probably other fly-by-wire aircraft): the control surfaces don't replay.** Writing them doesn't work: the A380 rewrites the surface simvars and its own animation L:vars every other frame. While it's frozen, the g its fly-by-wire reads sinks from about 1.0 to 0.55, so it commands full nose-up.

## Wishlist

- **Lights**
- **Weather and time of day** from the clip
- **Manual recording:** start and stop a clip yourself, as well as the automatic takeoffs and landings

## Aircraft

| Aircraft | Tested |
|---|---|
| iniBuilds A320neo V2 (default) | |
| iniBuilds A330 (default) | |
| Working Title 787-10 (default) | |
| Working Title 747-8 (default) | |
| iniBuilds A340 | |
| iniBuilds A350 | Yes |
| iniBuilds A380 | Yes |
| Fenix A320 | |
| Synaptic A220 | Yes |
| FlightSim Studio E-Jets | |
| PMDG 737 | Yes |
| PMDG 777 | Yes |
