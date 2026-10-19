// A fake control setup for the harness: the phases, texts and answers of src/core/ControlSetup.cpp,
// driven by a control strip beside the panel that plays the pilot and the aircraft.
window.harnessSetup = (function () {
  const STEPS = ["spoilers", "flaps", "throttle", "gear"];
  const TITLE = { spoilers: "Spoilers", flaps: "Flaps", throttle: "Throttle", gear: "Gear" };
  // The pilot's instructions, one at a time.
  const TASKS = {
    spoilers: ["Deploy the spoilers", "Retract the spoilers"],
    flaps: ["Lower the flaps one notch", "Wait for the flaps to extend", "Retract the flaps"],
    throttle: ["Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle",
      "Wait for the engines to spool down"],
    gear: ["Raise the gear", "Lower the gear"],
  };
  const HOLDS = { throttle: 2 };  // the first instructions that are holds (a ring fills while held)
  const HOLD_DETAIL = "Hold each position until it's ticked. On a lever with detents, the detent nearest halfway will do.";
  const QUESTIONS = {
    spoilers: "Did the spoilers come up and go back down?",
    flaps: "Did the flaps extend and retract?",
    throttle: "Did the engines spool up and back down?",
    gear: "Did the gear retract and extend again?",
  };
  const WATCH = { spoilers: "Watch the spoilers", flaps: "Watch the flaps", throttle: "Listen to the engines", gear: "Watch the gear" };
  // Candidate methods per control as on the A320neo V2, and whether the automatic check passes
  // each (failing ones are skipped without asking).
  const METHODS = {
    spoilers: [["key event SPOILERS_SET", false], ["simvars", false], ["L:var A320_SPOILERS_HANDLE_POSITION", true],
      ["L:var A320_SPOILERS_ANIM", true]],
    flaps: [["input event HANDLING_FLAPS (HANDLING_FLAPS_SET)", true], ["input event HANDLING_FLAPS (HANDLING_FLAPS_SET, as a fraction)", false],
      ["key event FLAPS_SET", true], ["simvars", false]],
    throttle: [["input event ENGINE_THROTTLE_# (ENGINE_THROTTLE_#_SET, as a fraction)", true],
      ["input event ENGINE_THROTTLE_# (ENGINE_THROTTLE_#_SET)", false], ["key events THROTTLEn_AXIS_SET_EX1", true], ["simvars", false]],
    gear: [["input event LANDING_GEAR_GEAR (LANDING_GEAR_GEAR_SET)", true], ["key events GEAR_SET/GEAR_UP/GEAR_DOWN", false]],
  };
  // The throttle lever's phases (aircraft whose lever simvar isn't the lever, like the A320neo V2).
  // Positions repeats the throttle's holds when the movement didn't show the lever's own variable.
  const POSITION_TASKS = TASKS.throttle.slice(0, 3);
  const POSITIONS_AGAIN = "Couldn't find the lever's own position in that movement. Once more, please.";
  const LEVER_TEXT = {
    query: ["Do the levers stay put under autothrust?",
      "On an Airbus the thrust levers sit in a detent (CL, FLX/MCT, TOGA) while the autothrust sets the thrust, so the " +
      "thrust this aircraft reports isn't where its levers are. Your movement didn't show which kind this is."],
    positions: ["", "The thrust this aircraft reports isn't where its levers are, so BetterReplay looks for the lever's own " +
      "position. Hold each position until it's ticked. On a lever with detents, the detent nearest halfway will do."],
    notice: ["Hands off the throttle", "BetterReplay now moves the throttle itself, from idle to full and back, to learn " +
      "where the lever's positions are, then tries reverse thrust if the aircraft has it. The engines spool up against " +
      "the parking brake for about 20 seconds."],
    sweep: ["Moving the throttle", "Keep your hands off the throttle while BetterReplay learns where the lever's positions are."],
  };
  const REVERSE_TEXT = {
    notice: "BetterReplay now tries reverse thrust itself, to learn how to replay it on this aircraft. The engines spool " +
      "up against the parking brake for a few seconds.",
    sweep: "Keep your hands off the throttle while BetterReplay tries reverse thrust.",
  };
  // Recording stops while the setup runs (ControlSetup.cpp: kTakeoffDetail, kDoneDetail).
  const PAUSE_TEXT = {
    takeoff: "Recording is paused until the setup is done, so this takeoff isn't saved.",
    done: "Recording starts again now, with these controls.",
  };
  // The gear step on the ground: held above the runway, then put back (kHoverDetail and on).
  const HOVER_TEXT = {
    hover: "No takeoff needed: BetterReplay holds the aircraft frozen just above the runway while you raise and lower " +
      "the gear, then puts it back.",
    user: "The aircraft is held frozen just above the runway until the gear step is done.",
    land: "Once the gear is down, BetterReplay puts the aircraft back on the runway.",
    lower: "The gear is still up. Once it's down, BetterReplay puts the aircraft back on the runway. Cancel setup puts it " +
      "back as it is.",
  };
  const AIRBORNE_SEC = 5;  // SetupFlow::kHoverAirborneSec
  const GEAR_WAIT_SEC = 6; // stands in for kLandGearWaitSec (30 s)
  const SWEEP_SEC = 8;
  const TASK_SEC = 1.6;   // the pilot does each instruction in this long, once started
  const HOLD_SEC = 3.2;   // a hold: half a second to get there, then the ring fills
  const SETTLE_SEC = 1.2; // then everything settles (the last instruction shows as done)
  const REPLAY_SEC = 3;
  const TIMEOUT_SEC = 20; // stands in for the 90-300 s the real setup waits

  // Starts at the gate with the engines off, so the "get ready" page shows first.
  const sim = { onGround: true, onRunway: false, taxiing: false, parkingBrake: false, engines: false, autoPilot: true,
    nothingWorks: false, airbus: true, unclear: false, leverHidden: false, hoverOnGround: false, gearLeftUp: false };
  const s = { declined: false, snoozed: false, results: null, methods: {}, active: false, plan: [], step: 0, phase: "",
    t: 0, moving: -1, task: 0, taskDone: false, hold: 0, method: 0, attempts: [], note: "", need: "", stoppedEarly: false,
    lever: false, hovering: false, gearUp: false, landThen: "", landPut: false };

  const control = () => s.plan[s.step];
  const groundPending = () => !s.results || ["spoilers", "flaps", "throttle"].some((c) => !s.results[c] || s.results[c] === "pending");

  function enterStep() {
    s.t = 0;
    s.moving = -1;
    s.method = 0;
    s.attempts = [];
    s.need = "";
    s.lever = false;
    s.reverseOnly = false;
    if (s.step >= s.plan.length) { s.phase = "done"; return; }
    s.phase = control() === "gear" && sim.onGround ? "hover" : "prepare";
  }
  function finish(state) {
    const c = control();
    s.results[c] = state;
    s.methods[c] = state === "ok" ? METHODS[c][s.method][0] +
      (s.lever && !s.reverseOnly && c === "throttle" ? ", lever from L:var INI_ENG#_THROTTLE_POS" : "") +
      (s.lever && c === "throttle" ? ", reverse by holding THROTTLE_DECR" : "") : "";
    if (s.hovering) { beginLand("next", state === "failed" && sim.gearLeftUp); return; }
    s.step++;
    enterStep();
  }
  // Held above the runway: back down once the gear is down (asking the pilot after GEAR_WAIT_SEC).
  function beginLand(then, gearUp) {
    s.phase = "land"; s.t = 0; s.landThen = then; s.landPut = false; s.gearUp = gearUp; s.taskDone = false;
  }
  function putBack() { s.hovering = false; s.landPut = true; s.phase = "land"; s.t = 0; }
  function afterLand() {
    s.landPut = false;
    if (s.landThen === "next") { s.step++; enterStep(); }
    else if (s.landThen === "takeoff") { s.phase = "takeoff"; s.t = 0; }
    else s.active = false;
  }
  function leaveHover(why) {
    s.note = "Couldn't set up the gear above the runway: " + why + ". It's set up after takeoff instead.";
    if (s.hovering) beginLand("takeoff", false); else s.phase = "takeoff";
  }
  function precondition() {
    const c = control();
    if (c === "gear") return sim.onGround ? "Take off" : "";
    if (!sim.onGround) return "Land and stop on the runway";
    if (sim.taxiing) return "Come to a stop";
    if (c === "throttle") {
      if (!sim.engines) return "Start the engines";
      if (!sim.parkingBrake) return "Set the parking brake";
    }
    return "";
  }
  function beginUser() { s.phase = "user"; s.t = 0; s.moving = -1; s.task = 0; s.taskDone = false; s.hold = 0; s.need = ""; }
  function beginPositions(note) {
    s.phase = "positions"; s.t = 0; s.moving = -1; s.task = 0; s.taskDone = false; s.hold = 0; s.note = note || "";
  }
  // No lever to calibrate (the simvar is the lever, or its levers move with the thrust): only reverse is tried.
  function reverseOnly() { s.lever = true; s.reverseOnly = true; s.phase = "notice"; s.note = ""; }
  // The lever's own variable, from the movement's holds (or, with "leverHidden", the holds once more).
  function leverFromMovement() { if (sim.leverHidden) beginPositions(POSITIONS_AGAIN); else { s.phase = "notice"; s.note = ""; } }
  // The pilot's instructions, `m` s after starting to move: sets the one showing and a hold's
  // progress; returns how long ago the last one was done (negative until then).
  function follow(list, holds, m) {
    let left = m;
    for (let i = 0; i < list.length; ++i) {
      const d = i < holds ? HOLD_SEC : TASK_SEC;
      if (left < d) {
        s.task = i;
        s.hold = i < holds ? Math.max(0, (left - 0.5) / (d - 0.5)) : 0;
        s.taskDone = false;
        return -1;
      }
      left -= d;
    }
    s.task = list.length - 1;
    s.hold = 0;
    s.taskDone = true;
    return left;
  }
  function pilotTime() {
    if (s.moving < 0 && sim.autoPilot && s.t > 1.5) s.moving = s.t;
    return s.moving < 0 ? -1 : s.t - s.moving;
  }
  function startMethod(i) { s.method = i; s.phase = "replay"; s.t = 0; s.stoppedEarly = false; }
  function nextMethod() {
    const c = control();
    if (s.method + 1 < METHODS[c].length) { startMethod(s.method + 1); return; }
    s.note = "Couldn't replay the " + c + " on this aircraft; replays will leave them alone.";
    finish("failed");
  }
  function evaluate() {
    const c = control();
    // On the A320neo V2 only the key events move the throttle.
    const passes = sim.airbus && c === "throttle" ? METHODS[c][s.method][0].indexOf("key events") === 0 : METHODS[c][s.method][1];
    if (s.stoppedEarly || (!sim.nothingWorks && passes)) {
      s.note = s.stoppedEarly ? "Stopped early: the aircraft started to roll. Is the parking brake set?" : "";
      s.phase = "ask";
      return;
    }
    s.attempts.push("failed");
    nextMethod();
  }

  function tick(dt) {
    if (!s.active) return;
    s.t += dt;
    const c = control();
    if (s.phase === "prepare") {
      s.need = precondition();
      if (!s.need) beginUser();
    } else if (s.phase === "takeoff") {
      if (!sim.onGround) beginUser();
    } else if (s.phase === "hover") {
      if (!s.hovering) {
        if (!sim.onGround) { beginUser(); return; }
        s.need = sim.taxiing ? "Come to a stop" : "";
        if (!s.need) { s.hovering = true; s.t = 0; }
      } else if (!sim.hoverOnGround && s.t > 0.5) {
        beginUser();
      } else if (s.t > AIRBORNE_SEC) {
        leaveHover("the aircraft still counted as on the ground there");
      }
    } else if (s.phase === "land" || s.phase === "lower") {
      if (s.landPut) { if (s.t > 0.5) afterLand(); return; }
      if (s.phase === "lower" && sim.autoPilot && s.t > 1.5) s.gearUp = false;  // the pilot lowers it
      if (!s.gearUp && s.t > 1) putBack();
      else if (s.gearUp && s.phase === "land" && s.t > GEAR_WAIT_SEC) { s.phase = "lower"; s.t = 0; }
    } else if (s.phase === "user") {
      const m = pilotTime();
      if (m < 0) { if (s.t > TIMEOUT_SEC) timeout(); return; }
      if (follow(TASKS[c], HOLDS[c] || 0, m) >= SETTLE_SEC) { s.note = ""; startMethod(0); }
    } else if (s.phase === "replay" && s.t > REPLAY_SEC) {
      evaluate();
    } else if (s.phase === "positions") {
      const m = pilotTime();
      if (m >= 0 && follow(POSITION_TASKS, 2, m) >= SETTLE_SEC) { s.note = ""; s.phase = "notice"; }
    } else if (s.phase === "notice") {
      s.need = sim.parkingBrake ? "" : "Set the parking brake";
    } else if (s.phase === "sweep" && s.t > SWEEP_SEC) {
      finish("ok");
    }
  }
  function timeout() {
    if (!s.active || s.phase !== "user") return;
    if (s.hovering && s.moving < 0) { leaveHover("the gear didn't move there"); return; }
    s.note = s.moving < 0 ? "Didn't see the " + control() + " move." : "The " + control() + " didn't go all the way and come back.";
    s.phase = "retry";
  }

  function start() {
    s.results = s.results || {};
    s.plan = sim.onGround ? STEPS.slice() : ["gear"];
    s.active = true;
    s.declined = false;
    s.snoozed = false;
    s.requested = false;
    s.step = 0;
    s.note = "";
    enterStep();
  }
  function answer(a) {
    if (a === "cancel" || a === "later") {
      if (a === "later") s.snoozed = true;
      if (s.phase === "land" || s.phase === "lower") {
        s.landThen = "stop";
        if (s.phase === "lower") putBack();  // as it is
      } else if (s.hovering) {
        beginLand("stop", s.phase === "user" && s.moving >= 0 && s.task === 1);  // cancelled with the gear up
      } else {
        s.active = false;
      }
      return true;
    }
    const p = s.phase;
    if (p === "ask" && a === "yes") {
      s.note = "";
      const keyEvents = control() === "throttle" && METHODS.throttle[s.method][0].indexOf("key events") === 0;
      if (!keyEvents) { finish("ok"); return true; }
      s.attempts = [];
      if (!sim.airbus) { reverseOnly(); return true; }
      s.lever = true;
      if (sim.unclear) s.phase = "query"; else leverFromMovement();
      return true;
    }
    if (p === "query" && (a === "yes" || a === "no")) {
      if (a === "no") { reverseOnly(); return true; }
      leverFromMovement();
      return true;
    }
    if (p === "notice" && a === "begin") { s.phase = "sweep"; s.t = 0; return true; }
    if (p === "ask" && a === "no") { s.attempts.push("rejected"); s.note = ""; nextMethod(); return true; }
    if (p === "ask" && a === "again") { startMethod(s.method); return true; }
    if (p === "retry" && a === "retry") { s.note = ""; beginUser(); return true; }
    if (p === "prepare" && a === "ready" && (control() === "gear" ? !sim.onGround : sim.onGround)) { beginUser(); return true; }
    if (p === "done" && a === "close") { s.active = false; return true; }
    if (a === "skip" && s.lever && p !== "sweep") { s.note = ""; s.lever = false; finish("ok"); return true; }
    if (a === "skip" && p !== "replay" && p !== "done" && p !== "sweep" && p !== "land" && p !== "lower") {
      s.note = "";
      finish("skipped");
      return true;
    }
    return false;
  }

  const ready = () => ({ parkingBrake: sim.parkingBrake, onRunway: sim.onGround && sim.onRunway, engines: sim.engines });
  function offer() {
    if (s.active || s.snoozed) return "";
    const r = ready();
    if (s.requested) return !sim.onGround ? "gear" : r.parkingBrake && r.onRunway && r.engines ? "new" : "wait";
    if (s.declined) return "";
    if (groundPending()) return r.parkingBrake && r.onRunway && r.engines ? "new" : "wait";
    return (!s.results.gear || s.results.gear === "pending") && !sim.onGround ? "gear" : "";
  }
  function status() {
    const have = s.declined ? "declined" : !s.results ? "none" :
      STEPS.every((c) => s.results[c] && s.results[c] !== "pending") ? "complete" : "partial";
    const out = { offer: offer(), requested: !!s.requested, ready: ready(), have: have, controls: [], active: s.active };
    if (s.results) {
      STEPS.forEach((c) => out.controls.push({ control: c, state: s.results[c] || "pending", method: s.methods[c] || "" }));
    }
    if (!s.active) return out;
    const done = s.phase === "done";
    const c = control() || "";
    const prompts = {
      prepare: s.need || (TASKS[c] || [""])[0], user: (TASKS[c] || [])[s.task], takeoff: "Take off",
      retry: "That didn't register", replay: WATCH[c], ask: QUESTIONS[c], done: "Setup finished",
      query: LEVER_TEXT.query[0], positions: POSITION_TASKS[s.task] || "", notice: s.need || LEVER_TEXT.notice[0],
      sweep: LEVER_TEXT.sweep[0], hover: s.need || "Lifting the aircraft off the runway",
      land: s.landPut || (!s.gearUp && s.t > 0.1) ? "Putting the aircraft back" : "Waiting for the gear to come down",
      lower: "Lower the gear",
    };
    const buttons = {
      prepare: (s.need ? ["ready"] : []).concat(["skip", "cancel"]), user: ["skip", "cancel"], takeoff: ["skip", "later"],
      retry: ["retry", "skip", "cancel"], replay: ["cancel"], ask: ["yes", "no", "again", "skip"], done: ["close"],
      query: ["yes", "no"], positions: ["skip", "cancel"], notice: ["begin", "skip", "cancel"], sweep: ["cancel"],
      hover: ["skip", "cancel"], lower: ["cancel"],
    }[s.phase] || [];
    const pilot = s.phase === "user" || s.phase === "positions" || s.phase === "lower";
    const holding = pilot && c === "throttle" && s.task < 2;
    const hoverDetail = s.phase === "user" ? (s.hovering ? HOVER_TEXT.user : "") : HOVER_TEXT[s.phase];
    return Object.assign(out, {
      phase: s.phase, step: done ? "" : c, title: done ? "Done" : TITLE[c], stepNo: done ? s.plan.length : s.step + 1,
      steps: s.plan.length, prompt: prompts[s.phase] || "", note: s.note,
      detail: s.reverseOnly && REVERSE_TEXT[s.phase] ? REVERSE_TEXT[s.phase] :
        LEVER_TEXT[s.phase] ? LEVER_TEXT[s.phase][1] : s.phase === "user" && holding ? HOLD_DETAIL :
        hoverDetail || PAUSE_TEXT[s.phase] || "",
      progress: s.phase === "sweep" ? Math.min(1, s.t / SWEEP_SEC) : 0,
      task: pilot && s.phase !== "lower" ? s.task : 0,
      tasks: done ? 0 : s.phase === "lower" ? 1 : s.phase === "positions" ? POSITION_TASKS.length : TASKS[c].length,
      taskDone: pilot && s.taskDone, holdTask: holding, hold: holding ? s.hold : 0,
      method: s.method + 1, methods: c ? METHODS[c].length : 0, attempts: s.attempts.slice(),
      plan: s.plan.map((x, i) => ({ control: x, state: i < s.step ? s.results[x] : i === s.step ? "current" : "next" })),
      buttons: buttons,
    });
  }
  // CommBus commands for the setup; returns [ok, error] or null for other commands.
  function command(cmd, playing) {
    switch (cmd.op) {
      case "setupStart":
        if (playing) return [false, "stop the replay first"];
        if (s.active) return [false, "the control setup is already running"];
        start();
        return [true, ""];
      case "setupAnswer":
        return s.active && answer(cmd.a) ? [true, ""] : [false, "'" + cmd.a + "' doesn't fit this step"];
      case "setupRequest":
        if (playing) return [false, "stop the replay first"];
        if (!s.active) { s.requested = true; s.snoozed = false; }
        return [true, ""];
      case "setupLater": s.active = false; s.snoozed = true; s.requested = false; return [true, ""];
      case "setupNever": s.active = false; s.declined = true; s.requested = false; return [true, ""];
      case "setupReset": s.active = false; s.declined = false; s.snoozed = false; s.results = null; s.methods = {}; return [true, ""];
      default: return null;
    }
  }

  // ---- the control strip
  function strip() {
    const box = document.createElement("div");
    box.id = "harness-strip";
    box.style.cssText = "position:fixed;top:16px;left:556px;width:230px;padding:10px 12px;background:#1b2530;" +
      "border:1px dashed #4a6070;border-radius:6px;color:#cfd8df;font:13px sans-serif;z-index:10;";
    const title = document.createElement("div");
    title.textContent = "Harness: the sim";
    title.style.cssText = "font-weight:bold;margin-bottom:8px;color:#f5b13d;";
    box.appendChild(title);
    const toggles = [["onGround", "On the ground (untick to take off)"], ["onRunway", "On a runway"], ["taxiing", "Taxiing"],
      ["parkingBrake", "Parking brake set"], ["engines", "Engines running"],
      ["autoPilot", "Do each movement automatically"], ["nothingWorks", "Nothing works on this aircraft"],
      ["airbus", "Throttle simvar isn't the lever (A320)"], ["unclear", "...and the movement doesn't tell"],
      ["leverHidden", "...and doesn't show the lever's variable"],
      ["hoverOnGround", "Counts as on the ground above the runway"], ["gearLeftUp", "A failed gear method leaves it up"]];
    toggles.forEach((t) => {
      const label = document.createElement("label");
      label.style.cssText = "display:block;margin:4px 0;cursor:pointer;";
      const input = document.createElement("input");
      input.type = "checkbox";
      input.id = "harness-" + t[0];
      input.checked = sim[t[0]];
      input.addEventListener("change", () => { sim[t[0]] = input.checked; });
      label.appendChild(input);
      label.appendChild(document.createTextNode(" " + t[1]));
      box.appendChild(label);
    });
    const actions = [
      ["Do the movement now", () => { if ((s.phase === "user" || s.phase === "positions") && s.moving < 0) s.moving = s.t; }],
      ["Time out", timeout],
      ["Aircraft rolls", () => { if (s.phase === "replay" && control() === "throttle") { s.stoppedEarly = true; evaluate(); } }],
      ["New flight", () => { s.active = false; s.snoozed = false; s.requested = false; s.hovering = false; }],  // (let go where it is)
    ];
    actions.forEach((a) => {
      const b = document.createElement("button");
      b.type = "button";
      b.textContent = a[0];
      b.style.cssText = "display:block;width:100%;margin-top:6px;padding:5px;background:#24313d;color:#e8eef3;" +
        "border:1px solid #3a4d5c;border-radius:4px;cursor:pointer;";
      b.addEventListener("click", a[1]);
      box.appendChild(b);
    });
    const help = document.createElement("div");
    help.style.cssText = "margin-top:8px;color:#8a9cab;font-size:12px;line-height:1.35;";
    help.textContent = "With no profile the panel waits for: parking brake set, on a runway, engines running. The " +
      "throttle step needs the parking brake too; the gear is done held above the runway (after takeoff if it counts as " +
      "on the ground there). Spoilers fail twice before an L:var passes, flaps pass first time.";
    box.appendChild(help);
    document.body.appendChild(box);
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", strip);
  else strip();

  return { status: status, command: command, tick: tick, sim: sim, state: s };
})();
