// BetterReplay toolbar panel: a stateless remote for the WASM module. Everything shown here comes
// from the module over CommBus; closing the panel loses nothing.
//
// Written as erasable TypeScript (types are stripped at build time, nothing is transformed) and
// ES2015-level JavaScript for the sim's Coherent GT browser: no class fields, optional chaining,
// nullish coalescing or async/await.

// ---- MSFS globals (from /JS/common.js and /JS/Services/CommBus.js)
declare class UIElement extends HTMLElement {
    connectedCallback(): void;
    disconnectedCallback(): void;
}
interface CommBusListener {
    on(name: string, callback: (payload: string) => void): void;
    off(name: string, callback: (payload: string) => void): void;
    callWasm(name: string, json: string): unknown;
}
declare function RegisterCommBusListener(callback?: () => void): CommBusListener;
declare function checkAutoload(): void;

// ---- protocol (mirrors src/module/CommBridge.cpp and Engine::statusJson / clipsJson)
interface PlayState {
    active: boolean;
    file: string;
    t: number;
    dur: number;
    speed: number;
    playing: boolean;
    preparing?: boolean;  // holding before a jump, e.g. lowering the gear before going to the runway
    eventT: number;
}
interface SetupControl {
    control: string;
    state: string;
    method: string;
}
interface SetupStep {
    control: string;
    state: string;  // a finished step's result ("ok", "failed", "skipped"), or "current" / "next"
}
// The control setup (src/core/ControlSetup.h); the step fields are there while it's active.
interface SetupStatus {
    offer: string;  // "new" (can start), "wait" (no profile, not ready yet), "gear" or ""
    ready?: { parkingBrake: boolean; onRunway: boolean; engines: boolean };  // what "wait" is waiting for
    have: string;   // the aircraft's profile: "none", "partial", "complete", "declined"
    controls: SetupControl[];
    active: boolean;
    phase?: string;
    step?: string;
    title?: string;
    stepNo?: number;
    steps?: number;
    prompt?: string;  // one short instruction or question
    task?: number;    // the pilot's instruction showing, of `tasks` in this step
    tasks?: number;
    taskDone?: boolean;  // the last one is done (things settling)
    method?: number;      // the method being tried (1-based), of `methods` in this step
    methods?: number;
    attempts?: string[];  // how each method before it ended: "failed", "rejected", "unavailable"
    note?: string;
    plan?: SetupStep[];
    buttons?: string[];
}
interface Status {
    rec: boolean;
    why: string;
    armed: boolean;
    armIn: number;
    phase: string;
    bufferSec: number;
    bufferMax?: number;  // how much the buffer holds
    aircraft: string;
    clipsRev: number;
    clipCount: number;
    lastSaved: string;
    error: string;
    play: PlayState;
    setup?: SetupStatus;
}
interface ClipInfo {
    file: string;
    event: string;
    utc: string;
    utcMs: number;
    aircraft: string;
    durationSec: number;
    touchdownVsFpm: number | null;
    peakG: number | null;
    bounces: number;
    auto: boolean;
}
interface ClipsPage {
    rev: number;
    total: number;
    offset: number;
    clips: ClipInfo[];
}
interface Reply {
    id: number;
    op: string;
    ok: boolean;
    error: string;
}
type Command = { [key: string]: string | number | boolean };

const CMD_EVENT = "BetterReplay.Cmd";
const STATUS_EVENT = "BetterReplay.Status";
const CLIPS_EVENT = "BetterReplay.Clips";
const REPLY_EVENT = "BetterReplay.Reply";
const POLL_ACTIVE_MS = 250;
const POLL_INACTIVE_MS = 2000;
const OFFLINE_AFTER_MS = 2500;
const LOG = "[BetterReplay]";

const MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
const PHASES: { [phase: string]: string } = {
    disarmed: "arming",
    ground: "on ground",
    airborne: "airborne",
    "takeoff-liftoff": "liftoff detected",
    "takeoff-climb": "capturing takeoff",
    "landing-touchdown": "touchdown detected",
    "landing-bounce": "bounce",
    "landing-rollout": "capturing landing",
};
const SETUP_BUTTONS: { [answer: string]: string } = {
    start: "Start setup",
    yes: "Yes",
    no: "No",
    again: "Watch again",
    skip: "Skip this step",
    retry: "Try again",
    ready: "Continue anyway",
    later: "Later",
    never: "Never for this aircraft",
    cancel: "Cancel setup",
    close: "Done",
};
// How prominent each answer is: the main choice, the alternatives beside it, and (anything not
// listed) the small ones underneath.
const SETUP_KIND: { [answer: string]: string } = {
    start: "primary",
    yes: "primary",
    retry: "primary",
    close: "primary",
    no: "secondary",
    again: "secondary",
    ready: "secondary",
    later: "secondary",
};
const SETUP_PHASES: { [phase: string]: string } = {
    wait: "Get ready",
    prepare: "Get ready",
    user: "Your turn",
    takeoff: "Take off",
    retry: "Didn't catch that",
    replay: "Watch",
    hold: "Watch",
    ask: "Your call",
    done: "All done",
};
// Phases where the pilot is doing something: when what they were told changes there, they did it.
const PILOT_PHASES: { [phase: string]: boolean } = { prepare: true, takeoff: true, user: true };
const TICK_HOLD_MS = 1100;  // a done instruction stays up, ticked, this long

const CONTROL_LABELS: { [control: string]: string } = {
    spoilers: "Spoilers",
    flaps: "Flaps",
    throttle: "Throttle",
    gear: "Gear",
};
const RESULT_LABELS: { [state: string]: string } = {
    ok: "Set up",
    failed: "Not replayable",
    skipped: "Skipped",
    pending: "Not set up",
};
// Icons: inline SVG drawn in the button's text colour.
const ICON_PLAY = '<svg viewBox="0 0 24 24"><path d="M8 5.6v12.8a1 1 0 0 0 1.53.85l10.1-6.4a1 1 0 0 0 0-1.7L9.53 4.75A1 1 0 0 0 8 5.6z" fill="currentColor"/></svg>';
const ICON_PAUSE = '<svg viewBox="0 0 24 24"><rect x="6" y="5" width="4.4" height="14" rx="1.2" fill="currentColor"/>' +
    '<rect x="13.6" y="5" width="4.4" height="14" rx="1.2" fill="currentColor"/></svg>';
const ICON_REPLAY = '<svg viewBox="0 0 24 24"><path d="M12 5V2L7.5 6 12 10V7a5 5 0 1 1-5 5H5a7 7 0 1 0 7-7z" fill="currentColor"/></svg>';
const ICON_SAVE = '<svg viewBox="0 0 24 24"><path d="M11 3h2v9.2l3.3-3.3 1.4 1.4-5.7 5.7-5.7-5.7 1.4-1.4 3.3 3.3V3zM5 18h14v2H5z" fill="currentColor"/></svg>';
const ICON_CHECK = '<svg viewBox="0 0 24 24"><path d="M9.5 16.2 5.3 12l-1.4 1.4 5.6 5.6L20.1 8.4 18.7 7z" fill="currentColor"/></svg>';
const ICON_CROSS = '<svg viewBox="0 0 24 24"><path d="M18.3 7.1 16.9 5.7 12 10.6 7.1 5.7 5.7 7.1l4.9 4.9-4.9 4.9 1.4 1.4 4.9-4.9 4.9 4.9 1.4-1.4-4.9-4.9z" fill="currentColor"/></svg>';
const ICON_TRASH = '<svg viewBox="0 0 24 24"><path d="M9 3h6l1 2h4v2H4V5h4l1-2zM6 9h12l-1 11a2 2 0 0 1-2 2H9a2 2 0 0 1-2-2L6 9z' +
    'm4 2v8h1.6v-8H10zm3.4 0v8H15v-8h-1.6z" fill="currentColor"/></svg>';

// ---- helpers

function parsePayload<T>(payload: unknown): T | null {
    if (typeof payload !== "string") return payload as T;
    const text = payload.replace(/\u0000+$/, "");  // the module sends C strings
    try {
        return JSON.parse(text) as T;
    } catch (e) {
        console.warn(LOG + " unreadable message", text);
        return null;
    }
}

function pad2(n: number): string {
    return n < 10 ? "0" + n : String(n);
}

function formatClock(seconds: number): string {
    const s = Math.max(0, Math.floor(seconds));
    return Math.floor(s / 60) + ":" + pad2(s % 60);
}

function formatLocalDate(utcMs: number): string {
    const d = new Date(utcMs);
    return d.getDate() + " " + MONTHS[d.getMonth()] + " " + pad2(d.getHours()) + ":" + pad2(d.getMinutes());
}

function eventLabel(event: string): string {
    if (event === "takeoff") return "Takeoff";
    if (event === "landing") return "Landing";
    return "Manual";
}

function escapeHtml(s: string): string {
    return String(s)
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/"/g, "&quot;");
}

function capitalize(s: string): string {
    return s ? s.charAt(0).toUpperCase() + s.slice(1) : s;
}

function byId<T extends HTMLElement>(id: string): T {
    return document.getElementById(id) as T;
}

// A clip's one-line summary: aircraft, length, and for landings the touchdown figures.
function clipFacts(c: ClipInfo): string[] {
    const parts = [formatClock(c.durationSec)];
    if (c.aircraft) parts.unshift(c.aircraft);
    if (c.event === "landing") {
        if (typeof c.touchdownVsFpm === "number") parts.push(Math.round(c.touchdownVsFpm) + " fpm");
        if (typeof c.peakG === "number") parts.push(c.peakG.toFixed(2) + " G");
        if (c.bounces > 0) parts.push(c.bounces + (c.bounces === 1 ? " bounce" : " bounces"));
    }
    return parts;
}

// Sets an element's text, hiding it when empty.
function setText(id: string, text: string): void {
    const el = byId(id);
    el.textContent = text;
    el.classList.toggle("hidden", !text);
}

// Sets an element's markup only when it changed (status arrives 4 times a second), hiding it
// when empty.
function setHtml(id: string, html: string): void {
    const el = byId(id);
    if (el.getAttribute("data-html") !== html) {
        el.innerHTML = html;
        el.setAttribute("data-html", html);
    }
    el.classList.toggle("hidden", !html);
}

function controlLabel(control: string): string {
    return CONTROL_LABELS[control] || capitalize(control);
}

// The setup's progress: one numbered dot per step, joined by lines.
function stepsHtml(plan: SetupStep[]): string {
    let html = "";
    for (let i = 0; i < plan.length; ++i) {
        const p = plan[i];
        if (i > 0) html += '<div class="br-step-line' + (p.state === "next" ? "" : " done") + '"></div>';
        html += '<div class="br-step ' + escapeHtml(p.state) + '"><span class="br-step-dot">' + (i + 1) + "</span>" +
            '<span class="br-step-name">' + escapeHtml(controlLabel(p.control)) + "</span></div>";
    }
    return html;
}

// What the finished setup found, per control.
function resultsHtml(controls: SetupControl[]): string {
    let html = "";
    for (let i = 0; i < controls.length; ++i) {
        const c = controls[i];
        if (c.state === "absent") continue;
        html += '<div class="br-result ' + escapeHtml(c.state) + '"><div class="br-result-head">' +
            '<span class="br-result-name">' + escapeHtml(controlLabel(c.control)) + "</span>" +
            '<span class="br-result-state">' + escapeHtml(RESULT_LABELS[c.state] || c.state) + "</span></div>" +
            (c.method ? '<div class="br-result-method">' + escapeHtml(c.method) + "</div>" : "") + "</div>";
    }
    return html;
}

// What the setup is waiting for, ticked off as it happens.
function checklistHtml(items: Array<[boolean, string]>): string {
    let html = "";
    for (let i = 0; i < items.length; ++i) {
        html += '<div class="br-check' + (items[i][0] ? " ok" : "") + '"><span class="br-check-box"></span><span>' +
            escapeHtml(items[i][1]) + "</span></div>";
    }
    return html;
}

// While replaying: one row per method tried in this step, the current one last. The methods still
// to come get invisible rows, so the block (centred in the page) is as tall from the start and
// the text above doesn't move up as rows appear.
function attemptsHtml(setup: SetupStatus): string {
    const phase = setup.phase || "";
    if (phase !== "replay" && phase !== "hold" && phase !== "ask") return "";
    const done = setup.attempts || [];
    let html = "";
    for (let i = 0; i < done.length; ++i) {
        html += '<div class="br-attempt failed"><span class="br-attempt-mark"></span>Method ' + (i + 1) + " failed</div>";
    }
    const n = setup.method || done.length + 1;
    html += '<div class="br-attempt current br-enter"><span class="br-attempt-mark"></span>' +
        (phase === "ask" ? "Method " + n + " passed the check" : "Trying method " + n) + "</div>";
    for (let i = n; i < (setup.methods || 0); ++i) {
        html += '<div class="br-attempt reserved"><span class="br-attempt-mark"></span>Trying method ' + (i + 1) + "</div>";
    }
    return html;
}

// The offer's outline: [heading, text] per item.
function outlineHtml(items: string[][]): string {
    let html = "";
    for (let i = 0; i < items.length; ++i) {
        html += '<div class="br-outline"><span class="br-step-dot">' + (i + 1) + "</span><div>" +
            '<span class="br-outline-title">' + escapeHtml(items[i][0]) + "</span>" +
            '<span class="br-outline-text">' + escapeHtml(items[i][1]) + "</span></div></div>";
    }
    return html;
}

// Walks up from an event target to the nearest element carrying `attr` (no Element.closest).
function findWithAttr(start: EventTarget | null, attr: string, stop: HTMLElement): HTMLElement | null {
    let el = start as HTMLElement | null;
    while (el && el !== stop) {
        if (el.getAttribute && el.getAttribute(attr) !== null) return el;
        el = el.parentElement;
    }
    return null;
}

// ---- panel

class BetterReplayPanel extends UIElement {
    declare ingameUi: HTMLElement | null;
    declare listener: CommBusListener | null;
    declare timer: number;
    declare active: boolean;
    declare status: Status | null;
    declare clips: ClipInfo[];
    declare pending: ClipInfo[];
    declare clipsRev: number;
    declare loadingRev: number;
    declare requestId: number;
    declare startedAt: number;
    declare lastPollAt: number;
    declare lastResponseAt: number;
    declare seekDragging: boolean;
    declare seekFraction: number;
    declare confirmFile: string;
    declare confirmTimer: number;
    declare toastTimer: number;
    declare saveState: string;  // the save button: "", "saving", "saved" or "failed"
    declare saveTimer: number;
    declare view: string;  // "main" or "clips" (the player shows whenever a clip plays)
    declare setupButtons: string;
    declare shownPrompt: string;   // the setup's main text as shown, and the step and phase it was in
    declare shownStep: string;
    declare shownPhase: string;
    declare tickOn: boolean;
    declare holding: boolean;      // showing a done instruction for a moment
    declare heldStatus: Status | null;
    declare handleStatus: (payload: string) => void;
    declare handleClips: (payload: string) => void;
    declare handleReply: (payload: string) => void;

    constructor() {
        super();
        this.ingameUi = null;
        this.listener = null;
        this.timer = 0;
        this.active = false;
        this.status = null;
        this.clips = [];
        this.pending = [];
        this.clipsRev = -1;
        this.loadingRev = -1;
        this.requestId = 0;
        this.startedAt = Date.now();
        this.lastPollAt = 0;
        this.lastResponseAt = 0;
        this.seekDragging = false;
        this.seekFraction = 0;
        this.confirmFile = "";
        this.confirmTimer = 0;
        this.toastTimer = 0;
        this.saveState = "";
        this.saveTimer = 0;
        this.view = "main";
        this.setupButtons = "";
        this.shownPrompt = "";
        this.shownStep = "";
        this.shownPhase = "";
        this.tickOn = false;
        this.holding = false;
        this.heldStatus = null;
        this.handleStatus = (payload: string) => this.onStatus(payload);
        this.handleClips = (payload: string) => this.onClips(payload);
        this.handleReply = (payload: string) => this.onReply(payload);
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.ingameUi = this.querySelector("ingame-ui");
        this.bindControls();
        if (this.ingameUi) {
            this.ingameUi.addEventListener("panelActive", () => this.setActive(true));
            this.ingameUi.addEventListener("panelInactive", () => this.setActive(false));
        }
        this.ensureListener();
        // Poll slowly even while hidden, quickly while open (status messages are tiny).
        this.timer = window.setInterval(() => this.tick(), POLL_ACTIVE_MS);
        this.send({ op: "status" });
    }

    disconnectedCallback(): void {
        window.clearInterval(this.timer);
        this.releaseListener();
        super.disconnectedCallback();
    }

    // ---- CommBus

    ensureListener(): void {
        if (this.listener || typeof RegisterCommBusListener !== "function") return;
        try {
            const listener = RegisterCommBusListener(() => this.send({ op: "status" }));
            listener.on(STATUS_EVENT, this.handleStatus);
            listener.on(CLIPS_EVENT, this.handleClips);
            listener.on(REPLY_EVENT, this.handleReply);
            this.listener = listener;
        } catch (e) {
            console.error(LOG + " CommBus registration failed", e);
        }
    }

    releaseListener(): void {
        if (!this.listener) return;
        try {
            if (typeof this.listener.off === "function") {
                this.listener.off(STATUS_EVENT, this.handleStatus);
                this.listener.off(CLIPS_EVENT, this.handleClips);
                this.listener.off(REPLY_EVENT, this.handleReply);
            }
        } catch (e) {
            // the view is being torn down
        }
        this.listener = null;
    }

    send(cmd: Command): void {
        this.ensureListener();
        if (!this.listener) return;
        this.requestId += 1;
        cmd.id = this.requestId;
        try {
            const result = this.listener.callWasm(CMD_EVENT, JSON.stringify(cmd)) as { catch?: (fn: (e: unknown) => void) => void };
            if (result && typeof result.catch === "function") {
                result.catch((e: unknown) => console.warn(LOG + " callWasm failed", e));
            }
        } catch (e) {
            console.warn(LOG + " callWasm threw", e);
        }
    }

    setActive(active: boolean): void {
        if (active && !this.active) {
            this.clipsRev = -1;  // always refresh the list when the panel opens
            this.lastPollAt = 0;
        }
        this.active = active;
    }

    tick(): void {
        const now = Date.now();
        const interval = this.active ? POLL_ACTIVE_MS : POLL_INACTIVE_MS;
        if (now - this.lastPollAt >= interval - 20) {
            this.lastPollAt = now;
            this.send({ op: "status" });
        }
        if (now - (this.lastResponseAt || this.startedAt) > OFFLINE_AFTER_MS) this.renderOffline();
    }

    onStatus(payload: string): void {
        const s = parsePayload<Status>(payload);
        if (!s) return;
        this.lastResponseAt = Date.now();
        this.status = s;
        this.renderStatus(s);
        this.renderPlayer(s.play);
        this.renderView();
        this.renderSetup(s);
        if (s.clipsRev !== this.clipsRev && s.clipsRev !== this.loadingRev) {
            this.loadingRev = s.clipsRev;
            this.pending = [];
            this.send({ op: "clips", offset: 0 });
        }
    }

    onClips(payload: string): void {
        const page = parsePayload<ClipsPage>(payload);
        if (!page) return;
        if (page.offset === 0) this.pending = [];
        if (page.offset !== this.pending.length) return;  // stale page from an earlier request
        for (let i = 0; i < page.clips.length; ++i) this.pending.push(page.clips[i]);
        if (this.pending.length < page.total && page.clips.length > 0) {
            this.send({ op: "clips", offset: this.pending.length });
            return;
        }
        this.clips = this.pending;
        this.pending = [];
        this.clipsRev = page.rev;
        this.loadingRev = -1;
        this.renderList();
    }

    onReply(payload: string): void {
        const r = parsePayload<Reply>(payload);
        if (!r) return;
        if (r.op === "saveLast") {
            this.setSaveState(r.ok ? "saved" : "failed", r.error);
        } else if (!r.ok) {
            this.toast(r.error || "Command failed");
        } else if (r.op === "delete") {
            this.toast("Clip deleted");
        }
        this.send({ op: "status" });
    }

    // ---- controls

    bindControls(): void {
        byId("br-save").addEventListener("click", () => {
            if (this.saveState) return;  // one save at a time; the button says how it went
            this.setSaveState("saving", "");
            this.send({ op: "saveLast" });
        });
        byId("br-open-clips").addEventListener("click", () => this.showView("clips"));
        byId("br-back").addEventListener("click", () => this.showView("main"));
        byId("br-stop").addEventListener("click", () => {
            this.send({ op: "stop" });
            this.showView("main");  // stopping goes back to the main screen
        });
        byId("br-playpause").addEventListener("click", () => {
            const playing = this.status && this.status.play.playing;
            this.send({ op: playing ? "pause" : "resume" });
        });

        const speeds = byId("br-speeds");
        speeds.addEventListener("click", (e: Event) => {
            const button = findWithAttr(e.target, "data-speed", speeds);
            if (button) this.send({ op: "speed", x: parseFloat(button.getAttribute("data-speed") || "1") });
        });

        const track = byId("br-seek");
        const fractionAt = (e: MouseEvent): number => {
            const r = track.getBoundingClientRect();
            const f = r.width > 0 ? (e.clientX - r.left) / r.width : 0;
            return f < 0 ? 0 : f > 1 ? 1 : f;
        };
        track.addEventListener("mousedown", (e: MouseEvent) => {
            this.seekDragging = true;
            this.seekFraction = fractionAt(e);
            this.renderSeek(this.seekFraction);
        });
        window.addEventListener("mousemove", (e: MouseEvent) => {
            if (!this.seekDragging) return;
            this.seekFraction = fractionAt(e);
            this.renderSeek(this.seekFraction);
        });
        window.addEventListener("mouseup", () => {
            if (!this.seekDragging) return;
            this.seekDragging = false;
            const dur = this.status ? this.status.play.dur : 0;
            this.send({ op: "seek", t: this.seekFraction * dur });
        });

        const setup = byId("br-setup");
        setup.addEventListener("click", (e: Event) => {
            const button = findWithAttr(e.target, "data-a", setup);
            if (button) this.setupAction(button.getAttribute("data-a") || "");
        });
        byId("br-setup-run").addEventListener("click", () => this.send({ op: "setupStart" }));

        const list = byId("br-list");
        list.addEventListener("click", (e: Event) => {
            const button = findWithAttr(e.target, "data-act", list);
            if (!button) return;
            const file = button.getAttribute("data-file") || "";
            const act = button.getAttribute("data-act");
            if (act === "play") {
                this.send({ op: "play", file: file });
            } else if (act === "delete") {
                this.confirmDelete(file);
            }
        });
    }

    showView(view: string): void {
        this.view = view;
        this.renderView();
    }

    // One screen at a time: the player while a clip plays, else the main screen or the clips.
    renderView(): void {
        const view = this.status && this.status.play.active ? "player" : this.view;
        const root = byId("br-root");
        root.classList.toggle("view-main", view === "main");
        root.classList.toggle("view-clips", view === "clips");
        root.classList.toggle("view-player", view === "player");
    }

    setupAction(a: string): void {
        const active = !!(this.status && this.status.setup && this.status.setup.active);
        if (a === "start") {
            this.send({ op: "setupStart" });
        } else if (a === "never") {
            this.send({ op: "setupNever" });
        } else if (a === "later" && !active) {
            this.send({ op: "setupLater" });
        } else {
            this.send({ op: "setupAnswer", a: a });
        }
    }

    confirmDelete(file: string): void {
        window.clearTimeout(this.confirmTimer);
        if (this.confirmFile === file) {
            this.confirmFile = "";
            this.send({ op: "delete", file: file });
        } else {
            this.confirmFile = file;
            this.confirmTimer = window.setTimeout(() => {
                this.confirmFile = "";
                this.renderList();
            }, 3000);
        }
        this.renderList();
    }

    findClip(file: string): ClipInfo | null {
        for (let i = 0; i < this.clips.length; ++i) {
            if (this.clips[i].file === file) return this.clips[i];
        }
        return null;
    }

    toast(message: string): void {
        const el = byId("br-toast");
        el.textContent = message;
        el.classList.remove("hidden");
        window.clearTimeout(this.toastTimer);
        this.toastTimer = window.setTimeout(() => el.classList.add("hidden"), 3500);
    }

    // ---- rendering

    renderOffline(): void {
        byId("br-dot").className = "br-dot offline";
        byId("br-main").className = "br-main offline";
        byId("br-state").textContent = "No response from the recorder";
        byId("br-detail").textContent = "Is the betterreplay-recorder package installed?";
    }

    renderStatus(s: Status): void {
        let dot = "";
        let state = "";
        let detail = "";
        if (s.play.active) {
            dot = "playback";
            state = "Replaying";
            detail = "Recording resumes when you stop";
        } else if (s.rec) {
            dot = s.armed ? "recording" : "arming";
            state = "Recording";
            detail = s.armed ? capitalize(PHASES[s.phase] || s.phase) : "Arming in " + Math.ceil(s.armIn) + " s";
        } else {
            state = "Recording paused";
            detail = capitalize(s.why);
        }
        byId("br-dot").className = "br-dot " + dot;
        byId("br-main").className = "br-main " + (dot || "paused");  // colours the buffer meter
        byId("br-state").textContent = state;
        byId("br-detail").textContent = detail;
        // The buffer fills up to what it holds, then rolls.
        const max = s.bufferMax || 0;
        byId("br-buffer").textContent = formatClock(s.bufferSec);
        byId("br-buffer-max").textContent = max > 0 ? "of " + formatClock(max) : "";
        byId("br-buffer-fill").style.width = (max > 0 ? Math.min(1, s.bufferSec / max) * 100 : 0).toFixed(1) + "%";
        this.renderSave();
        byId("br-open-count").textContent = String(s.clipCount);
        byId("br-clips-count").textContent = String(s.clipCount);

        const error = byId("br-error");
        error.textContent = s.error;
        error.classList.toggle("hidden", !s.error);
    }

    // The save button tells how the save went, in place: "Saving...", then "Saved" (or "Couldn't
    // save") for a moment, then back to itself.
    setSaveState(state: string, error: string): void {
        this.saveState = state;
        window.clearTimeout(this.saveTimer);
        if (state) {
            // A save that gets no reply gives the button back too.
            this.saveTimer = window.setTimeout(() => this.setSaveState("", ""), state === "saving" ? 5000 : 2200);
        }
        byId("br-save").title = state === "failed" ? error : "";
        this.renderSave();
    }

    renderSave(): void {
        const button = byId<HTMLButtonElement>("br-save");
        const state = this.saveState;
        let label = "Save last 5 minutes";
        const max = this.status ? this.status.bufferMax || 0 : 0;
        if (max > 0) {
            const minutes = Math.round(max / 60);
            label = "Save last " + (minutes === 1 ? "minute" : minutes + " minutes");
        }
        if (state === "saving") label = "Saving...";
        else if (state === "saved") label = "Saved";
        else if (state === "failed") label = "Couldn't save";
        byId("br-save-label").textContent = label;
        if (button.getAttribute("data-state") !== state) {
            button.setAttribute("data-state", state);
            button.className = "br-btn br-save" + (state ? " " + state : "");
            byId("br-save-icon").innerHTML = state === "saved" ? ICON_CHECK : state === "failed" ? ICON_CROSS : ICON_SAVE;
        }
        // Not disabled while it shows the outcome, so it keeps its colour.
        button.disabled = !state && (!this.status || this.status.play.active || this.status.bufferSec < 1);
    }

    // While the setup is offered or running it takes the whole panel.
    renderSetup(s: Status): void {
        if (this.holding) {
            this.heldStatus = s;  // shown once the done instruction has had its moment
            return;
        }
        const setup = s.setup;
        if (setup && setup.active && this.instructionDone(setup)) {
            // The pilot did what's showing: tick it, then move on.
            this.holding = true;
            this.heldStatus = s;
            this.setTick(true);
            window.setTimeout(() => {
                this.holding = false;
                this.shownPhase = "";  // that change has had its tick
                const held = this.heldStatus;
                this.heldStatus = null;
                if (held) this.renderSetup(held);
            }, TICK_HOLD_MS);
            return;
        }
        let kicker = "";
        let phase = "";
        let prompt = "";
        let detail = "";
        let note = "";
        let list = "";
        let plan: SetupStep[] = [];
        let buttons: string[] = [];
        if (setup && setup.active) {
            kicker = "Control setup";
            phase = setup.phase || "";
            prompt = setup.prompt || "";
            note = setup.note || "";
            plan = setup.plan || [];
            buttons = setup.buttons || [];
            if (phase === "done") list = resultsHtml(setup.controls);
        } else if (setup && setup.offer === "new") {
            kicker = "New aircraft";
            phase = "offer";
            prompt = "Set up replay controls for this aircraft?";
            detail = "BetterReplay learns how its spoilers, flaps, throttle and gear respond, so replays can move them.";
            list = outlineHtml([
                ["Here on the runway, about a minute",
                    "Move the spoilers, flaps and throttle when asked. Each time, it replays your movement and asks whether it looked right."],
                ["After takeoff", "Raise the gear and lower it again, once."],
            ]);
            buttons = ["start", "later", "never"];
        } else if (setup && setup.offer === "wait") {
            // No profile yet, and not where the setup can start: say what's missing.
            const r = setup.ready || { parkingBrake: false, onRunway: false, engines: false };
            kicker = "New aircraft";
            phase = "wait";
            prompt = "Set up replay controls for this aircraft";
            detail = "Line up on a runway with the engines running, ready for takeoff. The setup can start once these are ticked:";
            list = checklistHtml([
                [r.parkingBrake, "Parking brake set"],
                [r.onRunway, "On a runway"],
                [r.engines, "Engines running"],
            ]);
            buttons = ["later", "never"];
        } else if (setup && setup.offer === "gear") {
            kicker = "Control setup";
            phase = "offer";
            prompt = "One step left: the gear";
            detail = "Raise the gear and lower it again, staying slow. It then replays that and asks whether it looked right.";
            buttons = ["start", "later"];
        }

        const show = !!prompt;
        byId("br-root").classList.toggle("setup-mode", show);
        byId("br-setup").className = "br-setup phase-" + (phase || "none") + (setup && setup.active ? " stepping" : "") +
            (show ? "" : " hidden");
        if (show) {
            byId("br-setup-kicker").textContent = kicker;
            byId("br-setup-aircraft").textContent = s.aircraft || "";
            setHtml("br-setup-steps", stepsHtml(plan));
            setText("br-setup-phase", SETUP_PHASES[phase] || "");
            this.renderPrompt(prompt, !!(setup && setup.active && setup.taskDone));
            setText("br-setup-detail", detail);
            setHtml("br-setup-attempts", setup && setup.active ? attemptsHtml(setup) : "");
            setHtml("br-setup-list", list);
            setText("br-setup-note", note);
            this.renderSetupButtons(buttons, !!(setup && setup.active));
        } else {
            this.setupButtons = "";
        }
        const active = !!(setup && setup.active);
        this.shownStep = active && setup ? setup.step || "" : "";
        this.shownPhase = active ? phase : "";
        this.renderPlane(s);
    }

    // Whether what the panel shows the pilot to do has just been done: the setup moved on within
    // the same step (a skip moves to another step; a time-out isn't done).
    instructionDone(setup: SetupStatus): boolean {
        return !!PILOT_PHASES[this.shownPhase] && setup.step === this.shownStep && !!this.shownPrompt &&
            (setup.prompt || "") !== this.shownPrompt && setup.phase !== "retry";
    }

    // The main text, sliding in when it changes, with a tick when it's done.
    renderPrompt(text: string, done: boolean): void {
        if (text !== this.shownPrompt) {
            this.shownPrompt = text;
            const el = byId("br-setup-prompt");
            el.textContent = text;
            el.classList.remove("br-enter");
            void el.offsetWidth;  // restart the animation
            el.classList.add("br-enter");
        }
        this.setTick(done);
    }

    setTick(on: boolean): void {
        if (on === this.tickOn) return;
        this.tickOn = on;
        const tick = byId("br-setup-tick");
        tick.classList.remove("on");
        if (on) {
            void tick.offsetWidth;  // restart the animation
            tick.classList.add("on");
        }
    }

    // Big buttons for the main choices, small ones underneath for the rest. Rebuilt only when the
    // set changes, so a click isn't lost to a re-render.
    renderSetupButtons(buttons: string[], active: boolean): void {
        const key = buttons.join(",") + (active ? "+" : "");
        if (key === this.setupButtons) return;
        this.setupButtons = key;
        let main = "";
        let minor = "";
        for (let i = 0; i < buttons.length; ++i) {
            const a = buttons[i];
            const kind = SETUP_KIND[a] || "minor";
            const label = a === "later" && active ? "Finish later" : SETUP_BUTTONS[a] || a;
            const html = '<button type="button" class="br-btn ' + (kind === "minor" ? "br-link" : "br-big br-" + kind) +
                '" data-a="' + escapeHtml(a) + '">' + escapeHtml(label) + "</button>";
            if (kind === "minor") minor += html;
            else main += html;
        }
        setHtml("br-setup-btns", main);
        setHtml("br-setup-minor", minor);
    }

    // The aircraft card: its name, and whether replays can move its controls.
    renderPlane(s: Status): void {
        const setup = s.setup;
        byId("br-plane").classList.toggle("hidden", !s.aircraft && !setup);
        byId("br-aircraft").textContent = s.aircraft || "Unknown aircraft";
        byId("br-controls-pill").classList.toggle("hidden", !setup);
        byId("br-setup-run").classList.toggle("hidden", !setup);
        if (!setup) return;
        // Configured: every control the aircraft has replays. Partly: some do, some were skipped,
        // didn't pass, or are still to do (the gear, after takeoff).
        const profile = setup.have === "partial" || setup.have === "complete";
        let present = 0;
        let ok = 0;
        const lines: string[] = [];
        for (let i = 0; i < setup.controls.length; ++i) {
            const c = setup.controls[i];
            if (c.state === "absent") continue;
            ++present;
            if (c.state === "ok") ++ok;
            lines.push((CONTROL_LABELS[c.control] || c.control) + ": " +
                (c.state === "ok" ? c.method : RESULT_LABELS[c.state] || c.state));
        }
        const level = !profile ? "missing" : ok === present ? "ok" : ok > 0 ? "partial" : "missing";
        const pill = byId("br-controls-pill");
        pill.className = "br-pill " + level;
        pill.textContent = level === "ok" ? "Configured" : level === "partial" ? "Partially configured" : "Not configured";
        pill.title = lines.join("\n");
        const run = byId<HTMLButtonElement>("br-setup-run");
        run.textContent = profile ? "Redo" : "Set up";
        run.classList.toggle("br-primary", level !== "ok");
        run.disabled = s.play.active;
    }

    renderSeek(fraction: number): void {
        byId("br-seek-fill").style.width = (fraction * 100).toFixed(2) + "%";
    }

    renderPlayer(p: PlayState): void {
        const rows = byId("br-list").querySelectorAll(".br-clip");
        for (let i = 0; i < rows.length; ++i) {
            const row = rows[i] as HTMLElement;
            row.classList.toggle("playing", p.active && row.getAttribute("data-file") === p.file);
        }
        if (!p.active) return;

        const clip = this.findClip(p.file);
        byId("br-now").textContent = clip ? eventLabel(clip.event) + " · " + formatLocalDate(clip.utcMs) : p.file;
        byId("br-now-sub").textContent = p.preparing
            ? "Lowering the gear before the replay starts…"
            : clip
              ? clipFacts(clip).join(" · ")
              : "";
        if (!this.seekDragging) this.renderSeek(p.dur > 0 ? p.t / p.dur : 0);

        const marker = byId("br-seek-marker");
        marker.classList.toggle("hidden", !(p.eventT > 0 && p.dur > 0));
        if (p.dur > 0) marker.style.left = ((p.eventT / p.dur) * 100).toFixed(2) + "%";

        byId("br-time").textContent = formatClock(p.t);
        byId("br-dur").textContent = formatClock(p.dur);
        // The big button: pause while playing, play when paused, replay at the end.
        const atEnd = p.dur > 0 && p.t >= p.dur - 0.05;
        const mode = p.playing ? "pause" : atEnd ? "replay" : "play";
        const button = byId("br-playpause");
        if (button.getAttribute("data-mode") !== mode) {
            button.setAttribute("data-mode", mode);
            button.innerHTML = mode === "pause" ? ICON_PAUSE : mode === "replay" ? ICON_REPLAY : ICON_PLAY;
            button.title = mode === "pause" ? "Pause" : mode === "replay" ? "Replay" : "Play";
        }

        const buttons = byId("br-speeds").querySelectorAll("[data-speed]");
        for (let i = 0; i < buttons.length; ++i) {
            const b = buttons[i] as HTMLElement;
            const speed = parseFloat(b.getAttribute("data-speed") || "0");
            b.classList.toggle("active", Math.abs(speed - p.speed) < 1e-3);
        }
    }

    renderList(): void {
        const list = byId("br-list");
        if (this.clips.length === 0) {
            list.innerHTML = '<div class="br-empty">No clips yet. Takeoffs and landings are saved automatically.</div>';
            return;
        }
        const playing = this.status && this.status.play.active ? this.status.play.file : "";
        let html = "";
        for (let i = 0; i < this.clips.length; ++i) {
            const c = this.clips[i];
            const file = escapeHtml(c.file);
            const parts = clipFacts(c).map(escapeHtml);
            const confirming = this.confirmFile === c.file;
            html +=
                '<div class="br-clip' + (c.file === playing ? " playing" : "") + '" data-file="' + file + '">' +
                '<span class="br-badge ' + escapeHtml(c.event) + '">' + eventLabel(c.event) + "</span>" +
                '<div class="br-clip-text">' +
                '<div class="br-clip-title">' + formatLocalDate(c.utcMs) + "</div>" +
                '<div class="br-clip-sub">' + parts.join(" · ") + "</div>" +
                "</div>" +
                '<div class="br-clip-btns">' +
                '<button type="button" class="br-icon-btn br-play" title="Play" data-act="play" data-file="' + file + '">' +
                ICON_PLAY + "</button>" +
                // Delete asks once more: the bin turns into a red "Delete?".
                (confirming
                    ? '<button type="button" class="br-btn br-confirm-delete" data-act="delete" data-file="' + file + '">Delete?</button>'
                    : '<button type="button" class="br-icon-btn br-delete" title="Delete" data-act="delete" data-file="' + file + '">' +
                      ICON_TRASH + "</button>") +
                "</div></div>";
        }
        list.innerHTML = html;
    }
}

if (!window.customElements.get("ingamepanel-betterreplay")) {
    window.customElements.define("ingamepanel-betterreplay", BetterReplayPanel);
}
checkAutoload();
