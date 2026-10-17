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
    eventT: number;
}
interface Status {
    rec: boolean;
    why: string;
    armed: boolean;
    armIn: number;
    phase: string;
    bufferSec: number;
    aircraft: string;
    clipsRev: number;
    clipCount: number;
    lastSaved: string;
    error: string;
    play: PlayState;
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
    keep: boolean;
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
    return "Saved";
}

function eventBadge(event: string): string {
    if (event === "takeoff") return "TO";
    if (event === "landing") return "LDG";
    return "SAVE";
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
        if (!r.ok) {
            this.toast(r.error || "Command failed");
        } else if (r.op === "saveLast") {
            this.toast("Saved the last few minutes");
        } else if (r.op === "delete") {
            this.toast("Clip deleted");
        }
        this.send({ op: "status" });
    }

    // ---- controls

    bindControls(): void {
        byId("br-save").addEventListener("click", () => this.send({ op: "saveLast" }));
        byId("br-stop").addEventListener("click", () => this.send({ op: "stop" }));
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

        const list = byId("br-list");
        list.addEventListener("click", (e: Event) => {
            const button = findWithAttr(e.target, "data-act", list);
            if (!button) return;
            const file = button.getAttribute("data-file") || "";
            const act = button.getAttribute("data-act");
            if (act === "play") {
                this.send({ op: "play", file: file });
            } else if (act === "keep") {
                const clip = this.findClip(file);
                this.send({ op: "keep", file: file, keep: !(clip && clip.keep) });
            } else if (act === "delete") {
                this.confirmDelete(file);
            }
        });
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
            detail = s.armed ? PHASES[s.phase] || s.phase : "arming in " + Math.ceil(s.armIn) + " s";
        } else {
            state = "Recording paused";
            detail = capitalize(s.why);
        }
        detail += " · buffer " + formatClock(s.bufferSec);
        if (s.aircraft) detail += " · " + s.aircraft;

        byId("br-dot").className = "br-dot " + dot;
        byId("br-state").textContent = state;
        byId("br-detail").textContent = detail;
        byId<HTMLButtonElement>("br-save").disabled = s.play.active || s.bufferSec < 1;
        byId("br-count").textContent = String(s.clipCount);

        const error = byId("br-error");
        error.textContent = s.error;
        error.classList.toggle("hidden", !s.error);
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
        byId("br-player").classList.toggle("hidden", !p.active);
        if (!p.active) return;

        const clip = this.findClip(p.file);
        byId("br-now").textContent =
            "Playing: " + (clip ? eventLabel(clip.event) + " · " + formatLocalDate(clip.utcMs) : p.file);
        if (!this.seekDragging) this.renderSeek(p.dur > 0 ? p.t / p.dur : 0);

        const marker = byId("br-seek-marker");
        marker.classList.toggle("hidden", !(p.eventT > 0 && p.dur > 0));
        if (p.dur > 0) marker.style.left = ((p.eventT / p.dur) * 100).toFixed(2) + "%";

        byId("br-time").textContent = formatClock(p.t);
        byId("br-dur").textContent = formatClock(p.dur);
        const atEnd = p.dur > 0 && p.t >= p.dur - 0.05;
        byId("br-playpause").textContent = p.playing ? "Pause" : atEnd ? "Replay" : "Play";

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
            const parts = [formatClock(c.durationSec)];
            if (c.aircraft) parts.unshift(escapeHtml(c.aircraft));
            if (c.event === "landing") {
                if (typeof c.touchdownVsFpm === "number") parts.push(Math.round(c.touchdownVsFpm) + " fpm");
                if (typeof c.peakG === "number") parts.push(c.peakG.toFixed(2) + " G");
                if (c.bounces > 0) parts.push(c.bounces + (c.bounces === 1 ? " bounce" : " bounces"));
            }
            const confirming = this.confirmFile === c.file;
            html +=
                '<div class="br-clip' + (c.file === playing ? " playing" : "") + '" data-file="' + file + '">' +
                '<span class="br-badge ' + escapeHtml(c.event) + '">' + eventBadge(c.event) + "</span>" +
                '<div class="br-clip-text">' +
                '<div class="br-clip-title">' + eventLabel(c.event) + " · " + formatLocalDate(c.utcMs) + "</div>" +
                '<div class="br-clip-sub">' + parts.join(" · ") + "</div>" +
                "</div>" +
                '<div class="br-clip-btns">' +
                '<button type="button" class="br-btn br-primary" data-act="play" data-file="' + file + '">Play</button>' +
                '<button type="button" class="br-btn' + (c.keep ? " kept" : "") + '" data-act="keep" data-file="' + file + '">' +
                (c.keep ? "Kept" : "Keep") + "</button>" +
                '<button type="button" class="br-btn br-danger' + (confirming ? " confirm" : "") + '" data-act="delete" data-file="' + file + '">' +
                (confirming ? "Sure?" : "Delete") + "</button>" +
                "</div></div>";
        }
        list.innerHTML = html;
    }
}

if (!window.customElements.get("ingamepanel-betterreplay")) {
    window.customElements.define("ingamepanel-betterreplay", BetterReplayPanel);
}
checkAutoload();
