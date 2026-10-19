// Dev harness: serves the built toolbar panel outside the sim, with the MSFS globals mocked
// (UIElement, <ingame-ui>, RegisterCommBusListener) and a fake recorder module that speaks the
// same CommBus protocol as src/module/CommBridge.cpp. The control setup is faked in
// tools/harness-setup.js, with a strip beside the panel standing in for the pilot and aircraft.
// Run .\build.ps1 first, then:
//   node tools/panel-harness.mjs [port]      ->  http://localhost:8077/
import { createServer } from 'node:http';
import { readFileSync, existsSync } from 'node:fs';
import { join, extname, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const port = Number(process.argv[2] || 8077);
const panelDir = join(root, 'build', 'package', 'PackageSources', 'html_ui', 'InGamePanels', 'BetterReplayPanel');

const commonMock = `
class UIElement extends HTMLElement { connectedCallback() {} disconnectedCallback() {} }
class TemplateElement extends UIElement {}
function checkAutoload() {}
// <ingame-ui>: shows its children in a fixed-size frame and announces itself as active.
class IngameUiMock extends HTMLElement {
  connectedCallback() {
    this.style.cssText = 'display:block;width:520px;height:720px;border:1px solid #345;';
    setTimeout(() => this.dispatchEvent(new Event('panelActive')), 200);
  }
}
customElements.define('ingame-ui', IngameUiMock);
`;

// A fake recorder: same message shapes as Engine::statusJson / clipsJson and CommBridge replies.
const commBusMock = `
(function () {
  const now = Date.now();
  const clips = [];
  const aircraft = ['Cessna 172 Skyhawk', 'Airbus A320neo', 'A350-900 (Default Cabin)', 'TBM 930'];
  const airports = ['YMML', 'YSSY', 'NZAA', 'YBBN', 'YMAV', ''];  // '': none near, or recorded before airports
  for (let i = 0; i < 27; ++i) {
    const ev = i % 3 === 2 ? 'manual' : (i % 2 ? 'landing' : 'takeoff');
    clips.push({ file: 'clip' + i + '_' + ev + '.rpl', event: ev, utc: '', utcMs: now - i * 3600e3,
      aircraft: aircraft[i % 4], airport: ev === 'manual' ? '' : airports[i % 6], durationSec: ev === 'landing' ? 141 + i : 75 + i,
      touchdownVsFpm: ev === 'landing' ? -90 - i * 13 : null, peakG: ev === 'landing' ? 1.1 + i * 0.03 : null,
      bounces: ev === 'landing' ? i % 3 : 0, auto: ev !== 'manual' });
  }
  const state = { rev: 1, rec: true, why: '', armed: false, armIn: 6, phase: 'disarmed', buffer: 0,
    aircraft: 'Airbus A320neo', aircraftKnown: true, restoreLeft: 0,
    play: { active: false, file: '', t: 0, dur: 0, speed: 1, playing: false, eventT: 0 } };
  const listeners = {};
  // gearSec, flapsSec, enginesSec: Play first holds, frozen, until each has taken that long
  // ("lowering the gear", "extending the flaps", "spooling up the engines", together; 0: not part
  // of it); restoreSec: Stop keeps the aircraft frozen that long, "waiting for the gear" (10 s at
  // most, like EngineConfig::restoreMaxSec); fail: { op: 'error' } fails the next command of that
  // op with that error. setAircraft(title, known): another aircraft (known false: not identified
  // yet), which clips replay changing with it.
  window.harness = { state: state, clips: clips, log: [], setup: window.harnessSetup, gearSec: 0, flapsSec: 0,
    enginesSec: 0, restoreSec: 10, fail: {},
    setAircraft: (title, known) => {
      state.aircraft = title;
      state.aircraftKnown = known !== false;
      state.rev++;
    } };
  const playable = (c) => state.aircraftKnown && c.aircraft === state.aircraft;
  const noPrep = { gear: '', flaps: '', flapsIn: false, engines: '', enginesDown: false };
  const prepSteps = ['gear', 'flaps', 'engines'];

  setInterval(() => {
    window.harnessSetup.tick(0.1);
    if (state.restoreLeft > 0) {
      state.restoreLeft = Math.max(0, state.restoreLeft - 0.1);
    } else if (!state.play.active) {
      state.buffer = Math.min(600, state.buffer + 0.1);
      state.armIn = Math.max(0, state.armIn - 0.1);
      if (state.armIn === 0) { state.armed = true; state.phase = 'airborne'; }
    } else if (state.play.preparing) {
      // Like SimBridge::prepareJump: the steps together, each done when its time is up.
      const p = state.play;
      const prep = Object.assign({}, p.prep);
      prepSteps.forEach((s) => { if (prep[s] === 'active' && (p.left[s] -= 0.1) <= 0) prep[s] = 'done'; });
      p.prep = prep;
      if (!prepSteps.some((s) => prep[s] === 'active')) { p.preparing = false; p.prep = noPrep; }
    } else if (state.play.playing) {
      state.play.t = Math.min(state.play.dur, state.play.t + 0.1 * state.play.speed);
      if (state.play.t >= state.play.dur) state.play.playing = false;
    }
  }, 100);

  function emit(name, obj) {
    setTimeout(() => { (listeners[name] || []).forEach((fn) => fn(JSON.stringify(obj) + '\\u0000')); }, 15);
  }
  function status() {
    const restoring = !state.play.active && state.restoreLeft > 0;
    return { v: 1, rec: state.rec && !state.play.active && !restoring,
      why: state.play.active ? 'playback' : restoring ? 'restoring aircraft' : state.why,
      restoreIn: restoring ? state.restoreLeft : 0, restoreMax: 10,
      armed: state.armed, armIn: state.armIn, phase: state.phase, bufferSec: state.buffer, bufferMax: 600,
      aircraft: state.aircraft, clipsRev: state.rev, clipCount: clips.length, lastSaved: '', error: '',
      play: Object.assign({}, state.play), setup: window.harnessSetup.status() };
  }
  function reply(id, op, ok, error) { emit('BetterReplay.Reply', { id: id, op: op, ok: ok, error: error || '' }); }
  function handle(cmd) {
    window.harness.log.push(cmd);
    const setupReply = window.harnessSetup.command(cmd, state.play.active);
    if (setupReply) return reply(cmd.id, cmd.op, setupReply[0], setupReply[1]);
    const find = (f) => clips.findIndex((c) => c.file === f);
    const failure = window.harness.fail[cmd.op];
    if (failure && cmd.op !== 'status' && cmd.op !== 'clips') {
      delete window.harness.fail[cmd.op];
      return reply(cmd.id, cmd.op, false, failure);
    }
    switch (cmd.op) {
      case 'status': return emit('BetterReplay.Status', status());
      case 'clips': {
        // playable: like Engine::recordedOnThisAircraft (here by title alone).
        const off = cmd.offset || 0;
        const page = clips.slice(off, off + 20).map((c) => Object.assign({}, c, { playable: playable(c) }));
        return emit('BetterReplay.Clips', { rev: state.rev, total: clips.length, offset: off,
          aircraftKnown: state.aircraftKnown, clips: page });
      }
      case 'saveLast':
        clips.unshift({ file: 'manual_' + Date.now() + '.rpl', event: 'manual', utc: '', utcMs: Date.now(),
          aircraft: state.aircraft, airport: '', durationSec: state.buffer, touchdownVsFpm: null, peakG: null, bounces: 0, auto: false });
        state.rev++; return reply(cmd.id, cmd.op, true);
      case 'delete': { const i = find(cmd.file); if (i < 0) return reply(cmd.id, cmd.op, false, 'unknown clip');
        clips.splice(i, 1); state.rev++; if (state.play.file === cmd.file) state.play.active = false; return reply(cmd.id, cmd.op, true); }
      case 'play': { const i = find(cmd.file); if (i < 0) return reply(cmd.id, cmd.op, false, 'unknown clip');
        // Like Engine::play: only on the aircraft the clip was recorded on (harness.state.aircraft).
        if (!state.aircraftKnown) return reply(cmd.id, cmd.op, false, "the aircraft isn't identified yet");
        if (!playable(clips[i])) {
          return reply(cmd.id, cmd.op, false, 'recorded in the ' + clips[i].aircraft + ': switch to that aircraft to replay it');
        }
        state.restoreLeft = 0;
        const left = { gear: window.harness.gearSec, flaps: window.harness.flapsSec, engines: window.harness.enginesSec };
        const prep = Object.assign({}, noPrep);
        prepSteps.forEach((s) => { if (left[s] > 0) prep[s] = 'active'; });
        Object.assign(state.play, { active: true, file: cmd.file, t: 0, dur: clips[i].durationSec, playing: true,
          eventT: clips[i].event === 'landing' ? 120 : 0, preparing: prepSteps.some((s) => left[s] > 0), prep: prep,
          left: left }); return reply(cmd.id, cmd.op, true); }
      case 'pause': state.play.playing = false; return reply(cmd.id, cmd.op, state.play.active, state.play.active ? '' : 'nothing is playing');
      case 'resume': if (state.play.t >= state.play.dur) state.play.t = 0; state.play.playing = true; return reply(cmd.id, cmd.op, true);
      case 'seek': state.play.t = Math.max(0, Math.min(state.play.dur, cmd.t)); return reply(cmd.id, cmd.op, true);
      case 'speed': state.play.speed = Math.max(0.25, Math.min(4, cmd.x)); return reply(cmd.id, cmd.op, true);
      case 'stop': state.play.active = false; state.play.playing = false; state.play.preparing = false; state.play.prep = noPrep;
        state.restoreLeft = Math.min(10, window.harness.restoreSec); return reply(cmd.id, cmd.op, true);
      default: return reply(cmd.id, cmd.op, false, 'unknown command: ' + cmd.op);
    }
  }
  window.RegisterCommBusListener = function (cb) {
    if (cb) setTimeout(cb, 10);
    return {
      on: (name, fn) => { (listeners[name] = listeners[name] || []).push(fn); },
      off: (name, fn) => { listeners[name] = (listeners[name] || []).filter((f) => f !== fn); },
      callWasm: (name, json) => { if (name === 'BetterReplay.Cmd') handle(JSON.parse(json)); return Promise.resolve(); },
    };
  };
})();
`;

// The fake control setup and its control strip (tools/harness-setup.js), read on each request so
// edits show on reload.
const setupMock = () => readFileSync(join(root, 'tools', 'harness-setup.js'), 'utf8');

const mocks = {
  '/JS/common.js': () => commonMock,
  '/JS/Services/CommBus.js': () => setupMock() + commBusMock,
  '/JS/coherent.js': () => '',
  '/JS/dataStorage.js': () => '',
  '/JS/buttons.js': () => '',
  '/JS/Services/ToolBarPanels.js': () => '',
  '/SCSS/common.css': () => 'body { margin: 16px; background: #222; font-family: sans-serif; }',
};
const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml' };

createServer((req, res) => {
  const url = decodeURIComponent(req.url.split('?')[0]);
  if (url === '/') {
    res.writeHead(302, { Location: '/InGamePanels/BetterReplayPanel/BetterReplayPanel.html' });
    return res.end();
  }
  if (url in mocks) {
    res.writeHead(200, { 'Content-Type': types[extname(url)] || 'text/plain', 'Cache-Control': 'no-store' });
    return res.end(mocks[url]());
  }
  const prefix = '/InGamePanels/BetterReplayPanel/';
  const file = url.startsWith(prefix) ? join(panelDir, url.slice(prefix.length)) : null;
  if (file && !file.includes('..') && existsSync(file)) {
    res.writeHead(200, { 'Content-Type': types[extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-store' });
    return res.end(readFileSync(file));
  }
  res.writeHead(404);
  res.end();
}).listen(port, () => console.log(`panel harness on http://localhost:${port}/`));
