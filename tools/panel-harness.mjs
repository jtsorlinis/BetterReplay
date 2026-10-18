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
  const aircraft = ['Cessna 172 Skyhawk', 'Airbus A320neo', 'Boeing 737-800', 'TBM 930'];
  for (let i = 0; i < 27; ++i) {
    const ev = i % 3 === 2 ? 'manual' : (i % 2 ? 'landing' : 'takeoff');
    clips.push({ file: 'clip' + i + '_' + ev + '.rpl', event: ev, utc: '', utcMs: now - i * 3600e3,
      aircraft: aircraft[i % 4], durationSec: ev === 'landing' ? 141 + i : 75 + i,
      touchdownVsFpm: ev === 'landing' ? -90 - i * 13 : null, peakG: ev === 'landing' ? 1.1 + i * 0.03 : null,
      bounces: ev === 'landing' ? i % 3 : 0, auto: ev !== 'manual' });
  }
  const state = { rev: 1, rec: true, why: '', armed: false, armIn: 6, phase: 'disarmed', buffer: 0,
    aircraft: 'Airbus A320neo',
    play: { active: false, file: '', t: 0, dur: 0, speed: 1, playing: false, eventT: 0 } };
  const listeners = {};
  // prepareSec: Play first holds that long, "lowering the gear"; fail: { op: 'error' } fails the
  // next command of that op with that error.
  window.harness = { state: state, clips: clips, log: [], setup: window.harnessSetup, prepareSec: 0, fail: {} };

  setInterval(() => {
    window.harnessSetup.tick(0.1);
    if (!state.play.active) {
      state.buffer = Math.min(600, state.buffer + 0.1);
      state.armIn = Math.max(0, state.armIn - 0.1);
      if (state.armIn === 0) { state.armed = true; state.phase = 'airborne'; }
    } else if (state.play.preparing) {
      state.play.prepareLeft -= 0.1;
      if (state.play.prepareLeft <= 0) state.play.preparing = false;
    } else if (state.play.playing) {
      state.play.t = Math.min(state.play.dur, state.play.t + 0.1 * state.play.speed);
      if (state.play.t >= state.play.dur) state.play.playing = false;
    }
  }, 100);

  function emit(name, obj) {
    setTimeout(() => { (listeners[name] || []).forEach((fn) => fn(JSON.stringify(obj) + '\\u0000')); }, 15);
  }
  function status() {
    return { v: 1, rec: state.rec && !state.play.active, why: state.play.active ? 'playback' : state.why,
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
        const off = cmd.offset || 0;
        return emit('BetterReplay.Clips', { rev: state.rev, total: clips.length, offset: off, clips: clips.slice(off, off + 20) });
      }
      case 'saveLast':
        clips.unshift({ file: 'manual_' + Date.now() + '.rpl', event: 'manual', utc: '', utcMs: Date.now(),
          aircraft: 'Cessna 172 Skyhawk', durationSec: state.buffer, touchdownVsFpm: null, peakG: null, bounces: 0, auto: false });
        state.rev++; return reply(cmd.id, cmd.op, true);
      case 'delete': { const i = find(cmd.file); if (i < 0) return reply(cmd.id, cmd.op, false, 'unknown clip');
        clips.splice(i, 1); state.rev++; if (state.play.file === cmd.file) state.play.active = false; return reply(cmd.id, cmd.op, true); }
      case 'play': { const i = find(cmd.file); if (i < 0) return reply(cmd.id, cmd.op, false, 'unknown clip');
        // Like Engine::play: only on the aircraft the clip was recorded on (harness.state.aircraft).
        if (clips[i].aircraft !== state.aircraft) {
          return reply(cmd.id, cmd.op, false, 'recorded in the ' + clips[i].aircraft + ': switch to that aircraft to replay it');
        }
        Object.assign(state.play, { active: true, file: cmd.file, t: 0, dur: clips[i].durationSec, playing: true,
          eventT: clips[i].event === 'landing' ? 120 : 0, preparing: window.harness.prepareSec > 0,
          prepareLeft: window.harness.prepareSec }); return reply(cmd.id, cmd.op, true); }
      case 'pause': state.play.playing = false; return reply(cmd.id, cmd.op, state.play.active, state.play.active ? '' : 'nothing is playing');
      case 'resume': if (state.play.t >= state.play.dur) state.play.t = 0; state.play.playing = true; return reply(cmd.id, cmd.op, true);
      case 'seek': state.play.t = Math.max(0, Math.min(state.play.dur, cmd.t)); return reply(cmd.id, cmd.op, true);
      case 'speed': state.play.speed = Math.max(0.25, Math.min(4, cmd.x)); return reply(cmd.id, cmd.op, true);
      case 'stop': state.play.active = false; state.play.playing = false; state.play.preparing = false;
        return reply(cmd.id, cmd.op, true);
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
