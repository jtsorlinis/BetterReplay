// Dev harness: serves the built toolbar panel outside the sim, with the MSFS globals mocked
// (UIElement, <ingame-ui>, RegisterCommBusListener) and a fake recorder module that speaks the
// same CommBus protocol as src/module/CommBridge.cpp. Run .\build.ps1 first, then:
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
    play: { active: false, file: '', t: 0, dur: 0, speed: 1, playing: false, eventT: 0 } };
  const listeners = {};
  // A scripted control setup (src/core/ControlSetup.h): you "move" each control for 4 s, the
  // replay takes 3 s, then it asks.
  const STEPS = ['spoilers', 'flaps', 'throttle', 'gear'];
  const PROMPTS = { spoilers: 'Deploy the spoilers fully, then retract them.',
    flaps: 'Lower the flaps one notch, wait for them to extend, then retract them.',
    throttle: 'Advance the throttle to full, then back to idle.', gear: 'Raise the gear, then lower it again.' };
  const QUESTIONS = { spoilers: 'Did the spoilers come up and go back down?', flaps: 'Did the flaps extend and retract?',
    throttle: 'Did the engines spool up and back down?', gear: 'Did the gear retract and extend again?' };
  const WATCH = { spoilers: 'Watch the spoilers', flaps: 'Watch the flaps', throttle: 'Listen to the engines', gear: 'Watch the gear' };
  const METHODS = ['input event HANDLING_SPOILERS (HANDLING_SPOILERS_SET, as a fraction)', 'key event SPOILERS_SET', 'simvars'];
  const setup = { offer: 'new', declined: false, results: null, active: false, step: 0, phase: '', t: 0, method: 0, note: '' };
  window.harness = { state: state, clips: clips, log: [], setup: setup };
  function setupNext(state) {
    setup.results[STEPS[setup.step]] = state;
    setup.step++;
    setup.t = 0;
    setup.method = 0;
    if (setup.step >= STEPS.length) setup.phase = 'done';
    else setup.phase = STEPS[setup.step] === 'gear' ? 'takeoff' : 'user';
  }
  function setupStatus() {
    const have = setup.declined ? 'declined' : !setup.results ? 'none' : Object.keys(setup.results).length === 4 &&
      STEPS.every((s) => setup.results[s] !== 'pending') ? 'complete' : 'partial';
    const out = { offer: setup.active ? '' : setup.offer, have: have, controls: [], active: setup.active };
    if (setup.results) {
      STEPS.forEach((s) => out.controls.push({ control: s, state: setup.results[s] || 'pending',
        method: setup.results[s] === 'ok' ? METHODS[0] : '' }));
    }
    if (!setup.active) return out;
    const step = STEPS[setup.step] || '';
    const done = setup.phase === 'done';
    const buttons = { user: ['skip', 'cancel'], takeoff: ['skip', 'later'], replay: ['cancel'], ask: ['yes', 'no', 'again', 'skip'],
      retry: ['retry', 'skip', 'cancel'], done: ['close'] }[setup.phase] || [];
    const prompt = done ? 'Control setup finished.' : setup.phase === 'ask' ? QUESTIONS[step] : setup.phase === 'replay'
      ? WATCH[step] + ': replaying your movement. Hands off the controls.' : setup.phase === 'takeoff'
      ? 'Take off. Once you\\'re climbing, raise the gear, then lower it again. Stay well below the gear speed limit.' : PROMPTS[step];
    const detail = done ? STEPS.map((s) => s + ': ' + setup.results[s]).join(' · ') : setup.phase === 'replay'
      ? 'Method ' + (setup.method + 1) + ' of ' + METHODS.length + ': ' + METHODS[setup.method] + '.'
      : setup.phase === 'user' && setup.t > 2 ? 'Now retract them.' : '';
    return Object.assign(out, { phase: setup.phase, step: done ? '' : step, title: done ? 'Done' : step.charAt(0).toUpperCase() + step.slice(1),
      stepNo: done ? STEPS.length : setup.step + 1, steps: STEPS.length, prompt: prompt, detail: detail, note: setup.note, buttons: buttons });
  }
  function setupAnswer(a) {
    if (a === 'cancel' || a === 'later') { setup.active = false; if (a === 'later') setup.offer = ''; return true; }
    if (setup.phase === 'ask' && a === 'yes') { setup.note = STEPS[setup.step] + ' set up.'; setupNext('ok'); return true; }
    if (setup.phase === 'ask' && a === 'no') {
      setup.method++; setup.t = 0; setup.note = 'Trying the next method.';
      if (setup.method >= METHODS.length) setupNext('failed'); else setup.phase = 'replay';
      return true;
    }
    if (setup.phase === 'ask' && a === 'again') { setup.phase = 'replay'; setup.t = 0; return true; }
    if (setup.phase === 'retry' && a === 'retry') { setup.phase = 'user'; setup.t = 0; return true; }
    if (setup.phase === 'done' && a === 'close') { setup.active = false; return true; }
    if (a === 'skip' && setup.phase !== 'replay' && setup.phase !== 'done') { setupNext('skipped'); return true; }
    return false;
  }

  setInterval(() => {
    if (setup.active) {
      setup.t += 0.1;
      if (setup.phase === 'user' && setup.t > 4) { setup.phase = 'replay'; setup.t = 0; }
      else if (setup.phase === 'takeoff' && setup.t > 5) { setup.phase = 'user'; setup.t = 0; }
      else if (setup.phase === 'replay' && setup.t > 3) { setup.phase = 'ask'; setup.t = 0; }
    }
    if (!state.play.active) {
      state.buffer = Math.min(300, state.buffer + 0.1);
      state.armIn = Math.max(0, state.armIn - 0.1);
      if (state.armIn === 0) { state.armed = true; state.phase = 'airborne'; }
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
      armed: state.armed, armIn: state.armIn, phase: state.phase, bufferSec: state.buffer,
      aircraft: 'Cessna 172 Skyhawk', clipsRev: state.rev, clipCount: clips.length, lastSaved: '', error: '',
      play: Object.assign({}, state.play), setup: setupStatus() };
  }
  function reply(id, op, ok, error) { emit('BetterReplay.Reply', { id: id, op: op, ok: ok, error: error || '' }); }
  function handle(cmd) {
    window.harness.log.push(cmd);
    const find = (f) => clips.findIndex((c) => c.file === f);
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
        Object.assign(state.play, { active: true, file: cmd.file, t: 0, dur: clips[i].durationSec, playing: true,
          eventT: clips[i].event === 'landing' ? 120 : 0 }); return reply(cmd.id, cmd.op, true); }
      case 'pause': state.play.playing = false; return reply(cmd.id, cmd.op, state.play.active, state.play.active ? '' : 'nothing is playing');
      case 'resume': if (state.play.t >= state.play.dur) state.play.t = 0; state.play.playing = true; return reply(cmd.id, cmd.op, true);
      case 'seek': state.play.t = Math.max(0, Math.min(state.play.dur, cmd.t)); return reply(cmd.id, cmd.op, true);
      case 'speed': state.play.speed = Math.max(0.25, Math.min(4, cmd.x)); return reply(cmd.id, cmd.op, true);
      case 'stop': state.play.active = false; state.play.playing = false; return reply(cmd.id, cmd.op, true);
      case 'setupStart':
        if (state.play.active) return reply(cmd.id, cmd.op, false, 'stop the replay first');
        Object.assign(setup, { active: true, offer: '', step: 0, phase: 'user', t: 0, method: 0, note: '', results: setup.results || {} });
        return reply(cmd.id, cmd.op, true);
      case 'setupAnswer': {
        const ok = setup.active && setupAnswer(cmd.a);
        return reply(cmd.id, cmd.op, ok, ok ? '' : "'" + cmd.a + "' doesn't fit this step");
      }
      case 'setupLater': setup.active = false; setup.offer = ''; return reply(cmd.id, cmd.op, true);
      case 'setupNever': setup.active = false; setup.offer = ''; setup.declined = true; return reply(cmd.id, cmd.op, true);
      case 'setupReset': setup.active = false; setup.offer = 'new'; setup.declined = false; setup.results = null; return reply(cmd.id, cmd.op, true);
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

const mocks = {
  '/JS/common.js': commonMock,
  '/JS/Services/CommBus.js': commBusMock,
  '/JS/coherent.js': '',
  '/JS/dataStorage.js': '',
  '/JS/buttons.js': '',
  '/JS/Services/ToolBarPanels.js': '',
  '/SCSS/common.css': 'body { margin: 16px; background: #222; font-family: sans-serif; }',
};
const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml' };

createServer((req, res) => {
  const url = decodeURIComponent(req.url.split('?')[0]);
  if (url === '/') {
    res.writeHead(302, { Location: '/InGamePanels/BetterReplayPanel/BetterReplayPanel.html' });
    return res.end();
  }
  if (url in mocks) {
    res.writeHead(200, { 'Content-Type': types[extname(url)] || 'text/plain' });
    return res.end(mocks[url]);
  }
  const prefix = '/InGamePanels/BetterReplayPanel/';
  const file = url.startsWith(prefix) ? join(panelDir, url.slice(prefix.length)) : null;
  if (file && !file.includes('..') && existsSync(file)) {
    res.writeHead(200, { 'Content-Type': types[extname(file)] || 'application/octet-stream' });
    return res.end(readFileSync(file));
  }
  res.writeHead(404);
  res.end();
}).listen(port, () => console.log(`panel harness on http://localhost:${port}/`));
