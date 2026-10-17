// Prints a BetterReplay clip (.rpl): header, channel list and a sample of frames, or the whole
// thing as CSV. Mirrors src/core/ClipFormat.h.
// Usage: node tools/rpl-dump.mjs <clip.rpl> [--csv out.csv] [--every N]
import { readFileSync, writeFileSync } from 'node:fs';

const args = process.argv.slice(2);
const file = args[0];
if (!file) {
  console.error('usage: node tools/rpl-dump.mjs <clip.rpl> [--csv out.csv] [--every N]');
  process.exit(2);
}
const csvOut = args.includes('--csv') ? args[args.indexOf('--csv') + 1] : null;
const every = args.includes('--every') ? Number(args[args.indexOf('--every') + 1]) : 30;

const buf = readFileSync(file);
const view = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
const fail = (msg) => {
  console.error(`${file}: ${msg}`);
  process.exit(1);
};
if (buf.toString('latin1', 0, 4) !== 'BRPL') fail('not a BetterReplay clip');
const version = view.getUint16(4, true);
const EVENTS = ['manual', 'takeoff', 'landing'];
const STORAGE = { 1: ['f64', 8], 2: ['f32', 4], 3: ['u8', 1] };

let pos = 8;
let meta = null;
let channels = null;
let samples = null;
const readStr = (p) => {
  const n = view.getUint16(p, true);
  return [buf.toString('utf8', p + 2, p + 2 + n), p + 2 + n];
};

while (pos + 8 <= buf.length) {
  const id = buf.toString('latin1', pos, pos + 4);
  const size = view.getUint32(pos + 4, true);
  const p = pos + 8;
  if (p + size > buf.length) fail(`truncated ${id} chunk`);
  if (id === 'META') {
    const aircraft = readStr(p + 32)[0];
    meta = {
      event: EVENTS[view.getUint8(p)] || view.getUint8(p),
      bounces: view.getUint16(p + 2, true),
      utcStart: new Date(Number(view.getBigInt64(p + 4, true))).toISOString(),
      sampleRateHz: view.getFloat32(p + 12, true),
      frames: view.getUint32(p + 16, true),
      eventOffsetSec: view.getFloat32(p + 20, true),
      touchdownVsFpm: view.getFloat32(p + 24, true),
      peakG: view.getFloat32(p + 28, true),
      aircraft,
    };
  } else if (id === 'CHAN') {
    const count = view.getUint16(p, true);
    let q = p + 2;
    let offset = 4;
    channels = [];
    for (let i = 0; i < count; ++i) {
      const storage = view.getUint8(q);
      const bytes = view.getUint8(q + 2);
      const [key, next] = readStr(q + 3);
      channels.push({ key, storage, bytes, offset });
      offset += bytes;
      q = next;
    }
  } else if (id === 'SAMP') {
    samples = { frames: view.getUint32(p, true), stride: view.getUint32(p + 4, true), base: p + 8 };
  } else {
    console.log(`(skipping unknown chunk ${id}, ${size} bytes)`);
  }
  pos = p + size;
}
if (!meta || !channels || !samples) fail('missing META/CHAN/SAMP');

const valueAt = (frame, ch) => {
  const at = samples.base + frame * samples.stride + ch.offset;
  switch (ch.storage) {
    case 1: return view.getFloat64(at, true);
    case 2: return view.getFloat32(at, true);
    case 3: { const b = view.getUint8(at); return b === 255 ? NaN : b; }
    default: return NaN;
  }
};
const timeAt = (frame) => view.getFloat32(samples.base + frame * samples.stride, true);

console.log(`format v${version}`, meta);
console.log(`${channels.length} channels: ${channels.map((c) => `${c.key}:${(STORAGE[c.storage] || ['?'])[0]}`).join(' ')}`);
console.log(`${samples.frames} frames, ${timeAt(samples.frames - 1).toFixed(2)} s, stride ${samples.stride} bytes`);

const fmt = (v) => (Number.isInteger(v) ? String(v) : v.toFixed(Math.abs(v) < 1000 ? 4 : 1));
if (csvOut) {
  const lines = ['t,' + channels.map((c) => c.key).join(',')];
  for (let f = 0; f < samples.frames; ++f) lines.push([timeAt(f).toFixed(3), ...channels.map((c) => fmt(valueAt(f, c)))].join(','));
  writeFileSync(csvOut, lines.join('\n'));
  console.log(`wrote ${csvOut}`);
} else {
  const show = ['lat', 'lon', 'alt', 'agl', 'pitch', 'bank', 'hdg', 'gs', 'vs', 'ongnd', 'g', 'gear_c', 'flap_l', 'l_land'];
  const cols = show.map((k) => channels.find((c) => c.key === k)).filter(Boolean);
  console.log(['t', ...cols.map((c) => c.key)].map((s) => s.padStart(11)).join(''));
  for (let f = 0; f < samples.frames; f += every) {
    console.log([timeAt(f).toFixed(2), ...cols.map((c) => fmt(valueAt(f, c)))].map((s) => s.padStart(11)).join(''));
  }
}
