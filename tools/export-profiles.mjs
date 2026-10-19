// Copies the control profiles from the sim's work folder (aircraft.json) into profiles/, one file per
// aircraft (profiles/pmdg-777.json). build.ps1 combines them into the package's profiles/aircraft.json
// (tools/pack-profiles.mjs).
// An aircraft's file is found by the key inside it, so a renamed file still gets its updates. The
// work folder's profile replaces it unless the file's is newer (updatedUtcMs); files for aircraft
// the work folder doesn't have are left alone. Profiles with nothing set up (every control pending
// or skipped) aren't copied, and "never for this aircraft" isn't either.
// Usage: node tools/export-profiles.mjs [--from <work folder or aircraft.json>]
import { existsSync, mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const outDir = join(dirname(fileURLToPath(import.meta.url)), '..', 'profiles');
const defaultWork = join(process.env.APPDATA ?? '', 'Microsoft Flight Simulator 2024', 'WASM', 'MSFS2024',
  'betterreplay-recorder', 'work');

const args = process.argv.slice(2);
const from = args.includes('--from') ? args[args.indexOf('--from') + 1] : defaultWork;
if (!from) {
  console.error('usage: node tools/export-profiles.mjs [--from <work folder or aircraft.json>]');
  process.exit(2);
}
const inFile = existsSync(from) && statSync(from).isDirectory() ? join(from, 'aircraft.json') : from;
if (!existsSync(inFile)) {
  console.error(`${inFile} not found: set up an aircraft's controls in the sim first, or pass --from`);
  process.exit(1);
}
const fail = (msg) => {
  console.error(msg);
  process.exit(1);
};

const sim = JSON.parse(readFileSync(inFile, 'utf8'));
if (!Array.isArray(sim.aircraft)) fail(`${inFile}: no aircraft array`);

// The repo's files by the key in them.
mkdirSync(outDir, { recursive: true });
const files = new Map();
for (const name of readdirSync(outDir).filter((f) => f.endsWith('.json'))) {
  let p;
  try {
    p = JSON.parse(readFileSync(join(outDir, name), 'utf8'));
  } catch (e) {
    fail(`profiles/${name}: ${e.message}`);
  }
  if (!p.key) fail(`profiles/${name}: no key`);
  if (files.has(p.key)) fail(`profiles/${name} and profiles/${files.get(p.key).name} are both ${p.key}`);
  files.set(p.key, { name, profile: p });
}

// A new aircraft's file name: the last part of its key ("simobjects/airplanes/pmdg 777" -> "pmdg-777"),
// "title-<title>" for one known only by its title, and a number after it if that's taken.
const taken = new Set([...files.values()].map((f) => f.name.toLowerCase()));
const fileName = (key) => {
  const base = (key.startsWith('title:') ? 'title-' + key.slice(6) : key.split('/').pop())
    .replace(/[<>:"/\\|?*\x00-\x1f\s]+/g, '-').replace(/^[-.]+|[-.]+$/g, '') || 'aircraft';
  for (let n = 1; ; ++n) {
    const name = (n === 1 ? base : `${base}-${n}`) + '.json';
    if (!taken.has(name.toLowerCase())) {
      taken.add(name.toLowerCase());
      return name;
    }
  }
};

// The states the module follows a profile for (ok, failed) or that say what the aircraft lacks.
const setUp = (p) => Object.values(p.controls ?? {}).some((c) => ['ok', 'failed', 'absent'].includes(c.state));

// Indented, but arrays of plain values (the lever tables' [axis, lever] pairs) on one line, so
// diffs stay readable.
const format = (v, indent = '') => {
  const inner = indent + '  ';
  if (Array.isArray(v)) {
    if (v.every((x) => x === null || typeof x !== 'object')) return `[${v.map((x) => JSON.stringify(x)).join(', ')}]`;
    return `[\n${v.map((x) => inner + format(x, inner)).join(',\n')}\n${indent}]`;
  }
  if (v && typeof v === 'object') {
    const entries = Object.entries(v);
    if (entries.length === 0) return '{}';
    return `{\n${entries.map(([k, x]) => `${inner}${JSON.stringify(k)}: ${format(x, inner)}`).join(',\n')}\n${indent}}`;
  }
  return JSON.stringify(v);
};

const log = [];
for (const p of sim.aircraft) {
  if (!p.key || !setUp(p)) {
    log.push(`  skipped  ${p.key} (nothing set up)`);
    continue;
  }
  const { declined, ...profile } = p;
  const old = files.get(p.key);
  if (old && (old.profile.updatedUtcMs ?? 0) > (p.updatedUtcMs ?? 0)) {
    log.push(`  kept     profiles/${old.name} (newer than the work folder's)`);
    continue;
  }
  if (old && JSON.stringify(old.profile) === JSON.stringify(profile)) {
    log.push(`  same     profiles/${old.name}`);
    continue;
  }
  const name = old ? old.name : fileName(p.key);
  writeFileSync(join(outDir, name), format(profile) + '\n');
  log.push(`  ${old ? 'updated' : 'added  '}  profiles/${name} (${p.key})`);
}
for (const [key, { name }] of files) {
  if (!sim.aircraft.some((p) => p.key === key)) log.push(`  kept     profiles/${name} (not in the work folder)`);
}
console.log(`${inFile} -> profiles/`);
console.log(log.join('\n'));
