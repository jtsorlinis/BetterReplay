// Combines the included control profiles (profiles/*.json, one aircraft each, written by
// tools/export-profiles.mjs) into one file the way the module keeps them (aircraft.json), for the
// package's profiles/aircraft.json. build.ps1 runs it. Fails on a file that isn't a profile, or
// two files for the same aircraft.
// Usage: node tools/pack-profiles.mjs <profiles folder> <out.json>
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';

const [dir, out] = process.argv.slice(2);
if (!dir || !out) {
  console.error('usage: node tools/pack-profiles.mjs <profiles folder> <out.json>');
  process.exit(2);
}
const fail = (msg) => {
  console.error(msg);
  process.exit(1);
};

const byKey = new Map();
for (const name of readdirSync(dir).filter((f) => f.endsWith('.json')).sort()) {
  let p;
  try {
    p = JSON.parse(readFileSync(join(dir, name), 'utf8'));
  } catch (e) {
    fail(`${name}: ${e.message}`);
  }
  if (!p || typeof p.key !== 'string' || !p.key) fail(`${name}: no key`);
  if (!p.controls || typeof p.controls !== 'object') fail(`${name}: no controls`);
  if (byKey.has(p.key)) fail(`${name} and ${byKey.get(p.key).name} are both ${p.key}`);
  byKey.set(p.key, { name, p });
}
if (byKey.size === 0) fail(`no profiles in ${dir}`);

const aircraft = [...byKey.values()].map((e) => e.p).sort((a, b) => a.key.localeCompare(b.key));
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify({ version: 1, aircraft }));
console.log(`  ${aircraft.length} included control profile(s): ${aircraft.map((p) => p.key).join(', ')}`);
