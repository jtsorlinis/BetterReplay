// Sanity-checks a built module: required MSFS 2024 exports are present and every host import
// comes from a known API family (catches accidentally linking something the sim doesn't provide).
// Usage: node tools/check-wasm.mjs build/module/Release/BetterReplay.wasm
import { readFileSync } from 'node:fs';

const file = process.argv[2];
const mod = new WebAssembly.Module(readFileSync(file));
const exports = new Set(WebAssembly.Module.exports(mod).map((e) => e.name));
const imports = WebAssembly.Module.imports(mod);

const requiredExports = [
  'module_init',
  'module_deinit',
  'module_post_sim_systems_update',
  'GetSimConnectVersion',
  'GetExtensionVersion',
  'malloc',
  'free',
];
const allowedImport = (i) =>
  i.module === 'wasi_snapshot_preview1' ||
  (i.module === 'env' && /^(SimConnect_|fsCommBus|fsIO|fsVars|fsFlow|fsEvents)/.test(i.name));

let ok = true;
for (const name of requiredExports) {
  if (!exports.has(name)) {
    console.error(`missing export: ${name}`);
    ok = false;
  }
}
for (const i of imports) {
  if (!allowedImport(i)) {
    console.error(`unexpected import: ${i.module}.${i.name}`);
    ok = false;
  }
}
const env = imports.filter((i) => i.module === 'env').map((i) => i.name);
const wasi = imports.filter((i) => i.module === 'wasi_snapshot_preview1').map((i) => i.name);
console.log(`  ${file}: ${exports.size} exports, imports env[${env.length}] wasi[${wasi.join(', ')}]`);
process.exit(ok ? 0 : 1);
