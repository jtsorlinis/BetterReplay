// Compiles the panel's TypeScript to JavaScript by stripping type annotations (Node 22.13+/24's
// built-in module.stripTypeScriptTypes). Only erasable syntax is allowed; there is no type check.
// Usage: node tools/strip-types.mjs <in.ts> <out.js>
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import module from 'node:module';

const [input, output] = process.argv.slice(2);
if (!input || !output) {
  console.error('usage: node tools/strip-types.mjs <in.ts> <out.js>');
  process.exit(2);
}
if (typeof module.stripTypeScriptTypes !== 'function') {
  console.error(`Node ${process.version} has no module.stripTypeScriptTypes; use Node 22.13+ or 24+.`);
  process.exit(2);
}

const source = readFileSync(input, 'utf8');
let js;
try {
  js = module.stripTypeScriptTypes(source, { mode: 'strip' });
} catch (e) {
  console.error(`${input}: ${e.message}`);
  process.exit(1);
}
// Parse the result as a classic script to catch anything the strip step left behind.
try {
  new Function(js);
} catch (e) {
  console.error(`${output}: generated JavaScript does not parse: ${e.message}`);
  process.exit(1);
}
mkdirSync(dirname(output), { recursive: true });
writeFileSync(output, '// Generated from BetterReplayPanel.ts by tools/strip-types.mjs. Do not edit.\n' + js);
console.log(`  ${input} -> ${output}`);
