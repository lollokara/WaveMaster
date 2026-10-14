// gzip dist/* into ../data with flat names: index.html -> index.html.gz
import { readdirSync, readFileSync, writeFileSync, mkdirSync, rmSync, statSync } from 'node:fs';
import { gzipSync, constants } from 'node:zlib';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = join(root, 'dist'), dst = join(root, '..', 'data');
rmSync(dst, { recursive: true, force: true });
mkdirSync(dst, { recursive: true });
let total = 0;
for (const f of readdirSync(src)) {
  if (statSync(join(src, f)).isDirectory()) continue;
  const gz = gzipSync(readFileSync(join(src, f)), { level: constants.Z_BEST_COMPRESSION });
  writeFileSync(join(dst, f + '.gz'), gz);
  total += gz.length;
  console.log(`${(f + '.gz').padEnd(28)} ${gz.length} B`);
}
console.log(`total ${total} B (${(total / 1024).toFixed(1)} KiB gzipped)`);
