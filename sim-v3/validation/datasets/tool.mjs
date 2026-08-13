import { createHash } from 'node:crypto';
import { readdir, readFile, stat, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.dirname(fileURLToPath(import.meta.url));
const [command = 'inventory', datasetId] = process.argv.slice(2);

async function manifestIds() {
  return (await readdir(path.join(root, 'manifests')))
    .filter((name) => name.endsWith('.json'))
    .map((name) => name.slice(0, -5))
    .sort();
}

async function loadManifest(id) {
  if (!/^[a-z0-9-]+$/.test(id ?? '')) throw new Error('Dataset id must contain only lowercase letters, numbers, and hyphens');
  return JSON.parse(await readFile(path.join(root, 'manifests', `${id}.json`), 'utf8'));
}

async function filesUnder(directory, prefix = '') {
  const entries = await readdir(directory, { withFileTypes: true });
  const output = [];
  for (const entry of entries.sort((a, b) => a.name.localeCompare(b.name))) {
    const relative = path.posix.join(prefix, entry.name);
    const absolute = path.join(directory, entry.name);
    if (entry.isDirectory()) output.push(...await filesUnder(absolute, relative));
    else if (entry.isFile()) output.push({ absolute, relative });
  }
  return output;
}

async function sha256(filename) {
  const hash = createHash('sha256');
  hash.update(await readFile(filename));
  return hash.digest('hex');
}

async function inventory() {
  const rows = [];
  for (const id of await manifestIds()) {
    const manifest = await loadManifest(id);
    rows.push({ id, status: manifest.acquisition.status, evidenceAdmissible: manifest.evidenceAdmissible, source: manifest.primaryLandingPage });
  }
  process.stdout.write(`${JSON.stringify(rows, null, 2)}\n`);
}

async function lock(id) {
  const manifest = await loadManifest(id);
  if (!manifest.acquisition.localAcquisitionPermitted) {
    throw new Error(`${id}: acquisition is gated: ${manifest.acquisition.reason}`);
  }
  const rawDir = path.join(root, 'raw', id);
  if (!(await stat(rawDir).catch(() => null))?.isDirectory()) throw new Error(`${id}: no raw dataset directory at ${rawDir}`);
  const files = await filesUnder(rawDir);
  if (files.length === 0) throw new Error(`${id}: refusing to create an empty lock`);
  const lines = [];
  for (const file of files) lines.push(`${await sha256(file.absolute)}  ${file.relative}`);
  const lockPath = path.join(root, 'locks', `${id}.sha256`);
  await writeFile(lockPath, `${lines.join('\n')}\n`, { flag: 'wx' });
  process.stdout.write(`created ${lockPath} with ${files.length} file(s)\n`);
}

async function verify(id) {
  await loadManifest(id);
  const lockPath = path.join(root, 'locks', `${id}.sha256`);
  const lockText = await readFile(lockPath, 'utf8').catch(() => { throw new Error(`${id}: no checksum lock; evidence is not admissible`); });
  const rows = lockText.trim().split('\n').filter(Boolean);
  if (rows.length === 0) throw new Error(`${id}: checksum lock is empty`);
  for (const row of rows) {
    const match = /^([a-f0-9]{64})  ([^/].*)$/.exec(row);
    if (!match || match[2].split('/').includes('..')) throw new Error(`${id}: invalid lock row: ${row}`);
    const actual = await sha256(path.join(root, 'raw', id, ...match[2].split('/')));
    if (actual !== match[1]) throw new Error(`${id}: checksum mismatch for ${match[2]}`);
  }
  process.stdout.write(`${id}: verified ${rows.length} file(s)\n`);
}

try {
  if (command === 'inventory' && !datasetId) await inventory();
  else if (command === 'lock' && datasetId) await lock(datasetId);
  else if (command === 'verify' && datasetId) await verify(datasetId);
  else throw new Error('Usage: tool.mjs inventory | lock <dataset-id> | verify <dataset-id>');
} catch (error) {
  process.stderr.write(`${error.message}\n`);
  process.exitCode = 1;
}

