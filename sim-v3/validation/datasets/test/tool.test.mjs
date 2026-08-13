import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { spawnSync } from 'node:child_process';

const base = new URL('../', import.meta.url);
const aero = JSON.parse(await readFile(new URL('manifests/aero4river-v1.json', base)));
const simman = JSON.parse(await readFile(new URL('manifests/simman2014.json', base)));
const kvlcc2 = JSON.parse(await readFile(new URL('manifests/yasukawa-yoshimura-2015-kvlcc2.json', base)));
const wpcc = JSON.parse(await readFile(new URL('manifests/wpcc-v4.json', base)));
const marinRaw = JSON.parse(await readFile(new URL('manifests/kvlcc2-simman2008-third-party.json', base)));

assert.equal(aero.license.spdx, 'CC-BY-4.0');
assert.equal(aero.coordinateConvention.status, 'resolved-v1');
assert.equal(aero.evidenceAdmissible, true);
assert.equal(simman.acquisition.localAcquisitionPermitted, false);
assert.equal(simman.evidenceAdmissible, false);
assert.match(simman.scientificScope.claimLimit, /model structure/i);
assert.equal(kvlcc2.coefficientTransferToVehicleBPermitted, false);
assert.match(kvlcc2.scientificScope.claimLimit, /does not validate.*USV-scale coefficients/i);
assert.equal(kvlcc2.artifactContract.artifactKind, 'kvlcc2-mmg-reference-reproduction');
assert.equal(kvlcc2.artifactContract.evidenceStatus, 'reference-reproduction-only');
assert.equal(kvlcc2.artifactContract.trajectoryOverlayRole, 'qualitative-non-gating');
assert.equal(wpcc.version, '4');
assert.equal(wpcc.license.spdx, 'CC-BY-4.0');
assert.equal(wpcc.coordinateConvention.motionReferencePoint, '[Lpp/2, 0, WL]');
assert.equal(wpcc.channelPolicy.diagnosticReason.includes('EKF-derived'), true);
assert.equal(wpcc.scientificScope.coefficientTransferToVehicleBPermitted, false);
assert.equal(marinRaw.license.redistributionPermitted, false);
assert.equal(marinRaw.marin.columnMapping[10].status, 'identified-measured-input');
assert.equal(marinRaw.hsva.status, 'preprocessed-cache-only-fallback-staged');

const tool = new URL('../tool.mjs', import.meta.url);
const inventory = spawnSync(process.execPath, [tool.pathname, 'inventory'], { encoding: 'utf8' });
assert.equal(inventory.status, 0, inventory.stderr);
assert.deepEqual(JSON.parse(inventory.stdout).map(({ id }) => id), ['aero4river-v1', 'kvlcc2-simman2008-third-party', 'simman2014', 'wpcc-v4', 'yasukawa-yoshimura-2015-kvlcc2']);

const gated = spawnSync(process.execPath, [tool.pathname, 'lock', 'simman2014'], { encoding: 'utf8' });
assert.notEqual(gated.status, 0);
assert.match(gated.stderr, /acquisition is gated/);

const locked = spawnSync(process.execPath, [tool.pathname, 'verify', 'aero4river-v1'], { encoding: 'utf8' });
assert.equal(locked.status, 0, locked.stderr);
const wpccLocked = spawnSync(process.execPath, [tool.pathname, 'verify', 'wpcc-v4'], { encoding: 'utf8' });
assert.equal(wpccLocked.status, 0, wpccLocked.stderr);
const marinLocked = spawnSync(process.execPath, [tool.pathname, 'verify', 'kvlcc2-simman2008-third-party'], { encoding: 'utf8' });
assert.equal(marinLocked.status, 0, marinLocked.stderr);

console.log('dataset acquisition manifest tests passed');
