import assert from "node:assert/strict";
import {generate} from "./imu-sea-state-sweep.ts";

const first=generate(),second=generate();
assert.deepEqual(first,second,"sweep must be deterministic");
assert.equal(first.cells.length,7);
assert.deepEqual(first.cells.map(cell=>cell.beaufort),[0,2,4,6,8,10,12]);
assert.equal(first.cells[0].vibration_rms_mps2,0);
for(let i=1;i<first.cells.length;i++){
  assert.ok(first.cells[i].vibration_rms_mps2>first.cells[i-1].vibration_rms_mps2);
  assert.ok(first.cells[i].declared_accel_noise_std_mps2>first.cells[i-1].declared_accel_noise_std_mps2);
  assert.ok(first.cells[i].declared_gyro_noise_std_rad_s>first.cells[i-1].declared_gyro_noise_std_rad_s);
}
assert.match(first.claim_limit,/not physical vessel motion/);
console.log("Deterministic IMU sea-state sweep passed.");
