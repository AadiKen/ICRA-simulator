import assert from "node:assert/strict";
import {ConstantBathymetryField,ConstantEnvironmentalField,type BathymetryField} from "../src/index.ts";

const field=new ConstantEnvironmentalField({
  wind_ned_mps:[3,1,0],current_ned_mps:[.2,0,0],air_temperature_c:18,pressure_pa:100800,humidity_fraction:.7,rain_rate_mm_h:4,visibility_m:2500,fog_extinction_per_m:.0002,illumination_lux:800,wave:{amplitude_m:.6,period_s:5,phase_rad:.2,direction_rad:1},tide_m:.4,
  atmosphere:{wind_ned_mps:[3,1,0],air_temperature_c:18,pressure_pa:100800,humidity_fraction:.7,rain_rate_mm_h:4,visibility_m:2500,fog_extinction_per_m:.0002,illumination_lux:800,gust_ned_mps:[1,0,0],turbulence_intensity:.15,air_density_kg_m3:1.21,refractivity_n_units:320},
  water:{current_ned_mps:[.2,0,0],temperature_c:12,salinity_psu:34.5,pressure_pa:150000,sound_speed_profile:[{depth_m:0,sound_speed_mps:1495},{depth_m:50,sound_speed_mps:1488}],conductivity_s_m:4.2,turbidity_ntu:1.5,dissolved_oxygen_mg_l:8.1,ph:8.05},
  surface:{wave:{amplitude_m:.6,period_s:5,phase_rad:.2,direction_rad:1},tide_m:.4,sea_state_beaufort:3,significant_wave_height_m:1.2},
  electromagnetic:{rf_noise_floor_dbm:-104,rf_interference_factor:.1,magnetic_field_ned_t:[.00002,0,.00004],magnetic_declination_rad:.1,magnetic_inclination_rad:.8,magnetic_anomaly_ned_t:[0,.000001,0]},
  positioning:{sky_view_fraction:.8,visible_satellites:9,hdop:1.1,vdop:1.6,pdop:1.9,multipath_factor:.2,gnss_interference_factor:.05}
});
const sample=field.sample({position_ned_m:[4,5,0],time_s:2});
// Compatibility facade remains populated from the authoritative grouped values.
assert.deepEqual(sample.wind_ned_mps,[3,1,0]);assert.deepEqual(sample.current_ned_mps,[.2,0,0]);assert.equal(sample.air_temperature_c,18);assert.equal(sample.wave?.amplitude_m,.6);assert.equal(sample.tide_m,.4);
// One smoke query covers every new environmental group and field family.
assert.deepEqual(sample.atmosphere.gust_ned_mps,[1,0,0]);assert.equal(sample.atmosphere.turbulence_intensity,.15);assert.equal(sample.atmosphere.air_density_kg_m3,1.21);
assert.equal(sample.water.temperature_c,12);assert.equal(sample.water.salinity_psu,34.5);assert.equal(sample.water.pressure_pa,150000);assert.equal(sample.water.sound_speed_profile?.[1].sound_speed_mps,1488);assert.equal(sample.water.conductivity_s_m,4.2);assert.equal(sample.water.turbidity_ntu,1.5);assert.equal(sample.water.dissolved_oxygen_mg_l,8.1);assert.equal(sample.water.ph,8.05);
assert.equal(sample.surface.sea_state_beaufort,3);assert.equal(sample.surface.significant_wave_height_m,1.2);
assert.deepEqual(sample.electromagnetic.magnetic_field_ned_t,[.00002,0,.00004]);assert.equal(sample.electromagnetic.rf_noise_floor_dbm,-104);
assert.equal(sample.positioning.visible_satellites,9);assert.equal(sample.positioning.multipath_factor,.2);

const constant=new ConstantBathymetryField(42);assert.equal(constant.sample({position_ned_m:[0,0,0],time_s:0}).water_depth_m,42);
const spatial:BathymetryField={sample:({position_ned_m})=>({water_depth_m:20+position_ned_m[0]}),provenance:()=>({mode:"test-spatial"})};assert.equal(spatial.sample({position_ned_m:[7,0,0],time_s:0}).water_depth_m,27);
console.log("Grouped environment smoke tests passed.");
await import("./geography.test.ts");
await import("./weather.test.ts");
