/** Shared Surveyor wind estimate used by Node and serialized into VRX. */
export const SURVEYOR_WIND_ESTIMATE=Object.freeze({
  classification:"documented geometry-derived engineering estimate; not manufacturer or WAM-V data",
  exposed_height_m:0.42,
  exposed_height_reason:"0.17 m pontoon freeboard plus an estimated 0.25 m low-profile deck payload/superstructure",
  frontal_area_m2:0.3822,
  side_area_m2:0.7686,
  drag_coefficient:1.1,
  drag_coefficient_basis:"standard engineering approximation for a boxy/bluff small-craft superstructure",
  air_density_kg_m3:1.225,
  length_m:1.83,
  vrx_coeff_vector:[0.25754575,0.51783225,0],
  yaw_reason:"The centered, fore-aft symmetric profile has zero first-order aerodynamic center offset; no yaw coefficient is invented."
});

export function surveyorNodeWindConfig(){
  const e=SURVEYOR_WIND_ESTIMATE;
  return{enabled:true,rhoAir:e.air_density_kg_m3,frontalArea:e.frontal_area_m2,lateralArea:e.side_area_m2,length:e.length_m,C_X:(angle)=>e.drag_coefficient*Math.cos(angle),C_Y:(angle)=>e.drag_coefficient*Math.sin(angle),C_N:0};
}
