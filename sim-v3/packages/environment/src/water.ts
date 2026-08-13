export interface SoundSpeedPoint{depth_m:number;sound_speed_mps:number}
export interface AquaticSample{
  current_ned_mps:[number,number,number];
  temperature_c?:number;
  salinity_psu?:number;
  pressure_pa?:number;
  sound_speed_profile?:SoundSpeedPoint[];
  conductivity_s_m?:number;
  turbidity_ntu?:number;
  dissolved_oxygen_mg_l?:number;
  ph?:number;
  optical_attenuation_per_m?:number;
}
