export interface RegularWaveSample{amplitude_m:number;period_s:number;phase_rad:number;direction_rad:number}
export interface SurfaceSample{
  wave?:RegularWaveSample;
  tide_m?:number;
  sea_state_beaufort?:number;
  significant_wave_height_m?:number;
  dominant_wave_period_s?:number;
  mean_wave_direction_rad?:number;
}
