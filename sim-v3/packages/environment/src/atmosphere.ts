export interface AtmosphericSample{
  wind_ned_mps:[number,number,number];
  air_temperature_c:number;
  pressure_pa:number;
  humidity_fraction:number;
  rain_rate_mm_h:number;
  visibility_m:number;
  fog_extinction_per_m:number;
  illumination_lux:number;
  gust_ned_mps?:[number,number,number];
  turbulence_intensity?:number;
  air_density_kg_m3?:number;
  cloud_cover_fraction?:number;
  refractivity_n_units?:number;
  spectral_irradiance_w_m2_nm?:ReadonlyArray<{wavelength_nm:number;irradiance_w_m2_nm:number}>;
}
