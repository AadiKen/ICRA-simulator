export interface ElectromagneticSample{
  rf_noise_floor_dbm?:number;
  rf_interference_factor?:number;
  magnetic_field_ned_t?:[number,number,number];
  magnetic_declination_rad?:number;
  magnetic_inclination_rad?:number;
  magnetic_anomaly_ned_t?:[number,number,number];
}
