# KVLCC2 free-running model-test provenance

- Primary origin: SIMMAN 2008 workshop; free-running tests conducted by MARIN
  and HSVA on KVLCC2.
- Obtained via third-party redistribution in
  `martinlarsalbert/System-identification-of-Vessel-Manoeuvring-Models`, associated
  with Alexandersson et al., *System identification of Vessel Manoeuvring
  Models*, Ocean Engineering 266 (2022) 112905.
- This was **not obtained through direct SIMMAN access**.
- The source repository has no LICENSE. The cache is therefore not tracked,
  redistributed, or included in the anonymous release.
- The supplied handoff contained 28 in-scope files: 11 MARIN files, 16 HSVA
  files, and this source provenance. Its additional `wpcc-openwater` file was
  unrelated to this acquisition and was deliberately excluded.

Known physical-model differences are retained, not reconciled by overwriting:
MARIN uses scale 45.7, propeller diameter 0.204 m, and `x_G=0.2436 m`; the
Yasukawa/HMRI-oriented `kvlcc2-l7` configuration records 0.216 m and 0.25 m.
