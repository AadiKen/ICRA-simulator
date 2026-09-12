| Measure | BCOD | VRX 3.1.2 |
|---|---:|---:|
| Scenario-specific files | 1 | 1 |
| Configuration lines | 31 | 609 world + 95 launch |
| External assets | 0 | 10 |
| Setup commands | 1 | 2 container build stages |
| Measured setup | Not separately timed | 277.6s base + 99s builder |
| Runtime cross-check | Production scenario resolved | WAM-V spawned; sensors/scoring active (<120s bound) |

*Claim limit: This executed comparison measures configuration surface and local setup/runtime overhead. It does not validate BCOD physics, and without a controlled human study it does not establish scenario-authoring speed.*
