# Cache-only acquisition

Obtain authorized access to the exact SIMMAN 2008/MARIN/HSVA material, or place
the supplied third-party handoff under
`validation/datasets/raw/kvlcc2-simman2008-third-party/`. The directory is
gitignored. It must contain `SOURCE_PROVENANCE.md`, `kvlcc2-marin/`, and
`kvlcc2-hsva/`, but not the unrelated `wpcc-openwater/` directory.

Run `npm run verify:kvlcc2-marin` before import. If the cache is absent, the
command fails with acquisition instructions. These files must never enter the
anonymous release because reuse rights are unconfirmed.
