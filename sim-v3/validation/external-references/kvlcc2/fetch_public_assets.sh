#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$(cd "$(dirname "$0")" && pwd)}"
CACHE="$ROOT/cache"
KVLCC2="$CACHE/kvlcc2"
RELATED="$CACHE/related-work"
mkdir -p "$KVLCC2" "$RELATED"

fetch() {
  local url="$1" dest="$2"
  if [[ -s "$dest" ]]; then printf 'exists  %s\n' "$dest"; return; fi
  printf 'fetch   %s\n' "$dest"
  curl --fail --location --retry 4 --retry-delay 2 --connect-timeout 30 --output "$dest.part" "$url"
  mv "$dest.part" "$dest"
}

fetch_optional() {
  local url="$1" dest="$2" label="$3"
  if [[ -s "$dest" ]]; then printf 'exists  %s\n' "$dest"; return; fi
  printf 'fetch   %s\n' "$dest"
  if ! curl --fail --location --retry 1 --connect-timeout 30 --output "$dest.part" "$url"; then
    rm -f "$dest.part"
    printf 'WARN    %s is currently unavailable upstream; not a KVLCC2 comparison dependency.\n' "$label" >&2
  else
    mv "$dest.part" "$dest"
  fi
}

fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-Hull.zip' "$KVLCC2/KVLCC2-Hull.zip"
fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-Rudder-please-note-the-drawing-of-the-rudder-is-in-scale-1-58.00.zip' "$KVLCC2/KVLCC2-Rudder_scale_1-58.zip"
fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-Propeller-NMRI.zip' "$KVLCC2/KVLCC2-Propeller-NMRI.zip"
fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-propeller-HMRI.zip' "$KVLCC2/KVLCC2-Propeller-HMRI.zip"
fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-Propeller-Openwater-Data-NMRI-Model.zip' "$KVLCC2/KVLCC2-Propeller-Openwater-NMRI.zip"
fetch 'https://simman2014.dk/wp-content/uploads/2015/07/KVLCC2-Propeller-Openwater-Data-HMRI.pdf' "$KVLCC2/KVLCC2_Propeller_Openwater_HMRI.pdf"
fetch 'https://link.springer.com/content/pdf/10.1007/s00773-014-0293-y.pdf' "$KVLCC2/Yasukawa_Yoshimura_2015_MMG.pdf"
fetch_optional 'https://ittc.info/media/12240/75-02-06-06.pdf' "$KVLCC2/ITTC_7.5-02-06-06_Rev01_2024.pdf" 'ITTC 7.5-02-06-06 PDF'
fetch 'https://arxiv.org/pdf/2503.09203' "$RELATED/MarineGym_2503.09203.pdf"
fetch 'https://arxiv.org/pdf/2607.03072' "$RELATED/LOTUSim_2607.03072.pdf"
fetch 'https://arxiv.org/pdf/2504.06245' "$RELATED/Marine_Simulators_Review_2504.06245.pdf"
fetch 'https://www.jstage.jst.go.jp/article/jjasnaoe1968/1978/143/1978_143_113/_pdf/-char/en' "$RELATED/Ikeda_Himeno_Tanaka_1978_Roll_Damping.pdf"

printf '%s\n' 'SKIP    KVLCC2-Propeller-INSEAN.zipc: malformed upstream URL returns 404; NMRI and HMRI cover this asset.'

for archive in "$KVLCC2"/*.zip; do unzip -tq "$archive" >/dev/null || { printf 'Invalid ZIP (possibly HTML): %s\n' "$archive" >&2; exit 1; }; done
HULL_DIR="$KVLCC2/hull-iges"
mkdir -p "$HULL_DIR"
unzip -oq "$KVLCC2/KVLCC2-Hull.zip" -d "$HULL_DIR"
IGES_COUNT="$(find "$HULL_DIR" -type f \( -iname '*.igs' -o -iname '*.iges' \) | wc -l | tr -d ' ')"
if [[ "$IGES_COUNT" -lt 1 ]]; then printf 'KVLCC2-Hull.zip contains no IGES file.\n' >&2; exit 1; fi
printf 'verified hull archive: %s IGES file(s)\n' "$IGES_COUNT"

if [[ -f "$ROOT/SHA256SUMS.txt" ]]; then
  (cd "$ROOT" && shasum -a 256 -c SHA256SUMS.txt)
else
  printf 'Missing SHA256SUMS.txt\n' >&2; exit 1
fi

printf 'Public assets fetched and verified. Cache remains untracked.\n'
