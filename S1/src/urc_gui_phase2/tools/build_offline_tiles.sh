#!/usr/bin/env bash
# One-time setup: build the offline MDRS map tiles locally from OpenStreetMap data.
# No tile servers are contacted. Needs internet once (Overpass API, ~13 MB) and apt packages.
#
#   sudo apt-get install -y osmium-tool python3-mapnik fonts-dejavu-core   # once
#   tools/build_offline_tiles.sh                                          # ~1-2 min
#
# Steps: Overpass bbox query -> osmium export (GeoJSON) -> tools/render_tiles.py (Mapnik)
#        -> tools/tile_downloader.py --verify.
# Intermediate files go to build/ (git-ignored); tiles go to tiles/mdrs/.
# Data (c) OpenStreetMap contributors, ODbL: see docs/OFFLINE_MAP.md for attribution rules.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$ROOT/build"
OUT="${OUT:-$ROOT/tiles/mdrs}"
# South West North East: the tile plan's z12 tile extent around MDRS plus margin, so features
# crossing the edge of the mapped area are complete.
BBOX="${BBOX:-38.32,-110.87,38.50,-110.67}"
OVERPASS="${OVERPASS:-https://overpass-api.de/api/interpreter}"
UA="urc-gui-phase2-offline-map/1.0 (one-time educational project; +https://github.com/andrewtsomik/UMD-Loop-Phase-II-Technology-Challenge)"

for cmd in osmium curl python3; do
    command -v "$cmd" >/dev/null || { echo "missing $cmd; see header for the apt command" >&2; exit 1; }
done
python3 -c 'import mapnik' 2>/dev/null || { echo "missing python3-mapnik" >&2; exit 1; }

mkdir -p "$BUILD"
if [[ ! -s "$BUILD/mdrs.osm" || "${REFETCH:-0}" == 1 ]]; then
    echo "Fetching OSM data for bbox $BBOX from Overpass (one request, retried politely)..."
    query="[out:xml][timeout:90];(nwr($BBOX););(._;>;);out meta;"
    ok=0
    for attempt in 1 2 3 4; do
        code=$(curl -s -m 150 -A "$UA" --data-urlencode "data=$query" "$OVERPASS" \
               -o "$BUILD/mdrs.osm.part" -w '%{http_code}')
        # A busy Overpass answers 429/504 with an HTML page; only accept real OSM XML.
        if [[ "$code" == 200 ]] && head -c 300 "$BUILD/mdrs.osm.part" | grep -q '<osm '; then
            mv "$BUILD/mdrs.osm.part" "$BUILD/mdrs.osm"; ok=1; break
        fi
        echo "  attempt $attempt: HTTP $code, waiting $((20 * attempt)) s"
        sleep $((20 * attempt))
    done
    [[ $ok == 1 ]] || { echo "Overpass query failed; try again later or set OVERPASS=<mirror>" >&2; exit 1; }
fi

echo "Converting to GeoJSON..."
osmium export "$BUILD/mdrs.osm" -f geojson -o "$BUILD/mdrs.geojson" --overwrite

echo "Rendering tiles into $OUT ..."
python3 "$ROOT/tools/render_tiles.py" --geojson "$BUILD/mdrs.geojson" --out "$OUT"

echo "Verifying..."
python3 "$ROOT/tools/tile_downloader.py" --verify --out "$OUT"
