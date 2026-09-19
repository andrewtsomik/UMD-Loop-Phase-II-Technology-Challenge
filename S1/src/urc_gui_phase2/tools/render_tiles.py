#!/usr/bin/env python3
"""Render offline XYZ map tiles from an OpenStreetMap GeoJSON export, locally, with Mapnik.

Normally driven by tools/build_offline_tiles.sh; can be run on its own:

    python3 tools/render_tiles.py --geojson build/mdrs.geojson --dry-run
    python3 tools/render_tiles.py --geojson build/mdrs.geojson

Output is the same layout the downloader produced, <out>/{z}/{x}/{y}.png plus metadata.json,
so OfflineMapWidget needs no changes. The style is a small hand-written one (roads, tracks,
water, buildings, land use, cliffs, a few labels), not openstreetmap-carto; it deliberately
avoids PostGIS. Labels are placed per tile, so a label can differ slightly across a tile edge.

Map data (c) OpenStreetMap contributors, ODbL 1.0 (https://www.openstreetmap.org/copyright).
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from xml.sax.saxutils import escape

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tile_downloader as td  # noqa: E402  (plan_tiles, write_atomic, default_out_dir, MDRS_*)
from urc_gui_phase2.map_geometry import bbox_around, tile_bounds  # noqa: E402

ATTRIBUTION = '&copy; OpenStreetMap contributors (ODbL)'
FONT = 'DejaVu Sans Book'
FONT_BOLD = 'DejaVu Sans Bold'
BUFFER = 128  # px of neighbouring data rendered around each tile so lines/labels join up
EARTH_R = 6378137.0

# (line colour, casing colour, width in px at zoom 15) by highway class
ROADS = [
    (['motorway', 'trunk'], '#e892a2', '#dc2a67', 5.0),
    (['primary', 'primary_link'], '#fcd6a4', '#a06b00', 4.5),
    (['secondary', 'secondary_link'], '#f7fabf', '#707d05', 4.0),
    (['tertiary', 'tertiary_link', 'unclassified'], '#ffffff', '#8f8f8f', 3.5),
    (['residential', 'living_street', 'service'], '#ffffff', '#999999', 2.5),
    (['track'], '#996600', None, 1.4),
    (['path', 'footway', 'bridleway', 'cycleway'], '#bb4444', None, 1.0),
]


def _mercator(lon, lat):
    import math
    return (math.radians(lon) * EARTH_R,
            math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) * EARTH_R)


def _ds(path):
    return (f'<Datasource><Parameter name="type">geojson</Parameter>'
            f'<Parameter name="file">{escape(path)}</Parameter></Datasource>')


def _in(key, values):
    return '(' + ' or '.join(f"[{key}] = '{v}'" for v in values) + ')'


def style_xml(geojson: str, zoom: int) -> str:
    """Mapnik XML for one zoom level. Widths scale with zoom (x0.5 per zoom step down, min 0.5 px)."""
    k = max(0.35, 2 ** ((zoom - 15) * 0.6))
    ds = _ds(geojson)
    wgs = '+proj=longlat +datum=WGS84 +no_defs'

    def layer(name):
        return f'<Layer name="{name}" srs="{wgs}">{ds}<StyleName>{name}</StyleName></Layer>'

    def style(name, rules):
        return f'<Style name="{name}">{rules}</Style>'

    def rule(filt, *symbolizers):
        return f'<Rule><Filter>{filt}</Filter>{"".join(symbolizers)}</Rule>'

    def poly(colour):
        return f'<PolygonSymbolizer fill="{colour}" />'

    def line(colour, width, extra=''):
        return (f'<LineSymbolizer stroke="{colour}" stroke-width="{max(0.4, width * k):.2f}" '
                f'stroke-linejoin="round" stroke-linecap="round" {extra}/>')

    styles, layers = [], []

    def add(name, rules):
        styles.append(style(name, rules))
        layers.append(layer(name))

    add('landuse', ''.join([
        rule("[landuse] = 'farmland' or [landuse] = 'meadow' or [landuse] = 'grass'", poly('#dfe8c8')),
        rule("[landuse] = 'residential'", poly('#e0dfdf')),
        rule("[landuse] = 'industrial' or [landuse] = 'quarry'", poly('#ebdbe8')),
        rule("[natural] = 'bare_rock' or [natural] = 'scree'", poly('#e6ddd0')),
        rule("[natural] = 'scrub' or [natural] = 'wood'", poly('#dde8cf')),
        rule("[amenity] = 'parking'", poly('#eeeeee')),
    ]))
    add('water', rule("[natural] = 'water' or [waterway] = 'riverbank'", poly('#aad3df')))
    add('waterways', ''.join([
        rule(_in('waterway', ['river', 'canal']), line('#aad3df', 3.0)),
        rule(_in('waterway', ['stream', 'ditch', 'drain']), line('#aad3df', 1.4)),
    ]))
    add('cliffs', rule("[natural] = 'cliff'", line('#a0a0a0', 1.0, 'stroke-dasharray="3,2" ')))
    if zoom >= 14:
        add('buildings', rule("[building] != ''", poly('#d9d0c9'),
                              line('#bcaea1', 0.6)))
    for i, (classes, colour, casing, width) in enumerate(ROADS):
        filt = _in('highway', classes)
        if casing and zoom >= 13:
            add(f'road_case_{i}', rule(filt, line(casing, width + 1.6)))
        add(f'road_{i}', rule(filt, line(colour, width)))
    if zoom >= 15:
        add('road_names', rule(
            "[highway] != '' and [name] != ''",
            f'<TextSymbolizer face-name="{FONT}" size="{9 if zoom < 17 else 11}" fill="#333333" '
            f'halo-radius="1.5" halo-fill="#ffffff" placement="line" spacing="150" '
            f'minimum-distance="40">[name]</TextSymbolizer>'))
    if zoom >= 12:
        add('places', rule(
            "[place] != '' and [name] != ''",
            f'<TextSymbolizer face-name="{FONT_BOLD}" size="{10 + (zoom >= 14) * 2}" fill="#444444" '
            f'halo-radius="2" halo-fill="#ffffff" wrap-width="80">[name]</TextSymbolizer>'))
    if zoom >= 16:
        add('building_names', rule(
            "[building] != '' and [name] != ''",
            f'<TextSymbolizer face-name="{FONT}" size="9" fill="#555555" halo-radius="1.5" '
            f'halo-fill="#ffffff" wrap-width="60">[name]</TextSymbolizer>'))

    # Layers must be listed bottom-to-top; the order above already is.
    return (f'<Map srs="+proj=merc +a={EARTH_R} +b={EARTH_R} +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 '
            f'+k=1 +units=m +nadgrids=@null +wktext +no_defs" background-color="#f2efe9" '
            f'buffer-size="{BUFFER}">' + ''.join(styles) + ''.join(layers) + '</Map>')


def render_tile(mp, z, x, y) -> bytes:
    import mapnik
    south, west, north, east = tile_bounds(x, y, z)
    (x0, y0), (x1, y1) = _mercator(west, south), _mercator(east, north)
    mp.resize(256, 256)
    mp.zoom_to_box(mapnik.Box2d(x0, y0, x1, y1))
    image = mapnik.Image(256, 256)
    mapnik.render(mp, image)
    return image.tostring('png256')  # 8-bit palette PNG, like the OSM tiles: small files


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--geojson', required=True, help='GeoJSON from `osmium export`')
    p.add_argument('--lat', type=float, default=td.MDRS_LAT)
    p.add_argument('--lon', type=float, default=td.MDRS_LON)
    p.add_argument('--radius-km', type=float, default=2.5)
    p.add_argument('--min-zoom', type=int, default=12)
    p.add_argument('--max-zoom', type=int, default=17)
    p.add_argument('--out', default=td.default_out_dir())
    p.add_argument('--source', default='OpenStreetMap data via Overpass API (see docs/OFFLINE_MAP.md)')
    p.add_argument('--dry-run', action='store_true', help='print the plan and exit')
    args = p.parse_args(argv)
    if not (0 <= args.min_zoom <= args.max_zoom <= 19):
        p.error('need 0 <= min-zoom <= max-zoom <= 19')
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    bbox = bbox_around(args.lat, args.lon, args.radius_km * 1000.0)
    plan = td.plan_tiles(bbox, args.min_zoom, args.max_zoom)
    print(f'{len(plan)} tiles, zoom {args.min_zoom}-{args.max_zoom}, bbox '
          f'S{bbox[0]:.5f} W{bbox[1]:.5f} N{bbox[2]:.5f} E{bbox[3]:.5f}')
    if args.dry_run:
        return 0
    if not os.path.isfile(args.geojson):
        print(f'ERROR: {args.geojson} not found (run tools/build_offline_tiles.sh)', file=sys.stderr)
        return 1
    try:
        import mapnik
    except ImportError:
        print('ERROR: python3-mapnik missing: sudo apt-get install python3-mapnik fonts-dejavu-core',
              file=sys.stderr)
        return 1

    geojson = os.path.abspath(args.geojson)
    maps = {}
    for z in range(args.min_zoom, args.max_zoom + 1):
        mp = mapnik.Map(256, 256)
        mapnik.load_map_from_string(mp, style_xml(geojson, z))
        maps[z] = mp
    for i, (z, x, y) in enumerate(plan, 1):
        td.write_atomic(os.path.join(args.out, str(z), str(x), f'{y}.png'), render_tile(maps[z], z, x, y))
        if i % 100 == 0 or i == len(plan):
            print(f'  {i}/{len(plan)} tiles rendered')

    metadata = {
        'name': 'MDRS',
        'center': [args.lat, args.lon],
        'bounds': list(bbox),
        'min_zoom': args.min_zoom,
        'max_zoom': args.max_zoom,
        'tile_count': len(plan),
        'attribution': ATTRIBUTION,
        'source': args.source,
        'renderer': f'mapnik {mapnik.mapnik_version_string()} (tools/render_tiles.py)',
        'rendered_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    }
    with open(os.path.join(args.out, 'metadata.json'), 'w', encoding='utf-8') as handle:
        json.dump(metadata, handle, indent=2)
        handle.write('\n')
    print(f'Done. Tiles in {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
