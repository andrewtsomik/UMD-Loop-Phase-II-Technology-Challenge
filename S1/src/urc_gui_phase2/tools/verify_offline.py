#!/usr/bin/env python3
"""Demonstrate that the map works with no network. For judges and for you.

For a real demonstration, cut the network first, then run this:

  * in the VM: disable the network adapter (or `nmcli networking off`), or
  * per-process, as root: `sudo unshare -n sudo -u "$USER" env DISPLAY="$DISPLAY"
    XAUTHORITY="$XAUTHORITY" python3 tools/verify_offline.py --show`
    (a private network namespace with no usable interface). Plain
    `unshare -rn` without sudo is blocked on some Ubuntu releases.

    python3 tools/verify_offline.py            # headless self-check
    python3 tools/verify_offline.py --show     # visible window, stays open

Step [1] below tells you whether the network really was down.

It reports, in order:
  1. whether the network is really unreachable from this process,
  2. how many tiles Chromium loaded, and that every tile URL is file://,
  3. every request the browser engine tried to make to a non-local URL
     (must be none),
and exits 0 only if the map rendered from disk with zero blocked requests.
"""

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Belt and braces on top of the interceptor: make Chromium's DNS resolver fail
# for every hostname, so even a request that slipped past would not resolve.
os.environ.setdefault('QTWEBENGINE_CHROMIUM_FLAGS',
                      '--no-sandbox --host-resolver-rules="MAP * ~NOTFOUND"')

_HEADLESS = '--show' not in sys.argv
if _HEADLESS:
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtTest import QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from urc_gui_phase2.map_widget import MDRS_CENTER, OfflineMapWidget  # noqa: E402
from urc_gui_phase2.mission_model import MissionModel  # noqa: E402


def network_reachable() -> bool:
    for host in (('1.1.1.1', 53), ('8.8.8.8', 53)):
        try:
            with socket.create_connection(host, timeout=2):
                return True
        except OSError:
            continue
    return False


def wait_for(predicate, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        QTest.qWait(50)
    return predicate()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--tile-dir', default=None)
    parser.add_argument('--show', action='store_true', help='show the window and keep it open')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()

    reachable = network_reachable()
    print(f'[1] Network reachable from this process: {"YES (not a valid offline test!)" if reachable else "no"}')

    app = QApplication(sys.argv)
    widget = OfflineMapWidget(args.tile_dir)
    widget.resize(1000, 700)
    widget.show()

    model = MissionModel('verify')
    model.add('Demo A', MDRS_CENTER[0] + 0.0010, MDRS_CENTER[1] - 0.0010)
    model.add('Demo B', MDRS_CENTER[0] - 0.0008, MDRS_CENTER[1] + 0.0012)
    widget.set_waypoints(model.waypoints)
    widget.set_rover_position(*MDRS_CENTER)

    def js(script):
        box = []
        widget._map.getJsresponseForMap(script, box.append)
        wait_for(lambda: box, 5)
        return box[0] if box else None

    loaded = lambda: js('document.querySelectorAll("img.leaflet-tile-loaded").length') or 0  # noqa: E731
    ok_tiles = wait_for(lambda: loaded() > 0, args.timeout)
    QTest.qWait(1000)  # let any stray request happen
    urls = js('Array.from(document.querySelectorAll("img.leaflet-tile")).map(i => i.src)') or []
    non_file = [u for u in urls if not u.startswith('file:///') and not u.startswith('data:')]
    blocked = widget.blocked_requests

    print(f'[2] Tiles loaded in the browser: {loaded()}  (all file:// : {not non_file}; e.g. {urls[0] if urls else "-"})')
    print(f'    Local requests served: {widget._interceptor.allowed_count}')
    print(f'[3] Non-local requests blocked by the interceptor: {len(blocked)}')
    for url in blocked:
        print(f'      BLOCKED {url}')
    print(f'    Banner: {widget._banner.text()}')

    passed = ok_tiles and not non_file and not blocked
    if passed and reachable:
        print('RESULT: PASS, but the network was UP, so this shows only that the map used '
              'file:// tiles and made no non-local requests. Repeat with the network cut.')
    else:
        print('RESULT:', 'PASS - map rendered from local tiles with the network down, '
              'zero network requests' if passed else 'FAIL')
    if args.show:
        print('Window is open; close it to exit.')
        app.exec_()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
