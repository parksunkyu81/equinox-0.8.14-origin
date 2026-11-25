"""Build a compact mini-map database from an OSM extract and report its size.

Output: one zlib blob per ~1 km tile (0.01 deg lon x 0.01 deg lat), holding
  - car roads (motorway .. living_street, plus service): class, name, points
  - buildings: outline points
Coordinates are uint16 fractions of the tile, delta-encoded as int16.

usage (on a PC; needs `pip install osmium`, about 2 minutes for Korea):
  curl -LO https://download.geofabrik.de/asia/south-korea-latest.osm.pbf
  python build_minimap_db.py south-korea-latest.osm.pbf out_dir
  scp -P 8022 out_dir/minimap_kr.idx out_dir/minimap_kr.bin root@<eon>:/data/media/0/minimap/

Read by selfdrive/ui/qt/minimap.cc. 2026-11-24 extract: 1.22M roads,
1.39M buildings, 81k tiles, 84 MB.
"""
import os
import struct
import sys
import time
import zlib
from collections import defaultdict

import osmium

ROAD_CLASS = {
  'motorway': 0, 'motorway_link': 0, 'trunk': 1, 'trunk_link': 1,
  'primary': 2, 'primary_link': 2, 'secondary': 3, 'secondary_link': 3,
  'tertiary': 4, 'tertiary_link': 4, 'unclassified': 5, 'residential': 5,
  'living_street': 5, 'service': 6,
}
TILE = 0.01  # degrees


def tile_of(lon, lat):
  return int(lon // TILE), int(lat // TILE)


def q(v, base):
  return max(0, min(65535, int(round((v - base) / TILE * 65535))))


class H(osmium.SimpleHandler):
  def __init__(self):
    super().__init__()
    self.roads = defaultdict(list)      # tile -> [(cls, name, [(x, y)])]
    self.buildings = defaultdict(list)  # tile -> [[(x, y)]]
    self.n_road = self.n_bld = self.road_pts = self.bld_pts = 0

  def way(self, w):
    tags = w.tags
    hw = tags.get('highway')
    is_bld = 'building' in tags
    if hw not in ROAD_CLASS and not is_bld:
      return
    try:
      pts = [(n.location.lon, n.location.lat) for n in w.nodes]
    except osmium.InvalidLocationError:
      return
    if len(pts) < 2:
      return
    if hw in ROAD_CLASS:
      self.n_road += 1
      name = tags.get('name', '')
      # split a road at tile borders so each tile holds its own pieces
      cur_tile, cur = tile_of(*pts[0]), [pts[0]]
      for p in pts[1:]:
        t = tile_of(*p)
        cur.append(p)
        if t != cur_tile:
          self.roads[cur_tile].append((ROAD_CLASS[hw], name, cur))
          cur_tile, cur = t, [p]
      if len(cur) >= 2:
        self.roads[cur_tile].append((ROAD_CLASS[hw], name, cur))
      self.road_pts += len(pts)
    else:
      self.n_bld += 1
      lon = sum(p[0] for p in pts) / len(pts)
      lat = sum(p[1] for p in pts) / len(pts)
      self.buildings[tile_of(lon, lat)].append(pts)
      self.bld_pts += len(pts)


def encode_pts(pts, tx, ty):
  base_lon, base_lat = tx * TILE, ty * TILE
  out = bytearray()
  px = py = 0
  prev = None
  n = 0
  body = bytearray()
  for lon, lat in pts:
    x, y = q(lon, base_lon), q(lat, base_lat)
    if (x, y) == prev:
      continue
    body += struct.pack('<hh', (x - px + 32768) % 65536 - 32768, (y - py + 32768) % 65536 - 32768)
    px, py, prev = x, y, (x, y)
    n += 1
  out += struct.pack('<H', n) + body
  return out


def main():
  src, out_dir = sys.argv[1], sys.argv[2]
  os.makedirs(out_dir, exist_ok=True)
  t = time.time()
  h = H()
  h.apply_file(src, locations=True, idx='flex_mem')
  print(f'read {time.time() - t:.0f} s: roads {h.n_road:,} ({h.road_pts:,} pts), buildings {h.n_bld:,} ({h.bld_pts:,} pts)')

  tiles = set(h.roads) | set(h.buildings)
  sizes = {'roads_raw': 0, 'roads_z': 0, 'bld_raw': 0, 'bld_z': 0}
  index = bytearray()
  with open(os.path.join(out_dir, 'minimap_kr.bin'), 'wb') as f:
    for (tx, ty) in sorted(tiles):
      names = []
      name_idx = {}
      rb = bytearray()
      for cls, name, pts in h.roads.get((tx, ty), []):
        if name not in name_idx:
          name_idx[name] = len(names)
          names.append(name)
        rb += struct.pack('<BH', cls, name_idx[name]) + encode_pts(pts, tx, ty)
      nb = '\n'.join(names).encode('utf-8')
      roads_blob = struct.pack('<II', len(nb), len(h.roads.get((tx, ty), []))) + nb + rb
      bb = bytearray(struct.pack('<I', len(h.buildings.get((tx, ty), []))))
      for pts in h.buildings.get((tx, ty), []):
        bb += encode_pts(pts, tx, ty)
      rz, bz = zlib.compress(bytes(roads_blob), 9), zlib.compress(bytes(bb), 9)
      sizes['roads_raw'] += len(roads_blob); sizes['roads_z'] += len(rz)
      sizes['bld_raw'] += len(bb); sizes['bld_z'] += len(bz)
      off = f.tell()
      f.write(rz); f.write(bz)
      index += struct.pack('<hhIII', tx - 12000, ty - 3000, off, len(rz), len(bz))
  with open(os.path.join(out_dir, 'minimap_kr.idx'), 'wb') as f:
    f.write(index)
  mb = lambda b: b / 1024 / 1024
  print(f'tiles {len(tiles):,}  index {mb(len(index)):.1f} MB')
  print(f'roads:     raw {mb(sizes["roads_raw"]):7.1f} MB  zlib {mb(sizes["roads_z"]):7.1f} MB')
  print(f'buildings: raw {mb(sizes["bld_raw"]):7.1f} MB  zlib {mb(sizes["bld_z"]):7.1f} MB')
  print(f'total zlib {mb(sizes["roads_z"] + sizes["bld_z"] + len(index)):.1f} MB, done in {time.time() - t:.0f} s')
  # busiest tiles, for the EON bench
  busy = sorted(tiles, key=lambda k: len(h.roads.get(k, [])) + len(h.buildings.get(k, [])), reverse=True)[:5]
  for k in busy:
    print('busy tile', k, 'roads', len(h.roads.get(k, [])), 'buildings', len(h.buildings.get(k, [])))


if __name__ == '__main__':
  main()
