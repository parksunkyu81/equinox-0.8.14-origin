#pragma once

// Onroad mini-map drawn inside the turn-info box (design ①, 2026-11-25).
//
// Map data: OSM car roads + buildings for all of Korea, 84 MB of zlib tiles
// (0.01 deg each) built on a PC by tools/minimap/build_minimap_db.py and copied
// to MINIMAP_DIR. Route and car position come from roadLimitSpeed
// (selfdrive/navi_route.py: the Kakao route TmapNda posts, the phone's GPS
// matched to it; the EON has no GPS).
//
// Cost: a north-up layer (LAYER_PX square, about 1.3 km) is rendered on a
// worker thread and only again after the car has moved RELAYER_M from its
// centre; each paint draws that image rotated (GPU) plus the route polyline,
// a dot and an arrow. Measured on the EON: 7-13 ms per layer render with the
// busiest tiles, 9-29 ms per 3x3 tile load, both off the GUI thread.

#include <QImage>
#include <QPainter>
#include <QPolygonF>
#include <QString>
#include <QVector>

#include <atomic>
#include <memory>
#include <mutex>

#include "cereal/gen/cpp/log.capnp.h"

class MiniMap {
public:
  MiniMap();
  ~MiniMap();
  // True when the map files exist and roadLimitSpeed carries a usable position.
  bool active(const cereal::RoadLimitSpeed::Reader &nda);
  void draw(QPainter &p, const QRectF &rect, const cereal::RoadLimitSpeed::Reader &nda);

  struct State;   // shared with the worker thread

private:
  std::shared_ptr<State> st_;
};
