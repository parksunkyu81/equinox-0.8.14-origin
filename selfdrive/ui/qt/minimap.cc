#include "selfdrive/ui/qt/minimap.h"

#include <QFile>
#include <QHash>
#include <QPainterPath>
#include <QTransform>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <vector>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"

namespace {

const QString MINIMAP_DIR = "/data/media/0/minimap";
constexpr double TILE = 0.01;          // degrees, as in build_minimap_db.py
constexpr double KY = 111320.0;        // metres per degree of latitude
constexpr int LAYER_PX = 1024;         // north-up layer, square
constexpr double PX_PER_M = 0.8;       // 260 m ahead fills the 280 px map
constexpr double RELAYER_M = 200.0;    // re-render after moving this far from its centre
constexpr size_t TILE_CACHE_MAX = 30;
constexpr int MAX_LABELS = 40;         // label anchors kept per layer
constexpr int SHOWN_LABELS = 3;
constexpr double FILES_RECHECK_MS = 30000.;

// OSM class -> width [px] and fill, the same table as the PC samples.
constexpr float ROAD_W[7] = {13, 12, 11, 9, 7, 5, 3};
const QColor ROAD_FILL[7] = {QColor(232, 160, 70), QColor(215, 170, 90), QColor(150, 160, 178),
                             QColor(130, 140, 158), QColor(112, 122, 140), QColor(88, 98, 114),
                             QColor(72, 80, 94)};
const QColor MAP_BG(24, 29, 38);
const QColor BLD_FILL(44, 51, 64);
const QColor ROAD_CASING(40, 46, 58);

struct Road {
  int cls;
  QString name;
  QPolygonF ll;     // lon, lat
  QRectF bbox;      // in lon, lat
};
struct Tile {
  std::vector<Road> roads;
  std::vector<QPolygonF> blds;   // lon, lat
};
struct Label {
  QPointF ll;
  QString name;
};

int64_t tile_key(int tx, int ty) { return (int64_t)tx * 100000 + ty; }

bool inflate_blob(const uint8_t *src, size_t n, std::vector<uint8_t> &out) {
  out.resize(std::max<size_t>(n * 6, 4096));
  while (true) {
    uLongf len = out.size();
    const int r = uncompress(out.data(), &len, src, n);
    if (r == Z_OK) { out.resize(len); return true; }
    if (r != Z_BUF_ERROR || out.size() > (64u << 20)) return false;
    out.resize(out.size() * 2);
  }
}

// One delta-encoded point list (see build_minimap_db.py encode_pts).
bool read_pts(const std::vector<uint8_t> &b, size_t &p, int tx, int ty, QPolygonF &out) {
  if (p + 2 > b.size()) return false;
  uint16_t n; memcpy(&n, &b[p], 2); p += 2;
  if (p + 4u * n > b.size()) return false;
  out.reserve(n);
  int x = 0, y = 0;
  for (int i = 0; i < n; i++) {
    int16_t dx, dy;
    memcpy(&dx, &b[p], 2); memcpy(&dy, &b[p + 2], 2); p += 4;
    x = (x + dx) & 0xffff; y = (y + dy) & 0xffff;
    out << QPointF(tx * TILE + x / 65535.0 * TILE, ty * TILE + y / 65535.0 * TILE);
  }
  return true;
}

std::shared_ptr<Tile> parse_tile(const std::vector<uint8_t> &r, const std::vector<uint8_t> &b, int tx, int ty) {
  auto t = std::make_shared<Tile>();
  if (r.size() >= 8) {
    uint32_t nlen, nroads;
    memcpy(&nlen, &r[0], 4); memcpy(&nroads, &r[4], 4);
    if (8 + (size_t)nlen <= r.size()) {
      const QStringList names = QString::fromUtf8((const char *)&r[8], nlen).split('\n');
      size_t p = 8 + nlen;
      for (uint32_t i = 0; i < nroads && p + 3 <= r.size(); i++) {
        Road rd;
        rd.cls = std::min<int>(r[p], 6);
        uint16_t ni; memcpy(&ni, &r[p + 1], 2); p += 3;
        if (ni < names.size()) rd.name = names[ni];
        if (!read_pts(r, p, tx, ty, rd.ll)) break;
        rd.bbox = rd.ll.boundingRect();
        t->roads.push_back(std::move(rd));
      }
    }
  }
  if (b.size() >= 4) {
    uint32_t nb; memcpy(&nb, &b[0], 4);
    size_t p = 4;
    for (uint32_t i = 0; i < nb; i++) {
      QPolygonF poly;
      if (!read_pts(b, p, tx, ty, poly)) break;
      t->blds.push_back(std::move(poly));
    }
  }
  return t;
}

}  // namespace

struct MiniMap::State {
  // Worker-only (one worker at a time, guarded by busy).
  bool index_loaded = false;
  std::unordered_map<int64_t, std::array<uint32_t, 3>> index;   // off, roads len, buildings len
  FILE *bin = nullptr;
  std::unordered_map<int64_t, std::shared_ptr<Tile>> cache;

  // Shared with the GUI thread.
  std::mutex m;
  QImage layer;
  double layer_lat = 0., layer_lon = 0.;
  bool have_layer = false;
  QVector<Label> labels;
  std::atomic<bool> busy{false};
  std::atomic<bool> broken{false};   // map files unreadable: stop trying

  // GUI-only.
  int files = 0;            // 1 present, -1 missing, 0 not checked
  double files_checked = -1e12;

  ~State() { if (bin) fclose(bin); }

  bool load_index() {
    if (index_loaded) return bin != nullptr;
    index_loaded = true;
    QFile f(MINIMAP_DIR + "/minimap_kr.idx");
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QByteArray data = f.readAll();
    for (int i = 0; i + 16 <= data.size(); i += 16) {
      int16_t tx, ty; uint32_t off, rl, bl;
      memcpy(&tx, data.constData() + i, 2); memcpy(&ty, data.constData() + i + 2, 2);
      memcpy(&off, data.constData() + i + 4, 4); memcpy(&rl, data.constData() + i + 8, 4);
      memcpy(&bl, data.constData() + i + 12, 4);
      index[tile_key(tx + 12000, ty + 3000)] = {off, rl, bl};
    }
    bin = fopen((MINIMAP_DIR + "/minimap_kr.bin").toStdString().c_str(), "rb");
    return bin != nullptr;
  }

  std::shared_ptr<Tile> tile(int tx, int ty) {
    const int64_t k = tile_key(tx, ty);
    auto c = cache.find(k);
    if (c != cache.end()) return c->second;
    auto e = index.find(k);
    std::shared_ptr<Tile> t = std::make_shared<Tile>();
    if (e != index.end()) {
      const auto &[off, rl, bl] = e->second;
      std::vector<uint8_t> raw(rl + bl), r, b;
      if (fseek(bin, off, SEEK_SET) == 0 && fread(raw.data(), 1, raw.size(), bin) == raw.size() &&
          inflate_blob(raw.data(), rl, r) && inflate_blob(raw.data() + rl, bl, b)) {
        t = parse_tile(r, b, tx, ty);
      }
    }
    if (cache.size() >= TILE_CACHE_MAX) cache.clear();
    cache[k] = t;
    return t;
  }
};

namespace {

// Layer pixel of a lon/lat for a layer centred on (lat_c, lon_c).
inline QPointF layer_px(double lon, double lat, double lon_c, double lat_c, double kx) {
  return QPointF((lon - lon_c) * kx * PX_PER_M + LAYER_PX / 2.0,
                 LAYER_PX / 2.0 - (lat - lat_c) * KY * PX_PER_M);
}

void build_layer(std::shared_ptr<MiniMap::State> st, double lat_c, double lon_c) {
  if (!st->load_index()) { st->broken = true; st->busy = false; return; }
  const double kx = KY * std::cos(lat_c * M_PI / 180.0);
  const double half_m = LAYER_PX / 2.0 / PX_PER_M;
  const double dlon = half_m / kx, dlat = half_m / KY;
  const QRectF view(lon_c - dlon, lat_c - dlat, 2 * dlon, 2 * dlat);

  std::vector<std::shared_ptr<Tile>> tiles;
  for (int tx = (int)std::floor(view.left() / TILE); tx <= (int)std::floor(view.right() / TILE); tx++) {
    for (int ty = (int)std::floor(view.top() / TILE); ty <= (int)std::floor(view.bottom() / TILE); ty++) {
      tiles.push_back(st->tile(tx, ty));
    }
  }

  QImage img(LAYER_PX, LAYER_PX, QImage::Format_ARGB32_Premultiplied);
  img.fill(MAP_BG);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  auto to_px = [&](const QPolygonF &ll) {
    QPolygonF out; out.reserve(ll.size());
    for (const QPointF &q : ll) out << layer_px(q.x(), q.y(), lon_c, lat_c, kx);
    return out;
  };
  p.setPen(Qt::NoPen);
  p.setBrush(BLD_FILL);
  for (const auto &t : tiles) {
    for (const QPolygonF &b : t->blds) {
      if (!b.isEmpty() && view.contains(b[0])) p.drawPolygon(to_px(b));
    }
  }
  std::vector<std::pair<const Road *, QPolygonF>> vis;
  for (const auto &t : tiles) {
    for (const Road &r : t->roads) {
      if (r.bbox.intersects(view) || view.contains(r.bbox.topLeft())) vis.emplace_back(&r, to_px(r.ll));
    }
  }
  std::sort(vis.begin(), vis.end(), [](const auto &a, const auto &b) { return a.first->cls > b.first->cls; });
  p.setBrush(Qt::NoBrush);
  for (int pass = 0; pass < 2; pass++) {
    for (const auto &[r, poly] : vis) {
      p.setPen(QPen(pass == 0 ? ROAD_CASING : ROAD_FILL[r->cls], ROAD_W[r->cls] + (pass == 0 ? 3 : 0),
                    Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.drawPolyline(poly);
    }
  }
  p.end();

  // One label anchor per road name (bigger roads first): the middle vertex of
  // its longest piece in view.
  QHash<QString, std::pair<int, const Road *>> best;
  for (const auto &v : vis) {
    const Road *r = v.first;
    if (r->name.isEmpty() || r->cls > 4) continue;
    auto it = best.find(r->name);
    if (it == best.end() || r->ll.size() > it->first) best[r->name] = {r->ll.size(), r};
  }
  std::vector<const Road *> named;
  for (const auto &v : best) named.push_back(v.second);
  std::sort(named.begin(), named.end(), [](const Road *a, const Road *b) { return a->cls < b->cls; });
  QVector<Label> labels;
  for (const Road *r : named) {
    if (labels.size() >= MAX_LABELS) break;
    labels.push_back({r->ll[r->ll.size() / 2], r->name});
  }

  {
    std::lock_guard<std::mutex> lk(st->m);
    st->layer = img;
    st->layer_lat = lat_c;
    st->layer_lon = lon_c;
    st->labels = labels;
    st->have_layer = true;
  }
  st->busy = false;
}

double seg_dist(const QPointF &q, const QPointF &a, const QPointF &b) {
  const QPointF ab = b - a;
  const double l2 = QPointF::dotProduct(ab, ab);
  const double t = l2 > 0 ? std::clamp(QPointF::dotProduct(q - a, ab) / l2, 0.0, 1.0) : 0.0;
  const QPointF d = a + t * ab - q;
  return std::hypot(d.x(), d.y());
}

}  // namespace

MiniMap::MiniMap() : st_(std::make_shared<State>()) {}
MiniMap::~MiniMap() = default;

bool MiniMap::active(const cereal::RoadLimitSpeed::Reader &nda) {
  const double now = millis_since_boot();
  if (st_->files <= 0 && now - st_->files_checked > FILES_RECHECK_MS) {
    st_->files_checked = now;
    st_->files = QFile::exists(MINIMAP_DIR + "/minimap_kr.idx") && QFile::exists(MINIMAP_DIR + "/minimap_kr.bin") ? 1 : -1;
  }
  return st_->files > 0 && !st_->broken && nda.getMiniMapValid() && nda.getRouteLat().size() >= 2 &&
         nda.getRouteLat().size() == nda.getRouteLon().size();
}

void MiniMap::draw(QPainter &p, const QRectF &rect, const cereal::RoadLimitSpeed::Reader &nda) {
  const double lat = nda.getCarLat(), lon = nda.getCarLon();
  const double heading = nda.getCarHeading();

  QImage layer;
  QVector<Label> labels;
  double lat_c = lat, lon_c = lon;
  bool have_layer;
  {
    std::lock_guard<std::mutex> lk(st_->m);
    have_layer = st_->have_layer;
    if (have_layer) {
      layer = st_->layer;           // implicitly shared: no pixel copy
      labels = st_->labels;
      lat_c = st_->layer_lat;
      lon_c = st_->layer_lon;
    }
  }
  const double kx = KY * std::cos(lat_c * M_PI / 180.0);
  const QPointF car_l = layer_px(lon, lat, lon_c, lat_c, kx);
  const double off_m = std::hypot(car_l.x() - LAYER_PX / 2.0, car_l.y() - LAYER_PX / 2.0) / PX_PER_M;
  if (!st_->busy && (!have_layer || off_m > RELAYER_M)) {
    st_->busy = true;
    std::thread(build_layer, st_, lat, lon).detach();
  }

  // Car near the bottom of the map, heading up.
  const QPointF car_s(rect.center().x(), rect.bottom() - 52);
  QTransform T;
  T.translate(car_s.x(), car_s.y());
  T.rotate(-heading);
  T.translate(-car_l.x(), -car_l.y());

  QPainterPath clip;
  clip.addRoundedRect(rect, 22, 22);
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::SmoothPixmapTransform);
  p.setClipPath(clip);
  p.fillRect(rect, MAP_BG);

  p.save();
  p.setTransform(T, true);
  if (have_layer) p.drawImage(QPointF(0, 0), layer);

  // Route: grey behind the car, blue (white casing, soft glow) ahead.
  const auto rlat = nda.getRouteLat(), rlon = nda.getRouteLon();
  QPolygonF route;
  for (size_t i = 0; i < rlat.size(); i++) route << layer_px(rlon[i], rlat[i], lon_c, lat_c, kx);
  int split = 0;
  double best = 1e18;
  for (int i = 0; i + 1 < route.size(); i++) {
    const double d = seg_dist(car_l, route[i], route[i + 1]);
    if (d < best) { best = d; split = i; }
  }
  QPolygonF behind(route.mid(0, split + 1)), ahead;
  behind << car_l;
  ahead << car_l;
  for (int i = split + 1; i < route.size(); i++) ahead << route[i];
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(120, 128, 140), 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolyline(behind);
  p.setPen(QPen(QColor(60, 150, 255, 70), 22, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolyline(ahead);
  p.setPen(QPen(Qt::white, 12, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolyline(ahead);
  p.setPen(QPen(QColor(40, 130, 255), 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolyline(ahead);

  if (nda.getTurnLat() != 0. && nda.getTurnLon() != 0.) {
    const QPointF tp = layer_px(nda.getTurnLon(), nda.getTurnLat(), lon_c, lat_c, kx);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::white);
    p.drawEllipse(tp, 9, 9);
    p.setBrush(QColor(40, 130, 255));
    p.drawEllipse(tp, 6, 6);
  }
  p.restore();

  // Road names, screen-aligned, kept off the route ahead and off each other.
  QPolygonF ahead_s = T.map(ahead);
  configFont(p, "Open Sans", 20, "Bold");
  const QFontMetrics fm(p.font());
  QVector<QRectF> placed;
  for (const Label &lb : labels) {
    if (placed.size() >= SHOWN_LABELS) break;
    const QPointF s = T.map(layer_px(lb.ll.x(), lb.ll.y(), lon_c, lat_c, kx));
    const QRectF tr(s.x() - fm.horizontalAdvance(lb.name) / 2.0 - 4, s.y() - fm.height() / 2.0,
                    fm.horizontalAdvance(lb.name) + 8, fm.height());
    if (!rect.adjusted(6, 6, -6, -6).contains(tr)) continue;
    bool clash = tr.intersects(QRectF(car_s.x() - 32, car_s.y() - 32, 64, 64));   // keep off the car
    for (const QRectF &o : placed) clash = clash || o.intersects(tr);
    for (int i = 0; !clash && i + 1 < ahead_s.size(); i++) clash = seg_dist(s, ahead_s[i], ahead_s[i + 1]) < 16;
    if (clash) continue;
    placed.push_back(tr);
    const QPointF base(tr.left() + 4, tr.top() + fm.ascent());
    p.setPen(QColor(20, 24, 32, 230));
    for (const QPointF &o : {QPointF(-2, 0), QPointF(2, 0), QPointF(0, -2), QPointF(0, 2)}) p.drawText(base + o, lb.name);
    p.setPen(QColor(210, 218, 230));
    p.drawText(base, lb.name);
  }

  // Car arrow.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(255, 110, 50, 70));
  p.drawEllipse(car_s, 24, 24);
  QPolygonF arrow;
  arrow << car_s + QPointF(0, -18) << car_s + QPointF(13, 13) << car_s + QPointF(0, 6) << car_s + QPointF(-13, 13);
  p.setPen(QPen(Qt::white, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(QColor(255, 95, 40));
  p.drawPolygon(arrow);

  p.setClipping(false);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(255, 255, 255, 90), 1.5));
  p.drawRoundedRect(rect, 22, 22);
  p.restore();
}
