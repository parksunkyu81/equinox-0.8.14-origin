#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDateTime>
#include <QDebug>
#include <QGlyphRun>
#include <QTextLayout>
#include <QSound>
#include <QMouseEvent>
#include <QPainterPath>
#include <QStringList>
#include <algorithm>
#include <deque>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/paint_profile.h"
#include "selfdrive/ui/qt/util.h"

namespace {

// Laying a string out is the expensive half of drawing it, and the HUD redraws
// the same handful of strings frame after frame -- the repaint is driven by the
// camera (cameraview.cc schedules it on every VisionIPC frame), so at 20 fps.
// drawCenteredText below was paying for three layout passes per string: two
// QFontMetrics::boundingRect calls to find the width, then the draw itself.
//
// Memoise only the measurement. The rectangle handed to QPainter is exactly the
// one the same QFontMetrics call produced before, so nothing moves by a pixel;
// the draw still lays the string out once. Painting happens only on the Qt GUI
// thread, so plain statics need no locking.
constexpr int TEXT_RECT_CACHE_MAX = 4096;

struct FontTextRects {
  QFont font;
  QHash<QString, QRect> rects;
};

// Sub-tables are found by comparing QFont values -- a QFontDef field compare
// that allocates nothing -- rather than by QFont::key(), which rebuilds a
// joined string of every font field on each call and would cost more than it
// saves. A HUD draws with on the order of ten distinct fonts, so the scan is
// shorter than hashing one. std::deque keeps the returned reference valid as
// the list grows.
//
// The caller owns the table as a deliberately leaked static: a QFont destroyed
// after QApplication has torn down releases a font engine that no longer has a
// backend, so these must outlive it rather than run at static-destruction time.
QHash<QString, QRect> &textRectsFor(std::deque<FontTextRects> *tables, const QFont &font) {
  for (FontTextRects &t : *tables) {
    if (t.font == font) return t.rects;
  }
  tables->push_back({font, {}});
  return tables->back().rects;
}

// A HUD cycles through a bounded set of strings. Should something unbounded
// ever be drawn through here, drop the table rather than grow without limit.
template <typename Measure>
QRect cachedTextRect(QHash<QString, QRect> &rects, const QString &text, Measure measure) {
  auto it = rects.find(text);
  if (it == rects.end()) {
    if (rects.size() >= TEXT_RECT_CACHE_MAX) rects.clear();
    it = rects.insert(text, measure());
  }
  return it.value();
}
}

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_BACK, this);
  // CameraViewWidget::mouseReleaseEvent accepts every release (emit clicked),
  // so a tap on the HUD never reached OnroadWindow::mouseReleaseEvent: the
  // tile taps and the sidebar toggle all died. The full-size recorder_widget
  // used to sit on top and hide this until e24148e1 removed it. Taps go
  // straight through to OnroadWindow now; its hit tests map into nvg itself.
  nvg->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  road_view_layout->addWidget(nvg);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  // The record button used to sit in the top-right corner, which the thermal
  // panel now takes (rightColumnLayout). It is a free child now, outside the
  // layouts, parked above the 페달 신호 tile by placeRecorder() each update.
  recorder = new ScreenRecoder(this);
  recorder->move(bdr_s, bdr_s + 80);   // until the HUD has drawn the tile
  recorder->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }

  placeRecorder();

}

// Record button centred over the 페달 신호 tile, its bottom the tile group's
// own gap (19 px) above the tile, and kept 10 px inside the HUD's left edge
// (the 190 px button is wider than the 140 px tile at x 30..170).
void OnroadWindow::placeRecorder() {
  if (recorder == nullptr || nvg == nullptr) return;
  const QRect tile = nvg->pedalSignalTileRect();
  if (tile.isEmpty()) return;
  constexpr int GAP = 19;
  constexpr int EDGE = 10;
  const int bx = std::max(EDGE, tile.center().x() - recorder->width() / 2);
  const int by = std::max(EDGE, tile.top() - GAP - recorder->height());
  const QPoint target = nvg->mapTo(this, QPoint(bx, by));
  if (recorder->pos() != target) {
    recorder->move(target);
    recorder->raise();
    alerts->raise();
  }
}

bool NvgWindow::hitPedalTile(const QPoint &pos) const {
  // The tile is only drawn while the HUD is; a rect from a frame more than half
  // a second old is no longer on screen.
  return !pedal_tile_rect_.isEmpty() && millis_since_boot() - pedal_tile_drawn_t_ < 500.0 &&
         pedal_tile_rect_.contains(pos);
}

bool NvgWindow::hitAutoLaunchTile(const QPoint &pos) const {
  return !auto_launch_tile_rect_.isEmpty() && millis_since_boot() - auto_launch_tile_drawn_t_ < 500.0 &&
         auto_launch_tile_rect_.contains(pos);
}

// The TR tile no longer takes taps: there are no following-distance profiles
// to step through (dynamic_follow/follow_gap.py has one gap and no settings).

// Tapping the 페달 강도 tile steps CommaPedalResistance in the settings menu's
// order. controlsd and the planner re-read it on their own, without a restart.
static const char *const PEDAL_PROFILE_CYCLE[] = {"high", "mid", "low", "auto"};

static QString nextPedalProfile(const QString &cur) {
  constexpr int n = sizeof(PEDAL_PROFILE_CYCLE) / sizeof(PEDAL_PROFILE_CYCLE[0]);
  for (int i = 0; i < n; ++i) {
    if (cur == PEDAL_PROFILE_CYCLE[i]) {
      return PEDAL_PROFILE_CYCLE[(i + 1) % n];
    }
  }
  return PEDAL_PROFILE_CYCLE[0];
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();

  // A tap (not a swipe) on the 페달 강도 tile changes the profile, and goes no
  // further: it must not also open the sidebar or toggle the map.
  if (std::abs(dx) < 40 && std::abs(dy) < 40 && nvg->isVisible() &&
      nvg->hitPedalTile(nvg->mapFrom(this, endPos))) {
    Params params;
    const QString next = nextPedalProfile(
      QString::fromStdString(params.get("CommaPedalResistance")).trimmed().toLower());
    params.put("CommaPedalResistance", next.toStdString());
    nvg->refreshPedalProfile();
    return;
  }
  // Same for the 자동 출발 tile: a tap flips ActiveStopAccelBoost. The
  // planner's dynamic follow and controlsd read it live.
  if (std::abs(dx) < 40 && std::abs(dy) < 40 && nvg->isVisible() &&
      nvg->hitAutoLaunchTile(nvg->mapFrom(this, endPos))) {
    Params params;
    params.putBool("ActiveStopAccelBoost", !params.getBool("ActiveStopAccelBoost"));
    nvg->refreshPedalProfile();
    return;
  }
  if(std::abs(dx) > 250 || std::abs(dy) > 200) {

    if(std::abs(dx) < std::abs(dy)) {

      if(dy < 0) { // upward
        Params().remove("CalibrationParams");
        Params().remove("LiveParameters");
        QTimer::singleShot(1500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });

        QSound::play("../assets/sounds/reset_calibration.wav");
      }
      else { // downward
        QTimer::singleShot(500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });
      }
    }
    else if(std::abs(dx) > std::abs(dy)) {
      if(dx < 0) { // right to left
        if(recorder)
          recorder->toggle();
      }
      else { // left to right
        if(recorder)
          recorder->toggle();
      }
    }

    return;
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  startPos = e->pos();
  //QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_RGB_BACK);

  if(offroad && recorder) {
    recorder->stop(false);
  }

}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

// Bottom row (speed-limit board, speed panel, wheel / steering bar /
// confidence gauge, turn-info box) sits this much lower than it used to: the
// CPU / LatA line that it kept clear of moved up next to the clock
// (2026-11-24, on request). 43 rather than the line's full 50 px so the
// lowest box, the wheel's angle badge, still ends 10 px off the HUD's bottom.
static constexpr int BOTTOM_ROW_DROP = 43;

// Speed-limit board and the speed panel right of it. The board keeps half its
// old 30 px margin off the left edge and the panel half its old gap to the
// board (2026-11-24, on request). Everything anchored to the panel's right
// edge (wheel, steering bar, confidence gauge) moves with it.
static constexpr int SPEED_BOARD_X = 15;
static constexpr int SPEED_BOARD_W = 210;
static constexpr int SPEED_BOARD_PANEL_GAP = 15;

// Turn-signal chevron bands, shared by NvgWindow::drawTurnSignals and the ATC
// lane prompt in OnroadAlerts (both widgets fill the same stacked layout).
// Placed where the old pixmap indicator sat: vertically centred, and inboard
// rather than pinned to the screen edge. Still clear of the speed box on the
// left and the thermal tiles on the right, which both hug the edges. The left
// band sits 130 px further in (2026-11-22, on request), so its outermost
// chevron starts 12 px right of the 자동 출발 tile (338..478 x 498..638 on the
// 1880 x 1040 HUD): chevrons 490..640. The right band is unchanged.
static constexpr int TURN_BAND_W = 240;
static constexpr int TURN_BAND_GAP = 400;      // inner edge of the band, out from screen centre
static constexpr int TURN_LEFT_BAND_SHIFT = 130;
static constexpr int TURN_CHEVRONS = 3;
static constexpr int TURN_CHEVRON_STEP = 56;
static constexpr int TURN_CHEVRON_W = 38;
static constexpr int TURN_CHEVRON_H = 58;

static int turnBandX(int w, bool left) {
  return left ? w / 2 - TURN_BAND_GAP - TURN_BAND_W + TURN_LEFT_BAND_SHIFT : w / 2 + TURN_BAND_GAP;
}

// x of the innermost chevron's back edge (the end nearest the screen centre)
static int turnBandInner(int w, bool left) {
  const int band_x = turnBandX(w, left);
  return left ? band_x + TURN_BAND_W - 30 : band_x + 30;
}

// Three chevrons pointing outward; the first `lit` (inside-out) full colour,
// the rest faint.
static void drawTurnChevrons(QPainter &p, int w, int h, bool left, const QColor &color, int lit) {
  const int cy = h / 2;
  const int dir = left ? -1 : 1;
  const int inner = turnBandInner(w, left);
  for (int i = 0; i < TURN_CHEVRONS; i++) {
    const int x = inner + dir * i * TURN_CHEVRON_STEP;
    QColor c = color;
    c.setAlpha(i < lit ? 255 : 50);
    QPainterPath chevron;
    chevron.moveTo(x, cy - TURN_CHEVRON_H);
    chevron.lineTo(x + dir * TURN_CHEVRON_W, cy);
    chevron.lineTo(x, cy + TURN_CHEVRON_H);
    p.strokePath(chevron, QPen(c, 22, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  }
}

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }

  // Lane alerts between the two turn-signal chevron bands, at their height,
  // the text scaled down to fit the gap (2026-11-23, on request):
  //  - the ATC lane prompt draws the chevrons itself, the instruction's side
  //    lit amber and the other faint; the driver's blinker ends the prompt,
  //    so the live bands never draw over it;
  //  - the lane change lines (preLaneChangeLeft/Right, laneChange) come with
  //    the driver's blinker on, so NvgWindow::drawTurnSignals draws the live
  //    bands around them and only the text is drawn here.
  const bool atc_left = alert.type.startsWith("atcBlinkerLeft/");
  const bool atc_right = alert.type.startsWith("atcBlinkerRight/");
  // "좌회전 중 / 우회전 중" (turningIndicatorOn) sits here too: the turn
  // comes with the driver's blinker as a rule, so the live bands frame it.
  const bool lane_change = alert.type.startsWith("laneChange/") ||
                           alert.type.startsWith("preLaneChangeLeft/") ||
                           alert.type.startsWith("preLaneChangeRight/") ||
                           alert.type.startsWith("turningIndicatorOn/");
  if (atc_left || atc_right || lane_change) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);
    if (atc_left || atc_right) {
      const QColor amber(255, 175, 0);
      drawTurnChevrons(p, width(), height(), true, amber, atc_left ? TURN_CHEVRONS : 0);
      drawTurnChevrons(p, width(), height(), false, amber, atc_right ? TURN_CHEVRONS : 0);
    }

    const int gap_l = turnBandInner(width(), true) + 30;
    const int gap_r = turnBandInner(width(), false) - 30;
    const int gap_w = gap_r - gap_l;
    const int cy = height() / 2;

    auto fitted = [&](const QString &text, int size, const QString &style) {
      for (; size > 28; size -= 2) {
        configFont(p, "Open Sans", size, style);
        if (QFontMetrics(p.font()).horizontalAdvance(text) <= gap_w - 40) break;
      }
      return size;
    };
    const bool two = !alert.text2.isEmpty();
    const int s1 = fitted(alert.text1, 64, "Bold");
    const int s2 = two ? fitted(alert.text2, 38, "Regular") : 0;
    const int h1 = s1 + 26, h2 = two ? s2 + 20 : 0;
    const QRect box(gap_l, cy - (h1 + h2) / 2 - 18, gap_w, h1 + h2 + 36);

    const bool prompt = (atc_left || atc_right) ||
                        alert.status == cereal::ControlsState::AlertStatus::USER_PROMPT;
    p.setPen(QPen(prompt ? QColor(0xDA, 0x6F, 0x25) : QColor(0x17, 0x86, 0x44), 5));
    p.setBrush(QColor(0, 0, 0, 165));
    p.drawRoundedRect(box, 24, 24);
    p.setPen(Qt::white);
    configFont(p, "Open Sans", s1, "Bold");
    p.drawText(QRect(box.x(), box.y() + 18, box.width(), h1), Qt::AlignCenter, alert.text1);
    if (two) {
      configFont(p, "Open Sans", s2, "Regular");
      p.setPen(QColor(255, 255, 255, 220));
      p.drawText(QRect(box.x(), box.y() + 18 + h1, box.width(), h2), Qt::AlignCenter, alert.text2);
    }
    return;
  }

  // Every other alert keeps the original bottom band (2026-11-23, on request:
  // the upper / above-the-panel caption of d79d099c and 0e326748 is gone).
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {

}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);

  // neokii
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_acc = QPixmap("../assets/images/img_lat_icon.png");
  ic_lkas = QPixmap("../assets/images/img_long.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  // Unused-upstream asset, not under assets/images/ like the icons above.
  {
    // 0.85 of the wheel tile's own diameter, which is 20% over the shared
    // tile size (see drawBottomIcons' wheel_d), so the icon keeps the same
    // margin inside its disc as before.
    const int wheel_icon_size = static_cast<int>(radius * 6 / 5 * 0.85f);
    ic_wheel = QPixmap("../assets/img_chffr_wheel.png").scaled(wheel_icon_size, wheel_icon_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  }

  // Turn-info box icons (assets/navigation/).
  // carrot draws turn_l / turn_r / lane_change_l / lane_change_r / turn_u.
  {
    constexpr int TURN_ICON = 140;
    auto nav = [](const char *name) {
      return QPixmap(QString("../assets/navigation/%1.png").arg(name))
               .scaled(TURN_ICON, TURN_ICON, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    };
    ic_nav_turn_l = nav("direction_turn_left");
    ic_nav_turn_r = nav("direction_turn_right");
    ic_nav_fork_l = nav("direction_fork_slight_left");
    ic_nav_fork_r = nav("direction_fork_slight_right");
    ic_nav_uturn = nav("direction_turn_uturn");
  }

}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

/*
void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  float start_hue, end_hue;
  if (scene.end_to_end_long) {
    const auto &acceleration = (*s->sm)["modelV2"].getModelV2().getAcceleration();
    float acceleration_future = 0;
    if (acceleration.getZ().size() > 16) {
      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
    }
    start_hue = 60;
    // speed up: 120, slow down: 0
    end_hue = fmax(fmin(start_hue + acceleration_future * 30, 120), 0);

    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    end_hue = int(end_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0, whiteColor(200));
    bg.setColorAt(1, whiteColor(0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  painter.restore();
}*/

// 차선은 흰색, 경로 내부는 단색 초록으로 표시한다.
void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;

  // 1) lanelines: BLUE (alpha = prob, max 0.7)
  // The model always returns four lane lines: 0 = outer-left, 1 = ego-left,
  // 2 = ego-right, 3 = outer-right (see models/README.md "4 lanelines (outer
  // left, left, right, and outer right)"). Indices 0 and 3 are the far sides
  // of the neighbouring lanes, which is why every lane on the road used to be
  // drawn. Draw only 1..2 so just the lane we are in is outlined.
  //
  // Confidence is carried by alpha, which is hard to read at a glance while
  // driving, so a doubtful line turns orange as well. Colour rather than an
  // outline: the band's width is scaled by the same probability, so exactly
  // where an outline would be wanted the band is at its thinnest and a stroke
  // would swallow it -- and past a few metres it narrows below a pixel, where
  // any stroke is the whole line anyway.
  //
  // On the 2026-08-26 city route a quarter of engaged frames sat under 0.5, so
  // orange is meant to be seen regularly: it marks "do not trust this line",
  // not a fault. Below MIN_DRAW nothing is drawn at all -- the line's position
  // is meaningless there, and drawing it would show a lane line that the model
  // is not actually reporting.
  constexpr float LANE_LINE_WEAK_PROB = 0.5f;
  constexpr float LANE_LINE_MIN_DRAW_PROB = 0.05f;
  constexpr float LANE_LINE_WEAK_MIN_ALPHA = 0.45f;
  // Side Blind Zone Alert: the ego line on that side turns red (drawBlindSpot
  // adds the edge glow). 1 = ego-left, 2 = ego-right.
  const auto bsm_cs = (*s->sm)["carState"].getCarState();
  const bool bsm_side[3] = {false, bsm_cs.getLeftBlindspot(), bsm_cs.getRightBlindspot()};
  for (int i = 1; i <= 2; ++i) {
    const float prob = scene.lane_line_probs[i];
    if (prob < LANE_LINE_MIN_DRAW_PROB) {
      continue;
    }
    // Alpha carries confidence, capped at 0.85 rather than the old 0.7 -- at
    // 0.7 even a fully confident line let bright road surface through it.
    float a = std::clamp<float>(prob, 0.0f, 0.85f);
    if (prob < LANE_LINE_WEAK_PROB) {
      // ...but an orange line is a warning, and a warning that fades out is no
      // warning. Straight alpha put prob 0.10 at 10% opacity, which on screen
      // was indistinguishable from the sub-MIN_DRAW case of drawing nothing --
      // so the frames where the car can see least showed the least. Floor it.
      // Width still separates them (15 cm at 0.5 down to 11 cm at 0.1).
      a = std::max(a, LANE_LINE_WEAK_MIN_ALPHA);
      painter.setBrush(QColor::fromRgbF(1.0, 0.55, 0.0, a));   // orange
    } else {
      painter.setBrush(QColor::fromRgbF(0.0, 0.45, 1.0, a));   // blue
    }
    if (bsm_side[i]) {
      painter.setBrush(QColor::fromRgbF(0.92, 0.12, 0.12, std::max(a, 0.85f)));   // red: car in that blind zone
    }
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // Road edges are hidden so only the lane we are in is outlined. These are
  // the two road *boundaries* (where the road meets kerb/sidewalk), not lane
  // lines, and they run out to the sides at a much wider angle than the ego
  // lane -- they were the pale lines still crossing the view after the lane
  // loop above was narrowed to indices 1..2. They looked white rather than red
  // because alpha is 1.0 - road_edge_stds, so an uncertain edge washes out
  // against a bright road surface.
  //
  // Display only: road edges are still computed in ui.cc and still feed the
  // curve fallback (lane_planner.py's road-edge tier). Restore by
  // un-commenting.
  // for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
  //   painter.setBrush(QColor::fromRgbF(1.0, 0.0, 0.0, std::clamp<float>(1.0f - scene.road_edge_stds[i], 0.0f, 1.0f)));
  //   painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  // }

  // 2) PATH: show a red warning path when the model predicts a deep turn.
  float future_yaw = 0.0f;
  const auto &orientation = (*s->sm)["modelV2"].getModelV2().getOrientation();
  if (orientation.getZ().size() > 16) {
    future_yaw = std::abs(orientation.getZ()[16]);  // roughly 2.5 seconds ahead
  }

  QLinearGradient bg(0, height(), 0, height() / 4);
  if (future_yaw >= 0.12f) {
    bg.setColorAt(0.00, QColor(255, 22, 22, 210));   // sharp-turn warning
    bg.setColorAt(0.55, QColor(255, 48, 25, 185));
    bg.setColorAt(1.00, QColor(255, 92, 30, 0));
  } else {
    bg.setColorAt(0.00, QColor(18, 164, 226, 190));   // bright teal (2026-11-25, on request; was green 55,235,72)
    bg.setColorAt(0.50, QColor(18, 164, 226, 160));
    bg.setColorAt(1.00, QColor(18, 164, 226, 0));     // fade out
  }

  if (scene.turn_arrow_active) {
    // Turn desire out (ui.cc update_turn_arrow): the model path stays as a
    // faint band, since the steering still follows it, and the navigation's
    // turn is the arrow on top.
    QLinearGradient faint(0, height(), 0, height() / 4);
    faint.setColorAt(0.00, QColor(18, 164, 226, 70));
    faint.setColorAt(1.00, QColor(18, 164, 226, 0));
    painter.setBrush(faint);
    painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);
    if (scene.turn_arrow_ok) {
      // Screen-space arrow (ui.cc update_turn_arrow): up the lane from the
      // base, a quarter bend at the corner, then sideways to a head.
      const float sgn = scene.turn_arrow_left ? -1.f : 1.f;
      const QPointF b = scene.turn_arrow_base, c = scene.turn_arrow_corner;
      const float rise = std::max<float>(b.y() - c.y(), 1.f);
      const float r = std::min<float>(std::clamp<float>(rise * 0.45f, 50.f, 160.f), rise);  // bend radius [px]
      const float run = std::clamp<float>(width() * 0.14f, 160.f, 320.f);  // sideways leg [px]
      const float pen_w = std::clamp<float>(rise * 0.22f, 34.f, 70.f);
      const float head = pen_w * 1.25f;
      const QPointF knee(c.x(), c.y() + r);
      const QPointF out(c.x() + sgn * r, c.y());
      const QPointF end(c.x() + sgn * run, c.y());
      QPainterPath arrow_path(b);
      arrow_path.lineTo(knee);
      arrow_path.quadTo(c, out);
      arrow_path.lineTo(end);
      painter.setBrush(Qt::NoBrush);
      painter.setPen(QPen(QColor(30, 110, 255, 215), pen_w, Qt::SolidLine, Qt::FlatCap, Qt::RoundJoin));
      painter.drawPath(arrow_path);
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(30, 110, 255, 215));
      const QPointF tri[3] = {QPointF(end.x() + sgn * head * 1.3f, end.y()),
                              QPointF(end.x(), end.y() - head), QPointF(end.x(), end.y() + head)};
      painter.drawPolygon(tri, 3);
    }
  } else {
    painter.setBrush(bg);
    painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);
  }

  painter.restore();
}


// Lead marker: a rectangle outline around the lead car with the distance /
// speed / gap readout in a small pill inside the bottom edge (2026-11-24, on
// request; it replaced the red chevron and the big readout box under it).
// The outline colour carries the danger level drawHud works out: cyan safe,
// orange close, red too close, each a little thicker than the last.
void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data,
                         const QPointF &vd, const QString &info_text, int danger) {
  // Box sized from the model's own distance and the same focal scale the
  // road is projected with (ZOOM / fx, see updateFrameMat), so it shrinks as
  // the car pulls away: a car is ~1.9 m wide and ~1.5 m tall, plus a margin.
  const float d_rel = std::max(lead_data.getX()[0], 1.0f);
  const float zoom = uiState()->wide_camera ? ZOOM * 0.5f : ZOOM;
  const float bw = std::clamp(zoom * 2.4f / d_rel, 90.0f, 760.0f);
  const float bh = std::clamp(zoom * 1.9f / d_rel, 70.0f, 600.0f);
  // vd is the lead's rear projected a little above the road (ui.cc), where
  // the old chevron's tip sat; the car's wheels are a fifth of the box below.
  const float cx = std::clamp((float)vd.x(), bw / 2, width() - bw / 2);
  const float bottom = std::fmin(height() - 8.0f, (float)vd.y() + bh * 0.2f);
  const float top = bottom - bh;
  const float left = cx - bw / 2, right = cx + bw / 2;

  static const QColor kColors[3] = {QColor(0, 220, 230), QColor(255, 150, 0), QColor(255, 45, 45)};
  // 30% over the first 7 / 8 / 10, which read too thin on the road.
  static const int kWidths[3] = {9, 10, 13};
  danger = std::clamp(danger, 0, 2);

  // A full rectangle outline (2026-11-24, on request; it was four corner
  // brackets), small rounded corners only so the thick stroke's joins stay
  // clean.
  painter.save();
  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(kColors[danger], kWidths[danger], Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawRoundedRect(QRectF(left, top, right - left, bottom - top), 6, 6);

  if (!info_text.isEmpty()) {
    // Centred on the box, and one fixed size whatever the distance (on
    // request): the font never scales with the box, so near and far leads
    // read the same. A far lead's box can be narrower than the pill; it then
    // stays centred and reaches past both sides equally.
    //
    // Solid dark green under white text: about 5:1 contrast on its own, so it
    // reads over a bright sky, the dark car body or the green path alike. A
    // thin white rim keeps its edge visible where it sits on that green path.
    // 40 px with its padding: 30% over the 31 / 36 / 13 it had (on request).
    configFont(painter, "Open Sans", 40, "Bold");
    QFontMetrics metrics(painter.font());
    const int pill_w = metrics.horizontalAdvance(info_text) + 47;
    const int pill_h = metrics.height() + 17;
    QRect pill(static_cast<int>(cx - pill_w / 2.0f), static_cast<int>((top + bottom) / 2.0f - pill_h / 2.0f),
               pill_w, pill_h);
    pill.moveLeft(std::clamp(pill.left(), 12, std::max(12, width() - pill.width() - 12)));

    painter.setPen(QPen(QColor(255, 255, 255, 200), 2));
    painter.setBrush(QColor(0, 125, 60, 235));
    painter.drawRoundedRect(pill, pill_h / 2.0, pill_h / 2.0);
    painter.setPen(QColor(255, 255, 255));
    painter.drawText(pill, Qt::AlignCenter, info_text);
  }
  painter.restore();
}

void NvgWindow::paintGL() {
}

void NvgWindow::paintEvent(QPaintEvent *event) {
  paint_profile::begin();
  QPainter p;
  p.begin(this);

  p.beginNativePainting();
  CameraViewWidget::paintGL();
  p.endNativePainting();
  paint_profile::mark(paint_profile::CAMERA);

  UIState *s = uiState();
  if (s->worldObjectsVisible()) {
    drawHud(p);
  }

  p.end();
  paint_profile::mark(paint_profile::PAINTER_END);
  paint_profile::paint_done();

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15 && cur_draw_t - last_slow_frame_log_t >= 5000.) {
    LOGW("slow frame rate: %.2f fps", fps);
    last_slow_frame_log_t = cur_draw_t;
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000*5) {
    last_update_params = now;
    ui_update_params(uiState());
  }

  prev_draw_t = millis_since_boot();
}

// QRect::moveCenter rounds to an integer centre, and how far it rounds
// (0 or 0.5 px) depends on the parity of the glyph bounding box's width --
// consistent for any one string, but a single narrow glyph (e.g. "0") and a
// wider one (e.g. "40") drawn at the same x can land on opposite sides of
// that 0.5 px line. Each individually reads as "centred", but stacked next
// to each other (the cruise/current-speed/apply panel) the mismatch shows
// as one number looking off-centre relative to the others. Centring with
// float math instead removes the parity dependency, so every string lands
// on the exact same subpixel point regardless of its own width.
static void drawCenteredText(QPainter &p, int x, int y, const QString &text) {
  static auto *tables = new std::deque<FontTextRects>;
  const QRect real_rect = cachedTextRect(textRectsFor(tables, p.font()), text, [&] {
    QFontMetrics fm(p.font());
    QRect init_rect = fm.boundingRect(text);
    return fm.boundingRect(init_rect, 0, text);
  });
  const qreal draw_x = x - real_rect.width() / 2.0;

  // The width was memoised, but QPainter::drawText still itemised and shaped
  // the string (font fallback for Hangul, HarfBuzz) on every call, ~60 calls a
  // paint. Shape each (font, text) once into glyph runs whose baseline sits at
  // y = 0 and replay them; the glyphs, positions and pen are the same ones
  // drawText(QPointF) would produce, so nothing moves.
  struct FontGlyphRuns {
    QFont font;
    QHash<QString, QList<QGlyphRun>> runs;
  };
  static auto *run_tables = new std::deque<FontGlyphRuns>;
  FontGlyphRuns *table = nullptr;
  for (FontGlyphRuns &t : *run_tables) {
    if (t.font == p.font()) { table = &t; break; }
  }
  if (!table) {
    run_tables->push_back({p.font(), {}});
    table = &run_tables->back();
  }
  auto it = table->runs.find(text);
  if (it == table->runs.end()) {
    if (table->runs.size() >= TEXT_RECT_CACHE_MAX) table->runs.clear();
    QTextLayout layout(text, p.font(), p.device());
    QTextOption opt;
    opt.setWrapMode(QTextOption::NoWrap);
    layout.setTextOption(opt);
    layout.beginLayout();
    QTextLine line = layout.createLine();
    if (line.isValid()) {
      line.setLineWidth(1e6);
      line.setPosition(QPointF(0, -line.ascent()));
    }
    layout.endLayout();
    it = table->runs.insert(text, layout.glyphRuns());
  }
  const QPointF origin(draw_x, y);
  for (const QGlyphRun &run : it.value()) {
    p.drawGlyphRun(origin, run);
  }
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  drawCenteredText(p, x, y, text);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  p.setPen(color);
  drawCenteredText(p, x, y, text);
}

void NvgWindow::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
}

void NvgWindow::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  static auto *tables = new std::deque<FontTextRects>;
  const QRect rect = cachedTextRect(textRectsFor(tables, p.font()), text, [&] {
    QFontMetrics fm(p.font());
    QRect r = fm.boundingRect(text);
    r.adjust(-1, -1, 1, 1);
    return r;
  });
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void NvgWindow::drawHud(QPainter &p) {

  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  // Wall-clock timestamp, top-left -- lets a screenshot be matched back to
  // its exact rlog frame during later log analysis. Fixed at KST (UTC+9)
  // via toOffsetFromUtc() rather than trusting the device's own system
  // timezone setting, so this stays correct even if that's misconfigured.
  // The text only changes once a second, so cache the formatted string and
  // the QFont (configFont() rebuilds a QFont by family-name lookup on every
  // call) instead of paying that cost on every ~20Hz repaint.
  {
    static QFont ts_font = [] {
      QFont f("Open Sans");
      f.setPixelSize(40);
      f.setStyleName("Bold");
      return f;
    }();
    static QString cached_ts;
    static qint64 cached_sec = -1;
    const qint64 sec = QDateTime::currentSecsSinceEpoch();
    if (sec != cached_sec) {
      cached_sec = sec;
      const QDateTime kst = QDateTime::currentDateTimeUtc().toOffsetFromUtc(9 * 3600);
      cached_ts = kst.toString("yyyy-MM-dd HH:mm:ss");
    }
    p.setFont(ts_font);
    p.setPen(QColor(255, 255, 255, 220));
    p.drawText(bdr_s, 55, cached_ts);
  }
  // Where the CPU / LatA line starts: just right of the clock, on its
  // baseline (2026-11-24, on request; it used to run along the bottom edge).
  // Measured from an all-8s template so the line never shifts as the time
  // ticks.
  static const int info_x = [] {
    QFont f("Open Sans");
    f.setPixelSize(40);
    f.setStyleName("Bold");
    return bdr_s + QFontMetrics(f).horizontalAdvance("8888-88-88 88:88:88") + 30;
  }();

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);
  paint_profile::mark(paint_profile::HUD_HEAD);

  drawLaneLines(p, s);
  paint_profile::mark(paint_profile::LANES);

  auto leads = sm["modelV2"].getModelV2().getLeadsV3();
  const auto lead_one = sm["radarState"].getRadarState().getLeadOne();
  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  QString lead_info;
  if (lead_one.getStatus()) {
    const float v_ego = std::max(car_state.getVEgo(), 0.0f);
    const float d_rel = std::max(lead_one.getDRel(), 0.0f);
    const float v_lead = std::max(lead_one.getVLead(), 0.0f);
    if (v_ego > 1.0f) {
      const float time_gap = d_rel / v_ego;
      lead_info.sprintf("%.0f meters | %.0f km/h | %.2f s",
                        d_rel, v_lead * MS_TO_KPH, time_gap);
    } else {
      lead_info.sprintf("%.0f meters | %.0f km/h | -- s",
                        d_rel, v_lead * MS_TO_KPH);
    }
  }
  // Danger level for the lead's corner colour: 0 cyan, 1 orange, 2 red.
  // Two measures, whichever is worse:
  //  - time gap (distance / own speed): under 1.2 s orange, under 0.8 s red.
  //    1.2 rather than the textbook 2 s because the planner itself follows at
  //    about TR 1.45 -- a stricter line would paint normal following orange;
  //  - time to collision while closing in: under 4 s orange, under 2.5 s red,
  //    so a lead braking hard turns the box before the gap itself shrinks.
  // Near standstill (< 1 m/s) queueing close is normal: orange only inside
  // 2 m. A drop to a safer colour must hold 0.5 s, so the colour does not
  // flicker on a threshold; a rise shows at once.
  int lead_danger = 0;
  {
    static int shown_level = 0;
    static double hold_since = 0;
    int raw = 0;
    bool have = false;
    float d = 0.0f, v_lead = 0.0f;
    if (lead_one.getStatus()) {
      d = lead_one.getDRel();
      v_lead = lead_one.getVLead();
      have = true;
    } else if (leads[0].getProb() > .5) {
      d = leads[0].getX()[0];
      v_lead = leads[0].getV()[0];
      have = true;
    }
    if (have) {
      const float v_ego = std::max(car_state.getVEgo(), 0.0f);
      d = std::max(d, 0.0f);
      const float closing = v_ego - v_lead;
      const float ttc = closing > 0.5f ? d / closing : 1e3f;
      if (v_ego > 1.0f) {
        const float gap = d / v_ego;
        if (gap < 0.8f || ttc < 2.5f) raw = 2;
        else if (gap < 1.2f || ttc < 4.0f) raw = 1;
      } else if (d < 2.0f) {
        raw = 1;
      }
    }
    const double now = millis_since_boot();
    if (raw >= shown_level || now - hold_since >= 500.0) {
      shown_level = raw;
      hold_since = now;
    }
    lead_danger = shown_level;
  }

  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], lead_info, lead_danger);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1]);
  }

  paint_profile::mark(paint_profile::LEADS);

  //drawMaxSpeed(p);
  drawSpeed(p);
  paint_profile::mark(paint_profile::SPEED);
  drawSpeedLimit(p);
  paint_profile::mark(paint_profile::SPEED_LIMIT);
  drawThermal(p);
  paint_profile::mark(paint_profile::THERMAL);
  drawTurnSignals(p);
  drawBlindSpot(p);
  paint_profile::mark(paint_profile::TURN_SIGNALS);
  drawTurnInfo(p);
  paint_profile::mark(paint_profile::TURN_INFO);
  //drawGpsStatus(p);

  if(s->show_debug && width() > 1200)
    drawDebugText(p);

  const auto device_state = sm["deviceState"].getDeviceState();
  //const auto car_control = sm["carControl"].getCarControl();
  //const auto live_params = sm["liveParameters"].getLiveParameters();
  //const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  //const auto torque_state = controls_state.getLateralControlState().getTorqueState();

  //QColor orangeColor = QColor(52, 197, 66, 255);

  // Per core, not an average. One mean number is a value no core actually
  // has here: rtshield spins on the last core at FIFO 1 purely to keep it out
  // of idle, so it reads 100% for every healthy drive while the others sit
  // well below -- and averaging the four hid exactly the imbalance this line
  // is being read for. Cores are numbered from 1 to match how they read on
  // screen. Memory usage is gone with the average; it was never the number
  // anyone came to this line for.
  const auto cpu_usage_list = device_state.getCpuUsagePercent();
  constexpr int MAX_CPUS = 8;
  int cpus[MAX_CPUS] = {};
  const int n_cpus = std::min((int)cpu_usage_list.size(), MAX_CPUS);
  for (int i = 0; i < n_cpus; i++) {
    cpus[i] = cpu_usage_list[i];
  }

  // Nothing on this line moves at repaint rate: the core usages come from
  // deviceState at 2 Hz, and the rest are tuning values that hold still for
  // long stretches. Reformat only when one of them actually changes, keeping
  // the last string otherwise. This does not avoid the layout pass in
  // drawText -- that is unavoidable for a string drawn without measuring --
  // so it saves the formatting alone.
  const float info_values[3] = {
    controls_state.getLatAccelFactor(),
    controls_state.getFriction(),
    controls_state.getSteerRatio(),
  };
  // The clock's own size as the ceiling, so the two read as one header line,
  // and the floor below which it stops being readable at arm's length.
  constexpr int INFO_FONT_MAX = 40;
  constexpr int INFO_FONT_MIN = 24;
  constexpr int INFO_MARGIN = 20;
  static QString infoText;
  static int infoFontSize = INFO_FONT_MAX;
  static float last_values[3] = {};
  static int last_cpus[MAX_CPUS] = {};
  static int last_n_cpus = -1;
  if (infoText.isEmpty() || n_cpus != last_n_cpus ||
      !std::equal(cpus, cpus + n_cpus, last_cpus) ||
      !std::equal(std::begin(info_values), std::end(info_values), std::begin(last_values))) {
    std::copy(std::begin(info_values), std::end(info_values), std::begin(last_values));
    std::copy(cpus, cpus + n_cpus, last_cpus);
    last_n_cpus = n_cpus;

    infoText.clear();
    QString part;
    for (int i = 0; i < n_cpus; i++) {
      part.sprintf("CPU%d(%d%%) ", i + 1, cpus[i]);
      infoText += part;
    }
    // MIN_TR and DF_MOD were dropped from this line on request.
    part.sprintf("(LatA:%.3f,Fri:%.3f) SR(%.2f)",
                    info_values[0],
                    info_values[1],
                    info_values[2]
                    );
    infoText += part;

    // Four cores spelled out is a far longer line than the single average it
    // replaces -- long enough to run off the right edge at INFO_FONT_MAX -- so
    // the size is fitted to the width rather than fixed. Text advance is close
    // enough to linear in pixel size that one measurement at the maximum gives
    // the answer directly, and integer division rounds toward fitting. Done
    // here rather than per repaint: it is only worth measuring when the string
    // it measures has changed.
    configFont(p, "Open Sans", INFO_FONT_MAX, "Regular");
    const int avail = rect().width() - INFO_MARGIN - info_x;
    const int full_w = QFontMetrics(p.font()).horizontalAdvance(infoText);
    infoFontSize = (full_w > avail && full_w > 0)
                     ? std::max(INFO_FONT_MIN, INFO_FONT_MAX * avail / full_w)
                     : INFO_FONT_MAX;
    // The estimate treats advance as linear in pixel size, which is close but
    // not exact -- hinting and integer metrics move it a pixel or two either
    // way, and with three-digit percentages on every core the fit lands within
    // a few pixels of the edge. Confirm it, and step down if it does not fit,
    // which from a linear estimate is one step at most.
    while (infoFontSize > INFO_FONT_MIN) {
      configFont(p, "Open Sans", infoFontSize, "Regular");
      if (QFontMetrics(p.font()).horizontalAdvance(infoText) <= avail) break;
      infoFontSize--;
    }
  }


  // info
  configFont(p, "Open Sans", infoFontSize, "Regular");
  p.setPen(QColor(0, 255, 0, 255));
  p.drawText(info_x, 55, infoText);
  paint_profile::mark(paint_profile::INFO_LINE);


  drawBottomIcons(p);
  paint_profile::mark(paint_profile::BOTTOM_ICONS);
}

// Shared green -> amber -> red scale for the on-screen gauges. t = 0 is
// green (plenty of headroom), t = 1 red (at the limit). The confidence
// gauge reads the other way round -- more is better there -- so it passes
// 1 - dProb and lands on the same palette.
static QColor gaugeColor(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  const QColor safe(90, 200, 110), mid(230, 165, 40), danger(230, 55, 55);
  const QColor &a = (t < 0.5f) ? safe : mid;
  const QColor &b = (t < 0.5f) ? mid : danger;
  const float u = (t < 0.5f) ? (t / 0.5f) : ((t - 0.5f) / 0.5f);
  return QColor(a.red() + (b.red() - a.red()) * u,
                a.green() + (b.green() - a.green()) * u,
                a.blue() + (b.blue() - a.blue()) * u);
}

// Steering-effort gauge. The bowed bar is a fixed track; a coloured gauge
// fills out of its centre toward the side openpilot is steering to, its
// length proportional to how much of the available steering command is in
// use and its colour running green -> amber -> red as that approaches
// saturation. So a full-left saturated command reads as a red bar filling
// the entire left half, and the driver sees both direction and how close
// the system is to running out of authority in one glance.
//
// The gauge is driven by carControl.actuators.steer, the same normalized
// command the STEER MAX gauge below scales to 0..300, so the two never
// disagree.
//
// Riding on top is a separate outline-only marker for lane centring: where
// the car actually sits between the model's two lane lines, so a drift left
// or right is visible alongside the steering effort. The two answer
// different questions -- the gauge is what openpilot is DOING, the marker is
// where the car IS -- so they are deliberately kept as distinct shapes
// rather than folded into one indicator, which is what made the previous
// single marker ambiguous.
void NvgWindow::drawSteerGauge(QPainter &p, int cx, int cy, int w) {
  // Every vertical dimension here, and everything drawn inside the track, is
  // 30% over the sizes this gauge started at (12 / 46 / 30 / 40x30 / 32 / 8).
  // Scaled as a set so the gauge, the neutral reference and the lane marker
  // keep their proportions inside the thicker bar rather than one of them
  // being swallowed by it.
  constexpr int BOW = 16;      // how much the bar bows up in the middle
  constexpr int BAR_W = 60;    // track stroke width
  constexpr int GAUGE_W = 39;  // gauge stroke width, inset inside the track
  constexpr int REF_W = 52, REF_H = 39;
  constexpr int MARK = 42;     // lane-centring marker box size
  constexpr int MARK_PEN = 10; // marker outline thickness
  // Mirrors lane_planner.py CAMERA_OFFSET. The camera does not sit on the
  // car's centreline, so raw lane lines carry a constant lateral bias.
  constexpr float CAMERA_OFFSET_UI = -0.070f;

  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  const auto car_control = sm["carControl"].getCarControl();

  // Positive = LEFT, matching this fork's steeringAngleDeg convention (see
  // reference-sign-conventions) -- actuators.steer is commanded in the same
  // frame, so positive fills toward the bar's left end.
  const float steer_cmd = std::clamp(car_control.getActuators().getSteer(), -1.0f, 1.0f);
  // Hands on the wheel means the driver is steering, not openpilot, and the
  // command still being published is being overridden. Show nothing rather
  // than an animated gauge implying the system is driving the corner.
  const bool active = car_control.getLatActive() && !car_state.getSteeringPressed();

  // Smooth so the gauge glides instead of chattering frame to frame. Held
  // hard at neutral while inactive, so grabbing the wheel stops all motion
  // immediately instead of animating down.
  static float steer_shown = 0.0f;
  if (active) {
    steer_shown += (steer_cmd - steer_shown) * 0.18f;
  } else {
    steer_shown = 0.0f;
  }

  // Lane centring: -1 = hard left of the lane, +1 = hard right. Read from
  // the model's own lane lines, the same y values lane_planner blends.
  bool lane_valid = false;
  float lane_offset = 0.0f;
  const auto lls = sm["modelV2"].getModelV2().getLaneLines();
  const auto probs = sm["modelV2"].getModelV2().getLaneLineProbs();
  if (lls.size() >= 4 && probs.size() >= 4) {
    const auto ly = lls[1].getY();
    const auto ry = lls[2].getY();
    if (ly.size() > 0 && ry.size() > 0) {
      const float left = ly[0], right = ry[0];
      const float lane_w = right - left;
      if (probs[1] > 0.3f && probs[2] > 0.3f && lane_w > 2.0f && lane_w < 4.5f) {
        // modelV2 y is +right, so a positive lane centre means the lane sits
        // to the right of the car -- the car is left of centre. Flip the sign
        // so the marker travels the way the car does.
        const float centre = (left + right) / 2.0f + CAMERA_OFFSET_UI;
        lane_offset = std::clamp(-centre / (lane_w / 2.0f), -1.0f, 1.0f);
        lane_valid = true;
      }
    }
  }
  static float lane_shown = 0.0f;
  lane_shown += ((lane_valid ? lane_offset : 0.0f) - lane_shown) * 0.18f;

  const int half = w / 2;
  const int x0 = cx - half, x1 = cx + half;
  const int y_end = cy + BOW;

  // The track is a quadratic bezier, so the gauge has to follow it exactly
  // rather than being a straight rectangle. t = 0 is the left end, 1 the
  // right, 0.5 the centre.
  auto bar_point = [&](float t) {
    const float mt = 1.0f - t;
    return QPointF(mt * mt * x0 + 2.0f * mt * t * cx + t * t * x1,
                   mt * mt * y_end + 2.0f * mt * t * (cy - BOW) + t * t * y_end);
  };

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);

  QPainterPath bar;
  bar.moveTo(x0, y_end);
  bar.quadTo(cx, cy - BOW, x1, y_end);
  p.strokePath(bar, QPen(QColor(255, 255, 255, active ? 210 : 110), BAR_W,
                         Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  const float ratio = std::abs(steer_shown);
  if (active && ratio > 0.01f) {
    // Positive (left) walks t down from the centre toward 0, negative
    // (right) up toward 1, reaching the very end at full saturation.
    const float t_end = 0.5f - 0.5f * steer_shown;
    constexpr int STEPS = 24;
    QPainterPath gauge;
    gauge.moveTo(bar_point(0.5f));
    for (int i = 1; i <= STEPS; i++) {
      gauge.lineTo(bar_point(0.5f + (t_end - 0.5f) * (float)i / STEPS));
    }
    p.strokePath(gauge, QPen(gaugeColor(ratio), GAUGE_W,
                             Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  }

  // Neutral reference at the centre, over the gauge so its round cap never
  // bulges past centre.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(40, 44, 48, active ? 235 : 110));
  p.drawRoundedRect(QRectF(cx - REF_W / 2.0, cy - REF_H / 2.0, REF_W, REF_H), 5, 5);

  // Lane-centring marker, drawn last so it stays readable over the gauge.
  // Outline only -- no fill -- so the gauge colour and the black centre
  // reference both stay visible where it overlaps them, which is exactly
  // where the car is centred and the two shapes sit on top of each other.
  const QPointF mark_pt = bar_point((lane_shown + 1.0f) / 2.0f);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(254, 32, 32, lane_valid ? 255 : 120), MARK_PEN,
                Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawRoundedRect(QRectF(mark_pt.x() - MARK / 2.0, mark_pt.y() - MARK / 2.0,
                           MARK, MARK), 7, 7);

  p.restore();
}

// Scene-understanding confidence gauge: a vertical white track with a ball
// that rises and shifts red -> amber -> green as lateralPlan.dProb climbs.
// dProb is the model's own published confidence in the lane geometry it is
// currently steering to (see lane_planner.py) -- the same quantity behind
// the dProb-based analysis used to size the fallback thresholds elsewhere
// in this file/repo, reused here as a general "how sure is it right now"
// readout rather than adding a second confidence source.
void NvgWindow::drawConfidenceGauge(QPainter &p, int cx, int top_y, int bottom_y) {
  // Sized 20% up from the original 28 / 44 / 257 so the ball's height reads at
  // a glance from the driver's seat rather than needing a look. The span grows
  // about its own centre, so the gauge stays centred on the same point and
  // still clears the temperature panel beside it (664..972 inside its
  // 652..976).
  constexpr int TRACK_W = 41;   // 28 * 1.2 * 1.2
  constexpr int BALL_D = 64;    // 44 * 1.2 * 1.2
  constexpr int BALL_PEN = 10;  // black rim, drawn outside the fill
  const int mid = (top_y + bottom_y) / 2;
  const int half_span = (bottom_y - top_y) * 6 / 10;   // 0.5 * 1.2
  top_y = mid - half_span;
  bottom_y = mid + half_span;

  const SubMaster &sm = *(uiState()->sm);
  const auto lp = sm["lateralPlan"].getLateralPlan();
  const float target = std::clamp(lp.getDProb(), 0.0f, 1.0f);

  // Smooth like the lane-alignment marker so the ball glides rather than
  // jumping every model frame.
  static float shown = 0.0f;
  shown += (target - shown) * 0.15f;

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);

  // White track, matching the lane-alignment bar's look.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(255, 255, 255, 90));
  p.drawRoundedRect(QRectF(cx - TRACK_W / 2.0, top_y, TRACK_W, bottom_y - top_y),
                    TRACK_W / 2.0, TRACK_W / 2.0);

  // Red -> amber -> green across the confidence range: the shared gauge
  // scale, inverted because here a high value is the good end.
  const QColor ball_color = gaugeColor(1.0f - shown);

  // Ball travels the track's inner extent so it never pokes past the
  // rounded caps at either end. Measured from the rim, not the fill, or the
  // outline is what pokes out instead.
  const float ball_r = (BALL_D + BALL_PEN) / 2.0f;
  const float travel_top = top_y + ball_r;
  const float travel_bottom = bottom_y - ball_r;
  const float ball_y = travel_bottom - shown * (travel_bottom - travel_top);

  // Black rim so the ball still reads as a distinct object where its colour
  // lands close to the track's own white or to a bright road surface behind.
  p.setPen(QPen(blackColor(255), BALL_PEN));
  p.setBrush(ball_color);
  p.drawEllipse(QPointF(cx, ball_y), BALL_D / 2.0, BALL_D / 2.0);
  p.setPen(Qt::NoPen);

  p.restore();
}

// Label for a TmapNda safety event. The codes are TmapNda's nSdiType scheme,
// passed through road_speed_limiter.py as roadLimitSpeed.camType (TmapNda
// translates Kakao's KNSafetyCode into the same numbers). Note 33 is wildlife,
// not school zone -- school zone is 20.
static constexpr int SPEED_BUMP_CAM_TYPE = 22;
// TmapNda sends a fixed 150 m once a bump's distance reaches 0 (just passed).
// It is not a real bump -- the same rule as SpeedBumpPlanner.PLACEHOLDER_DIST.
static constexpr int BUMP_PLACEHOLDER_DIST = 150;
static constexpr int PARKING_CAM_TYPE = 17;

// Distance (m) of the speed bump to show, or -1 for none. A bump is shown only
// while controlsd's SpeedBumpPlanner is approaching one, so TmapNda's late
// reports of a bump already passed (and its 150 m placeholder) never reach
// the screen. The number is TmapNda's own reported distance; when TmapNda has
// dropped the bump before reaching it, the planner's estimate stands in.
static int shownBumpDist(const SubMaster &sm) {
  if (!sm["controlsState"].getControlsState().getSpeedBumpTracking()) return -1;
  auto nda = sm["roadLimitSpeed"].getRoadLimitSpeed();
  if (nda.getActive() <= 0) return -1;
  int d = nda.getBumpLeftDist();
  if (d <= 0 && nda.getCamType() == SPEED_BUMP_CAM_TYPE) d = nda.getCamLimitSpeedLeftDist();
  if (d <= 0 || d == BUMP_PLACEHOLDER_DIST) {
    d = (int)std::lround(sm["controlsState"].getControlsState().getSpeedBumpDist());
  }
  return std::max(0, d);
}

static QString ndaSafetyLabel(int cam_type) {
  switch (cam_type) {
    case 1: case 8: return "단속카메라";
    case 2: case 3: case 4: return "구간단속";
    case 6: return "신호단속";
    case 7: return "이동식카메라";
    case 9: return "버스전용";
    case 11: return "갓길감시";
    case 12: return "끼어들기";
    case 13: return "교통정보수집";
    case 15: return "과적단속";
    case 16: return "적재불량";
    case 17: return "주차단속";
    case 19: return "철길건널목";
    case 20: return "스쿨존";
    case 22: return "방지턱";
    case 25: return "휴게소";
    case 26: return "톨게이트";
    case 27: return "안개주의";
    case 29: return "사고다발";
    case 30: return "급커브";
    case 32: return "급경사";
    case 33: return "야생동물";
    // TmapNda's live incidents (IncidentRepository), not SDI codes.
    case 90: return "사고발생";
    case 91: return "공사중";
    case 92: return "재난발생";
    default: return "안전정보";
  }
}

// Right-hand column, laid out bottom up (2026-11-22, on request):
//   1. the turn-info box in carrot's corner (hud_renderer: right edge 10 px
//      off the screen edge, 300 tall), raised by INFO_CLEAR so the CPU / LatA
//      line along the bottom (baseline h - 15) stays readable. Narrowed from
//      carrot's 790 to 639 (x 1271..1910 on 1920 px) to leave room for the
//      half-length steering bar and the confidence gauge left of it;
//   2. the brake / ACC / auto-hold / LKAS status block above it, with the
//      confidence gauge beside its brake / auto-hold column;
//   3. the temperature panel above that.
// One function so drawThermal, drawBottomIcons and drawTurnInfo agree.
//
// 2026-11-24: the box is 699 wide (x 1171..1870 on the 1880 px HUD), taking
// the 60 px the bottom row gave up when the speed panel moved left, and it
// drops by BOTTOM_ROW_DROP now that the CPU line is no longer under it.
//
// Later on 2026-11-24 the thermal strip moved from above the status block to
// between it and the turn-info box (on request), so from the bottom up it is
// now: turn-info box, thermal strip, status block. The strip's height is a
// fixed THERMAL_H rather than measured from its fonts, so this function can
// place the block above it without drawing anything.
RightColumnLayout rightColumnLayout(int w, int h) {
  constexpr int BOX_W = 699, BOX_H = 300;
  constexpr int INFO_CLEAR = 50 - BOTTOM_ROW_DROP;
  constexpr int GAP = 20;
  constexpr int STATUS_PANEL_GAP = 19;
  RightColumnLayout l;
  l.turn_box = QRect(w - 10 - BOX_W, h - 10 - BOX_H - INFO_CLEAR, BOX_W, BOX_H);
  l.thermal_bottom = l.turn_box.top() - GAP;
  l.thermal_top = l.thermal_bottom - THERMAL_H;
  l.status_bottom = l.thermal_top - STATUS_PANEL_GAP;
  return l;
}

// 페달 상태 icons (design G, chosen 2026-11-25), standard shapes so they read
// without learning: Google Material Design arrow_upward / arrow_downward /
// keyboard_double_arrow_up outlines (24 x 24 grid, Apache 2.0) for 가속 /
// 감속 / BOOST, and the ISO 2575 brake tell-tale (!) for 브레이크. Vector
// shapes, so no asset files. Each fits a box s wide and tall centred on c.
enum class PedalIcon { Accel, Decel, Brake, Boost };

static void drawPedalIcon(QPainter &p, QPointF c, float s, PedalIcon kind, const QColor &color) {
  static const QPointF ARROW_UP[] = {{4, 12}, {5.41, 13.41}, {11, 7.83}, {11, 20}, {13, 20},
                                     {13, 7.83}, {18.58, 13.42}, {20, 12}, {12, 4}};
  static const QPointF ARROW_DOWN[] = {{4, 12}, {5.41, 10.59}, {11, 16.17}, {11, 4}, {13, 4},
                                       {13, 16.17}, {18.58, 10.58}, {20, 12}, {12, 20}};
  static const QPointF DOUBLE_UP_LOW[] = {{6, 17.59}, {7.41, 19}, {12, 14.42}, {16.59, 19},
                                          {18, 17.59}, {12, 11.59}};
  static const QPointF DOUBLE_UP_HIGH[] = {{6, 11}, {7.41, 12.41}, {12, 7.83}, {16.59, 12.41},
                                           {18, 11}, {12, 5}};
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);
  const float x = c.x(), y = c.y();

  auto material = [&](const QPointF *pts, int n) {
    QPolygonF poly;
    for (int i = 0; i < n; i++) {
      poly << QPointF(x + (pts[i].x() - 12) * s / 24, y + (pts[i].y() - 12) * s / 24);
    }
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    p.drawPolygon(poly);
  };

  switch (kind) {
    case PedalIcon::Accel: material(ARROW_UP, 9); break;
    case PedalIcon::Decel: material(ARROW_DOWN, 9); break;
    case PedalIcon::Boost:
      material(DOUBLE_UP_LOW, 6);
      material(DOUBLE_UP_HIGH, 6);
      break;
    case PedalIcon::Brake: {
      // (!) -- a ring with an exclamation mark between two side arcs, at
      // 0.8 of the box so its arcs sit as wide as the arrows' heads.
      const float h = s * 0.8f, r = h * 0.33f, pen = h * 0.10f;
      p.setPen(QPen(color, pen, Qt::SolidLine, Qt::RoundCap));
      p.setBrush(Qt::NoBrush);
      p.drawEllipse(QPointF(x, y), r, r);
      const float ra = h * 0.50f;
      const QRectF outer(x - ra, y - ra, 2 * ra, 2 * ra);
      p.drawArc(outer, 135 * 16, 90 * 16);    // left arc
      p.drawArc(outer, -45 * 16, 90 * 16);    // right arc
      p.setPen(Qt::NoPen);
      p.setBrush(color);
      const float bw = h * 0.10f;
      p.drawRoundedRect(QRectF(x - bw / 2, y - r * 0.62f, bw, r * 0.80f), bw / 2, bw / 2);
      p.drawEllipse(QPointF(x, y + r * 0.50f), bw * 0.62f, bw * 0.62f);
      break;
    }
  }
  p.restore();
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  paint_profile::sub_begin();
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();
  auto controls_state = sm["controlsState"].getControlsState();
  const bool stop_accel_boost_active = controls_state.getStopAccelBoostActive();

  // 하단 원형 2줄 시작점
  const int icon_start_x = 600;
  const int icon_step = radius + 50;
  const int row_gap = 25;

  // Keep the inter-row gap at half of its former 50 px value.
  int x = icon_start_x;
  const int y1 = rect().bottom() - footer_h / 2 - 10 + BOTTOM_ROW_DROP;
  const int y2 = y1 - radius - row_gap;

  // Wheel then steering bar, both on the upper row. The wheel is placed off
  // the speed panel's real right edge (drawSpeed publishes it, and it runs
  // first in paintGL) rather than off a column index -- at column 0 it
  // overlapped that panel. The bar takes whatever is left up to the row's
  // original right end, so it narrows instead of anything else moving.
  // The wheel and the bar sit two fifths of the row pitch below the upper row
  // -- a further 20% of the pitch down from the one fifth they started at, to
  // clear the tile rows above them. Expressed against the pitch rather than as
  // a pixel count so it keeps its proportion if radius or row_gap change.
  const int row_drop = (y1 - y2) * 2 / 5;
  // The bar alone then drops a further tenth of the pitch (on request); the
  // wheel stays on wheel_row_cy, the line both used to share.
  const int wheel_row_cy = y2 + row_drop;
  // The bar's centre line is the wheel's own centre (wheel_cy below, 865 on
  // 1080 px), so the two read as one unit. It used to sit 7 px higher.
  const int bar_cy = wheel_row_cy + (y1 - y2) * 3 / 20;

  constexpr int WHEEL_GAP = 22;
  // The wheel is the one tile that is not part of a grid of equals -- it sits
  // alone against the steering bar -- so it is sized 20% over the shared tile
  // diameter to read at a glance without disturbing any of the others. Its
  // own diameter, not `radius`, drives the anchors below, so the WHEEL_GAP
  // clearances hold at the new size and only the bar gives up the width.
  const int wheel_d = radius * 6 / 5;   // 116 -> 139
  // drawSteerGauge strokes the bar with a round cap, so the ink reaches half
  // the stroke width past the path's endpoint. Measure the gap from that
  // visual edge: at a 16 px gap measured from the endpoint the cap actually
  // overlapped the wheel by 7 px.
  constexpr int BAR_CAP = 60 / 2;   // BAR_W / 2 in drawSteerGauge
  // The bar and the gauge after it keep clear of the turn-info box always, not
  // only while the box shows, so nothing changes length mid-drive.
  constexpr int GAUGE_GAP = 22;
  constexpr int GAUGE_HALF_W = (64 + 10) / 2;   // BALL_D + BALL_PEN in drawConfidenceGauge
  constexpr int GAUGE_BOX_GAP = 20;
  const int wheel_cx = (speed_panel_right_ > 0
                        ? speed_panel_right_ + WHEEL_GAP + wheel_d / 2
                        : icon_start_x);
  const int bar_left = wheel_cx + wheel_d / 2 + WHEEL_GAP + BAR_CAP;
  // Half the length the bar had before the turn-info box arrived (on request,
  // 2026-11-22): its right cap centre was at the end of the icon row (1488 on
  // 1920 px), 725 px from bar_left; now 362, ink 733..1155. Still starting
  // WHEEL_GAP right of the wheel. Capped so the gauge after it stays
  // GAUGE_BOX_GAP clear of the turn-info box, which rightColumnLayout narrows
  // to make exactly this room (both land on 1125 at the current sizes).
  const int bar_right_orig = icon_start_x + (icon_step * 5) + radius / 2;
  const int bar_right_max = rightColumnLayout(width(), height()).turn_box.left()
                            - GAUGE_BOX_GAP - 2 * GAUGE_HALF_W - GAUGE_GAP - BAR_CAP;
  const int bar_right = std::min(bar_left + (bar_right_orig - bar_left) / 2, bar_right_max);
  if (bar_right > bar_left) {
    drawSteerGauge(p, (bar_left + bar_right) / 2, bar_cy, bar_right - bar_left);
  }

  // Confidence gauge right of the steering bar, centred on the wheel's line,
  // with the extent it had in the bottom icon rows (y2 - radius/2 ..
  // y1 + radius/2, drawn 1.2x about its centre).
  {
    const int half = ((y1 + radius / 2) - (y2 - radius / 2)) / 2;
    drawConfidenceGauge(p, bar_right + BAR_CAP + GAUGE_GAP + GAUGE_HALF_W, bar_cy - half, bar_cy + half);
  }
  paint_profile::sub_mark(paint_profile::BI_GAUGES);

  float cur_speed = std::max(0.0, car_state.getVEgo() * MS_TO_KPH);
  // The old shared str / str2 / textColor scratch locals are gone: every tile
  // now builds its own strings and colour. The commented-out blocks below
  // still reference them, so restoring one means declaring them again.

  /*
  // Previous steering-angle display (kept for easy restoration).
  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  float textSize = 48.f;
  textColor = QColor(255, 255, 255, 200);

  str.sprintf("%.0f°", steer_angle);
  configFont(p, "Open Sans", textSize, "Bold");
  textColor = QColor(255, 255, 255, 200);
  drawTextWithColor(p, x, y1 - 20, str, textColor);

  str2.sprintf("%.0f°", desire_angle);
  configFont(p, "Open Sans", textSize, "Bold");
  textColor = QColor(155, 255, 155, 200);
  drawTextWithColor(p, x, y1 + 50, str2, textColor);
  p.setOpacity(1.0);
  */

  // 1. PEDAL MAX -- value only; the tile is drawn with the right-hand status
  // block below, left of PEDAL LEVEL and at that block's size.
  // GM CarController clips the command sent to the comma pedal to 0.00..0.85.
  // actuatorsOutput.gas is the post-controller value that matches the CAN output.
  constexpr float comma_pedal_min = 0.0f;
  constexpr float comma_pedal_max = 0.85f;
  const float comma_pedal = std::clamp(car_control.getActuatorsOutput().getGas(),
                                        comma_pedal_min, comma_pedal_max);
  const float comma_pedal_ratio = (comma_pedal - comma_pedal_min) /
                                  (comma_pedal_max - comma_pedal_min);

  // 조향 신호 -- the steering torque actually sent to the EPS, 0..STEER_MAX
  // (300). It is actuatorsOutput, after the driver-override and rate limits,
  // so with a hand on the wheel it can read below the steering bar, which
  // shows the controller's request. Drawn as a ring tile beside 페달 상태.
  constexpr int STEER_MAX_COUNTS = 300;
  const float steer_ratio = std::min(1.0f, std::abs(car_control.getActuatorsOutput().getSteer()));
  const int steer_torque = static_cast<int>(std::lround(steer_ratio * STEER_MAX_COUNTS));

  // 2. VISION DIST -- hidden, kept for easy restoration.
  // The shared textSize local that used to live here is gone with the tiles
  // that read it (TR, PEDAL STATUS); those now size their own text.
  /*
  float textSize = 48.f;
  x = icon_start_x + (icon_step * 4);

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  auto lead_vision = sm["modelV2"].getModelV2().getLeadsV3()[0];
  float vision_dist = lead_vision.getProb() > .5 ? (lead_vision.getX()[0] - 1.5) : 0;
  //float vision_second = vision_dist / cur_speed;    // [거리 / 속력]

  // Orange Color if less than 15ｍ / Red Color if less than 5ｍ
  if (lead_vision.getProb()) {
    if (vision_dist < 15) {
      textColor = QColor(255, 127, 0, 200);
    } else if (vision_dist < 5) {
      textColor = QColor(255, 0, 0, 200);
    } else {
      textColor = QColor(120, 255, 120, 200);
    }
    str.sprintf("%.1f", vision_dist);
  } else {
    str = "──";
  }

  configFont(p, "Open Sans", 27, "Bold");
  drawText(p, x, y2-14, "DIST", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+35, str, textColor);
  p.setOpacity(1.0);
  */

  // 3. LKAS (swapped column with WHEEL, per user request)
  // carControl.latActive is the exact flag the GM CarController gates LKAS on
  // (apply_control_activation: steer faults, minSteerSpeed, standstill, angle
  // limit). Read it instead of re-deriving from speed -- the old
  // "cur_speed > 10" copy showed ON while steering was actually cut, and
  // duplicated GM_MIN_STEER_SPEED_KPH as a literal.
  //const bool lkas_bool = car_state.getLkasEnable();
  const bool lat_active = car_control.getLatActive();
  //const bool engaged = controls_state.getEnabled();
  const float min_steer_kph = sm["carParams"].getCarParams().getMinSteerSpeed() * MS_TO_KPH;

  // Value/colour only -- LKAS itself is drawn in the right-hand status column
  // stacked above the temperature panel, at the end of this function.
  QString lkas_str;
  QColor lkas_color;
  if (lat_active && cur_speed > min_steer_kph) {
    lkas_str = "ON";
    lkas_color = QColor(120, 255, 120, 200);
  }
  else if (lat_active && cur_speed < min_steer_kph) {
    // City stop-and-go: steering is cut by the speed gate and comes back on
    // its own above it. Amber, not red -- nothing is broken. This is checked
    // BEFORE the fault branch on purpose: the PSCM reports
    // LKATorqueDeliveredStatus == 2 (steerFaultTemporary) while stopped or
    // creeping simply because it is not delivering LKA torque there. In the
    // 2026-08-26 city drive every one of those 24.9s sat under 10 km/h, so
    // ranking the fault first would paint normal stop-and-go red.
    lkas_str = "OFF";
    lkas_color = QColor(255, 175, 0, 220);
  }
  else  {
    // City stop-and-go: steering is cut by the speed gate and comes back on
    // its own above it. Amber, not red -- nothing is broken. This is checked
    // BEFORE the fault branch on purpose: the PSCM reports
    // LKATorqueDeliveredStatus == 2 (steerFaultTemporary) while stopped or
    // creeping simply because it is not delivering LKA torque there. In the
    // 2026-08-26 city drive every one of those 24.9s sat under 10 km/h, so
    // ranking the fault first would paint normal stop-and-go red.
    lkas_str = "OFF";
    lkas_color = QColor(255, 175, 0, 220);
  }

  // 5. AI driving profiles
  x = icon_start_x + (icon_step * 4);

  /*
  // Previous CURV status display (disabled, kept for easy restoration).
  bool curv = controls_state.getCurvDriving();
  int curvSpeed = (int)controls_state.getCurvSpeed();

  QColor circleColor2;

  if (curv) {
    circleColor2 = QColor(0, 200, 83, 235);
  } else {
    circleColor2 = QColor(0, 0, 0, 235);
    curvSpeed = 0;
  }

  p.setPen(Qt::NoPen);
  p.setBrush(circleColor2);
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  QString curvText;
  if (curv) {
    curvText = "CURV:ON";
  } else {
    curvText = "CURV:OFF";
  }

  QColor curvColor = QColor(255, 255, 255, 200);
  configFont(p, "Open Sans", 40.f, "Bold");
  drawTextWithColor(p, x, y1 - 20, curvText, curvColor);

  QString strCurvSpeed = QString("%1km/h").arg(curvSpeed);
  configFont(p, "Open Sans", 40.f, "Bold");
  drawTextWithColor(p, x, y1 + 50, strCurvSpeed, curvColor);
  p.setOpacity(1.0);
  */

  // Community menu value:
  //   CommaPedalResistance = high | mid | low | auto
  // Refresh once per second instead of reading Params on every UI frame.
  // In AUTO the tile is relabelled 페달 AUTO and shows what the learner picked
  // (CommaPedalAutoStatus, written by controlsd), or its progress -- "9/15"
  // usable acceleration episodes -- while it is still learning on MID.
  static uint64_t last_ai_profile_update = 0;
  static QString ai_pedal_profile = "MID";
  static bool ai_pedal_auto = false;
  static bool ai_pedal_learning = false;
  const uint64_t ai_profile_now = millis_since_boot();

  if (pedal_profile_refresh_ || last_ai_profile_update == 0 ||
      ai_profile_now - last_ai_profile_update >= 1000) {
    last_ai_profile_update = ai_profile_now;
    pedal_profile_refresh_ = false;

    Params params;
    QString pedal_profile = QString::fromStdString(
      params.get("CommaPedalResistance")).trimmed().toUpper();
    ai_pedal_auto = pedal_profile == "AUTO";
    ai_pedal_learning = false;
    if (ai_pedal_auto) {
      const QString status = QString::fromStdString(
        params.get("CommaPedalAutoStatus")).trimmed();
      if (status.startsWith("learn ")) {
        pedal_profile = status.mid(6);
        ai_pedal_learning = true;
      } else {
        pedal_profile = status.toUpper();
        if (pedal_profile != "HIGH" && pedal_profile != "MID" && pedal_profile != "LOW") {
          pedal_profile = "0/15";
          ai_pedal_learning = true;
        }
      }
    } else if (pedal_profile != "HIGH" && pedal_profile != "MID" &&
               pedal_profile != "LOW") {
      pedal_profile = "MID";
    }
    ai_pedal_profile = pedal_profile;
    // 자동 출발 tile: Community "Activate stop accel boost", same refresh.
    auto_launch_on_ = params.getBool("ActiveStopAccelBoost");
  }

  // Colour encodes the level so the setting reads at a glance without having
  // to read the word. The tile itself is drawn with the right-hand status
  // block below, above ACC and at that block's size.
  QColor aiProfileColor;
  if (ai_pedal_learning) {
    aiProfileColor = QColor(160, 160, 160, 220);
  } else if (ai_pedal_profile == "HIGH") {
    aiProfileColor = QColor(254, 32, 32, 220);
  } else if (ai_pedal_profile == "MID") {
    aiProfileColor = QColor(255, 185, 15, 220);
  } else {
    aiProfileColor = QColor(255, 255, 255, 200);
  }


  // ================================================================================================================ //
  x = 140;
  x = icon_start_x;

  // 2. TR Value -- values only; the tile is drawn above the speed panel at
  // the end of this function.
  const float tr_value = controls_state.getDynamicTRValue();
  //int cruise_gap = car_state.getCruiseGap();
  // There is no profile name to show any more: the label is just "TR".
  const QString tr_label = "TR";
  QString tr_str;
  tr_str.sprintf("%.2f", tr_value);

  /*
  // 1. SPEED

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    textColor = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    textColor = QColor(255, a, a, 230);
  }

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y2-20, "SPEED", 200);

  str.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);*/

  // 1. PEDAL -- state and colour only; the tile is drawn above the speed
  // panel at the end of this function.
  //
  // Shown as an icon rather than a word (on request, 2026-11-25): 가속 green
  // up arrow, 감속 red down arrow, 브레이크 amber brake symbol (!), BOOST a
  // double up chevron. Shape and colour both differ, so it reads at a glance.
  //
  // It used to follow the planner's actuators.accel: > 0 가속, == 0 브레이크,
  // < 0 감속. On this car accel is never negative -- every decel request is
  // exactly 0 (the comma pedal cannot brake) -- so 감속 never showed and every
  // coast, slowdown and standstill showed 브레이크: 21% of the 11-26 drives
  // with no brake pressed. And the pedal follows the plan through its rise
  // limiter (0.4 s median, 1.5 s p90 behind an accel > 0), so 가속 led the car.
  // Now it shows what the car is doing: the driver's brake, the pedal command
  // actually sent (actuatorsOutput.gas) or the driver's gas, and otherwise
  // 감속 (coasting). 가속 comes on at once; 가속 -> 감속 waits PEDAL_DECEL_HOLD_MS
  // so a brief dip of the pedal does not flash the red arrow.
  constexpr float PEDAL_ON = 0.03f, PEDAL_OFF = 0.015f;
  constexpr double PEDAL_DECEL_HOLD_MS = 500.;
  static PedalIcon pedal_state = PedalIcon::Decel;
  static double pedal_low_since = -1.;
  const double pedal_now = millis_since_boot();
  const float pedal_out = car_control.getActuatorsOutput().getGas();
  if (pedal_out > PEDAL_OFF) {
    pedal_low_since = -1.;
  } else if (pedal_low_since < 0.) {
    pedal_low_since = pedal_now;
  }
  if (car_state.getBrakePressed()) {
    pedal_state = PedalIcon::Brake;
  } else if (stop_accel_boost_active) {
    pedal_state = PedalIcon::Boost;
  } else if (car_state.getGasPressed() || pedal_out > PEDAL_ON) {
    pedal_state = PedalIcon::Accel;
  } else if (pedal_state == PedalIcon::Accel || pedal_state == PedalIcon::Boost) {
    if (pedal_low_since >= 0. && pedal_now - pedal_low_since >= PEDAL_DECEL_HOLD_MS) {
      pedal_state = PedalIcon::Decel;
    }
  } else {
    pedal_state = PedalIcon::Decel;
  }
  const PedalIcon pedal_icon = pedal_state;
  QColor pedalStatusColor;
  switch (pedal_icon) {
    case PedalIcon::Accel: pedalStatusColor = QColor(120, 255, 120, 255); break;
    case PedalIcon::Brake: pedalStatusColor = QColor(255, 185, 15, 255); break;
    case PedalIcon::Boost: pedalStatusColor = QColor(225, 255, 239, 255); break;
    case PedalIcon::Decel:
    default:
      // Grey while openpilot is not engaged: nothing is being asked of the car.
      pedalStatusColor = controls_state.getEnabled() ? QColor(254, 32, 32, 255)
                                                     : QColor(200, 200, 200, 200);
      break;
  }

  // ACC is drawn in the right-hand status column below; only its state is
  // read here.
  const bool acc_bool = car_state.getAdaptiveCruise();

  /*// 5. long control state
  x = icon_start_x + (icon_step * 1);
  int longControlState = (int)controls_state.getLongControlState();
  const char* long_state[] = {"꺼짐", "켜짐", "정지", "출발"};
  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  str = long_state[longControlState];
  textColor = QColor(120, 255, 120, 200);

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y1-20, "LONG", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y1+50, str, textColor);
  p.setOpacity(1.0);*/

  // 5. STEER MAX -- hidden, kept for easy restoration. The same normalized
  // actuators.steer it scaled to 0..300 still drives the steering gauge on
  // the lane bar, so nothing is lost by hiding this tile.
  /*
  x = icon_start_x + (icon_step * 1);
  constexpr float steer_max = 300.0f;
  const float steer_command = std::clamp(std::abs(car_control.getActuators().getSteer()) * steer_max,
                                         0.0f, steer_max);
  const float steer_ratio = steer_command / steer_max;
  const QRectF steer_ring(x - radius / 2 + 7, y1 - radius / 2 + 7,
                          radius - 14, radius - 14);

  p.setPen(QPen(QColor(55, 61, 74, 255), 3));
  p.setBrush(QColor(55, 61, 74, 235));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  // Full-scale background ring and clockwise live-command ring.
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(118, 126, 139, 180), 9, Qt::SolidLine, Qt::FlatCap));
  p.drawEllipse(steer_ring);
  p.setPen(QPen(QColor(164, 210, 70, 255), 9, Qt::SolidLine, Qt::FlatCap));
  p.drawArc(steer_ring, 90 * 16, -static_cast<int>(steer_ratio * 360.0f * 16.0f));

  str = "STEER MAX";
  configFont(p, "Open Sans", 20, "Bold");
  drawText(p, x, y1 - 17, str, 230);

  str2.sprintf("%.0f", steer_command);
  textColor = QColor(255, 255, 255, 245);
  configFont(p, "Open Sans", 36, "Bold");
  drawTextWithColor(p, x, y1 + 21, str2, textColor);
  p.setOpacity(1.0);
  p.setBrush(Qt::NoBrush);
  p.setPen(Qt::NoPen);
  */

  // 6. STEER / DESIRE -- hidden, kept for easy restoration. A signed dual
  // ring plus the STEER/DESIRE readout; the WHEEL tile below still shows the
  // same live steeringAngleDeg.
  /*
  x = icon_start_x + (icon_step * 2);
  {
    constexpr float steer_desire_max_deg = 120.0f;
    const float steer_angle_deg = car_state.getSteeringAngleDeg();
    const float desire_angle_deg = car_control.getActuators().getSteeringAngleDeg();

    const QRectF steer_outer_ring(x - radius / 2 + 4, y2 - radius / 2 + 4, radius - 8, radius - 8);
    const QRectF steer_inner_ring(x - radius / 2 + 18, y2 - radius / 2 + 18, radius - 36, radius - 36);

    p.setPen(Qt::NoPen);
    p.setBrush(blackColor(200));
    p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

    // ISO 8855: positive angle = left turn. A physical wheel turning left
    // spins counter-clockwise, so fill the ring CCW (positive Qt span) for
    // positive angles and CW (negative span) for negative/right angles --
    // same sign convention as the WHEEL rotation below.
    auto draw_signed_ring = [&](const QRectF &ring_rect, const QColor &track_color,
                                 const QColor &fill_color, float angle_deg) {
      p.setBrush(Qt::NoBrush);
      p.setPen(QPen(track_color, 7, Qt::SolidLine, Qt::FlatCap));
      p.drawEllipse(ring_rect);
      const float ratio = std::clamp(std::abs(angle_deg) / steer_desire_max_deg, 0.0f, 1.0f);
      const float signed_span = (angle_deg >= 0.0f ? 1.0f : -1.0f) * ratio * 360.0f;
      p.setPen(QPen(fill_color, 7, Qt::SolidLine, Qt::FlatCap));
      p.drawArc(ring_rect, 90 * 16, static_cast<int>(signed_span * 16.0f));
    };

    draw_signed_ring(steer_outer_ring, QColor(255, 0, 0, 40), QColor(255, 0, 0, 255), steer_angle_deg);
    draw_signed_ring(steer_inner_ring, QColor(155, 255, 155, 46), QColor(155, 255, 155, 255), desire_angle_deg);

    p.setBrush(Qt::NoBrush);
    p.setPen(Qt::NoPen);

    str2.sprintf("STEER %.0f°", steer_angle_deg);
    textColor = QColor(120, 255, 120, 200);  // green
    configFont(p, "Open Sans", 32, "Bold");
    drawTextWithColor(p, x, y2 - 5, str2, textColor);

    str2.sprintf("DESIRE %.0f°", desire_angle_deg);
    textColor = QColor(120, 255, 120, 200);  // green
    configFont(p, "Open Sans", 32, "Bold");
    drawTextWithColor(p, x, y2 + 32, str2, textColor);
    p.setOpacity(1.0);
  }
  */

  // 7. WHEEL -- immediately left of the steering bar, on the same line. The
  // live steeringAngleDeg rendered as a physically rotating wheel icon
  // instead of a number, so it reads together with the bar's effort and
  // lane-centring rather than as a separate row. Not present in the original
  // source; uses the previously-unused ../assets/img_chffr_wheel.png.
  x = wheel_cx;
  {
    const float steer_angle_deg = car_state.getSteeringAngleDeg();
    const bool hands_on_wheel = car_state.getSteeringPressed();
    // The wheel sits three twentieths of the row pitch below the bar -- a
    // further 10% of the pitch down from the one twentieth it started at -- so
    // its centre sits clear of the bar's spine rather than near it.
    const int wheel_cy = wheel_row_cy + (y1 - y2) * 3 / 20;

    p.setPen(Qt::NoPen);
    // Green when hands-off (system driving alone), black when the driver is
    // holding the wheel -- gives an at-a-glance signal matching steeringPressed.
    p.setBrush(hands_on_wheel ? blackColor(220) : QColor(23, 134, 68, 220));
    p.drawEllipse(x - wheel_d / 2, wheel_cy - wheel_d / 2, wheel_d, wheel_d);

    p.save();
    p.translate(x, wheel_cy);
    // QPainter::rotate() is clockwise for positive angles, opposite of the
    // drawArc() convention above -- negate so positive (left) still spins CCW.
    p.rotate(-steer_angle_deg);
    p.setOpacity(1.0);
    p.drawPixmap(-ic_wheel.width() / 2, -ic_wheel.height() / 2, ic_wheel);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 59, 59, 255));
    p.drawEllipse(QPointF(0, -wheel_d * 0.42), 4.5, 4.5);
    p.restore();

    // Angle readout, centred under the wheel. The icon shows direction and
    // magnitude at a glance but never the number, so the exact angle goes
    // below it in the same badge as the lead-distance readout on the road --
    // green on a translucent black rounded rect -- so both live numbers read
    // as one kind of thing rather than two unrelated styles.
    //
    // The badge is sized from the widest string the tile can ever show
    // ("-888") rather than from the live text, so a changing angle never
    // resizes the box or shifts it sideways under the wheel.
    // The steering torque used to follow in brackets; it moved to the 조향 신호
    // ring tile beside 페달 상태.
    const QString angle_text = QString("%1°")
                                 .arg(static_cast<int>(std::lround(steer_angle_deg)));

    configFont(p, "Open Sans", 43, "Bold");
    QFontMetrics angle_fm(p.font());
    // The NDA badge above uses the same angle-only width as its minimum.
    const int badge_min_w = angle_fm.horizontalAdvance("-888°") + 38;
    const int angle_w = badge_min_w;
    const int angle_h = angle_fm.height() + 24;
    // Centred on the wheel, but kept off the speed panel to its left, the same
    // rule as the NDA badge: the torque value makes this box wider than the wheel.
    int angle_left = x - angle_w / 2;
    if (speed_panel_right_ > 0) angle_left = std::max(angle_left, speed_panel_right_ + 10);
    angle_left = std::clamp(angle_left, 12, std::max(12, width() - angle_w - 12));
    QRect angle_rect(angle_left, wheel_cy + wheel_d / 2 + 10, angle_w, angle_h);

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 160));
    p.drawRoundedRect(angle_rect, 10, 10);
    p.setPen(QColor(0, 255, 0, 255));
    p.drawText(angle_rect, Qt::AlignCenter, angle_text);
    p.setPen(Qt::NoPen);

    // NDA badge, centred above the wheel in the same style as the angle
    // badge below it. Shown only while TmapNda is connected. With no safety
    // event ahead it names the road (고속도로 / 일반도로; TmapNda sends the
    // road class only with its 자동 길안내 주행 option on, otherwise every road
    // reads 일반도로). With an event ahead it names the event and its
    // distance ("방지턱 85m", no limit), in yellow so it stands apart from the
    // plain road label.
    auto nda = sm["roadLimitSpeed"].getRoadLimitSpeed();
    if (nda.getActive() > 0) {
      const int cam_type = nda.getCamType();
      const int cam_limit = nda.getCamLimitSpeed();
      const int cam_dist = nda.getCamLimitSpeedLeftDist();
      const int sec_limit = nda.getSectionLimitSpeed();
      const int sec_dist = nda.getSectionLeftDist();

      auto dist_text = [](int m) {
        return m < 1000 ? QString("%1m").arg(m) : QString::number(m / 1000.0, 'f', 1) + "km";
      };

      QString nda_text;
      bool event = true;
      // Bump first, and only while the planner is approaching one (see
      // shownBumpDist). A bump in the first SDI slot otherwise shows nothing.
      const int bump_dist = shownBumpDist(sm);
      if (bump_dist >= 0) {
        nda_text = ndaSafetyLabel(SPEED_BUMP_CAM_TYPE) + " " + dist_text(bump_dist);
      } else if (cam_dist > 0 && cam_type != SPEED_BUMP_CAM_TYPE && (cam_type != 0 || cam_limit > 0)) {
        nda_text = ndaSafetyLabel(cam_type == 0 ? 1 : cam_type) + " " + dist_text(cam_dist);
      } else if (sec_limit > 0 && sec_dist > 0) {
        nda_text = "구간단속 " + dist_text(sec_dist);
      } else {
        nda_text = nda.getIsHighway() ? "고속도로" : "일반도로";
        event = false;
      }

      QFontMetrics nda_fm(p.font());
      const int nda_w = std::max(badge_min_w, nda_fm.horizontalAdvance(nda_text) + 38);
      const int nda_h = angle_h;
      // Centred on the wheel, but never over the speed panel to its left: a
      // Hangul label is wider than the wheel, and centred it ran 15-35 px into
      // that panel's right edge. It shifts right instead.
      int nda_left = x - nda_w / 2;
      if (speed_panel_right_ > 0) nda_left = std::max(nda_left, speed_panel_right_ + 10);
      nda_left = std::clamp(nda_left, 12, std::max(12, width() - nda_w - 12));
      QRect nda_rect(nda_left, wheel_cy - wheel_d / 2 - 10 - nda_h, nda_w, nda_h);

      p.setBrush(QColor(0, 0, 0, 160));
      p.drawRoundedRect(nda_rect, 10, 10);
      p.setPen(event ? QColor(255, 200, 0, 255) : QColor(0, 255, 0, 255));
      p.drawText(nda_rect, Qt::AlignCenter, nda_text);
      p.setPen(Qt::NoPen);
    }
  }
  // Includes the value/colour reads and the 1 s Params refresh above.
  paint_profile::sub_mark(paint_profile::BI_WHEEL);

  // 8. Right-hand status block, sitting above the temperature panel and
  // centred on it:
  //
  //     BRAKE   ACC
  //     HOLD    LKAS
  //
  // Laid out 2x2 rather than as one tall column so the block stays short --
  // two circles plus their gap still fit inside the panel's background width.
  // The split also puts the two icon tiles in the left column and the two
  // labelled text tiles in the right, so each column reads consistently.
  // Skipped if drawThermal has not run yet (it does, earlier in paintGL) so a
  // zero panel top can never park these at the top of the screen.
  if (thermal_panel_top_ > 0) {
    // The tiles no longer have to fit inside the temperature panel's 213 px
    // background; they only have to keep the panel's own 21 px margin off the
    // screen edge, so the block is right-aligned to the panel rather than
    // centred on it and overhangs to the left.
    //
    // Every tile in this block is 20% over the size it was, gaps and the
    // panel clearance included, so the group scales as one piece instead of
    // growing discs inside the old spacing. The label and value sizes follow
    // on their own: drawTileText fits both to the disc, and its two ceilings
    // below are raised by the same fifth so the short strings that were
    // already sitting on them grow too.
    //
    // The wheel is deliberately not part of this -- it is drawn from its own
    // wheel_d above and keeps the size it has, which the discs now match.
    //
    // At 140 the right-hand block runs 1605..1899 x 408..700 on 1920 x 1080,
    // with the thermal panel above it and the turn-info box below
    // (rightColumnLayout).
    constexpr int SD = 140;        // status circle diameter (116 * 1.2)
    constexpr int SGAP_X = 14;     // horizontal gap between the two columns
    constexpr int SGAP_Y = 12;     // vertical gap between the two rows
    constexpr int PANEL_GAP = 19;  // clearance above the temperature panel

    // Label over value, the pair centred on the disc by their actual ink.
    // Fixed baseline offsets scaled from the 116 px tiles do not survive the
    // move: once the label shrinks to fit, they leave a 17 px hole between
    // the two and push the value 17 px below the disc's centre.
    //
    // The label shrinks until it fits, because "PEDAL LEVEL" is four times
    // the width of "ACC" and a fixed 20 px would run off the edge. The value
    // is measured from "8" rather than the live string so a changing reading
    // can never shift the pair.
    // size_ref is the widest string this tile can ever show. Both the label
    // and the value are fitted to the disc, and the value is measured from
    // size_ref rather than the live text so switching state (ON -> OFF,
    // 가속 -> 브레이크) never resizes or reflows it.
    // The tile is a disc, so a flat width budget is the wrong shape: the two
    // lines sit at the top and bottom of the text block, where the disc has
    // narrowed, not across its widest part. Fitting each line to the chord at
    // its own height is what keeps the long labels inside their circle -- the
    // old flat SD - 8 budget already pushed "PEDAL LEVEL" 3 px past the disc
    // at SD 100, and would have pushed it 11 px past at 116.
    //
    // Both caps are ceilings, not fixed sizes: short values ("ON", "MID") take
    // the full 42 px, while a long size_ref settles well below it.
    //
    // A label carrying a newline is stacked over two lines, each measured at
    // its own height. That buys the label width but costs the value: the
    // taller block pushes the value down into the narrower part of the disc,
    // which is why 페달 상태 stayed on one line.
    // label_lift raises the label alone, as a fraction of the text block's own
    // height, so a tile whose value has been scaled up can buy the two lines
    // back their separation without moving the value or the disc.
    auto drawTileText = [&](int cx_, int cy, const QString &label,
                            const QString &value, const QColor &value_color,
                            int usable_r, int label_alpha,
                            const QString &size_ref, float value_scale,
                            float label_lift = 0.0f, float label_scale = 1.0f) {
      constexpr int TEXT_GAP = 8;   // between the label block and the value
      constexpr int LINE_GAP = 2;   // between two label lines
      const QStringList label_lines = label.split('\n');
      int label_pt = 34;   // ceiling, 28 * 1.2
      int value_pt = 50;   // ceiling, 42 * 1.2
      int label_h = 0, value_h = 0, total_h = 0;

      // Shrinking a line changes the block's height, which changes the chord
      // both lines are measured against, so this converges rather than solving
      // in one pass. The guard is a belt-and-braces bound: each turn drops a
      // size by one and both have floors, so it always terminates well before.
      // Widest the disc is at a line whose outer edge is |y| from the centre.
      const auto chord_at = [&](int y_outer) {
        const float k = (float)usable_r * usable_r - (float)y_outer * y_outer;
        return k > 1.0f ? (int)(2.0f * std::sqrt(k)) : 0;
      };

      // The fit below depends only on the label, size_ref, the disc radius and
      // the two scales -- never on the live value -- yet it ran on every paint:
      // up to 96 rounds of QFontMetrics and tightBoundingRect (glyph outlines)
      // per tile. On 2026-11-25 drawBottomIcons was 31 of the 48 ms a paint
      // took, which held the onroad view to ~8 fps. Fit once per key, reuse.
      struct TileFit { int label_pt, value_pt, label_h, value_h, total_h; };
      static auto *fit_cache = new QHash<QString, TileFit>;
      const QString fit_key = label + QChar(0x1f) + size_ref + QChar(0x1f) +
                              QString::number(usable_r) + QChar(0x1f) +
                              QString::number(value_scale) + QChar(0x1f) +
                              QString::number(label_scale);
      const auto cached_fit = fit_cache->constFind(fit_key);
      if (cached_fit != fit_cache->constEnd()) {
        label_pt = cached_fit->label_pt;
        value_pt = cached_fit->value_pt;
        label_h = cached_fit->label_h;
        value_h = cached_fit->value_h;
        total_h = cached_fit->total_h;
      } else {
        for (int guard = 0; guard < 96; guard++) {
          configFont(p, "Open Sans", label_pt, "Bold");
          const QFontMetrics fm_label(p.font());
          // One line height for every label line: they are all caps, so their
          // ink heights match and even spacing is what reads as a block.
          label_h = 0;
          for (const QString &line : label_lines) {
            label_h = std::max(label_h, fm_label.tightBoundingRect(line).height());
          }

          configFont(p, "Open Sans", value_pt, "Bold");
          const QFontMetrics fm_value(p.font());
          // Height from "8" rather than the live text, so a changing value can
          // never reflow the block.
          value_h = fm_value.tightBoundingRect("8").height();
          const int value_w = fm_value.width(size_ref);

          const int n = label_lines.size();
          total_h = label_h * n + LINE_GAP * (n - 1) + TEXT_GAP + value_h;
          const int half = total_h / 2;

          // Each line is measured where it actually sits: the second label line
          // is nearer the centre than the first, so it gets a wider chord even
          // though "STATUS" is the longer word.
          int label_over = -usable_r;
          for (int i = 0; i < n; i++) {
            const int line_top = -half + i * (label_h + LINE_GAP);
            const int outer = std::max(std::abs(line_top), std::abs(line_top + label_h));
            label_over = std::max(label_over,
                                  fm_label.width(label_lines[i]) - chord_at(outer));
          }
          const int value_over = value_w - chord_at(half);
          if (label_over <= 0 && value_over <= 0) {
            break;
          }
          // Give up whichever line overhangs more, so they close in together
          // instead of one starving the other.
          if (value_over >= label_over && value_pt > 14) {
            value_pt--;
          } else if (label_pt > 9) {
            label_pt--;
          } else if (value_pt > 14) {
            value_pt--;
          } else {
            break;
          }
        }

        // Same escape hatch as value_scale below, for the label. The two pedal
        // tiles need it: "PEDAL MAX" and "PEDAL LEVEL" are long enough that the
        // fit drives them down to 13 and 15 px, which is a caption rather than a
        // label you can read at a glance. Applied before value_scale so the
        // block height is recomputed once, from the final label size.
        if (label_scale != 1.0f) {
          label_pt = (int)(label_pt * label_scale + 0.5f);
          configFont(p, "Open Sans", label_pt, "Bold");
          const QFontMetrics fm_scaled(p.font());
          label_h = 0;
          for (const QString &line : label_lines) {
            label_h = std::max(label_h, fm_scaled.tightBoundingRect(line).height());
          }
          const int n = label_lines.size();
          total_h = label_h * n + LINE_GAP * (n - 1) + TEXT_GAP + value_h;
        }

        // Applied after the fit, so it deliberately breaks out of the disc. The
        // pedal-state tile needs it: its widest state, 브레이크, pins the value
        // at 24 px, which is too small to read at a glance while driving.
        if (value_scale != 1.0f) {
          value_pt = (int)(value_pt * value_scale + 0.5f);
          configFont(p, "Open Sans", value_pt, "Bold");
          value_h = QFontMetrics(p.font()).tightBoundingRect("8").height();
          const int n = label_lines.size();
          total_h = label_h * n + LINE_GAP * (n - 1) + TEXT_GAP + value_h;
        }
        fit_cache->insert(fit_key, TileFit{label_pt, value_pt, label_h, value_h, total_h});
      }

      const int top = cy - total_h / 2;

      const int lift = (int)(total_h * label_lift + 0.5f);

      configFont(p, "Open Sans", label_pt, "Bold");
      for (int i = 0; i < label_lines.size(); i++) {
        // drawText takes the baseline, which for these all-caps lines is the
        // bottom of the ink.
        drawText(p, cx_, top + (i + 1) * label_h + i * LINE_GAP - lift,
                 label_lines[i], label_alpha);
      }
      QColor c = value_color;
      configFont(p, "Open Sans", value_pt, "Bold");
      drawTextWithColor(p, cx_, top + total_h, value, c);
      p.setOpacity(1.0);
    };

    // value_scale > 1 lets the value spill past the disc on purpose; see the
    // note where it is applied.
    auto statusCircle = [&](int cx_, int cy, const QString &label,
                            const QString &value, const QColor &value_color,
                            const QString &size_ref, float value_scale = 1.0f,
                            float label_lift = 0.0f, float label_scale = 1.0f) {
      p.setPen(Qt::NoPen);
      p.setBrush(blackColor(200));
      p.drawEllipse(cx_ - SD / 2, cy - SD / 2, SD, SD);
      // A hair inside the disc's edge so the glyphs don't graze it.
      drawTileText(cx_, cy, label, value, value_color, SD / 2 - 3, 200, size_ref,
                   value_scale, label_lift, label_scale);
    };

    // Same tile as statusCircle plus a progress ring, so the live comma-pedal
    // command still reads as a filling arc rather than only as a number.
    auto statusRing = [&](int cx_, int cy, const QString &label,
                          const QString &value, float ratio,
                          const QString &size_ref,
                          float label_lift = 0.0f, float label_scale = 1.0f) {
      p.setPen(QPen(QColor(55, 61, 74, 255), 3));
      p.setBrush(QColor(55, 61, 74, 235));
      p.drawEllipse(cx_ - SD / 2, cy - SD / 2, SD, SD);

      const QRectF ring(cx_ - SD / 2 + 7, cy - SD / 2 + 7, SD - 14, SD - 14);
      p.setBrush(Qt::NoBrush);
      p.setPen(QPen(QColor(118, 126, 139, 180), 8, Qt::SolidLine, Qt::FlatCap));
      p.drawEllipse(ring);
      p.setPen(QPen(QColor(255, 0, 0, 255), 8, Qt::SolidLine, Qt::FlatCap));
      p.drawArc(ring, 90 * 16,
                -static_cast<int>(std::clamp(ratio, 0.0f, 1.0f) * 360.0f * 16.0f));

      // Inside the ring, not just inside the disc: the ring's circle is inset
      // 7 and stroked 8, so its inner edge is at SD / 2 - 11.
      drawTileText(cx_, cy, label, value, QColor(255, 255, 255, 245), SD / 2 - 15, 230,
                   size_ref, 1.0f, label_lift, label_scale);
      p.setBrush(Qt::NoBrush);
      p.setPen(Qt::NoPen);
    };

    // Right edge flush with the panel below, so the block keeps that panel's
    // margin off the screen edge and grows to the left. Centred on the panel
    // it would come within 6 px of the edge.
    const int block_cx = thermal_panel_right_ > 0
                         ? thermal_panel_right_ - SD - SGAP_X / 2
                         : thermal_panel_cx_;
    const int col_r = block_cx + (SD + SGAP_X) / 2;
    // Anchored from the bottom so the block always grows away from the panel.
    // Two rows of icon/state pairs; the pedal tiles moved over to the speed
    // panel's group, where they sit with the other pedal readings.
    // The block now sits on top of the turn-info box with the thermal panel
    // above it (rightColumnLayout); it used to stand on the thermal panel.
    const int row_b = rightColumnLayout(width(), height()).status_bottom - SD / 2;
    const int row_t = row_b - SD - SGAP_Y;

    // The brake and auto-hold icons that filled the left column are gone (on
    // request, 2026-11-26); ACC and LKAS keep their place on the right.
    statusCircle(col_r, row_t, "ACC", acc_bool ? "ON" : "OFF",
                 acc_bool ? QColor(120, 255, 120, 200) : QColor(254, 32, 32, 200), "OFF");
    statusCircle(col_r, row_b, "LKAS", lkas_str, lkas_color, "OFF");
    paint_profile::sub_mark(paint_profile::BI_STATUS);

    // The four pedal/following readings ride above the NDA/HDA badge, at the
    // same size and horizontal gap as the block on the right -- so the two
    // groups read as one system rather than two sizes of tile. Anchored from
    // the badge up, so the pair nearest it stays put and the pedal pair stacks
    // above that. Skipped until drawSpeedLimit has published the badge's box
    // (it runs earlier in drawHud, and publishes even when it draws nothing).
    if (nda_badge_top_ > 0) {
      const int speed_row = nda_badge_top_ - PANEL_GAP - SD / 2;
      const int speed_row_top = speed_row - SD - SGAP_Y;
      // Centred on the badge, but pushed right if that would take the left
      // column off the screen. The badge sits at x 30 with its centre at 135,
      // so a pair of 116 px discs cleared the edge by 13 px and a pair of 140
      // px ones does not -- centred, the left disc would start at -12. The
      // group therefore holds the badge's centre until it cannot, then keeps
      // the badge's own 30 px margin instead and overhangs to the right, the
      // same trade the right-hand block makes against the screen edge.
      constexpr int EDGE_MARGIN = 30;
      const int group_w = SD * 2 + SGAP_X;
      const int group_cx = std::max(nda_badge_cx_, EDGE_MARGIN + group_w / 2);
      const int scol_l = group_cx - (SD + SGAP_X) / 2;
      const int scol_r = group_cx + (SD + SGAP_X) / 2;

      QString pedal_max_str;
      pedal_max_str.sprintf("%.0f", comma_pedal * 100.0f);
      // 페달 신호 is matched to 페달 상태 the same way 페달 강도 is, so all
      // four tiles in this group carry one label size and one value size.
      // The scale is chosen for the rendered result, not copied: this tile
      // fits inside the progress ring rather than the disc, and measures its
      // value against a two-digit "88", so its own fit lands well below the
      // others and 1.40 is what brings it level with them. Every factor here
      // multiplies that tile's own fitted size, so they ride the disc: the
      // group scaled to SD 140 without one of them being retuned.
      statusRing(scol_l, speed_row_top, "페달 신호", pedal_max_str,
                 comma_pedal_ratio, "88", 0.10f, 1.40f);
      pedal_signal_tile_rect_ = QRect(scol_l - SD / 2, speed_row_top - SD / 2, SD, SD);
      // 페달 강도 sits below 페달 상태 (swapped on request). It is matched to
      // 페달 상태 above it, so the right-hand column
      // reads as one pair: same Korean label, same tenth-of-a-block lift, and
      // a value that renders at the same size.
      //
      // Matched on the rendered size, not on the 1.60 factor. The factor
      // multiplies each tile's own fitted size, and the fit depends on
      // size_ref: 페달 상태 measures against the wide 브레이크 and settles
      // small, this tile measures against the much narrower HIGH and settles
      // high. The same 1.60 here would render visibly larger than its pair,
      // spill well past its own disc and reach the PEDAL MAX disc beside it.
      // 1.17 lands level with the pair and clears that disc.
      statusCircle(scol_r, speed_row, ai_pedal_auto ? "페달 AUTO" : "페달 강도", ai_pedal_profile,
                   aiProfileColor, "HIGH", 1.17f, 0.10f);
      pedal_tile_rect_ = QRect(scol_r - SD / 2, speed_row - SD / 2, SD, SD);
      pedal_tile_drawn_t_ = millis_since_boot();

      // 자동 출발, right of 페달 강도 on its row (below 조향 신호). Shows and,
      // on a tap, flips the Community "Activate stop accel boost" toggle
      // (ActiveStopAccelBoost). Matched to 페달 강도: same size_ref and scale.
      // On the 1880 x 1040 HUD it spans 338..478 x 455..595; the left blinker
      // band starts right of it (drawTurnSignals).
      const int auto_launch_x = scol_r + SD + SGAP_X;
      statusCircle(auto_launch_x, speed_row, "자동 출발", auto_launch_on_ ? "ON" : "OFF",
                   auto_launch_on_ ? QColor(120, 255, 120, 220) : QColor(254, 32, 32, 220),
                   "HIGH", 1.17f, 0.10f);
      auto_launch_tile_rect_ = QRect(auto_launch_x - SD / 2, speed_row - SD / 2, SD, SD);
      auto_launch_tile_drawn_t_ = millis_since_boot();

      statusCircle(scol_l, speed_row,
                   tr_label, tr_str, QColor(120, 255, 120, 200), "8.88");
      // 페달 상태: no label, a ring in the state's colour and one large
      // standard icon (design G, chosen 2026-11-25), so it reads at a glance.
      {
        p.setPen(Qt::NoPen);
        p.setBrush(blackColor(200));
        p.drawEllipse(scol_r - SD / 2, speed_row_top - SD / 2, SD, SD);
        p.save();
        p.setRenderHint(QPainter::Antialiasing);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(pedalStatusColor, 8));
        p.drawEllipse(QPointF(scol_r, speed_row_top), SD / 2 - 6, SD / 2 - 6);
        p.restore();
        drawPedalIcon(p, QPointF(scol_r, speed_row_top), SD * 5 / 7, pedal_icon,
                      pedalStatusColor);
      }

      // 조향 신호, right of 페달 상태 on the same row and in the same style as
      // 페달 신호: red ring filling with the torque sent to the EPS (0..300),
      // the value in the middle. Sized against "300" so a three-digit value
      // stays inside the ring.
      statusRing(scol_r + SD + SGAP_X, speed_row_top, "조향 신호",
                 QString::number(steer_torque), steer_ratio, "300", 0.10f, 1.40f);
      paint_profile::sub_mark(paint_profile::BI_TILES);
    }
  }
}

/*
void NvgWindow::drawSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  // std::max 타입 에러 방지(전부 float로 통일)
  float v_ego = sm["carState"].getCarState().getVEgo();
  float conv = s->scene.is_metric ? (float)MS_TO_KPH : (float)MS_TO_MPH;
  float cur_speed = std::max(0.0f, v_ego * conv);

  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color(255, 255, 255, 230);
  if (accel > 0) {
    int a = (int)(255.f - (180.f * (accel / 2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230);
  } else {
    int a = (int)(255.f - (255.f * (-accel / 3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230);
  }

  // 위치(기존과 동일)
  const int x = rect().center().x() - 150;
  const int y_speed = 460;
  const int y_unit  = 540;

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  const QString unit = s->scene.is_metric ? "km/h" : "mph";

  // =========================
  // 고정 배경(템플릿 기준) + 폭 20% 확대
  // =========================
  const QString speed_template = "888";   // 3자리 폭 기준(고정)
  const QString unit_template  = "km/h";  // 둘 중 긴 쪽 기준(고정)

  // 템플릿으로 "기준 배경" 계산
  configFont(p, "Open Sans", 176, "Bold");
  QFontMetricsF fmSpeed(p.font());
  QRectF rSpeedT = fmSpeed.boundingRect(speed_template);
  QRectF speedRectT(x - rSpeedT.width() / 2.0,
                    y_speed - fmSpeed.ascent(),
                    rSpeedT.width(),
                    fmSpeed.height());

  configFont(p, "Open Sans", 66, "Regular");
  QFontMetricsF fmUnit(p.font());
  QRectF rUnitT = fmUnit.boundingRect(unit_template);
  QRectF unitRectT(x - rUnitT.width() / 2.0,
                   y_unit - fmUnit.ascent(),
                   rUnitT.width(),
                   fmUnit.height());

  // 템플릿 두 줄을 감싸는 기본 배경 + 패딩
  QRectF bgBase = speedRectT.united(unitRectT).adjusted(-28, -18, 28, 18);

  // 폭 20% 확대 + (고정) 센터 유지
  const qreal w = bgBase.width() * 1.2;
  const qreal h = bgBase.height();        // 높이도 고정(원하시면 *1.1 같은 조절 가능)
  const QPointF c = bgBase.center();
  QRectF bgFixed(c.x() - w / 2.0, c.y() - h / 2.0, w, h);

  // ---- 2) 반투명 검정 배경(바깥 레이어) ----
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 160));
  p.drawRoundedRect(bgFixed, 22, 22);

  // ---- 3) 텍스트(안쪽 레이어) ----
  configFont(p, "Open Sans", 176, "Bold");
  drawTextWithColor(p, x, y_speed, speed, color);

  configFont(p, "Open Sans", 66, "Regular");
  drawText(p, x, y_unit, unit, 200);

  p.restore();
}*/

void NvgWindow::drawSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  // -------------------------
  // Current speed value
  // -------------------------
  const auto car_state = sm["carState"].getCarState();
  const float v_ego = car_state.getVEgo();
  // This compact driving panel is always expressed in km/h. carState.vEgo
  // is m/s, while cruiseMaxSpeed/applyMaxSpeed below are already km/h.
  const float cur_speed = std::max(0.0f, v_ego * (float)MS_TO_KPH);

  const float accel = car_state.getAEgo();

  // speedColor (가감속에 따라 변화)
  QColor speedColor(255, 255, 255, 230);
  if (accel > 0) {
    int a = (int)(255.f - (180.f * (accel / 2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    speedColor = QColor(a, a, 255, 230);
  } else {
    int a = (int)(255.f - (255.f * (-accel / 3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    speedColor = QColor(255, a, a, 230);
  }

  // -------------------------
  // Main speed position (기존 동일)
  // -------------------------
  const int x = rect().center().x() - 150;
  const int speed_y_offset = 400;
  const int y_speed = 460 + speed_y_offset;
  const int y_unit  = 540 + speed_y_offset;

  QString speed;
  speed.sprintf("%.0f", cur_speed);

  // -------------------------
  // Speed background (template-based)
  // -------------------------
  const QString speed_template = "888";
  const QString unit_template  = "km/h";

  configFont(p, "Open Sans", 176, "Bold");
  QFontMetricsF fmSpeed(p.font());
  QRectF rSpeedT = fmSpeed.boundingRect(speed_template);
  QRectF speedRectT(x - rSpeedT.width() / 2.0,
                    y_speed - fmSpeed.ascent(),
                    rSpeedT.width(),
                    fmSpeed.height());

  configFont(p, "Open Sans", 66, "Regular");
  QFontMetricsF fmUnit(p.font());
  QRectF rUnitT = fmUnit.boundingRect(unit_template);
  QRectF unitRectT(x - rUnitT.width() / 2.0,
                   y_unit - fmUnit.ascent(),
                   rUnitT.width(),
                   fmUnit.height());

  QRectF bgBase = speedRectT.united(unitRectT).adjusted(-28, -18, 28, 18);

  // 폭 20% 확대 + 센터 유지
  const qreal bgW = bgBase.width() * 1.2;
  const qreal bgH = bgBase.height();
  const QPointF bgC = bgBase.center();
  QRectF bgFixed(bgC.x() - bgW / 2.0, bgC.y() - bgH / 2.0, bgW, bgH);

  // -------------------------
  // Background color (30% brighter)
  // -------------------------
  QColor bgBright30(77, 77, 77, 160);

  // ✅ 패널 배경을 10% 더 투명하게 (alpha 160 -> 144)
  QColor panelBgColor(77, 77, 77, 144);

  // -------------------------
  // Cruise/Apply panel (left)
  // -------------------------
  const auto controls_state = sm["controlsState"].getControlsState();
  const float applyMaxSpeed_kph  = controls_state.getApplyMaxSpeed();
  const float cruiseMaxSpeed_kph = controls_state.getCruiseMaxSpeed();
  const bool is_cruise_set = (cruiseMaxSpeed_kph > 0.f && cruiseMaxSpeed_kph < 255.f);

  auto to_display_speed = [](float kph) -> int {
    if (kph <= 0.f) return 0;
    return (int)(kph + 0.5f);
  };

  // ✅ 패널 폭 10% 증가 (콘텐츠 폭 기준으로 같이 확대)
  // 2026-11-22: 20% narrower on request (bg 290 -> 232 px). The right edge
  // stays where it was, so the wheel, the steering bar and everything else
  // anchored to speed_panel_right_ keeps its place; only the left edge moves
  // in (261 -> 319 on 1920 px). The widest text, the current speed at 100 px
  // ("888" 198 px in Inter Bold, from the font's advance widths), still fits
  // with ~17 px each side.
  constexpr qreal PANEL_W_SCALE = 0.8;
  const qreal panel_content_w = 220.0 * 1.10 * PANEL_W_SCALE;
  const qreal panel_content_h = bgFixed.height();
  const qreal panel_bg_w = panel_content_w * 1.2;
  const qreal panel_bg_h = panel_content_h;

  // How far below the reference box the panel sits: the bottom row's shared
  // drop (see BOTTOM_ROW_DROP).
  const qreal group_drop = BOTTOM_ROW_DROP;

  // The left edge is pinned to the speed-limit board rather than derived from
  // the reference box: SPEED_BOARD_PANEL_GAP right of the board (x 240 on the
  // 1880 px HUD). The reference box is still what sets the panel's height.
  QRectF panelBg(SPEED_BOARD_X + SPEED_BOARD_W + SPEED_BOARD_PANEL_GAP,
                 bgFixed.top() + group_drop,
                 panel_bg_w,
                 panel_bg_h);

  QRectF panelContent(panelBg.center().x() - panel_content_w / 2.0,
                      panelBg.top(),
                      panel_content_w,
                      panel_content_h);

  // Publish the panel box for the tiles drawBottomIcons stacks above it
  // (see the members' comment in onroad.h).
  speed_panel_top_ = (int)panelBg.top();
  speed_panel_cx_ = (int)panelBg.center().x();
  speed_panel_right_ = (int)panelBg.right();
  speed_panel_bottom_ = (int)std::lround(panelBg.top() + panelBg.height());

  // 패널 배경 (✅ 더 투명)
  p.setPen(Qt::NoPen);
  p.setBrush(panelBgColor);
  p.drawRoundedRect(panelBg, 22, 22);

  const int panel_cx = (int)panelContent.center().x();

  // 3줄을 패널 세로 중앙 기준으로 균등 배치.
  const int panel_cy = (int)panelContent.center().y();
  const int lineGap = (int)(panelContent.height() * 0.33);

  // 폰트
  const int unifiedFont = 70;     // Cruise/Apply
  const int unifiedSpdFont = 100;  // Current Speed

  // drawTextWithColor 는 y 를 베이스라인으로 쓴다. 숫자를 원하는 높이에
  // "가운데" 놓으려면 자기 글자 높이의 절반만큼 내려야 한다. 높이 기준은
  // 실제 문자열이 아니라 "8" 로 고정해서, 값이 바뀌어도 줄이 흔들리지 않게 한다.
  auto baselineAt = [&](int center_y) {
    QFontMetrics fm(p.font());
    return center_y + fm.boundingRect("8").height() / 2;
  };

  // Colors
  QColor cruiseGreen(120, 255, 120, 200);
  QColor naWhite(255, 255, 255, 180);
  QColor applyOrange(255, 127, 0, 200);

  // Cruise
  QString strCruise;
  configFont(p, "Inter", unifiedFont, "Bold");
  const int y_cruise = baselineAt(panel_cy - lineGap);
  if (is_cruise_set) {
    strCruise.sprintf("%d", to_display_speed(cruiseMaxSpeed_kph));
    drawTextWithColor(p, panel_cx, y_cruise, strCruise, cruiseGreen);
  } else {
    strCruise = "N/A";
    drawTextWithColor(p, panel_cx, y_cruise, strCruise, naWhite);
  }

  // Current Speed (숫자만, speedColor 유지)
  QColor curSpeedColor = speedColor;
  QString strCur;
  strCur.sprintf("%d", (int)(cur_speed + 0.5f));
  configFont(p, "Inter", unifiedSpdFont, "Bold");
  // 가운데 줄만 줄 간격의 1/20 만큼 위로. 픽셀 상수가 아니라 간격 비율이라
  // 패널 높이가 바뀌어도 비율이 유지된다.
  const int cur_speed_lift = lineGap / 20;
  drawTextWithColor(p, panel_cx, baselineAt(panel_cy - cur_speed_lift), strCur, curSpeedColor);

  // Apply
  QString strApply;
  if (is_cruise_set && applyMaxSpeed_kph > 0.f) {
    strApply.sprintf("%d", to_display_speed(applyMaxSpeed_kph));
  } else {
    strApply = "MAX";
  }
  configFont(p, "Inter", unifiedFont, "Bold");
  drawTextWithColor(p, panel_cx, baselineAt(panel_cy + lineGap), strApply, applyOrange);

  p.restore();
}






QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

/*
void NvgWindow::drawSpeedLimit(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

  const auto controls_state = sm["controlsState"].getControlsState();

  float applyMaxSpeed = controls_state.getApplyMaxSpeed();
  float cruiseMaxSpeed = controls_state.getCruiseMaxSpeed();
  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  int activeNDA = roadLimitSpeed.getActive();
  int roadLimit_Speed = roadLimitSpeed.getRoadLimitSpeed();
  int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();
  int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
  int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0)
  {
      int w = 120;
      int h = 54;
      //int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      //int y = 40 - bdr_s;
      int y = 80 - bdr_s;

      p.setOpacity(1.f);
      //p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
      p.drawPixmap(280, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
  }

  const int x_start = 30;
  const int y_start = 30;

  int board_width = 210;
  int board_height = 384;

  const int corner_radius = 32;
  int max_speed_height = 210;

  QColor bgColor = QColor(0, 0, 0, 166);

  {
    // draw board
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);

    if(limit_speed > 0 && left_dist > 0) {
      board_width = limit_speed < 100 ? 210 : 230;
      board_height = max_speed_height + board_width;

      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height-board_width/2), corner_radius, corner_radius);
      path.addRoundedRect(QRectF(x_start, y_start+corner_radius, board_width, board_height-corner_radius), board_width/2, board_width/2);
    }
    else if(roadLimit_Speed > 0 && roadLimit_Speed < 200) {
      board_height = 485;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }
    else {
      max_speed_height = 235;
      board_height = max_speed_height;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }

    p.setPen(Qt::NoPen);
    p.fillPath(path.simplified(), bgColor);
  }

  QString str;

  // Max Speed
  {
    p.setPen(QColor(255, 255, 255, 230));

    if(is_cruise_set) {
      configFont(p, "Inter", 80, "Bold");
      str.sprintf( "%d", (int)(cruiseMaxSpeed + 0.5));
    }
    else {
      configFont(p, "Inter", 60, "Bold");
      str = "N/A";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 35);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }


  // applyMaxSpeed
  {
    p.setPen(QColor(255, 255, 255, 180));

    configFont(p, "Inter", 50, "Bold");
    if(is_cruise_set && applyMaxSpeed > 0) {
      str.sprintf( "%d", (int)(applyMaxSpeed + 0.5));
    }
    else {
      str = "MAX";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start + max_speed_height/2, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 24);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }

  //
  if(limit_speed > 0 && left_dist > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = 18;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, 25));
    p.drawEllipse(board_rect);

    p.setPen(QPen(Qt::black, padding));

    str.sprintf("%d", limit_speed);
    configFont(p, "Inter", 70, "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    // left dist
    QRect rcLeftDist;
    QString strLeftDist;

    if(left_dist < 1000)
      strLeftDist.sprintf("%dm", left_dist);
    else
      strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

    QFont font("Inter");
    font.setPixelSize(55);
    font.setStyleName("Bold");

    QFontMetrics fm(font);
    int width = fm.width(strLeftDist);

    padding = 10;

    int center_x = x_start + board_width / 2;
    rcLeftDist.setRect(center_x - width / 2, y_start+board_height+15, width, font.pixelSize()+10);
    rcLeftDist.adjust(-padding*2, -padding, padding*2, padding);

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rcLeftDist, 20, 20);

    configFont(p, "Inter", 55, "Bold");
    p.setBrush(Qt::NoBrush);
    p.setPen(QColor(255, 255, 255, 230));
    p.drawText(rcLeftDist, Qt::AlignCenter|Qt::AlignVCenter, strLeftDist);
  }
  else if(roadLimit_Speed > 0 && roadLimit_Speed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", 35, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimit_Speed);
      configFont(p, "Inter", 75, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
  }

  p.restore();
}*/

// Speed bump warning sign (red rounded triangle, amber face, black bump),
// drawn as vectors so it needs no image asset and stays sharp at any size.
// Coordinates are in a 1920 x 1695 design box (the reference sign's own
// proportions) and are fitted, centred, into `area`.
static void drawSpeedBumpSign(QPainter &p, const QRectF &area) {
  const qreal DW = 1920.0, DH = 1695.0;
  const qreal s = std::min(area.width() / DW, area.height() / DH);
  const QPointF o(area.center().x() - DW * s / 2, area.center().y() - DH * s / 2);
  auto P = [&](qreal x, qreal y) { return QPointF(o.x() + x * s, o.y() + y * s); };

  p.save();
  p.setRenderHint(QPainter::Antialiasing, true);

  // Rounded corners: fill a smaller triangle and stroke it with a thick,
  // round-joined pen of the same colour.
  const QPolygonF red_tri(QVector<QPointF>{P(960, 147), P(120, 1600), P(1800, 1600)});
  auto rounded = [&](const QColor &c, qreal pen_w) {
    QPen pen(c, pen_w * s, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(c);
    p.drawPolygon(red_tri);
  };
  rounded(Qt::white, 230);              // thin white rim
  rounded(QColor(0xD3, 0x1F, 0x25), 180);

  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0xF8, 0xA7, 0x0F));
  p.drawPolygon(QPolygonF(QVector<QPointF>{P(960, 259), P(240, 1505), P(1680, 1505)}));

  // Bump: a base bar with a dome on top.
  QPainterPath bump;
  bump.moveTo(P(452, 1321));
  bump.lineTo(P(590, 1321));
  bump.cubicTo(P(700, 1175), P(830, 1135), P(960, 1135));
  bump.cubicTo(P(1090, 1135), P(1220, 1175), P(1329, 1321));
  bump.lineTo(P(1468, 1321));
  bump.lineTo(P(1468, 1413));
  bump.lineTo(P(452, 1413));
  bump.closeSubpath();
  p.setBrush(Qt::black);
  p.drawPath(bump);

  p.restore();
}

// Parking enforcement sign (red ring and slash over a black "P", the slash
// broken around the letter), drawn as vectors in a 512 x 512 design box
// fitted, centred, into `area`.
static void drawParkingSign(QPainter &p, const QRectF &area) {
  const qreal D = 512.0;
  const qreal s = std::min(area.width(), area.height()) / D;
  const QColor face(0xF8, 0xF8, 0xF8);
  const QColor red(0xFF, 0x2D, 0x2D);

  p.save();
  p.setRenderHint(QPainter::Antialiasing, true);
  p.translate(area.center().x() - D * s / 2, area.center().y() - D * s / 2);
  p.scale(s, s);

  // Face
  p.setPen(Qt::NoPen);
  p.setBrush(face);
  p.drawEllipse(QPointF(256, 256), 226, 226);

  // Slash, upper-left to lower-right. Its ends run under the ring.
  p.setPen(QPen(red, 36, Qt::SolidLine, Qt::FlatCap));
  p.drawLine(QPointF(112, 124), QPointF(388, 400));

  // "P", fitted to its box regardless of the font's metrics. A face-coloured
  // stroke first cuts the slash's gap around the letter.
  QFont font("Inter");
  font.setPixelSize(300);
  font.setBold(true);
  QPainterPath glyph;
  glyph.addText(0, 0, font, "P");
  const QRectF g = glyph.boundingRect();
  const QRectF box(190, 140, 175, 225);
  QTransform fit;
  fit.translate(box.left(), box.top());
  fit.scale(box.width() / g.width(), box.height() / g.height());
  fit.translate(-g.left(), -g.top());
  glyph = fit.map(glyph);

  p.setPen(QPen(face, 56, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(face);
  p.drawPath(glyph);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0x1E, 0x1E, 0x1E));
  p.drawPath(glyph);

  // Ring last, so the slash ends and the gap stroke stay beneath it.
  p.setPen(QPen(red, 36));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(QPointF(256, 256), 190, 190);

  p.restore();
}

// SpeedLimit만 표시하고, Cruise/Apply(=MaxSpeed 박스)는 완전히 제거
// + 보드 위치20% 아래 이동(클램프)
// + roadLimit_Speed(SPEED LIMIT 박스)일 때 NDA/HDA 아이콘을 박스 위에 표시
void NvgWindow::drawSpeedLimit(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

  const int activeNDA = roadLimitSpeed.getActive();
  const int roadLimit_Speed = roadLimitSpeed.getRoadLimitSpeed();
  const int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
  const int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();
  const int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
  const int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  // A bump in the first SDI slot is drawn as the bump sign below, never as a
  // "10" camera board (road_speed_limiter fills its limit with 10).
  const bool cam_is_bump = roadLimitSpeed.getCamType() == SPEED_BUMP_CAM_TYPE;

  if (!cam_is_bump && camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  } else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  // Speed bump ahead, only while the planner is approaching one.
  const int bump_dist = shownBumpDist(sm);
  const bool show_bump = bump_dist >= 0;

  // Parking enforcement camera ahead (first SDI slot). It carries no speed
  // limit, so it never reached the camera board before.
  const bool show_parking = !show_bump && activeNDA > 0 &&
                            roadLimitSpeed.getCamType() == PARKING_CAM_TYPE && camLimitSpeedLeftDist > 0;

  // A sign (bump, then parking) takes the board's place; the camera/section
  // board returns once it is gone.
  const bool show_sign = show_bump || show_parking;
  const int sign_dist = show_bump ? bump_dist : camLimitSpeedLeftDist;
  const bool show_cam_or_section = !show_sign && (limit_speed > 0 && left_dist > 0);
  const bool show_road = (roadLimit_Speed > 0 && roadLimit_Speed < 200);

  // ============================================================
  // ✅ 15% 확대 스케일
  // ============================================================
  const float k = 1.00f;
  auto S = [&](int v) -> int { return (int)std::lround(v * k); };

  // ---- layout base ----
  const int x_start = SPEED_BOARD_X;
  const int base_y_start = 70;        // 위치는 유지
  const int corner_radius = S(32);
  const QColor bgColor(0, 0, 0, 166);

  // ---- 20% 아래로 이동 (화면 하단 넘어가면 clamp) + 현재 보이던 위치에서 400 더 내리는 것----
  // + BOTTOM_ROW_DROP with the rest of the bottom row.
  const int desired_shift = (int)std::lround(height() * 0.20f) + 400 + BOTTOM_ROW_DROP;

  // Publish where the NDA/HDA badge sits, for the pedal tiles drawBottomIcons
  // stacks above it (see the members' comment in onroad.h). Always measured
  // against the road-limit board's geometry, and published before the early
  // return below, so the tiles hold one position instead of jumping between
  // board types or vanishing on a road with no published limit.
  {
    const int road_needed_h = S(275);
    const int road_max_shift = std::max(0, height() - (base_y_start + road_needed_h) - S(20));
    const int road_y_start = base_y_start + std::clamp(desired_shift, 0, road_max_shift);
    nda_badge_top_ = std::max(0, road_y_start - S(54) - S(10));
    nda_badge_cx_ = x_start + S(210) / 2;
  }

  if (!show_sign && !show_cam_or_section && !show_road) {
    p.restore();
    return;
  }

  int needed_h = 0;
  if (show_sign) {
    needed_h = S(210) + S(110);
  } else if (show_cam_or_section) {
    const int board_w = S(SPEED_BOARD_W);
    // 원형 표지(보드) + 아래 거리 pill 여유
    needed_h = board_w + S(110);
  } else { // show_road
    needed_h = S(275);
  }

  const int max_shift = std::max(0, height() - (base_y_start + needed_h) - S(20));
  const int y_shift = std::clamp(desired_shift, 0, max_shift);
  int y_start = base_y_start + y_shift;

  // A sign or camera board carries a distance pill under it, and the clamp
  // above left 50 px of empty road below that pill against the speed panel's
  // ~20 beside it. Its pill's bottom now lines up with the panel's bottom
  // edge (2026-11-24, on request), so the two share one bottom margin; the
  // board, and the NDA icon above it, come down with it. drawSpeed runs first
  // in drawHud and publishes the panel's edge. The pill ends board_h + 90
  // below y_start: 15 gap + 55 font + 10 + 10 pad (draw_dist_pill).
  if ((show_sign || show_cam_or_section) && speed_panel_bottom_ > 0) {
    const int board_h = S(SPEED_BOARD_W);
    const int pill_end = board_h + S(15) + S(55) + S(10) + S(10);
    y_start = std::clamp(speed_panel_bottom_ - pill_end, base_y_start, height() - pill_end);
  }

  // ---- NDA/HDA 아이콘 ----
  // 어느 표지가 그려지든 그 표지 바로 위에 중앙 정렬한다. CAM/SECTION 분기는
  // x_icon = 280 으로 고정돼 있었는데, 그 자리가 속도 패널(x 268~558) 안쪽이라
  // 배지가 패널 위에 겹쳐 그려졌다. 아래 폭만 분기하면 두 경우가 같아진다.
  if (activeNDA > 0) {
    const int w = S(120);
    const int h = S(54);
    const int top_margin = S(10);
    const int board_width = S(SPEED_BOARD_W);
    const int x_icon = x_start + (board_width - w) / 2;
    const int y_icon = std::max(0, y_start - h - top_margin);

    p.setOpacity(1.f);
    p.drawPixmap(x_icon, y_icon, w, h, (activeNDA == 1) ? ic_nda : ic_hda);
  }

  p.setOpacity(1.f);

  QString str;

  // Distance pill centred under a board: "85m" / "1.2km".
  auto draw_dist_pill = [&](int board_width, int board_height, int dist) {
    QString s_dist;
    if (dist < 1000) s_dist.sprintf("%dm", dist);
    else s_dist.sprintf("%.1fkm", dist / 1000.f);

    QFont font("Inter");
    font.setPixelSize(S(55));
    font.setStyleName("Bold");
    const int w_txt = QFontMetrics(font).width(s_dist);
    const int pad = S(10);
    const int center_x = x_start + board_width / 2;

    QRect rc(center_x - w_txt / 2, y_start + board_height + S(15), w_txt, font.pixelSize() + S(10));
    rc.adjust(-pad * 2, -pad, pad * 2, pad);

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rc, S(20), S(20));

    configFont(p, "Inter", S(55), "Bold");
    p.setBrush(Qt::NoBrush);
    p.setPen(QColor(255, 255, 255, 230));
    p.drawText(rc, Qt::AlignCenter | Qt::AlignVCenter, s_dist);
  };

  // ------------------------------------------------------------
  // 방지턱 / 주차단속 (표지 + 아래 남은 거리 pill)
  // ------------------------------------------------------------
  if (show_sign) {
    const int board_width = S(210);
    const int board_height = board_width;

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);

    const int pad = S(14);
    const QRectF sign_rect(x_start + pad, y_start + pad, board_width - 2 * pad, board_height - 2 * pad);
    if (show_bump) drawSpeedBumpSign(p, sign_rect);
    else drawParkingSign(p, sign_rect);

    draw_dist_pill(board_width, board_height, sign_dist);
  }

  // ------------------------------------------------------------
  // CAM/SECTION 제한속도 (원형 표지 + 아래 거리 pill)
  // ------------------------------------------------------------
  else if (show_cam_or_section) {
    const int board_width = S(SPEED_BOARD_W);
    const int board_height = board_width;

    // background
    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);

    // inner circle
    QRect board_rect(x_start, y_start, board_width, board_width);

    int padding = S(14);
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = S(18);
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, S(25)));
    p.drawEllipse(board_rect);

    // speed text
    p.setPen(QPen(Qt::black, padding));
    str.sprintf("%d", limit_speed);
    // The board no longer widens to 230 for a three-digit limit -- the speed
    // panel is only SPEED_BOARD_PANEL_GAP to its right now -- so the number
    // shrinks instead to stay inside the red ring.
    configFont(p, "Inter", S(limit_speed < 100 ? 70 : 58), "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    draw_dist_pill(board_width, board_height, left_dist);
  }

  // ------------------------------------------------------------
  // 일반 도로 제한속도 (SPEED LIMIT 박스)
  // ------------------------------------------------------------
  else if (show_road) {
    const int board_width = S(210);
    const int board_height = S(275);

    // background
    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);

    QRectF board_rect(x_start, y_start, board_width, board_height);

    int padding = S(14);
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius - padding / 2, corner_radius - padding / 2);

    padding = S(10);
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius - S(12), corner_radius - S(12));

    // "SPEED LIMIT"
    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", S(35), "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height() / 2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + S(20));
      p.setPen(QColor(0, 0, 0, 255));
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    // road limit number
    {
      str.sprintf("%d", roadLimit_Speed);
      configFont(p, "Inter", S(75), "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(),
                   board_rect.y() + board_rect.height() / 2,
                   board_rect.width(),
                   board_rect.height() / 2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + S(3));
      p.setPen(QColor(0, 0, 0, 255));
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
  }

  p.setOpacity(1.f);
  p.restore();
}



template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
  std::vector<T> xData(x_list);
  std::vector<T> yData(y_list);
  int size = xData.size();

  int i = 0;
  if(x >= xData[size - 2]) {
    i = size - 2;
  }
  else {
    while ( x > xData[i+1] ) i++;
  }
  T xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
  if (!extrapolate) {
    if ( x < xL ) yR = yL;
    if ( x > xR ) yL = yR;
  }

  T dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}

// LaneChangeLowSpeed [km/h] for the blinker label, read once per blinker episode
// (the setting only changes offroad): -1 = read it on the next blinker.
static int lane_change_low_speed_kph = -1;

// Side Blind Zone Alert from the car (carState.left/rightBlindspot, GM 0x142
// byte 0 bits 6/7), shown on that side so the driver can check it against the
// mirror light: the left/right assignment came from a statistical match on the
// 2026-11-25 drive and is not confirmed yet.
void NvgWindow::drawBlindSpot(QPainter &p) {
  const auto car_state = (*uiState()->sm)["carState"].getCarState();
  const bool sides[2] = {car_state.getLeftBlindspot(), car_state.getRightBlindspot()};
  if (!sides[0] && !sides[1]) return;
  // A red glow fading in from that screen edge, readable from the corner of
  // the eye without reading anything, and a small car marker low on that edge
  // (the radar watches beside and behind the car, not the lane ahead).
  constexpr int GLOW_W = 150;
  constexpr int CAR_W = 44, CAR_H = 70, CAR_MARGIN = 26;
  const int top = header_h;
  const int h = height() - top;
  p.save();
  p.setPen(Qt::NoPen);
  for (int i = 0; i < 2; i++) {
    if (!sides[i]) continue;
    const bool left = i == 0;
    const int x0 = left ? 0 : width() - GLOW_W;
    QLinearGradient glow(left ? 0 : width(), 0, left ? GLOW_W : width() - GLOW_W, 0);
    glow.setColorAt(0.0, QColor(235, 30, 30, 200));
    glow.setColorAt(1.0, QColor(235, 30, 30, 0));
    p.setBrush(glow);
    p.drawRect(x0, top, GLOW_W, h);

    const int cx = left ? CAR_MARGIN : width() - CAR_MARGIN - CAR_W;
    const int cy = height() * 2 / 3 - CAR_H / 2;
    p.setBrush(QColor(255, 255, 255, 235));
    p.drawRoundedRect(cx, cy, CAR_W, CAR_H, 12, 12);
    p.setBrush(QColor(235, 30, 30, 235));
    p.drawRoundedRect(cx + 7, cy + 10, CAR_W - 14, 18, 5, 5);   // windscreen
    p.drawRoundedRect(cx + 7, cy + CAR_H - 22, CAR_W - 14, 12, 4, 4);   // rear window
  }
  p.restore();
}

void NvgWindow::drawTurnSignals(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  const bool left_on = car_state.getLeftBlinker();
  const bool right_on = car_state.getRightBlinker();

  // nothing to show with both off, and hazards shouldn't paint both edges
  if (left_on == right_on) {
    lane_change_low_speed_kph = -1;
    p.setOpacity(1.0);
    return;
  }
  const bool left = left_on;

  // The blinker alone doesn't say whether openpilot will actually move the car
  // over. Pull the planner's lane change state so the indicator can tell the
  // driver which of the three situations they're in.
  const auto lp = sm["lateralPlan"].getLateralPlan();
  const auto lc_state = lp.getLaneChangeState();
  const bool changing = lc_state == cereal::LateralPlan::LaneChangeState::LANE_CHANGE_STARTING ||
                        lc_state == cereal::LateralPlan::LaneChangeState::LANE_CHANGE_FINISHING;
  const bool armed = lc_state == cereal::LateralPlan::LaneChangeState::PRE_LANE_CHANGE;

  QColor color;
  QString label;
  if (changing) {
    color = QColor(0, 225, 120);
    label = "차선변경 중";
  } else if (armed) {
    color = QColor(255, 175, 0);
    // With a navigation (ATC) instruction: from 15 km/h the blinker (a tap is
    // enough) moves the car one lane when there is room, under 15 the driver
    // does it by hand (desire_helper.py ATC_TAP_SPEED_MIN). Otherwise: under 30 km/h a nudge starts it
    // (LANE_CHANGE_LOW_SPEED, the LaneChangeLowSpeed param), from there the
    // blinker alone after 1 s.
    const QString atc = QString::fromUtf8(sm["roadLimitSpeed"].getRoadLimitSpeed().getAtcType().cStr());
    const bool atc_lane = (atc.startsWith("fork ") || atc.startsWith("atc ") || atc.startsWith("turn ")) &&
                          !atc.contains("prepare");
    if (lane_change_low_speed_kph < 0) {
      const int kph = QString::fromStdString(Params().get("LaneChangeLowSpeed")).toInt();
      lane_change_low_speed_kph = (kph == 40 || kph == 50 || kph == 60) ? kph : 30;
    }
    const bool fast = car_state.getVEgo() >= lane_change_low_speed_kph / 3.6f;
    if (atc_lane) label = car_state.getVEgo() >= 15 / 3.6f ? "자동 대기" : "직접 변경";
    else label = (lp.getAutoLaneChangeEnabled() && fast) ? "자동 대기" : "핸들 밀기";
  } else {
    // planner isn't arming: not engaged, or under LANE_CHANGE_SPEED_MIN
    color = QColor(210, 210, 210);
    // 10 = desire_helper.py LANE_CHANGE_SPEED_MIN
    label = (uiState()->status == STATUS_ENGAGED && car_state.getVEgo() < 10 / 3.6f) ? "10km/h+" : "";
  }

  // Geometry: TURN_BAND_* / TURN_CHEVRON_* above OnroadAlerts.
  const int band_w = TURN_BAND_W;
  const int band_x = turnBandX(width(), left);
  const int cy = height() / 2;
  const int ch = TURN_CHEVRON_H;
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);

  // Chevrons and label only -- no halo behind them, so the path view stays
  // unobstructed where the indicator overlaps it.

  // Three chevrons firing inside-out, the way a sequential turn signal does.
  const int n = TURN_CHEVRONS;
  const double period = changing ? 460.0 : 800.0;
  const double phase = fmod(millis_since_boot(), period) / period;
  const int lit = phase < 0.82 ? (int)(phase / 0.82 * n) + 1 : 0;
  drawTurnChevrons(p, width(), height(), left, color, lit);

  if (!label.isEmpty()) {
    configFont(p, "Open Sans", 40, "Bold");
    p.setPen(color);
    // Left: above the chevrons. Below them (y 608..668) the shifted band's
    // label would land on the cruise panel's top edge (x 300..532 from y 638).
    // Both labels are centred 60 px inboard of their band (2026-11-24): on the
    // left the lowered 조향 신호 ring reaches x 478 at that height, and on the
    // right the thermal strip now starts at x 1521 just below the chevrons.
    const int label_y = left ? cy - ch - 30 - 60 : cy + ch + 30;
    const int label_x = left ? band_x + 60 : band_x - 60;
    p.drawText(QRect(label_x, label_y, band_w, 60), Qt::AlignCenter, label);
  }

  p.restore();
}

// Text with a dark drop shadow (carrot draws its HUD text with a border and a
// shadow). y is the baseline; centre = true centres it on x.
static void drawShadowText(QPainter &p, qreal x, qreal y, const QString &text,
                           const QColor &color, bool centre) {
  if (text.isEmpty()) return;
  const qreal w = centre ? QFontMetricsF(p.font()).horizontalAdvance(text) : 0.;
  const QPointF pos(x - w / 2., y);
  p.setPen(QColor(0, 0, 0, 200));
  p.drawText(pos + QPointF(3, 3), text);
  p.setPen(color);
  p.drawText(pos, text);
}

// AutoTurnControl turn-info box: carrot hud_renderer.py _draw_turn_info_hud()
// (carrot-wip 77dd3335): carrot's contents and offsets in its bottom-right
// corner, the box narrowed from 790 to 639 (see rightColumnLayout).
// Data: road_speed_limiter.py -> roadLimitSpeed.
// Differences: the ATC backdrop is drawn only while ATC acts ("none" is
// truthy in carrot and paints it solid green even with ATC off), and a
// cancelled fork is grey. Shown only while a route is being guided, like carrot.
void NvgWindow::drawTurnInfo(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  if (sm.rcv_frame("roadLimitSpeed") == 0 ||
      nanos_since_boot() - sm.rcv_time("roadLimitSpeed") > 1000000000ULL) {
    return;   // road_speed_limiter not publishing: never show a frozen box
  }
  const auto nda = sm["roadLimitSpeed"].getRoadLimitSpeed();
  const int go_dist = nda.getGoPosDist();
  const int go_time = nda.getGoPosTime();
  if (!(go_dist > 0 && go_time > 0)) return;

  if (width() < 1200) return;                   // carrot skips narrow screens too
  // carrot's corner (rightColumnLayout): 699 wide, 10 px off the right edge
  // and 17 px off the bottom. Every font is up a step from carrot's on
  // request (2026-11-24); the offsets below are re-spaced for them.
  const QRect box = rightColumnLayout(width(), height()).turn_box;
  const int box_w = box.width();
  const int BOX_H = box.height();
  const int tbt_x = box.left();
  const int box_y = box.top();
  const int tbt_y = box_y + 60;                 // carrot: box top = tbt_y - 60

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);

  p.setPen(QPen(QColor(255, 255, 255), 2));
  p.setBrush(QColor(0, 0, 0, 120));
  p.drawRoundedRect(QRectF(tbt_x, box_y, box_w, BOX_H), 30, 30);

  const QColor white(255, 255, 255);

  // guidance text (elided: the box is narrower than carrot's)
  configFont(p, "Open Sans", 52, "Bold");
  {
    const QFontMetrics fm(p.font());
    drawShadowText(p, tbt_x + 20, tbt_y - 8,
                   fm.elidedText(QString::fromUtf8(nda.getTbtMainText().cStr()), Qt::ElideRight, box_w - 40),
                   white, false);
  }

  // turn icon, ATC state backdrop, distance
  const int turn_info = nda.getXTurnInfo();
  if (turn_info > 0) {
    const int bx = tbt_x + 100;
    const int by = tbt_y + 95;   // 10 lower than carrot: the guidance line grew

    const QString atc = QString::fromUtf8(nda.getAtcType().cStr());
    if (!atc.isEmpty() && !atc.startsWith("none")) {
      QColor fill(0, 228, 48);                                       // acting
      if (atc.contains("prepare")) fill = QColor(0, 255, 0, 100);    // not yet in range
      else if (atc.contains("canceled")) fill = QColor(150, 150, 150, 170);
      p.setPen(QPen(QColor(0, 0, 0), 1));
      p.setBrush(fill);
      p.drawRoundedRect(QRectF(bx - 80, by - 85, 160, 222), 15, 15);
    }

    const QPixmap *icon = nullptr;
    switch (turn_info) {
      case 1: icon = &ic_nav_turn_l; break;
      case 2: icon = &ic_nav_turn_r; break;
      case 3: icon = &ic_nav_fork_l; break;
      case 4: icon = &ic_nav_fork_r; break;
      case 7: icon = &ic_nav_uturn; break;
      default: break;
    }
    if (icon != nullptr && !icon->isNull()) {
      p.drawPixmap(bx - icon->width() / 2, by - icon->height() / 2, *icon);
    } else {
      const QString t = turn_info == 6 ? QString("TG")
                      : turn_info == 8 ? QString("목적지")
                      : QString("감속:%1").arg(turn_info);
      configFont(p, "Open Sans", 42, "Bold");
      drawShadowText(p, bx, by + 14, t, white, true);
    }

    const int dist = (int)nda.getXDistToTurn();
    if (dist > 0) {
      const QString d = dist < 1000 ? QString("%1 m").arg(dist)
                                    : QString("%1 km").arg(dist / 1000.0, 0, 'f', 1);
      configFont(p, "Open Sans", 52, "Bold");
      drawShadowText(p, bx, by + 115, d, white, true);
    }
  }

  // camera description (green tag) or road name
  const QString sdi = QString::fromUtf8(nda.getSdiDescr().cStr());
  const QString road = QString::fromUtf8(nda.getPosRoadName().cStr());
  configFont(p, "Open Sans", 50, "Bold");
  const int label_x = tbt_x + 200;
  const int label_base = tbt_y + 210;
  if (!sdi.isEmpty()) {
    // Elide against the room left in the box (pill ends 10 px inside its right
    // edge), then size the pill to what is drawn. Eliding to the text's own
    // rounded advance cut short labels ("사고다…") with half the box empty.
    const QFontMetricsF fm(p.font());
    const qreal avail = box_w - (label_x - tbt_x) - 20;
    const QString shown = fm.elidedText(sdi, Qt::ElideRight, avail);
    const qreal tw = fm.horizontalAdvance(shown);
    const qreal th = fm.height();
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 228, 48));
    p.drawRoundedRect(QRectF(label_x - 10, label_base - fm.ascent() - 4, tw + 20, th + 8), 10, 10);
    drawShadowText(p, label_x, label_base, shown, white, false);
  } else if (!road.isEmpty()) {
    drawShadowText(p, label_x, label_base, road, white, false);
  }

  // arrival time and distance left
  // The arrival line steps down from 56 px until it ends 24 px inside the
  // right edge (a 3-digit minute count is the long case; in the 699 px box
  // it settles near 46). The distance left is not tied to it any more: it is
  // short, so it keeps 56 px.
  const QDateTime eta = QDateTime::currentDateTimeUtc().toOffsetFromUtc(9 * 3600).addSecs(go_time);
  const QString eta_text = QString("도착: %1분(%2)").arg(go_time / 60.0, 0, 'f', 1).arg(eta.toString("HH:mm"));
  const int eta_avail = box_w - 195 - 24;
  int eta_size = 56;
  for (; eta_size > 34; eta_size -= 2) {
    configFont(p, "Open Sans", eta_size, "Bold");
    if (QFontMetrics(p.font()).horizontalAdvance(eta_text) <= eta_avail) break;
  }
  configFont(p, "Open Sans", eta_size, "Bold");
  drawShadowText(p, tbt_x + 195, tbt_y + 74, eta_text, white, false);
  configFont(p, "Open Sans", 56, "Bold");
  drawShadowText(p, tbt_x + 310, tbt_y + 140, QString("%1km").arg(go_dist / 1000.0, 0, 'f', 1), white, false);

  p.restore();
}

/*
void NvgWindow::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 120;
  int h = 100;
  int x = width() - w - 30;
  int y = 30;

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Open Sans", 40, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);
  p.setOpacity(1.);
}*/

void NvgWindow::drawThermal(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto deviceState = sm["deviceState"].getDeviceState();

  const auto cpuTempC = deviceState.getCpuTempC();
  float ambientTemp = deviceState.getAmbientTempC();

  float cpuTemp = 0.f;
  if (std::size(cpuTempC) > 0) {
    for (int i = 0; i < (int)std::size(cpuTempC); i++) {
      cpuTemp += cpuTempC[i];
    }
    cpuTemp = cpuTemp / (float)std::size(cpuTempC);
  }

  // =========================
  // Layout: one horizontal strip of three tiles (2026-11-24, on request; it
  // used to be a 213 x 309 column). It sits between the status block above
  // and the turn-info box below (rightColumnLayout), right edge 21 px off the
  // screen edge like the block. THERMAL_W x THERMAL_H is 15% over the
  // block-wide 294 x 77 it had, so it now reaches 44 px left of the block.
  // =========================
  constexpr int RIGHT_MARGIN = 21;
  constexpr int STRIP_W = THERMAL_W;
  const int pad = 16;
  const int col_w = (STRIP_W - pad * 2) / 3;
  const int value_label_gap = 12;

  // Size the value font so the widest reading ("100%") fits its column with a
  // few px to spare; measured from fixed reference strings, never the live
  // values, so a changing reading can never resize the strip. At 338 wide it
  // lands near 38 px (it was 32).
  // The fit and the two ink heights depend only on col_w, so they are worked
  // out once per width: the loop below used to run on every paint, up to 22
  // tightBoundingRect calls (glyph outlines) each time.
  constexpr int LABEL_PT = 25;   // 22 + 15%
  static int fit_col_w = -1, value_pt = 42, val_ink_h = 0, lab_ink_h = 0;
  if (col_w != fit_col_w) {
    fit_col_w = col_w;
    value_pt = 42;
    for (; value_pt > 20; value_pt--) {
      configFont(p, "Open Sans", value_pt, "Bold");
      if (QFontMetrics(p.font()).tightBoundingRect("100%").width() <= col_w - 6) break;
    }
    // Ink heights, not line boxes: every string here is digits / caps with no
    // descender (see the note this replaced in git history).
    configFont(p, "Open Sans", value_pt, "Bold");
    val_ink_h = QFontMetrics(p.font()).tightBoundingRect("8°%").height();
    configFont(p, "Open Sans", LABEL_PT, "Bold");
    lab_ink_h = QFontMetrics(p.font()).tightBoundingRect("A").height();
  }

  // The strip's box is fixed (rightColumnLayout places the status block from
  // it); the two text lines are centred in it.
  const RightColumnLayout col = rightColumnLayout(width(), height());
  const int total_h = col.thermal_bottom - col.thermal_top;
  const int right = width() - RIGHT_MARGIN;
  const int left = right - STRIP_W;
  const int top = col.thermal_top;
  const int text_top = top + (total_h - (val_ink_h + value_label_gap + lab_ink_h)) / 2;

  // Publish the panel's box: the status block takes its right edge from it
  // (see the members' comment in onroad.h).
  thermal_panel_top_ = top;
  thermal_panel_cx_ = left + STRIP_W / 2;
  thermal_panel_right_ = right;

  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(QRect(left, top, STRIP_W, total_h), 16, 16);

  // Faint dividers between the three columns.
  p.setPen(QPen(QColor(255, 255, 255, 38), 2));
  for (int i = 1; i < 3; i++) {
    const int dx = left + pad + col_w * i;
    p.drawLine(dx, top + 11, dx, top + total_h - 11);
  }

  const int value_base = text_top + val_ink_h;
  const int label_base = value_base + value_label_gap + lab_ink_h;
  // Each string centred on its column by its own ink, so digits look optically
  // centred rather than advance-centred.
  // Ink rects memoised per font and string, like drawCenteredText's: the
  // readings take a bounded set of values.
  static auto *ink_tables = new std::deque<FontTextRects>;
  auto inkRect = [&](const QString &text) {
    return cachedTextRect(textRectsFor(ink_tables, p.font()), text,
                          [&] { return QFontMetrics(p.font()).tightBoundingRect(text); });
  };
  auto drawTile = [&](int col, const QString &value, const QString &label, const QColor &valColor) {
    const int cx = left + pad + col_w * col + col_w / 2;
    configFont(p, "Open Sans", value_pt, "Bold");
    const QRect vi = inkRect(value);
    p.setPen(valColor);
    p.drawText(cx - vi.width() / 2 - vi.left(), value_base, value);

    configFont(p, "Open Sans", LABEL_PT, "Bold");
    const QRect li = inkRect(label);
    p.setPen(QColor(0, 255, 0, 220));
    p.drawText(cx - li.width() / 2 - li.left(), label_base, label);
  };

  // BAT (coloured by the CPU temperature, as before)
  QString batStr;
  batStr.sprintf("%d%%", deviceState.getBatteryPercent());
  int r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  int g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  drawTile(0, batStr, "BAT", QColor(r, g, 200, 220));

  // CPU
  QString cpuStr;
  cpuStr.sprintf("%.0f°C", cpuTemp);
  drawTile(1, cpuStr, "CPU", QColor(r, g, 200, 220));

  // AMB (ambient)
  QString ambStr;
  ambStr.sprintf("%.0f°C", ambientTemp);
  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  drawTile(2, ambStr, "AMB", QColor(r, g, 200, 220));

  p.restore();
}




/*void NvgWindow::drawDebugText(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  QString str;

  int y = 200;
  //const int height = 60;

  const int text_x = width()/2 + 200;
  //const int text_x = 40;

  auto controls_state = sm["controlsState"].getControlsState();
  //auto car_control = sm["carControl"].getCarControl();
  //auto car_state = sm["carState"].getCarState();

  const char* bucketPointsStr = controls_state.getBucketPoints().cStr();

  configFont(p, "Open Sans", 27, "Bold");
  p.setPen(QColor(0, 255, 0, 255));
  p.setRenderHint(QPainter::TextAntialiasing);

  // |가 아니라 이미 \n로 단락 구분된 문자열을 받는다고 가정
  QString bucketStr(bucketPointsStr);

  // 텍스트 그리기: QRect와 TextWordWrap 사용
  int textWidth = 900;   // 영역 너비, 필요에 따라 조정
  int textHeight = 1000; // 영역 높이, 필요에 따라 조정
  QRect textRect(text_x, y, textWidth, textHeight);

  // Qt::TextWordWrap 옵션으로 \n 단락 반영
  p.drawText(textRect, Qt::AlignLeft | Qt::TextWordWrap, bucketStr);
}*/

void NvgWindow::drawDebugText(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  // Starts under the clock / CPU header line (baseline 55).
  int y = 130;
  const int height = 60;

  const int text_x = width()/2 + 200;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  //float applyAccel = controls_state.getApplyAccel();
  //float aReqValue = controls_state.getAReqValue();
  //float aReqValueMin = controls_state.getAReqValueMin();
  //float aReqValueMax = controls_state.getAReqValueMax();

  //int sccStockCamAct = (int)controls_state.getSccStockCamAct();
  //int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

  float vEgo = car_state.getVEgo();
  float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  float vPid = controls_state.getVPid();
  float upAccelCmd = controls_state.getUpAccelCmd();
  float uiAccelCmd = controls_state.getUiAccelCmd();
  float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"꺼짐", "켜짐", "정지", "출발"};

  configFont(p, "Open Sans", 50, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Accel: %.3f\n", accel);
  p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  //p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  //p.drawText(text_x, y, str);

  //auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  //auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  //float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  //float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  //y += height;
  //str.sprintf("Lead: %.1f/%.1f/%.1f\n", radar_dist, vision_dist, (radar_dist - vision_dist));
  //p.drawText(text_x, y, str);
}
