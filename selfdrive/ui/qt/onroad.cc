#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDateTime>
#include <QDebug>
#include <QSound>
#include <QMouseEvent>
#include <QPainterPath>
#include <algorithm>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

namespace {
constexpr float COMFORT_BRAKE = 2.5f;
constexpr float STOP_DISTANCE = 5.5f;

float desired_follow_distance(float v_ego, float v_lead, float t_follow) {
  float v_diff_offset = 0.0f;
  if (v_lead > v_ego) {
    v_diff_offset = std::clamp(v_lead - v_ego, 0.0f, STOP_DISTANCE / 2.0f);
    v_diff_offset = std::max(v_diff_offset * ((10.0f - v_ego) / 10.0f), 0.0f);
  }

  const float safe_obstacle_distance =
    (v_ego * v_ego) / (2.0f * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE;
  const float stopped_equivalence_distance =
    (v_lead * v_lead) / (2.0f * COMFORT_BRAKE) + v_diff_offset;
  return std::max(safe_obstacle_distance - stopped_equivalence_distance, 0.0f);
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

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setContentsMargins(0, 0, 0, 0);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignTop);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
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
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();
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

  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  startPos = e->pos();
  //QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

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
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
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
    const int wheel_icon_size = static_cast<int>(radius * 0.85f);
    ic_wheel = QPixmap("../assets/img_chffr_wheel.png").scaled(wheel_icon_size, wheel_icon_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
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
    bg.setColorAt(0.00, QColor(55, 235, 72, 190));    // green
    bg.setColorAt(0.50, QColor(55, 235, 72, 160));
    bg.setColorAt(1.00, QColor(55, 235, 72, 0));      // fade out
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  painter.restore();
}


void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data,
                         const QPointF &vd, bool is_radar, const QString &info_text) {
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  const float v_rel = lead_data.getV()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  if (!info_text.isEmpty()) {
    painter.save();
    // +20% overall on 2026-08-27: the font and the padding both, so the box
    // grows with the text instead of tightening around it. 36 -> 43 px, and the
    // padding scaled to match (32 -> 38, 20 -> 24). The rounded-rect radius
    // below follows for the same reason.
    configFont(painter, "Open Sans", 43, "Bold");
    QFontMetrics metrics(painter.font());
    const int text_width = metrics.horizontalAdvance(info_text) + 38;
    const int text_height = metrics.height() + 24;
    QRect text_rect(static_cast<int>(x - text_width / 2), static_cast<int>(y + sz + 12),
                    text_width, text_height);
    if (text_rect.bottom() > height() - 12) {
      text_rect.moveBottom(static_cast<int>(y - 12));
    }
    text_rect.moveLeft(std::clamp(text_rect.left(), 12, width() - text_rect.width() - 12));

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 160));
    painter.drawRoundedRect(text_rect, 10, 10);
    painter.setPen(whiteColor());
    painter.drawText(text_rect, Qt::AlignCenter, info_text);
    painter.restore();
  }
}

void NvgWindow::paintGL() {
}

void NvgWindow::paintEvent(QPaintEvent *event) {
  QPainter p;
  p.begin(this);

  p.beginNativePainting();
  CameraViewWidget::paintGL();
  p.endNativePainting();

  UIState *s = uiState();
  if (s->worldObjectsVisible()) {
    drawHud(p);
  }

  p.end();

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
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  const qreal draw_x = x - real_rect.width() / 2.0;
  p.drawText(QPointF(draw_x, y), text);
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
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
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

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);

  drawLaneLines(p, s);

  auto leads = sm["modelV2"].getModelV2().getLeadsV3();
  const auto lead_one = sm["radarState"].getRadarState().getLeadOne();
  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  QString lead_info;
  if (lead_one.getStatus()) {
    const float v_ego = std::max(car_state.getVEgo(), 0.0f);
    const float d_rel = std::max(lead_one.getDRel(), 0.0f);
    const float v_lead = std::max(lead_one.getVLead(), 0.0f);
    const float desired_distance = desired_follow_distance(v_ego, v_lead,
                                                           controls_state.getDynamicTRValue());
    if (v_ego > 1.0f) {
      const float time_gap = d_rel / v_ego;
      lead_info.sprintf("%.0f meters (Desired:%.0f) | %.0f km/h | %.2f s",
                        d_rel, desired_distance, v_lead * MS_TO_KPH, time_gap);
    } else {
      lead_info.sprintf("%.0f meters (Desired:%.0f) | %.0f km/h | -- s",
                        d_rel, desired_distance, v_lead * MS_TO_KPH);
    }
  }
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0], lead_info);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }

  //drawMaxSpeed(p);
  drawSpeed(p);
  drawSpeedLimit(p);
  drawThermal(p);
  drawRestArea(p);
  drawTurnSignals(p);
  //drawGpsStatus(p);

  if(s->show_debug && width() > 1200)
    drawDebugText(p);

  const auto device_state = sm["deviceState"].getDeviceState();
  //const auto car_control = sm["carControl"].getCarControl();
  //const auto live_params = sm["liveParameters"].getLiveParameters();
  //const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  //const auto torque_state = controls_state.getLateralControlState().getTorqueState();

  //QColor orangeColor = QColor(52, 197, 66, 255);

  float cpu_usage = 0.0f;
  const auto cpu_usage_list = device_state.getCpuUsagePercent();

  if (cpu_usage_list.size() > 0) {
    for (const auto usage : cpu_usage_list) {
      cpu_usage += usage;
    }
    cpu_usage /= cpu_usage_list.size();
  }

  QString infoText;
  infoText.sprintf("CPU(%.0f%%) MEM(%d%%) (LatA:%.3f,Fri:%.3f) SR(%.2f) MIN_TR(%.1f) DF_MOD(%.1f)",
                      cpu_usage,
                      device_state.getMemoryUsagePercent(),
                      controls_state.getLatAccelFactor(),
                      controls_state.getFriction(),
                      controls_state.getSteerRatio(),
                      controls_state.getMinTR(),
                      controls_state.getGlobalDfMod()
                      );


  // info
  configFont(p, "Open Sans", 43, "Regular");
  p.setPen(QColor(0, 255, 0, 255));
  p.drawText(rect().left() + 20, rect().height() - 15, infoText);


  drawBottomIcons(p);
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
  constexpr int BOW = 12;      // how much the bar bows up in the middle
  constexpr int BAR_W = 46;    // track stroke width
  constexpr int GAUGE_W = 30;  // gauge stroke width, inset inside the track
  constexpr int REF_W = 40, REF_H = 30;
  constexpr int MARK = 32;     // lane-centring marker box size
  constexpr int MARK_PEN = 8;  // marker outline thickness
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
  constexpr int TRACK_W = 28;
  constexpr int BALL_D = 44;

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
  // rounded caps at either end.
  const float travel_top = top_y + BALL_D / 2.0f;
  const float travel_bottom = bottom_y - BALL_D / 2.0f;
  const float ball_y = travel_bottom - shown * (travel_bottom - travel_top);

  p.setBrush(ball_color);
  p.drawEllipse(QPointF(cx, ball_y), BALL_D / 2.0, BALL_D / 2.0);

  p.restore();
}

void NvgWindow::drawBottomIcons(QPainter &p) {
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
  const int y1 = rect().bottom() - footer_h / 2 - 10;
  const int y2 = y1 - radius - row_gap;

  // Steering-effort gauge sits above the upper icon row, spanning its width.
  drawSteerGauge(p, icon_start_x + (icon_step * 5) / 2,
                 y2 - radius / 2 - 52, icon_step * 5 + radius);

  // Confidence gauge sits beside the ACC/LKAS column (icon_step * 5),
  // spanning the same vertical extent as that stacked pair.
  drawConfidenceGauge(p, icon_start_x + (icon_step * 6),
                      y2 - radius / 2, y1 + radius / 2);

  float cur_speed = std::max(0.0, car_state.getVEgo() * MS_TO_KPH);
  QString str;
  QString str2;
  QColor textColor = QColor(255, 255, 255, 200);

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

  // 1. PEDAL MAX
  // GM CarController clips the command sent to the comma pedal to 0.00..0.85.
  // actuatorsOutput.gas is the post-controller value that matches the CAN output.
  constexpr float comma_pedal_min = 0.0f;
  constexpr float comma_pedal_max = 0.85f;
  const float comma_pedal = std::clamp(car_control.getActuatorsOutput().getGas(),
                                        comma_pedal_min, comma_pedal_max);
  const float comma_pedal_ratio = (comma_pedal - comma_pedal_min) /
                                  (comma_pedal_max - comma_pedal_min);
  const QRectF pedal_ring(x - radius / 2 + 7, y1 - radius / 2 + 7,
                          radius - 14, radius - 14);

  p.setPen(QPen(QColor(55, 61, 74, 255), 3));
  p.setBrush(QColor(55, 61, 74, 235));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  // Full-scale background ring and clockwise live comma-pedal command ring.
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(118, 126, 139, 180), 9, Qt::SolidLine, Qt::FlatCap));
  p.drawEllipse(pedal_ring);
  p.setPen(QPen(QColor(255, 0, 0, 255), 9, Qt::SolidLine, Qt::FlatCap));
  p.drawArc(pedal_ring, 90 * 16,
            -static_cast<int>(comma_pedal_ratio * 360.0f * 16.0f));

  str = "PEDAL MAX";
  configFont(p, "Open Sans", 20, "Bold");
  drawText(p, x, y1 - 17, str, 230);

  str2.sprintf("%.0f", comma_pedal * 100.0f);
  textColor = QColor(255, 255, 255, 245);
  configFont(p, "Open Sans", 36, "Bold");
  drawTextWithColor(p, x, y1 + 21, str2, textColor);
  p.setOpacity(1.0);
  p.setBrush(Qt::NoBrush);
  p.setPen(Qt::NoPen);

  float textSize = 34.f;

  // 2. VISION DIST -- hidden, kept for easy restoration.
  // This assignment stays OUTSIDE the comment on purpose: tiles further down
  // (TR mode, PEDAL STATUS) draw with whatever textSize holds, and hiding it
  // along with the rest would silently shrink them from 48 to 34 px.
  textSize = 48.f;
  /*
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

  // AUTO HOLD is drawn in the right-hand status column below; only its state
  // is read here.
  const int autohold = car_state.getAutoHold();

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
  //   CommaPedalResistance = high | mid | low
  // Refresh once per second instead of reading Params on every UI frame.
  static uint64_t last_ai_profile_update = 0;
  static QString ai_pedal_profile = "MID";
  const uint64_t ai_profile_now = millis_since_boot();

  if (last_ai_profile_update == 0 || ai_profile_now - last_ai_profile_update >= 1000) {
    last_ai_profile_update = ai_profile_now;

    QString pedal_profile = QString::fromStdString(
      Params().get("CommaPedalResistance")).toUpper();
    if (pedal_profile != "HIGH" && pedal_profile != "MID" &&
        pedal_profile != "LOW") {
      pedal_profile = "MID";
    }
    ai_pedal_profile = pedal_profile;
  }

  // Circular dial matching the PEDAL STATUS / ACC gauges: title on the disc,
  // value under it. Colour encodes the level rather than being fixed green,
  // so the setting reads at a glance without looking at the word.
  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  QColor aiProfileColor;
  if (ai_pedal_profile == "HIGH") {
    aiProfileColor = QColor(254, 32, 32, 220);
  } else if (ai_pedal_profile == "MID") {
    aiProfileColor = QColor(255, 185, 15, 220);
  } else {
    aiProfileColor = QColor(255, 255, 255, 200);
  }

  configFont(p, "Open Sans", 20, "Bold");
  drawText(p, x, y1 - 14, "PEDAL LEVEL", 200);

  configFont(p, "Open Sans", 38, "Bold");
  drawTextWithColor(p, x, y1 + 35, ai_pedal_profile, aiProfileColor);
  p.setOpacity(1.0);


  // ================================================================================================================ //
  x = 140;
  x = icon_start_x;

  // 2. TR Value
  x = icon_start_x + icon_step;
  float tr_value = controls_state.getDynamicTRValue();
  auto tr_mode = controls_state.getDynamicTRMode();
  //int cruise_gap = car_state.getCruiseGap();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  str.sprintf("%s", tr_mode.cStr());
  str2.sprintf("%.2f", tr_value);

  configFont(p, "Open Sans", textSize, "Bold");
  //textColor = QColor(255, 255, 255, 200);  white
  textColor = QColor(120, 255, 120, 200);   // green


  configFont(p, "Open Sans", 27, "Bold");
  drawText(p, x, y2-14, str, 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+35, str2, textColor);
  p.setOpacity(1.0);

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

  // 1. PEDAL
  x = icon_start_x;
  float accel = car_control.getActuators().getAccel();

  p.setPen(Qt::NoPen);
  p.setBrush(stop_accel_boost_active ? QColor(0, 170, 90, 235) : blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  if(accel > 0) {
    //str = "ACCEL";
    str = "가속";
    textColor = QColor(120, 255, 120, 200);
  }
  else if(accel == 0.0) {
    //str = "──";
    str = "브레이크";
    textColor = QColor(255, 185, 15, 200);
  }
  else {
    //str = "DECEL";
    str = "감속";
    textColor = QColor(254, 32, 32, 200);
  }

  // Keep the existing PEDAL gauge layout. Only its active color and state
  // text change while the confirmed stop-and-go launch assist is operating.
  if(stop_accel_boost_active) {
    str = "BOOST";
    textColor = QColor(225, 255, 239, 255);
  }

  configFont(p, "Open Sans", 20, "Bold");
  drawText(p, x, y2-14, "PEDAL STATUS", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+35, str, textColor);
  p.setOpacity(1.0);

  // ACC and BRAKE moved to the right-hand status column below; only their
  // state is read here.
  const bool acc_bool = car_state.getAdaptiveCruise();
  const bool brake_valid = car_state.getBrakePressed();

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

  // 7. WHEEL -- same live steeringAngleDeg as "STEER" above, rendered as a
  // physically rotating wheel icon instead of a number. Not present in the
  // original source; uses the previously-unused ../assets/img_chffr_wheel.png.
  // (swapped column with LKAS, per user request)
  x = icon_start_x + (icon_step * 2);
  {
    const float steer_angle_deg = car_state.getSteeringAngleDeg();
    const bool hands_on_wheel = car_state.getSteeringPressed();

    p.setPen(Qt::NoPen);
    // Green when hands-off (system driving alone), black when the driver is
    // holding the wheel -- gives an at-a-glance signal matching steeringPressed.
    p.setBrush(hands_on_wheel ? blackColor(220) : QColor(23, 134, 68, 220));
    p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

    p.save();
    p.translate(x, y1);
    // QPainter::rotate() is clockwise for positive angles, opposite of the
    // drawArc() convention above -- negate so positive (left) still spins CCW.
    p.rotate(-steer_angle_deg);
    p.setOpacity(1.0);
    p.drawPixmap(-ic_wheel.width() / 2, -ic_wheel.height() / 2, ic_wheel);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 59, 59, 255));
    p.drawEllipse(QPointF(0, -radius * 0.42), 4.5, 4.5);
    p.restore();
  }

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
    constexpr int SD = 88;         // status circle diameter
    constexpr int SGAP_X = 12;     // horizontal gap between the two columns
    constexpr int SGAP_Y = 10;     // vertical gap between the two rows
    constexpr int PANEL_GAP = 16;  // clearance above the temperature panel

    auto statusCircle = [&](int cx_, int cy, const QString &label,
                            const QString &value, const QColor &value_color) {
      p.setPen(Qt::NoPen);
      p.setBrush(blackColor(200));
      p.drawEllipse(cx_ - SD / 2, cy - SD / 2, SD, SD);
      configFont(p, "Open Sans", 20, "Bold");
      drawText(p, cx_, cy - 10, label, 200);
      QColor c = value_color;
      configFont(p, "Open Sans", 28, "Bold");
      drawTextWithColor(p, cx_, cy + 27, value, c);
      p.setOpacity(1.0);
    };

    auto statusIcon = [&](int cx_, int cy, QPixmap &img, bool on) {
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(0, 0, 0, on ? 77 : 26));
      p.drawEllipse(cx_ - SD / 2, cy - SD / 2, SD, SD);
      p.setOpacity(on ? 1.0f : 0.15f);
      const int isz = (SD / 2) * 1.5;
      p.drawPixmap(cx_ - isz / 2, cy - isz / 2, isz, isz, img);
      p.setOpacity(1.0);
    };

    const int col_l = thermal_panel_cx_ - (SD + SGAP_X) / 2;
    const int col_r = thermal_panel_cx_ + (SD + SGAP_X) / 2;
    // Anchored from the bottom so the block always grows away from the panel.
    const int row_b = thermal_panel_top_ - PANEL_GAP - SD / 2;
    const int row_t = row_b - SD - SGAP_Y;

    statusIcon(col_l, row_t, ic_brake, brake_valid);
    if (autohold >= 0) {
      statusIcon(col_l, row_b,
                 autohold > 1 ? ic_autohold_warning : ic_autohold_active,
                 autohold > 0);
    }
    statusCircle(col_r, row_t, "ACC", acc_bool ? "ON" : "OFF",
                 acc_bool ? QColor(120, 255, 120, 200) : QColor(254, 32, 32, 200));
    statusCircle(col_r, row_b, "LKAS", lkas_str, lkas_color);
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
  const qreal panel_content_w = 220.0 * 1.10;
  const qreal panel_content_h = bgFixed.height();
  const qreal panel_bg_w = panel_content_w * 1.2;
  const qreal panel_bg_h = panel_content_h;

  // 가운데(좌측 패널 ↔ 메인 속도 박스) 가로 여백
  const int baseGap = 24;
  const int extraGapX = 14;
  const int gap = baseGap + extraGapX;

  QRectF panelBg(bgFixed.left() - gap - panel_bg_w,
                 bgFixed.top(),
                 panel_bg_w,
                 panel_bg_h);

  QRectF panelContent(panelBg.center().x() - panel_content_w / 2.0,
                      panelBg.top(),
                      panel_content_w,
                      panel_content_h);

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
  drawTextWithColor(p, panel_cx, baselineAt(panel_cy), strCur, curSpeedColor);

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

  if (camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  } else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  const bool show_cam_or_section = (limit_speed > 0 && left_dist > 0);
  const bool show_road = (roadLimit_Speed > 0 && roadLimit_Speed < 200);

  if (!show_cam_or_section && !show_road) {
    p.restore();
    return;
  }

  // ============================================================
  // ✅ 15% 확대 스케일
  // ============================================================
  const float k = 1.00f;
  auto S = [&](int v) -> int { return (int)std::lround(v * k); };

  // ---- layout base ----
  const int x_start = 30;             // 위치는 유지
  const int base_y_start = 70;        // 위치는 유지
  const int corner_radius = S(32);
  const QColor bgColor(0, 0, 0, 166);

  // ---- 20% 아래로 이동 (화면 하단 넘어가면 clamp) + 현재 보이던 위치에서 400 더 내리는 것----
  const int desired_shift = (int)std::lround(height() * 0.20f) + 400;

  int needed_h = 0;
  if (show_cam_or_section) {
    const int board_w = S((limit_speed < 100) ? 210 : 230);
    // 원형 표지(보드) + 아래 거리 pill 여유
    needed_h = board_w + S(110);
  } else { // show_road
    needed_h = S(275);
  }

  const int max_shift = std::max(0, height() - (base_y_start + needed_h) - S(20));
  const int y_shift = std::clamp(desired_shift, 0, max_shift);
  const int y_start = base_y_start + y_shift;

  // ---- NDA/HDA 아이콘 ----
  // 요구사항: roadLimit_Speed(SPEED LIMIT 박스) 있으면 박스 "위"에 표시
  if (activeNDA > 0) {
    const int w = S(120);
    const int h = S(54);
    p.setOpacity(1.f);

    if (show_road) {
      // SPEED LIMIT 박스 위 중앙 정렬
      const int board_width = S(210);
      const int top_margin = S(10);
      const int x_icon = x_start + (board_width - w) / 2;
      int y_icon = y_start - h - top_margin;
      y_icon = std::max(0, y_icon);

      p.drawPixmap(x_icon, y_icon, w, h, (activeNDA == 1) ? ic_nda : ic_hda);
    } else {
      // CAM/SECTION일 때는 기존 위치 유지(크기만 확대), 이동량만 반영
      const int x_icon = 280;
      const int base_nda_y = 80 - bdr_s;
      const int y_icon = base_nda_y + y_shift;

      p.drawPixmap(x_icon, y_icon, w, h, (activeNDA == 1) ? ic_nda : ic_hda);
    }
  }

  p.setOpacity(1.f);

  QString str;

  // ------------------------------------------------------------
  // CAM/SECTION 제한속도 (원형 표지 + 아래 거리 pill)
  // ------------------------------------------------------------
  if (show_cam_or_section) {
    const int board_width = S((limit_speed < 100) ? 210 : 230);
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
    configFont(p, "Inter", S(70), "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    // left dist pill
    QRect rcLeftDist;
    QString strLeftDist;

    if (left_dist < 1000) strLeftDist.sprintf("%dm", left_dist);
    else strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

    QFont font("Inter");
    font.setPixelSize(S(55));
    font.setStyleName("Bold");

    QFontMetrics fm(font);
    int w_txt = fm.width(strLeftDist);

    padding = S(10);
    const int center_x = x_start + board_width / 2;

    rcLeftDist.setRect(center_x - w_txt / 2,
                       y_start + board_height + S(15),
                       w_txt,
                       font.pixelSize() + S(10));
    rcLeftDist.adjust(-padding * 2, -padding, padding * 2, padding);

    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(rcLeftDist, S(20), S(20));

    configFont(p, "Inter", S(55), "Bold");
    p.setBrush(Qt::NoBrush);
    p.setPen(QColor(255, 255, 255, 230));
    p.drawText(rcLeftDist, Qt::AlignCenter | Qt::AlignVCenter, strLeftDist);
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



QPixmap NvgWindow::get_icon_iol_com(const char* key) {
  auto item = ic_oil_com.find(key);
  if(item == ic_oil_com.end()) {
    QString str;
    str.sprintf("../assets/images/oil_com/%s.png", key);

    QPixmap icon = QPixmap(str);
    ic_oil_com[key] = icon;
    return icon;
  }
  else
    return item.value();
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

void NvgWindow::drawRestArea(QPainter &p) {
  if(width() < 1850)
    return;

  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  auto restAreaList = roadLimitSpeed.getRestArea();

  int length = std::size(restAreaList);

  int yPos = 0;
  for(int i = length-1; i >= 0; i--) {
    auto restArea = restAreaList[i];
    auto image = restArea.getImage();
    auto title = restArea.getTitle();
    auto oilPrice = restArea.getOilPrice();
    auto distance = restArea.getDistance();

    if(title.size() > 0 && distance.size() > 0) {
      drawRestAreaItem(p, yPos, image, title, oilPrice, distance, i == 0);
      yPos += 200 + 25;
    }
  }
}

void NvgWindow::drawRestAreaItem(QPainter &p, int yPos, capnp::Text::Reader image, capnp::Text::Reader title,
        capnp::Text::Reader oilPrice, capnp::Text::Reader distance, bool lastItem) {

  int mx = 20;
  int my = 5;

  int box_width = Hardware::TICI() ? 580 : 510;
  int box_height = 200;

  int icon_size = 70;

  //QRect rc(30, 30, 184, 202); // MAX box
  QRect rc(184+30+30, 30 + yPos, box_width, box_height);
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 5, 5);

  if(lastItem)
    p.setPen(QColor(255, 255, 255, 200));
  else
    p.setPen(QColor(255, 255, 255, 150));

  int x = rc.left() + mx;
  int y = rc.top() + my;

  configFont(p, "Open Sans", 60, "Bold");
  p.drawText(x, y+60+5, title.cStr());

  QPixmap icon = get_icon_iol_com(image.cStr());
  p.drawPixmap(x, y + box_height/2 + 5, icon_size, icon_size, icon);

  configFont(p, "Open Sans", 50, "Bold");
  p.drawText(x + icon_size + 15, y + box_height/2 + 50 + 5, oilPrice.cStr());

  configFont(p, "Open Sans", 60, "Bold");

  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(distance.cStr());

  p.drawText(rc.left()+rc.width()-rect.width()-mx-5, y + box_height/2 + 60, distance.cStr());
}

void NvgWindow::drawTurnSignals(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  const bool left_on = car_state.getLeftBlinker();
  const bool right_on = car_state.getRightBlinker();

  // nothing to show with both off, and hazards shouldn't paint both edges
  if (left_on == right_on) {
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
    label = lp.getAutoLaneChangeEnabled() ? "자동 대기" : "핸들 밀기";
  } else {
    // planner isn't arming: not engaged, or under LANE_CHANGE_SPEED_MIN
    color = QColor(210, 210, 210);
    label = (uiState()->status == STATUS_ENGAGED && car_state.getVEgo() < 50 / 3.6f) ? "50km/h+" : "";
  }

  // Placed where the old pixmap indicator sat: vertically centred, and inboard
  // rather than pinned to the screen edge. Still clear of the speed box on the
  // left and the thermal tiles on the right, which both hug the edges.
  const int band_w = 240;
  const int band_gap = 400;   // inner edge of the band, out from screen centre
  const int band_x = left ? width() / 2 - band_gap - band_w : width() / 2 + band_gap;
  const int cy = height() / 2;
  const int dir = left ? -1 : 1;
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);

  // Chevrons and label only -- no halo behind them, so the path view stays
  // unobstructed where the indicator overlaps it.

  // Three chevrons firing inside-out, the way a sequential turn signal does.
  const int n = 3;
  const int step = 56;
  const int cw = 38;
  const int ch = 58;
  const double period = changing ? 460.0 : 800.0;
  const double phase = fmod(millis_since_boot(), period) / period;
  const int lit = phase < 0.82 ? (int)(phase / 0.82 * n) + 1 : 0;
  const int inner = left ? band_x + band_w - 30 : band_x + 30;

  for (int i = 0; i < n; i++) {
    const int x = inner + dir * i * step;
    QColor c = color;
    c.setAlpha(i < lit ? 255 : 50);
    QPainterPath chevron;
    chevron.moveTo(x, cy - ch);
    chevron.lineTo(x + dir * cw, cy);
    chevron.lineTo(x, cy + ch);
    p.strokePath(chevron, QPen(c, 22, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  }

  if (!label.isEmpty()) {
    configFont(p, "Open Sans", 40, "Bold");
    p.setPen(color);
    p.drawText(QRect(band_x, cy + ch + 30, band_w, 60), Qt::AlignCenter, label);
  }

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
  // 레이아웃(세로형 + 오른쪽 아래 디버그 표시 위쪽)
  // =========================
  const int tile_w = 185;
  // Space between one tile's label and the next tile's reading. The layout
  // this replaced left 38 px here; halved from the 32 px first tried, since
  // that band was the most visibly empty part of the panel.
  const int gap = 16;
  const int pad = 14;

  // Size each tile to the glyphs' actual ink, not to the font's line box.
  // Open Sans reports ascent 1.07em + descent 0.29em, but every string here
  // is digits/caps with no descender, so its ink is only cap height
  // (~0.71em). Laying out by line box therefore left ~45% of each tile
  // structurally empty, which is what made the panel look padded out.
  //
  // Measured from fixed reference strings, never from the live values, so a
  // changing reading (9% -> 100%, 5C -> 45C) can never change the tile
  // height and shift the whole panel. "8" covers digits, "°" and "%" the
  // tallest symbols the value line can show; labels are all caps.
  const int value_label_gap = 20;
  configFont(p, "Open Sans", 56, "Bold");
  const int val_ink_h = QFontMetrics(p.font()).tightBoundingRect("8°%").height();
  configFont(p, "Open Sans", 31, "Bold");
  const int lab_ink_h = QFontMetrics(p.font()).tightBoundingRect("A").height();

  const int tile_h = val_ink_h + value_label_gap + lab_ink_h;
  const int total_w = tile_w;
  const int total_h = tile_h * 3 + gap * 2;

  // 오른쪽 여백 기준 고정. 작은 해상도에서는 최소 x=35 보장.
  const int x_calc = width() - tile_w - 35;
  const int x = x_calc > 35 ? x_calc : 35;

  // 화면 맨 아래로 내림. 배경 사각형의 아래변이 하단 디버그 텍스트의
  // baseline(paintGL에서 rect().height() - 15에 그림)과 같은 높이에 오도록
  // 맞춰서, 둘이 하나의 하단 줄처럼 읽히게 한다. 배경은 타일보다 pad 만큼
  // 더 내려가므로 그만큼 빼 준다.
  // 디버그 텍스트는 왼쪽(left + 20)에서 시작하고 이 패널은 오른쪽 끝에
  // 붙으므로 같은 높이에 있어도 가로로 겹치지 않는다.
  constexpr int debug_text_up = 15;
  const int y_calc = rect().bottom() - debug_text_up - pad - total_h;
  const int y = y_calc > 80 ? y_calc : 80;

  // Publish the panel's box for the status column drawBottomIcons stacks on
  // top of it (see the members' comment in onroad.h).
  thermal_panel_top_ = y - pad;
  thermal_panel_cx_ = x + tile_w / 2;

  // ✅ 배경: 투명 검정 + 라운드
  QRect bg_rect(x - pad, y - pad, total_w + pad * 2, total_h + pad * 2);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(bg_rect, 18, 18);

  // Draw each string from its own ink box rather than centring it in a rect:
  // the rect form re-introduces exactly the line-box padding the tile heights
  // above were sized to remove. Baselines sit at the bottom of each reference
  // ink block (valid because none of these strings has a descender), and the
  // horizontal centre uses the string's own ink width so the digits look
  // optically centred rather than advance-centred.
  auto drawTile = [&](int ty, const QString &value, const QString &label, const QColor &valColor) {
    configFont(p, "Open Sans", 56, "Bold");
    const QRect vi = QFontMetrics(p.font()).tightBoundingRect(value);
    p.setPen(valColor);
    p.drawText(x + (tile_w - vi.width()) / 2 - vi.left(), ty + val_ink_h, value);

    configFont(p, "Open Sans", 31, "Bold");
    const QRect li = QFontMetrics(p.font()).tightBoundingRect(label);
    p.setPen(QColor(0, 255, 0, 220));
    p.drawText(x + (tile_w - li.width()) / 2 - li.left(),
               ty + val_ink_h + value_label_gap + lab_ink_h, label);
  };

  // =========================
  // BAT
  // =========================
  QString batStr;
  batStr.sprintf("%d%%", deviceState.getBatteryPercent());

  int r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  int g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  drawTile(y, batStr, "BAT.L", QColor(r, g, 200, 220));

  // =========================
  // CPU
  // =========================
  QString cpuStr;
  cpuStr.sprintf("%.0f°C", cpuTemp);

  r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  drawTile(y + (tile_h + gap), cpuStr, "CPU", QColor(r, g, 200, 220));

  // =========================
  // AMBIENT
  // =========================
  QString ambStr;
  ambStr.sprintf("%.0f°C", ambientTemp);

  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  drawTile(y + (tile_h + gap) * 2, ambStr, "AMBIENT", QColor(r, g, 200, 220));

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

  int y = 80;
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
