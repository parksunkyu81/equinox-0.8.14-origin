#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>
#include <algorithm>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

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
  }
  else if (scene.end_to_end) {
    const auto &orientation = (*s->sm)["modelV2"].getModelV2().getOrientation();
    float orientation_future = 0;
    if (orientation.getZ().size() > 16) {
      orientation_future = std::abs(orientation.getZ()[16]);  // 2.5 seconds
    }
    // straight: 112, in turns: 70
    float curve_hue = fmax(70, 112 - (orientation_future * 420));
    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    curve_hue = int(curve_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.75 / 1.5, QColor::fromHslF(curve_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(curve_hue / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0, whiteColor(200));
    bg.setColorAt(1, whiteColor(0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  painter.restore();
}*/

// 차선 흰색→녹색, PATH는 E2E 여부와 무관하게 항상 녹/노/빨 그라데이션, painter save/restore 짝 정상화
void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;

  // 1) lanelines: WHITE -> GREEN (alpha = prob, max 0.7)
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    const float a = std::clamp<float>(scene.lane_line_probs[i], 0.0f, 0.7f);
    painter.setBrush(QColor::fromRgbF(0.0, 1.0, 0.0, a));
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // road edges (원본 유지)
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0.0, 0.0, std::clamp<float>(1.0f - scene.road_edge_stds[i], 0.0f, 1.0f)));
    painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }

  // 2) PATH: ALWAYS green~yellow~red gradient (accel/curve based)
  const auto model = (*s->sm)["modelV2"].getModelV2();

  float accel_future = 0.f;
  const auto &acc = model.getAcceleration();
  if (acc.getX().size() > 16) {
    accel_future = acc.getX()[16];  // ~2.5s
  }

  float orient_future = 0.f;
  const auto &ori = model.getOrientation();
  if (ori.getZ().size() > 16) {
    orient_future = std::abs(ori.getZ()[16]);  // ~2.5s
  }

  // accel: + => greener, - => redder (0~120)
  float hue_acc = std::clamp(60.f + accel_future * 30.f, 0.f, 120.f);
  // curve: bigger => redder (0~120)
  float hue_curve = std::clamp(120.f - orient_future * 600.f, 0.f, 120.f);

  float end_hue = std::min(hue_acc, hue_curve);
  end_hue = int(end_hue * 100.f + 0.5f) / 100.f;

  QLinearGradient bg(0, height(), 0, height() / 4);
  bg.setColorAt(0.0, QColor::fromHslF(120.f / 360.f, 0.97, 0.56, 0.45));    // green
  bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360.f, 1.00, 0.68, 0.35));  // mid
  bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360.f, 1.00, 0.68, 0.00));  // fade

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  painter.restore();
}


void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar) {
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
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
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

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
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

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);

  drawLaneLines(p, s);

  auto leads = sm["modelV2"].getModelV2().getLeadsV3();
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0]);
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

  const auto controls_state = sm["controlsState"].getControlsState();
  //const auto device_State = sm["deviceState"].getDeviceState();
  //const auto car_control = sm["carControl"].getCarControl();
  //const auto live_params = sm["liveParameters"].getLiveParameters();
  //const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  //const auto torque_state = controls_state.getLateralControlState().getTorqueState();

  //QColor orangeColor = QColor(52, 197, 66, 255);

  /*float cpuTemp = 0;
  auto cpuList = device_State.getCpuTempC();

  if (cpuList.size() > 0) {
     for(int i = 0; i < cpuList.size(); i++)
         cpuTemp += cpuList[i];
     cpuTemp /= cpuList.size();
  }

  int cpuUsage = 0;
  auto cpuUsageList = device_State.getCpuUsagePercent();

  if (cpuUsageList.size() > 0) {
     for(int i = 0; i < cpuUsageList.size(); i++)
         cpuUsage += cpuUsageList[i];
     cpuUsage /= cpuUsageList.size();
  }*/

  // BAT(%d) HW(CPU %.1f ℃, %d, MEM %d)
  /*
  device_State.getBatteryPercent(),
                      cpuTemp,
                      cpuUsage,
                      device_State.getMemoryUsagePercent(),
  */
  QString infoText;
  infoText.sprintf("TORQUE(LatAccel:%.2f,Friction:%.2f) TCO(%.2f) SR(%.2f) SAD(%.2f) CURVE(%.2f) MIN_TR(%.1f) DF_MOD(%.1f)",
                      controls_state.getLatAccelFactor(),
                      controls_state.getFriction(),
                      controls_state.getTotalCameraOffset(),
                      controls_state.getSteerRatio(),
                      controls_state.getSteerActuatorDelay(),
                      controls_state.getSccCurvatureFactor(),
                      controls_state.getMinTR(),
                      controls_state.getGlobalDfMod()
                      );


  // info
  configFont(p, "Open Sans", 43, "Regular");
  p.setPen(QColor(0, 255, 0, 255));
  p.drawText(rect().left() + 20, rect().height() - 15, infoText);


  drawBottomIcons(p);
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();
  auto controls_state = sm["controlsState"].getControlsState();

  // 하단 원형 2줄 시작점
  const int icon_start_x = 600;

  // 1. 핸들 토크 각도
  int x = icon_start_x;
  const int y1 = rect().bottom() - footer_h / 2 - 10;

  float cur_speed = std::max(0.0, car_state.getVEgo() * MS_TO_KPH);
  QString str;
  QString str2;
  float img_alpha;
  float bg_alpha;
  QColor textColor = QColor(255, 255, 255, 200);

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

  // 2. VISION DIST
  x = radius / 2 + (bdr_s * 2) + (radius + 50);

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  auto lead_vision = sm["modelV2"].getModelV2().getLeadsV3()[0];
  float vision_dist = lead_vision.getProb() > .5 ? (lead_vision.getX()[0] - 1.5) : 0;
  //float vision_second = vision_dist / cur_speed;    // [거리 / 속력]

  textSize = 48.f;

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

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y1-20, "DIST", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y1+50, str, textColor);
  p.setOpacity(1.0);

  // 3. LKAS
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 2);
  bool lkas_bool = car_state.getLkasEnable();

  textSize = 48.f;

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  if(lkas_bool == true and cur_speed > 10) {
    str = "ON";
    textColor = QColor(120, 255, 120, 200);
  }
  else if(lkas_bool == true and cur_speed <= 10) {
    str = "OFF";
    textColor = QColor(254, 32, 32, 200);
  }
  else {
    str = "OFF";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y1-20, "LKAS", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y1+50, str, textColor);
  p.setOpacity(1.0);

  // 4.auto hold
  int autohold = car_state.getAutoHold();
  if(autohold >= 0) {
    x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 3);
    img_alpha = autohold > 0 ? 1.0f : 0.15f;
    bg_alpha = autohold > 0 ? 0.3f : 0.1f;
    drawIcon(p, x, y1, autohold > 1 ? ic_autohold_warning : ic_autohold_active,
            QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
    p.setOpacity(1.0);
  }


  // ================================================================================================================ //
  x = 140;
  const int y2 = rect().bottom() - (footer_h / 2) - (radius + 50) - 10;

  // 1.TR Value
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


  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y2-20, str, 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str2, textColor);
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

  // 2. PEDAL
  x = radius / 2 + (bdr_s * 2) + (radius + 50);
  float accel = car_control.getActuators().getAccel();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  if(accel > 0) {
    str = "ACCEL";
    textColor = QColor(120, 255, 120, 200);
  }
  else if(accel == 0.0) {
    str = "──";
    textColor = QColor(255, 185, 15, 200);
  }
  else {
    str = "DECEL";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y2-20, "PEDAL", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

  // 3. ACC
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 2);
  bool acc_bool = car_state.getAdaptiveCruise();
  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textColor = QColor(255, 255, 255, 200);

  if(acc_bool == true) {
    str = "ON";
    textColor = QColor(120, 255, 120, 200);
  }
  else {
    str = "OFF";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y2-20, "ACC", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

  // 4. brake
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 3);
  bool brake_valid = car_state.getBrakePressed();
  img_alpha = brake_valid ? 1.0f : 0.15f;
  bg_alpha = brake_valid ? 0.3f : 0.1f;
  drawIcon(p, x, y2, ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  p.setOpacity(1.0);

  // 5. long control state
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 4);
  int longControlState = (int)controls_state.getLongControlState();
  const char* long_state[] = {"off", "pid", "stopping", "starting"};
  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(200));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  str = long_state[longControlState];
  textColor = QColor(120, 255, 120, 200);

  configFont(p, "Open Sans", 38, "Bold");
  drawText(p, x, y2-20, "LONG", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

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
  const float conv = s->scene.is_metric ? (float)MS_TO_KPH : (float)MS_TO_MPH;
  const float cur_speed = std::max(0.0f, v_ego * conv);

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

  auto to_display_speed = [&](float kph) -> int {
    if (kph <= 0.f) return 0;
    if (s->scene.is_metric) return (int)(kph + 0.5f);
    return (int)(kph * (float)KM_TO_MILE + 0.5f);
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

  // 3줄 배치 + 중간 여백
  const int midGapY = 25;

  int y_cruise = (int)(panelContent.top() + panelContent.height() * 0.30) - midGapY;
  int y_curspd = (int)(panelContent.top() + panelContent.height() * 0.55);
  int y_apply  = (int)(panelContent.top() + panelContent.height() * 0.80) + midGapY;

  // 폰트
  const int unifiedFont = 70;     // Cruise/Apply
  const int unifiedSpdFont = 100;  // Current Speed

  // Colors
  QColor cruiseGreen(120, 255, 120, 200);
  QColor naWhite(255, 255, 255, 180);
  QColor applyOrange(255, 127, 0, 200);

  // Cruise
  QString strCruise;
  configFont(p, "Inter", unifiedFont, "Bold");
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
  drawTextWithColor(p, panel_cx, y_curspd, strCur, curSpeedColor);

  // Apply
  QString strApply;
  if (is_cruise_set && applyMaxSpeed_kph > 0.f) {
    strApply.sprintf("%d", to_display_speed(applyMaxSpeed_kph));
  } else {
    strApply = "MAX";
  }
  configFont(p, "Inter", unifiedFont, "Bold");
  drawTextWithColor(p, panel_cx, y_apply, strApply, applyOrange);

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
  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.setOpacity(1.);
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
  // 레이아웃(가로형 + 좌상단 여유)
  // =========================
  const int x = 35;
  const int y = 425;   // 기존 25 -> 425 로 변경

  const int tile_w = 185;
  const int tile_h = 145;

  const int gap = 20;   // ✅ 타일 사이 간격: 18 -> 20 (10% 증가 반영)
  const int pad = 16;

  const int total_w = tile_w * 3 + gap * 2;
  const int total_h = tile_h;

  // ✅ 배경: 투명 검정 + 라운드
  QRect bg_rect(x - pad, y - pad, total_w + pad * 2, total_h + pad * 2);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(bg_rect, 18, 18);

  auto drawTile = [&](int tx, const QString &value, const QString &label, const QColor &valColor) {
    const int label_h = 48;                 // 라벨 영역 높이
    const int value_label_gap = 5;          // ✅ 값/라벨 사이 세로 여백(약 10% 느낌으로 추가)

    // ✅ valueRect 아래를 줄여서 labelRect와 사이에 빈 공간을 만들기
    QRect valueRect(tx, y, tile_w, tile_h - label_h - value_label_gap);
    QRect labelRect(tx, y + tile_h - label_h, tile_w, label_h);

    // ✅ 값 폰트: 20% 확대 유지
    configFont(p, "Open Sans", 60, "Bold");
    p.setPen(valColor);
    p.drawText(valueRect, Qt::AlignCenter, value);

    // ✅ 라벨 폰트: 20% 확대 유지
    configFont(p, "Open Sans", 35, "Bold");
    p.setPen(QColor(0, 255, 0, 220));
    p.drawText(labelRect, Qt::AlignCenter, label);
  };

  // =========================
  // BAT
  // =========================
  QString batStr;
  batStr.sprintf("%d%%", deviceState.getBatteryPercent());

  int r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  int g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  drawTile(x, batStr, "BAT.L", QColor(r, g, 200, 220));

  // =========================
  // CPU
  // =========================
  QString cpuStr;
  cpuStr.sprintf("%.0f°C", cpuTemp);

  r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  drawTile(x + (tile_w + gap), cpuStr, "CPU", QColor(r, g, 200, 220));

  // =========================
  // AMBIENT
  // =========================
  QString ambStr;
  ambStr.sprintf("%.0f°C", ambientTemp);

  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  drawTile(x + (tile_w + gap) * 2, ambStr, "AMBIENT", QColor(r, g, 200, 220));

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

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

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
