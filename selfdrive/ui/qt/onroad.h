#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"

#include <QTimer>
#include <QMap>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"


// ***** onroad widgets *****

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class NvgWindow : public CameraViewWidget {
  Q_OBJECT

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0);

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data,
                const QPointF &vd, bool is_radar, const QString &info_text = {});
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  double prev_draw_t = 0;
  double last_slow_frame_log_t = 0;
  FirstOrderFilter fps_filter;

  uint64_t last_update_params;

  // neokii
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color);
  void drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color);
  void paintEvent(QPaintEvent *event) override;

  // 원형 사이즈
  //const int radius = 192;
  const int radius = 116;
  const int img_size = (radius / 2) * 1.5;

  // neokii
  QPixmap ic_brake;
  QPixmap ic_autohold_warning;
  QPixmap ic_autohold_active;
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;
  QPixmap ic_turn_signal_l;
  QPixmap ic_turn_signal_r;
  QPixmap ic_satellite;
  QPixmap ic_acc;
  QPixmap ic_lkas;
  QPixmap ic_wheel;

  QMap<QString, QPixmap> ic_oil_com;

  void drawMaxSpeed(QPainter &p);
  void drawSpeed(QPainter &p);
  void drawBottomIcons(QPainter &p);
  void drawSteerGauge(QPainter &p, int cx, int cy, int w);
  void drawConfidenceGauge(QPainter &p, int cx, int top_y, int bottom_y);

  // Where drawThermal put its panel this frame, so the status icons stacked
  // above it can line up without re-deriving the panel's font-metric-driven
  // height. drawThermal runs before drawBottomIcons in paintGL, so these are
  // current by the time the stack is drawn.
  int thermal_panel_top_ = 0;
  int thermal_panel_cx_ = 0;
  int thermal_panel_right_ = 0;

  // Same idea for the speed panel: drawSpeed runs before drawBottomIcons in
  // drawHud, so what anchors off it sees its real box rather than re-deriving
  // a font-metric-driven layout. Only the right edge is read now -- the wheel
  // is placed off it; the pedal tiles that used to sit above this panel moved
  // to the NDA badge.
  int speed_panel_top_ = 0;
  int speed_panel_cx_ = 0;
  int speed_panel_right_ = 0;

  // Where the NDA/HDA badge sits above the speed-limit board. drawSpeedLimit
  // publishes it before its own early return, so the pedal tiles stacked above
  // it keep their place on the roads where no limit is being shown at all.
  int nda_badge_top_ = 0;
  int nda_badge_cx_ = 0;
  void drawSpeedLimit(QPainter &p);
  void drawSteer(QPainter &p);
  void drawRestArea(QPainter &p);
  void drawTurnSignals(QPainter &p);
  void drawGpsStatus(QPainter &p);
  void drawDebugText(QPainter &p);
  void drawThermal(QPainter &p);
  void drawHud(QPainter &p);

private:
  QPixmap get_icon_iol_com(const char* key);
  void drawRestAreaItem(QPainter &p, int yPos, capnp::Text::Reader image, capnp::Text::Reader title,
                        capnp::Text::Reader oilPrice, capnp::Text::Reader distance, bool lastItem);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

  void paintEvent(QPaintEvent *event) override;

private:
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

  // neokii
private:
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
