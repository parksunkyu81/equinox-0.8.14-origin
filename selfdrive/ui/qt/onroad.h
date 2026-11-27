#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"

#include <QTimer>
#include <QMap>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"

// Right-hand onroad column, bottom up: turn-info box (carrot's spot), the
// thermal strip above it, the brake / ACC / auto-hold / LKAS status block
// above that. Computed in onroad.cc.
constexpr int THERMAL_W = 338;   // 2 x 140 + 14 status block width, +15%
constexpr int THERMAL_H = 89;    // 77 px it started at, +15%
struct RightColumnLayout {
  QRect turn_box;
  int status_bottom = 0;    // bottom edge of the brake / ACC / auto-hold / LKAS block
  int thermal_top = 0;      // the thermal strip's background, top and bottom
  int thermal_bottom = 0;
};
RightColumnLayout rightColumnLayout(int w, int h);


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

  // 페달 강도 tile: tapping it cycles CommaPedalResistance. Returns true when
  // pos (in this widget's coordinates) is inside the tile as it was last
  // drawn, and that drawing is recent enough to still be on screen.
  bool hitPedalTile(const QPoint &pos) const;
  // The tile re-reads CommaPedalResistance once a second; this makes the next
  // frame read it at once.
  void refreshPedalProfile() { pedal_profile_refresh_ = true; update(); }
  // 자동 출발 tile: tapping it flips ActiveStopAccelBoost. Same rules as
  // hitPedalTile; refreshPedalProfile also re-reads this tile's state.
  bool hitAutoLaunchTile(const QPoint &pos) const;
  // TR tile: tapping it cycles FollowGapLevel SHT -> MID -> LONG. Same rules
  // as hitPedalTile; refreshPedalProfile also re-reads this tile's level.
  bool hitTrTile(const QPoint &pos) const;
  // 페달 신호 tile as last drawn (this widget's coordinates; empty until the
  // first HUD frame). OnroadWindow parks the record button above it.
  QRect pedalSignalTileRect() const { return pedal_signal_tile_rect_; }

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  // danger: 0 safe (cyan corners), 1 close (orange), 2 too close (red).
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data,
                const QPointF &vd, const QString &info_text = {}, int danger = 0);
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
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;
  QPixmap ic_turn_signal_l;
  QPixmap ic_turn_signal_r;
  QPixmap ic_satellite;
  QPixmap ic_acc;
  QPixmap ic_lkas;
  QPixmap ic_wheel;

  // AutoTurnControl turn-info box (carrot hud_renderer _draw_turn_info_hud).
  QPixmap ic_nav_turn_l;
  QPixmap ic_nav_turn_r;
  QPixmap ic_nav_fork_l;
  QPixmap ic_nav_fork_r;
  QPixmap ic_nav_uturn;
  void drawTurnInfo(QPainter &p);

  void drawMaxSpeed(QPainter &p);
  void drawSpeed(QPainter &p);
  void drawTrafficLight(QPainter &p);
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
  // Bottom edge of the speed panel; drawSpeedLimit lines the sign / camera
  // board's distance pill up with it.
  int speed_panel_bottom_ = 0;

  // Where the NDA/HDA badge sits above the speed-limit board. drawSpeedLimit
  // publishes it before its own early return, so the pedal tiles stacked above
  // it keep their place on the roads where no limit is being shown at all.
  int nda_badge_top_ = 0;
  int nda_badge_cx_ = 0;

  // Where the 페달 강도 tile was last drawn, and when, for hitPedalTile.
  QRect pedal_tile_rect_;
  QRect pedal_signal_tile_rect_;
  double pedal_tile_drawn_t_ = 0;
  bool pedal_profile_refresh_ = false;
  // Where the 자동 출발 tile was last drawn, and the toggle it shows.
  QRect auto_launch_tile_rect_;
  double auto_launch_tile_drawn_t_ = 0;
  bool auto_launch_on_ = false;
  // Where the TR tile was last drawn, and the FollowGapLevel it shows.
  QRect tr_tile_rect_;
  double tr_tile_drawn_t_ = 0;
  QString follow_gap_level_ = "MID";
  void drawSpeedLimit(QPainter &p);
  void drawSteer(QPainter &p);
  void drawTurnSignals(QPainter &p);
  void drawBlindSpot(QPainter &p);
  void drawGpsStatus(QPainter &p);
  void drawDebugText(QPainter &p);
  void drawThermal(QPainter &p);
  void drawHud(QPainter &p);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

  void paintEvent(QPaintEvent *event) override;

private:
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QHBoxLayout* split;

  // neokii
private:
  ScreenRecoder* recorder = nullptr;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;
  void placeRecorder();

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
