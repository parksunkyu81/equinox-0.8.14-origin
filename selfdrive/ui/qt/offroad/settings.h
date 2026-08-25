#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>
#include <QStackedLayout>


#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/qt/widgets/controls.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void reviewTrainingGuide();
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void reviewTrainingGuide();
  void showDriverView();
  void closeSettings();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  LabelControl *gitBranchLbl;
  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class C2NetworkPanel: public QWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  LabelControl *ipaddress;
};




class SelectCar : public QWidget {
  Q_OBJECT
public:
  explicit SelectCar(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selectedCar();

};

class LateralControl : public QWidget {
  Q_OBJECT
public:
  explicit LateralControl(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();

};

class DynamicTRGap : public QWidget {
  Q_OBJECT
public:
  explicit DynamicTRGap(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();

};

class MinTR : public QWidget {
  Q_OBJECT
public:
  explicit MinTR(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();

};

class GlobalDfMod : public QWidget {
  Q_OBJECT
public:
  explicit GlobalDfMod(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();

};

// Live view of the road-facing (rear) camera, for checking lens condition --
// haze, dirt, condensation or focus directly degrade laneLineProbs, since the
// model is fed these raw frames. Runs offroad by holding IsDriverViewEnabled,
// which is what starts camerad while parked; camerad publishes the rear RGB
// stream unconditionally once running.
class TestCamera : public QWidget {
  Q_OBJECT
public:
  explicit TestCamera(QWidget* parent = 0);

protected:
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

private:
  Params params;
  CameraViewWidget* cameraView = nullptr;
  QLabel* statusLabel = nullptr;

signals:
  void backPress();
};

class CommunityPanel : public QWidget {
  Q_OBJECT

private:
  QStackedLayout* main_layout = nullptr;
  QWidget* homeScreen = nullptr;
  SelectCar* selectCar = nullptr;
  LateralControl* lateralControl = nullptr;
  DynamicTRGap* dynamicTRGap = nullptr;
  MinTR* minTR = nullptr;
  GlobalDfMod* globalDfMod = nullptr;
  TestCamera* testCamera = nullptr;

  QWidget* homeWidget;

public:
  explicit CommunityPanel(QWidget *parent = nullptr);
};

