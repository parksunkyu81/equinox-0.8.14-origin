#pragma once

#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

// Live view of the forward camera, with a capture button. The driver camera
// already had a preview; this is the one that matters for how the road is
// seen -- mount angle, glare off the dashboard, anything on the lens.
class RoadViewWindow : public QWidget {
  Q_OBJECT

public:
  explicit RoadViewWindow(QWidget *parent = 0);

signals:
  void done();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  void capture();

  CameraViewWidget *cameraView;
  QLabel *status;
  Params params;
};
