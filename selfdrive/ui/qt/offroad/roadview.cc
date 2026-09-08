#include "selfdrive/ui/qt/offroad/roadview.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QImage>
#include <QStackedLayout>
#include <QVBoxLayout>

// Where a captured frame lands. /data/media/0 is the storage the drive
// segments already live on, so it survives a reboot and can be pulled off
// the device the same way a route is.
const QString CAPTURE_DIR = "/data/media/0";

RoadViewWindow::RoadViewWindow(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);

  QStackedLayout *layout = new QStackedLayout(this);
  layout->setStackingMode(QStackedLayout::StackAll);

  cameraView = new CameraViewWidget("camerad", VISION_STREAM_RGB_BACK, false, this);
  layout->addWidget(cameraView);

  // Controls sit on top of the camera. Transparent everywhere the buttons are
  // not, so the frame underneath stays visible.
  QWidget *overlay = new QWidget(this);
  overlay->setAttribute(Qt::WA_TranslucentBackground);
  QVBoxLayout *ol = new QVBoxLayout(overlay);
  ol->setContentsMargins(40, 40, 40, 40);

  status = new QLabel("camera starting", overlay);
  status->setStyleSheet("font-size: 40px; color: white; background-color: rgba(0, 0, 0, 150); padding: 16px; border-radius: 8px;");
  status->setAlignment(Qt::AlignCenter);
  ol->addWidget(status, 0, Qt::AlignTop | Qt::AlignHCenter);
  ol->addStretch();

  QHBoxLayout *btns = new QHBoxLayout();
  QPushButton *shot = new QPushButton("Take photo", overlay);
  QPushButton *close = new QPushButton("Close", overlay);
  const QString btn_style = "QPushButton { font-size: 45px; font-weight: 500; color: white; background-color: #393939; border-radius: 10px; padding: 30px 60px; }";
  shot->setStyleSheet(btn_style);
  close->setStyleSheet(btn_style);
  btns->addWidget(shot);
  btns->addStretch();
  btns->addWidget(close);
  ol->addLayout(btns);

  layout->addWidget(overlay);
  layout->setCurrentWidget(overlay);

  QObject::connect(shot, &QPushButton::clicked, this, &RoadViewWindow::capture);
  QObject::connect(close, &QPushButton::clicked, this, &RoadViewWindow::done);
  QObject::connect(cameraView, &CameraViewWidget::vipcThreadFrameReceived, [=]() {
    if (status->text() == "camera starting") {
      status->setText("live");
    }
  });
}

void RoadViewWindow::showEvent(QShowEvent *event) {
  status->setText("camera starting");
  // Same flag the driver preview uses. camerad is marked driverview=True in
  // process_config, so manager brings it up offroad for either preview and
  // stops it again on the way out.
  params.putBool("IsDriverViewEnabled", true);
}

void RoadViewWindow::hideEvent(QHideEvent *event) {
  params.putBool("IsDriverViewEnabled", false);
}

void RoadViewWindow::capture() {
  // grabFramebuffer() reads back what was actually drawn, so the saved file is
  // the frame on screen rather than a second, differently exposed grab.
  QImage img = cameraView->grabFramebuffer();
  if (img.isNull()) {
    status->setText("no frame yet");
    return;
  }

  QString fn = CAPTURE_DIR + "/camera_" +
               QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".jpg";
  if (img.save(fn, "JPEG", 90)) {
    status->setText("saved " + fn);
  } else {
    status->setText("could not write " + fn);
  }
}
