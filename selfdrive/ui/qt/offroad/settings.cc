#include "selfdrive/ui/qt/offroad/settings.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QScroller>
#include <QListView>
#include <QListWidget>

namespace {
constexpr double kTorqueLatAccelFactorMin = 0.5;
constexpr double kTorqueLatAccelFactorMax = 4.5;
constexpr double kTorqueLatAccelFactorStep = 0.01;
constexpr double kTorqueLatAccelFactorDefault = 2.0;
const QString kTorqueTunePath = "/data/ntune/lat_torque_v4.json";

double clampTorqueLatAccelFactor(double value) {
  return std::max(kTorqueLatAccelFactorMin, std::min(value, kTorqueLatAccelFactorMax));
}

double readTorqueLatAccelFactor() {
  double value = kTorqueLatAccelFactorDefault;
  QFile tune_file(kTorqueTunePath);
  if (tune_file.open(QIODevice::ReadOnly)) {
    const QJsonValue tune_value = QJsonDocument::fromJson(tune_file.readAll())
                                  .object().value("latAccelFactor");
    if (tune_value.isDouble()) {
      value = tune_value.toDouble();
    }
  }
  return clampTorqueLatAccelFactor(value);
}

void writeTorqueLatAccelFactor(double value) {
  value = std::round(clampTorqueLatAccelFactor(value) * 100.0) / 100.0;

  QJsonObject tune;
  QFile existing_tune(kTorqueTunePath);
  if (existing_tune.open(QIODevice::ReadOnly)) {
    tune = QJsonDocument::fromJson(existing_tune.readAll()).object();
  }
  tune.insert("latAccelFactor", value);

  QFile tune_file(kTorqueTunePath);
  if (tune_file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    tune_file.write(QJsonDocument(tune).toJson(QJsonDocument::Indented));
    tune_file.close();
  }
}
}  // namespace

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggles{
    {
      "OpenpilotEnabledToggle",
      "Enable openpilot",
      "Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off.",
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "IsLdwEnabled",
      "Enable Lane Departure Warnings",
      "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h).",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsRHD",
      "Enable Right-Hand Drive",
      "Allow openpilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat.",
      "../assets/offroad/icon_openpilot_mirrored.png",
    },
    {
      "IsMetric",
      "Use Metric System",
      "Display speed in km/h instead of mph.",
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      "Record and Upload Driver Camera",
      "Upload data from the driver facing camera and help improve the driver monitoring algorithm.",
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "DisengageOnAccelerator",
      "Disengage On Accelerator Pedal",
      "When enabled, pressing the accelerator pedal will disengage openpilot.",
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      "Show ETA in 24h format",
      "Use 24h format instead of am/pm",
      "../assets/offroad/icon_metric.png",
    },
#endif

  };

  Params params;

  if (params.getBool("DisableRadar_Allow")) {
    toggles.push_back({
      "DisableRadar",
      "openpilot Longitudinal Control",
      "openpilot will disable the car's radar and will take over control of gas and brakes. Warning: this disables AEB!",
      "../assets/offroad/icon_speed_limit.png",
    });
  }

  for (auto &[param, title, desc, icon] : toggles) {
    auto toggle = new ParamControl(param, title, desc, icon, this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    //if (!locked) {
    //  connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
    //}
    addItem(toggle);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl("Dongle ID", getDongleId().value_or("N/A")));
  addItem(new LabelControl("Serial", params.get("HardwareSerial").c_str()));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton("Soft restart");
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton("Reset Calibration");
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration and live params?", this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  addItem(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl("Driver Camera", "PREVIEW",
                                   "Preview the driver facing camera to help optimize device mounting position for best driver monitoring experience. (vehicle must be off)");
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", " ");
  connect(resetCalibBtn, &ButtonControl::showDescription, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      params.remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl("Review Training Guide", "REVIEW", "Review the rules, features, and limitations of openpilot");
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm("Are you sure you want to review the training guide?", this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl("Regulatory", "VIEW", "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      RichTextDialog::alert(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  /*QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      btn->setEnabled(offroad);
    }
  });*/

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton("Reboot");
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton("Power Off");
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (Hardware::TICI()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      "openpilot requires the device to be mounted within 4° left or right and "
      "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.";
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += QString(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "down" : "up",
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "left" : "right");
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm("Are you sure you want to reboot?", this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert("Disengage to Reboot", this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm("Are you sure you want to power off?", this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert("Disengage to Power Off", this);
  }
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  gitBranchLbl = new LabelControl("Git Branch");
  gitCommitLbl = new LabelControl("Git Commit");
  osVersionLbl = new LabelControl("OS Version");
  versionLbl = new LabelControl("Version", "", QString::fromStdString(params.get("ReleaseNotes")).trimmed());
  lastUpdateLbl = new LabelControl("Last Update Check", "", "The last time openpilot successfully checked for an update. The updater only runs while the car is off.");
  updateBtn = new ButtonControl("Check for Update", "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    if (params.getBool("IsOffroad")) {
      fs_watch->addPath(QString::fromStdString(params.getParamPath("LastUpdateTime")));
      fs_watch->addPath(QString::fromStdString(params.getParamPath("UpdateFailedCount")));
      updateBtn->setText("CHECKING");
      updateBtn->setEnabled(false);
    }
    std::system("pkill -1 -f selfdrive.updated");
  });


  auto uninstallBtn = new ButtonControl("Uninstall " + getBrand(), "UNINSTALL");
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm("Are you sure you want to uninstall?", this)) {
      params.putBool("DoUninstall", true);
    }
  });
  connect(uiState(), &UIState::offroadTransition, uninstallBtn, &QPushButton::setEnabled);

  QWidget *widgets[] = {versionLbl, lastUpdateLbl, updateBtn, gitBranchLbl, gitCommitLbl, osVersionLbl, uninstallBtn};
  for (QWidget* w : widgets) {
    addItem(w);
  }

  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    if (path.contains("UpdateFailedCount") && std::atoi(params.get("UpdateFailedCount").c_str()) > 0) {
      lastUpdateLbl->setText("failed to fetch update");
      updateBtn->setText("CHECK");
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  auto tm = params.get("LastUpdateTime");
  if (!tm.empty()) {
    lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
  }

  versionLbl->setText(getBrandVersion());
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText("CHECK");
  updateBtn->setEnabled(true);
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

C2NetworkPanel::C2NetworkPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);

  ListWidget *list = new ListWidget();
  list->setSpacing(30);
  // wifi + tethering buttons
#ifdef QCOM
  auto wifiBtn = new ButtonControl("Wi-Fi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  list->addItem(wifiBtn);

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  list->addItem(tetheringBtn);
#endif
  ipaddress = new LabelControl("IP Address", "");
  list->addItem(ipaddress);

  // SSH key management
  list->addItem(new SshToggle());
  list->addItem(new SshControl());
  layout->addWidget(list);
  layout->addStretch(1);
}

void C2NetworkPanel::showEvent(QShowEvent *event) {
  ipaddress->setText(getIPAddress());
}

QString C2NetworkPanel::getIPAddress() {
  std::string result = util::check_output("ifconfig wlan0");
  if (result.empty()) return "";

  const std::string inetaddrr = "inet addr:";
  std::string::size_type begin = result.find(inetaddrr);
  if (begin == std::string::npos) return "";

  begin += inetaddrr.length();
  std::string::size_type end = result.find(' ', begin);
  if (end == std::string::npos) return "";

  return result.substr(begin, end - begin).c_str();
}

QWidget *network_panel(QWidget *parent) {
#ifdef QCOM
  return new C2NetworkPanel(parent);
#else
  return new Networking(parent);
#endif
}

void SettingsWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("← Back");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      font-weight: bold;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(300, 110);
  sidebar_layout->addSpacing(10);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignRight);
  sidebar_layout->addSpacing(10);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);

  QList<QPair<QString, QWidget *>> panels = {
    {"Device", device},
    {"Network", network_panel(this)},
    {"Toggles", new TogglesPanel(this)},
    {"Software", new SoftwarePanel(this)},
    {"Community", new CommunityPanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({"Navigation", map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 60px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != "Network" ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}


/////////////////////////////////////////////////////////////////////////

TestCamera::TestCamera(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  statusLabel = new QLabel("camera starting", this);
  statusLabel->setObjectName("testCameraStatus");
  statusLabel->setAlignment(Qt::AlignCenter);
  // The saved-file message is a long absolute path. main_layout of
  // CommunityPanel is a QStackedLayout, which sizes to its widest child, so
  // without these this panel's width demand would widen the whole Community
  // menu -- same reason the labels in CommunityPanel's own layout set these.
  statusLabel->setWordWrap(true);
  statusLabel->setMinimumWidth(0);
  statusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  main_layout->addWidget(statusLabel, 0);

  // VISION_STREAM_RGB_BACK is the road-facing (rear) camera -- the same stream
  // onroad.cc's NvgWindow renders. zoom=false shows the uncropped frame, which
  // is what makes edge haze and lens dirt visible.
  cameraView = new CameraViewWidget("camerad", VISION_STREAM_RGB_BACK, false, this);
  cameraView->setMinimumWidth(0);
  cameraView->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  connect(cameraView, &CameraViewWidget::vipcThreadFrameReceived, this, [=](VisionBuf *) {
    if (statusLabel->isVisible() && !showing_capture_result) {
      statusLabel->hide();
    }
  });
  main_layout->addWidget(cameraView, 1);

  QPushButton* capture = new QPushButton("Capture");
  capture->setObjectName("testCameraCaptureBtn");
  capture->setFixedSize(500, 100);
  connect(capture, &QPushButton::clicked, [=]() { saveFrame(); });
  main_layout->addWidget(capture, 0, Qt::AlignHCenter);

  setStyleSheet(R"(
    #testCameraStatus {
      font-size: 45px;
      color: #dddddd;
      padding: 20px;
    }
    #testCameraCaptureBtn {
      font-size: 50px;
      padding: 20px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
  )");
}

void TestCamera::saveFrame() {
  // Read the vision stream directly instead of grabFramebuffer(). The on-screen
  // image is not the sensor data: cameraview.cc's fragment shader applies an EON
  // display compensation (dz = 0.0627, so black is lifted to 16/255 and the whole
  // range is compressed), and the widget letterboxes and rescales it. Measured on
  // a real capture: on-screen content occupied levels 16..187 while the letterbox
  // beside it was a true 0, and the same camera's raw fcamera.hevc frames span the
  // full 0..255. Saving the source buffer keeps the file usable for judging lens
  // haze/focus, which is the whole point of this screen.
  //
  // Its own short-lived client keeps this off CameraViewWidget's vipc thread --
  // that thread's frame pointer is only valid until it recycles the buffer, while
  // this runs entirely on the GUI thread where the button signal lands.
  VisionIpcClient vipc("camerad", VISION_STREAM_RGB_BACK, true);
  if (!vipc.connect(false)) {
    showing_capture_result = true;
    statusLabel->setText("capture failed: camerad not streaming yet");
    statusLabel->show();
    return;
  }

  VisionBuf *buf = vipc.recv(nullptr, 250);
  if (buf == nullptr || buf->addr == nullptr || buf->width == 0 || buf->height == 0) {
    showing_capture_result = true;
    statusLabel->setText("capture failed: no frame yet");
    statusLabel->show();
    return;
  }

  // camerad's RGB buffers are BGR24 with a row stride (rgb_to_yuv.cl reads
  // b,g,r from bytes 0,1,2). Qt 5.12 has no Format_BGR888, so wrap the same
  // bytes as RGB888 and swap. rgbSwapped() deep-copies, which also detaches
  // the image from the mapped buffer before it is recycled.
  if (buf->stride < buf->width * 3) {
    showing_capture_result = true;
    statusLabel->setText("capture failed: unexpected buffer stride");
    statusLabel->show();
    return;
  }
  QImage img = QImage((const uchar *)buf->addr, buf->width, buf->height,
                      buf->stride, QImage::Format_RGB888).rgbSwapped();
  if (img.isNull()) {
    showing_capture_result = true;
    statusLabel->setText("capture failed: could not build image");
    statusLabel->show();
    return;
  }

  // Same volume the drive logs live on (/data/media/0/realdata), so this is
  // reachable over scp and survives a reboot, unlike /tmp.
  const QString dir = "/data/media/0/camera_test";
  if (!QDir().mkpath(dir)) {
    showing_capture_result = true;
    statusLabel->setText("capture failed: cannot create " + dir);
    statusLabel->show();
    return;
  }

  const QString path = dir + "/camtest_" +
      QDateTime::currentDateTimeUtc().addSecs(9 * 3600).toString("yyyyMMdd_HHmmss") + ".png";
  showing_capture_result = true;
  if (img.save(path, "PNG")) {
    statusLabel->setText("saved: " + path);
  } else {
    statusLabel->setText("capture failed: cannot write " + path);
  }
  statusLabel->show();
}

void TestCamera::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  showing_capture_result = false;
  statusLabel->setText("camera starting");
  statusLabel->show();
  // camerad is a driverview process: offroad it only runs while this param is
  // set (see manager.py's ensure_running). Onroad it is already running and
  // this is a no-op that thermald's not_driver_view condition ignores, because
  // startup_conditions are only evaluated while offroad.
  params.putBool("IsDriverViewEnabled", true);
}

void TestCamera::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
  params.putBool("IsDriverViewEnabled", false);
}

CommunityPanel::CommunityPanel(QWidget* parent) : QWidget(parent) {

  main_layout = new QStackedLayout(this);

  homeScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
  vlayout->setContentsMargins(0, 20, 0, 20);

  //QString selected = QString::fromStdString(Params().get("SelectedCar"));

  //QPushButton* selectCarBtn = new QPushButton(selected.length() ? selected : "Select your car");
  //selectCarBtn->setObjectName("selectCarBtn");
  //selectCarBtn->setStyleSheet("margin-right: 30px;");
  //selectCarBtn->setFixedSize(350, 100);
  //connect(selectCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(selectCar); });
  //vlayout->addSpacing(10);
  //vlayout->addWidget(selectCarBtn, 0, Qt::AlignRight);
  //vlayout->addSpacing(10);

  homeWidget = new QWidget(this);
  QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
  homeWidget->setObjectName("homeWidget");
  homeWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  toggleLayout->setContentsMargins(0, 0, 0, 0);

  main_layout->addWidget(homeScreen);

  /*selectCar = new SelectCar(this);
  connect(selectCar, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(selectCar, &SelectCar::selectedCar, [=]() {

     QString selected = QString::fromStdString(Params().get("SelectedCar"));
     selectCarBtn->setText(selected.length() ? selected : "Select your car");
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(selectCar);*/

  QString following_distance_profile = QString::fromStdString(
    Params().get("FollowingDistanceProfile"));
  if (following_distance_profile != "short" && following_distance_profile != "mid" &&
      following_distance_profile != "long") {
    following_distance_profile = "mid";
    Params().put("FollowingDistanceProfile", "mid");
  }
  QLabel* followingDistanceProfileLabel = new QLabel(
    "Adjusting distance to the vehicle ahead", homeWidget);
  followingDistanceProfileLabel->setObjectName("followingDistanceProfileLabel");
  followingDistanceProfileLabel->setWordWrap(true);
  followingDistanceProfileLabel->setMinimumWidth(0);
  followingDistanceProfileLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  QComboBox* followingDistanceProfileCombo = new QComboBox(homeWidget);
  followingDistanceProfileCombo->setObjectName("followingDistanceProfileCombo");
  followingDistanceProfileCombo->addItems(QStringList() << "SHORT" << "MID" << "LONG");
  followingDistanceProfileCombo->setCurrentText(following_distance_profile.toUpper());
  connect(followingDistanceProfileCombo, &QComboBox::currentTextChanged,
          [=](const QString &selected) {
    Params().put("FollowingDistanceProfile", selected.toLower().toStdString());
  });
  QHBoxLayout* layoutBtn_0 = new QHBoxLayout();
  layoutBtn_0->setContentsMargins(0, 0, 0, 0);
  layoutBtn_0->addWidget(followingDistanceProfileLabel, 1);
  layoutBtn_0->addWidget(followingDistanceProfileCombo, 0);

  QString comma_pedal_resistance = QString::fromStdString(
    Params().get("CommaPedalResistance")).toLower();
  if (comma_pedal_resistance != "high" && comma_pedal_resistance != "mid" &&
      comma_pedal_resistance != "low") {
    comma_pedal_resistance = "mid";
    Params().put("CommaPedalResistance", "mid");
  }
  QLabel* commaPedalResistanceLabel = new QLabel(
    "Comma pedal resistance adjustment", homeWidget);
  commaPedalResistanceLabel->setObjectName("commaPedalResistanceLabel");
  commaPedalResistanceLabel->setWordWrap(true);
  commaPedalResistanceLabel->setMinimumWidth(0);
  commaPedalResistanceLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  QComboBox* commaPedalResistanceCombo = new QComboBox(homeWidget);
  commaPedalResistanceCombo->setObjectName("commaPedalResistanceCombo");
  commaPedalResistanceCombo->addItems(QStringList() << "HIGH" << "MID" << "LOW");
  commaPedalResistanceCombo->setCurrentText(comma_pedal_resistance.toUpper());
  connect(commaPedalResistanceCombo, &QComboBox::currentTextChanged,
          [=](const QString &selected) {
    Params().put("CommaPedalResistance", selected.toLower().toStdString());
  });
  QHBoxLayout* layoutBtn_5 = new QHBoxLayout();
  layoutBtn_5->setContentsMargins(0, 0, 0, 0);
  layoutBtn_5->addWidget(commaPedalResistanceLabel, 1);
  layoutBtn_5->addWidget(commaPedalResistanceCombo, 0);

  QString dynamicTR_Gap = QString::fromStdString(Params().get("DynamicTRGap"));
  if(dynamicTR_Gap.length() == 0)
    dynamicTR_Gap = "auto";
  QPushButton* dynamicTRGapBtn = new QPushButton("Dynamic Follow Profile : " + dynamicTR_Gap);
  dynamicTRGapBtn->setObjectName("dynamicTRGapBtn");
  dynamicTRGapBtn->setMinimumWidth(0);
  dynamicTRGapBtn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

  connect(dynamicTRGapBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(dynamicTRGap); });
  dynamicTRGap = new DynamicTRGap(this);
  connect(dynamicTRGap, &DynamicTRGap::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(dynamicTRGap, &DynamicTRGap::selected, [=]() {
     QString dynamicTR_gap = QString::fromStdString(Params().get("DynamicTRGap"));
     if(dynamicTR_gap.length() == 0)
       dynamicTR_gap = "auto";
     dynamicTRGapBtn->setText("Dynamic Follow Profile : " + dynamicTR_gap);
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(dynamicTRGap);
  QHBoxLayout* layoutBtn_1 = new QHBoxLayout();
  layoutBtn_1->setContentsMargins(0, 0, 0, 0);
  layoutBtn_1->addWidget(dynamicTRGapBtn);
  layoutBtn_1->addSpacing(10);

  // =============================================================================================================== //
  QString min_tr = QString::fromStdString(Params().get("minTR"));
  if(min_tr.length() == 0)
    min_tr = "0.9";
  QPushButton* minTrBtn = new QPushButton("Dynamic Follow Min TR (0.85 to 1.3, DEF:0.9) : " + min_tr);
  minTrBtn->setObjectName("minTrBtn");
  minTrBtn->setMinimumWidth(0);
  minTrBtn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

  connect(minTrBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(minTR); });
  minTR = new MinTR(this);
  connect(minTR, &MinTR::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(minTR, &MinTR::selected, [=]() {
     QString min_tr = QString::fromStdString(Params().get("minTR"));
     if(min_tr.length() == 0)
       min_tr = "0.9";
     minTrBtn->setText("Dynamic Follow Min TR (0.85 to 1.3, DEF:0.9) : " + min_tr);
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(minTR);
  QHBoxLayout* layoutBtn_2 = new QHBoxLayout();
  layoutBtn_2->setContentsMargins(0, 0, 0, 0);
  layoutBtn_2->addWidget(minTrBtn);
  layoutBtn_2->addSpacing(10);
  // =============================================================================================================== //
  QString global_df_mod = QString::fromStdString(Params().get("globalDfMod"));
  if(global_df_mod.length() == 0)
    global_df_mod = "1.0";
  QPushButton* globalDfModBtn = new QPushButton("Dynamic Follow multiplier (0.85 to 2.5) : " + global_df_mod);
  globalDfModBtn->setObjectName("globalDfModBtn");
  globalDfModBtn->setMinimumWidth(0);
  globalDfModBtn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

  connect(globalDfModBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(globalDfMod); });
  globalDfMod = new GlobalDfMod(this);
  connect(globalDfMod, &GlobalDfMod::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(globalDfMod, &GlobalDfMod::selected, [=]() {
     QString global_df_mod = QString::fromStdString(Params().get("globalDfMod"));
     if(global_df_mod.length() == 0)
       global_df_mod = "1.0";
     globalDfModBtn->setText("Dynamic Follow multiplier (0.85 to 2.5) : " + global_df_mod);
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(globalDfMod);
  QHBoxLayout* layoutBtn_4 = new QHBoxLayout();
  layoutBtn_4->setContentsMargins(0, 0, 0, 0);
  layoutBtn_4->addWidget(globalDfModBtn);
  layoutBtn_4->addSpacing(10);
  // =============================================================================================================== //

  QPushButton* testCameraBtn = new QPushButton("Test Camera");
  testCameraBtn->setObjectName("testCameraBtn");
  testCameraBtn->setMinimumWidth(0);
  testCameraBtn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  connect(testCameraBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(testCamera); });

  testCamera = new TestCamera(this);
  connect(testCamera, &TestCamera::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  main_layout->addWidget(testCamera);
  QHBoxLayout* layoutBtn_testCamera = new QHBoxLayout();
  layoutBtn_testCamera->setContentsMargins(0, 0, 0, 0);
  layoutBtn_testCamera->addWidget(testCameraBtn);
  layoutBtn_testCamera->addSpacing(10);
  // =============================================================================================================== //

  QString lateral_control = QString::fromStdString(Params().get("LateralControl"));
  if(lateral_control.length() == 0)
    lateral_control = "TORQUE";

  QPushButton* lateralControlBtn = new QPushButton(lateral_control);
  lateralControlBtn->setObjectName("lateralControlBtn");
  lateralControlBtn->setMinimumWidth(0);
  lateralControlBtn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  lateralControlBtn->hide();  // Temporarily keep the lateral control selector out of Community.
  connect(lateralControlBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(lateralControl); });

  lateralControl = new LateralControl(this);
  connect(lateralControl, &LateralControl::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(lateralControl, &LateralControl::selected, [=]() {

     QString lateral_control = QString::fromStdString(Params().get("LateralControl"));
     if(lateral_control.length() == 0)
       lateral_control = "TORQUE";
     lateralControlBtn->setText(lateral_control);
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(lateralControl);
  QHBoxLayout* layoutBtn_3 = new QHBoxLayout();
  layoutBtn_3->setContentsMargins(0, 0, 0, 0);
  layoutBtn_3->addWidget(lateralControlBtn);
  layoutBtn_3->addSpacing(10);
  // =============================================================================================================== //


  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn_3, 0);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn_1, 1);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn_2, 1);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn_4, 1);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn_testCamera, 1);
  vlayout->addSpacing(10);
  // SettingsWindow already wraps every panel in a ScrollView. Keeping a
  // second ScrollView here reduced the lower menu width by its viewport and
  // scrollbar, and created nested horizontal/vertical scrolling on EON.
  vlayout->addWidget(homeWidget, 0);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(0x29, 0x29, 0x29));
  setAutoFillBackground(true);
  setPalette(pal);

  setStyleSheet(R"(
    #back_btn, #selectCarBtn, #lateralControlBtn, #cruiseGapBtn, #dynamicTRGapBtn, #minTrBtn, #globalDfModBtn, #testCameraBtn {
      font-size: 50px;
      margin: 0px;
      padding: 20px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
    #followingDistanceProfileLabel, #commaPedalResistanceLabel {
      font-size: 42px;
      color: #dddddd;
      padding: 20px;
    }
    #torqueLatAccelLabel {
      font-size: 38px;
      color: #dddddd;
      padding: 20px;
    }
    #torqueLatAccelAdjustBtn, #torqueLatAccelPresetBtn {
      font-size: 38px;
      padding: 10px 20px;
      color: #dddddd;
      background-color: #444444;
      border: 0px;
      border-radius: 20px;
    }
    #followingDistanceProfileCombo, #commaPedalResistanceCombo {
      min-width: 300px;
      min-height: 100px;
      font-size: 45px;
      padding: 10px 25px;
      color: #dddddd;
      background-color: #444444;
      border: 0px;
      border-radius: 20px;
    }
    #followingDistanceProfileCombo QAbstractItemView, #commaPedalResistanceCombo QAbstractItemView {
      font-size: 45px;
      min-height: 300px;
      color: #dddddd;
      background-color: #393939;
      selection-background-color: #555555;
    }
  )");

  QList<ParamControl*> toggles;

  /*toggles.append(new ParamControl("UseClusterSpeed",
                                            "Use Cluster Speed",
                                            "Use cluster speed instead of wheel speed.",
                                            "../assets/offroad/icon_road.png",
                                            this));*/

  /*toggles.append(new ParamControl("LongControlEnabled",
                                            "Enable HKG Long Control",
                                            "warnings: it is beta, be careful!! Openpilot will control the speed of your car",
                                            "../assets/offroad/icon_road.png",
                                            this));*/

  /*toggles.append(new ParamControl("MadModeEnabled",
                                            "Enable Lead Safe speed Control",
                                            "For use in city driving or on blocked roads.",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));*/

  /*toggles.append(new ParamControl("IsLdwsCar",
                                            "LDWS",
                                            "If your car only supports LDWS, turn it on.",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));*/

  toggles.append(new ParamControl("LaneChangeEnabled",
                                            "Enable Lane Change Assist",
                                            "Perform assisted lane changes with openpilot by checking your surroundings for safety, activating the turn signal and gently nudging the steering wheel towards your desired lane. openpilot is not capable of checking if a lane change is safe. You must continuously observe your surroundings to use this feature.",
                                            "../assets/offroad/icon_road.png",
                                            this));

  toggles.append(new ParamControl("AutoLaneChangeEnabled",
                                            "Enable Auto Lane Change(Nudgeless)",
                                            "warnings: it is beta, be careful!!",
                                            "../assets/offroad/icon_road.png",
                                            this));

  toggles.append(new ParamControl("SccSmootherSlowOnCurves",
                                            "Enable Slow On Curves",
                                            "",
                                            "../assets/offroad/icon_road.png",
                                            this));

  /*toggles.append(new ParamControl("SccSmootherSyncGasPressed",
                                            "Sync set speed on gas pressed",
                                            "",
                                            "../assets/offroad/icon_road.png",
                                            this));*/

  /*toggles.append(new ParamControl("StockNaviDecelEnabled",
                                            "Stock Navi based deceleration",
                                            "Use the stock navi based deceleration for longcontrol",
                                            "../assets/offroad/icon_road.png",
                                            this));*/

  /*toggles.append(new ParamControl("KeepSteeringTurnSignals",
                                            "Keep steering while turn signals",
                                            "",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));*/

  /*toggles.append(new ParamControl("HapticFeedbackWhenSpeedCamera",
                                            "Haptic feedback (speed-cam alert)",
                                            "Haptic feedback when a speed camera is detected",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));*/


  toggles.append(new ParamControl("ShowDebugUI",
                                            "Show Debug UI",
                                            "",
                                            "../assets/offroad/icon_shell.png",
                                            this));

  toggles.append(new ParamControl("ActiveStopAccelBoost",
                                            "Activate stop accel boost",
                                            "Reduce lead-start delay and add up to 40% initial acceleration boost above 1 km/h.",
                                            "../assets/offroad/icon_road.png",
                                            this));

  /*toggles.append(new ParamControl("E2ELong",
                                          "Enable E2E Long",
                                          "Activate E2E Long. It may work unexpectedly. Be careful.",
                                          "../assets/offroad/icon_shell.png",
                                          this));*/

  /*toggles.append(new ParamControl("CustomTREnabled",
                                          "Custom TR Enable",
                                          "to use Custom TR not 1.45(comma default).",
                                          "../assets/offroad/icon_shell.png",
                                          this));*/

  toggles.append(new ParamControl("closeToRoadEdge",
                                          "Close to road edge",
                                          "",
                                          "../assets/offroad/icon_road.png",
                                          this));

  toggles.append(new ParamControl("DrivingStyleAI",
                                          "Driving Style AI Integration",
                                          "Learn the driver's acceleration, braking, and following preferences. Predictive coasting eases the comma pedal for a slowing lead, an approaching curve, or a speed limit while natural deceleration learns in shadow mode.",
                                          "../assets/offroad/icon_road.png",
                                          this));

  toggles.append(new ParamControl("PredictiveBrakeAlert",
                                          "Enable predictive brake alert",
                                          "Show a visual alert and play the configured Korean voice when learned natural deceleration is not enough.",
                                          "../assets/offroad/icon_road.png",
                                          this));

  toggles.append(new ParamControl("AutoShutdown",
                                          "Enable auto shutdown",
                                          "Automatically power off the EON 180 seconds after external power is disconnected and driving has stopped.",
                                          "../assets/offroad/icon_shell.png",
                                          this));

  // Keep titles short: AbstractControl renders them in a QPushButton with no
  // word wrap, so a long title widens the panel and adds a horizontal scrollbar.
  toggles.append(new ParamControl("CurveFallbackDisabled",
                                          "Disable Curve Lane Fallback",
                                          "커브 중 차선을 놓쳤을 때 직전에 기억해둔 차선 모양을 대신 쓰는 보조 기능을 끕니다. 끄면 일반 차선 혼합만 사용하며, 이는 공식 오픈파일럿과 같은 동작입니다.\n\n측정 결과: 이 보조 기능은 전체 주행의 1.4%에서만 동작했고 보탠 조향량은 1~5도 수준이었습니다. 급커브(핸들 68도 이상)에서는 기억해둔 데이터가 12%만 살아있어 거의 기여하지 못했습니다.\n\n같은 도로를 켠 채와 끈 채로 각각 주행해 비교해보세요.\n\n다음 주행부터 적용됩니다.",
                                          "../assets/offroad/icon_road.png",
                                          this));

  for(ParamControl *toggle : toggles) {
    if(main_layout->count() != 0) {
      toggleLayout->addWidget(horizontal_line());
    }
    toggleLayout->addWidget(toggle);
  }

  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addLayout(layoutBtn_0);
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addLayout(layoutBtn_5);

  // Keep the torque factor control at the bottom of Community. nTune watches
  // its JSON file, so these changes apply without changing the control logic.
  auto torqueLatAccelLabel = new QLabel(homeWidget);
  torqueLatAccelLabel->setObjectName("torqueLatAccelLabel");
  torqueLatAccelLabel->setWordWrap(true);
  torqueLatAccelLabel->setMinimumWidth(0);
  torqueLatAccelLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  auto decreaseTorqueLatAccelBtn = new QPushButton("-", homeWidget);
  decreaseTorqueLatAccelBtn->setObjectName("torqueLatAccelAdjustBtn");
  auto increaseTorqueLatAccelBtn = new QPushButton("+", homeWidget);
  increaseTorqueLatAccelBtn->setObjectName("torqueLatAccelAdjustBtn");
  auto cityTorqueLatAccelBtn = new QPushButton("City", homeWidget);
  cityTorqueLatAccelBtn->setObjectName("torqueLatAccelPresetBtn");
  auto expressTorqueLatAccelBtn = new QPushButton("Highway", homeWidget);
  expressTorqueLatAccelBtn->setObjectName("torqueLatAccelPresetBtn");

  for (auto button : {decreaseTorqueLatAccelBtn, increaseTorqueLatAccelBtn,
                      cityTorqueLatAccelBtn, expressTorqueLatAccelBtn}) {
    button->setMinimumHeight(100);
  }
  decreaseTorqueLatAccelBtn->setFixedWidth(100);
  increaseTorqueLatAccelBtn->setFixedWidth(100);

  auto updateTorqueLatAccelLabel = [=](double value) {
    value = clampTorqueLatAccelFactor(value);
    torqueLatAccelLabel->setText(
      QString("Torque latAccelFactor (0.50 ~ 4.50) : %1\nStep Scale : x0.01")
        .arg(value, 0, 'f', 2));
  };
  auto setTorqueLatAccelFactor = [=](double value) {
    value = clampTorqueLatAccelFactor(value);
    writeTorqueLatAccelFactor(value);
    updateTorqueLatAccelLabel(value);
  };
  updateTorqueLatAccelLabel(readTorqueLatAccelFactor());

  connect(decreaseTorqueLatAccelBtn, &QPushButton::clicked, [=]() {
    setTorqueLatAccelFactor(readTorqueLatAccelFactor() - kTorqueLatAccelFactorStep);
  });
  connect(increaseTorqueLatAccelBtn, &QPushButton::clicked, [=]() {
    setTorqueLatAccelFactor(readTorqueLatAccelFactor() + kTorqueLatAccelFactorStep);
  });
  connect(cityTorqueLatAccelBtn, &QPushButton::clicked, [=]() {
    setTorqueLatAccelFactor(1.6);
  });
  connect(expressTorqueLatAccelBtn, &QPushButton::clicked, [=]() {
    setTorqueLatAccelFactor(1.7);
  });

  QHBoxLayout* torqueLatAccelLayout = new QHBoxLayout();
  torqueLatAccelLayout->setContentsMargins(0, 0, 0, 0);
  torqueLatAccelLayout->setSpacing(10);
  torqueLatAccelLayout->addWidget(torqueLatAccelLabel, 1);
  torqueLatAccelLayout->addWidget(decreaseTorqueLatAccelBtn);
  torqueLatAccelLayout->addWidget(increaseTorqueLatAccelBtn);
  torqueLatAccelLayout->addWidget(cityTorqueLatAccelBtn);
  torqueLatAccelLayout->addWidget(expressTorqueLatAccelBtn);
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addLayout(torqueLatAccelLayout);

}

LateralControl::LateralControl(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton(tr("Back"));
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  //list->setAttribute(Qt::WA_AcceptTouchEvents, true);
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  QStringList items = {"TORQUE", "INDI", "PID"};
  list->addItems(items);
  list->setCurrentRow(0);

  QString selectedControl = QString::fromStdString(Params().get("LateralControl"));

  int index = 0;
  for(QString item : items) {
    if(selectedControl == item) {
        list->setCurrentRow(index);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    Params().put("LateralControl", list->currentItem()->text().toStdString());
    emit selected();

    QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });

    });

  main_layout->addWidget(list);
}

DynamicTRGap::DynamicTRGap(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  QStringList items = {"traffic", "stock", "roadtrip", "auto"};
  list->addItems(items);
  list->setCurrentRow(0);

  QString selectedControl = QString::fromStdString(Params().get("DynamicTRGap"));

  int index = 0;
  for(QString item : items) {
    if(selectedControl == item) {
        list->setCurrentRow(index);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    //Params().put("LateralControl", list->currentItem()->text().toStdString());
    Params().put("DynamicTRGap", list->currentItem()->text().toStdString());
    emit selected();

    QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", false);
      });

    });

  main_layout->addWidget(list);
}

MinTR::MinTR(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  QStringList items = {"0.8", "0.9", "1.0", "1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2", "2.3", "2.4", "2.5", "2.6", "2.7"};
  list->addItems(items);
  list->setCurrentRow(0);

  QString selectedControl = QString::fromStdString(Params().get("minTR"));

  int index = 0;
  for(QString item : items) {
    if(selectedControl == item) {
        list->setCurrentRow(index);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    //Params().put("LateralControl", list->currentItem()->text().toStdString());
    Params().put("minTR", list->currentItem()->text().toStdString());
    emit selected();

    QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", false);
      });

    });

  main_layout->addWidget(list);
}

GlobalDfMod::GlobalDfMod(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  QStringList items = {"0.8", "0.9", "1.0", "1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2", "2.3", "2.4", "2.5"};
  list->addItems(items);
  list->setCurrentRow(0);

  QString selectedControl = QString::fromStdString(Params().get("globalDfMod"));

  int index = 0;
  for(QString item : items) {
    if(selectedControl == item) {
        list->setCurrentRow(index);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    //Params().put("LateralControl", list->currentItem()->text().toStdString());
    Params().put("globalDfMod", list->currentItem()->text().toStdString());
    emit selected();

    QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", false);
      });

    });

  main_layout->addWidget(list);
}
