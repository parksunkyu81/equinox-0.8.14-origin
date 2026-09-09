#include "selfdrive/ui/qt/offroad/roadview.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QDateTime>
#include <QHBoxLayout>
#include <QImage>
#include <QStackedLayout>
#include <QVBoxLayout>

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/mat.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/util.h"

// Where a captured frame lands. /data/media/0 is the storage the drive
// segments already live on, so it survives a reboot and can be pulled off
// the device the same way a route is.
const QString CAPTURE_DIR = "/data/media/0";

namespace {

// ModelFrame keeps these as members, so they cannot be reached from here
// without building one, which would want an OpenCL context the UI has no other
// use for. Same numbers as selfdrive/modeld/models/commonmodel.h.
const int MODEL_WIDTH = 512;
const int MODEL_HEIGHT = 256;

// The inverse of camerad's rgb_to_yuv.cl, which is BT.601 studio swing.
inline void yuv_to_rgb(int y, int u, int v, uint8_t *out) {
  const int c = y - 16, d = u - 128, e = v - 128;
  out[0] = std::clamp((298 * c + 409 * e + 128) >> 8, 0, 255);
  out[1] = std::clamp((298 * c - 100 * d - 208 * e + 128) >> 8, 0, 255);
  out[2] = std::clamp((298 * c + 516 * d + 128) >> 8, 0, 255);
}

QImage yuv420_to_image(const uint8_t *y, const uint8_t *u, const uint8_t *v, int w, int h) {
  QImage img(w, h, QImage::Format_RGB888);
  for (int j = 0; j < h; j++) {
    uint8_t *row = img.scanLine(j);
    const uint8_t *yr = y + j * w;
    const uint8_t *ur = u + (j / 2) * (w / 2);
    const uint8_t *vr = v + (j / 2) * (w / 2);
    for (int i = 0; i < w; i++) {
      yuv_to_rgb(yr[i], ur[i / 2], vr[i / 2], row + i * 3);
    }
  }
  return img;
}

// A CPU copy of modeld's transforms/transform.cl, fixed-point rounding
// included, so what lands in the file is what the model is handed rather than
// something that merely looks like it.
void warp_plane(const uint8_t *src, int src_w, int src_h,
                uint8_t *dst, int dst_w, int dst_h, const mat3 &m) {
  const int INTER_BITS = 5;
  const int INTER_TAB_SIZE = 1 << INTER_BITS;
  const int COEF_BITS = 15;
  const int COEF_SCALE = 1 << COEF_BITS;

  auto sat_short = [](int x) { return std::clamp(x, -32768, 32767); };

  for (int dy = 0; dy < dst_h; dy++) {
    for (int dx = 0; dx < dst_w; dx++) {
      float X0 = m.v[0] * dx + m.v[1] * dy + m.v[2];
      float Y0 = m.v[3] * dx + m.v[4] * dy + m.v[5];
      float W = m.v[6] * dx + m.v[7] * dy + m.v[8];
      W = W != 0.0f ? INTER_TAB_SIZE / W : 0.0f;
      const int X = (int)rintf(X0 * W), Y = (int)rintf(Y0 * W);

      const int sx = sat_short(X >> INTER_BITS), sy = sat_short(Y >> INTER_BITS);
      const int ax = X & (INTER_TAB_SIZE - 1), ay = Y & (INTER_TAB_SIZE - 1);

      auto at = [&](int x, int y) {
        return (x >= 0 && x < src_w && y >= 0 && y < src_h) ? (int)src[y * src_w + x] : 0;
      };

      const float tabx = (float)ax / INTER_TAB_SIZE, taby = (float)ay / INTER_TAB_SIZE;
      const int itab0 = sat_short((int)nearbyintf((1.f - taby) * (1.f - tabx) * COEF_SCALE));
      const int itab1 = sat_short((int)nearbyintf((1.f - taby) * tabx * COEF_SCALE));
      const int itab2 = sat_short((int)nearbyintf(taby * (1.f - tabx) * COEF_SCALE));
      const int itab3 = sat_short((int)nearbyintf(taby * tabx * COEF_SCALE));

      const int val = at(sx, sy) * itab0 + at(sx + 1, sy) * itab1 +
                      at(sx, sy + 1) * itab2 + at(sx + 1, sy + 1) * itab3;
      dst[dy * dst_w + dx] = (uint8_t)std::clamp((val + (1 << (COEF_BITS - 1))) >> COEF_BITS, 0, 255);
    }
  }
}

// The matrix modeld builds in update_calibration(). Picking columns commutes
// with the multiply, so taking columns 0, 1 and 3 out of the extrinsic matrix
// before applying the intrinsics gives what modeld gets from Eigen, without
// pulling Eigen into the UI.
bool model_projection(mat3 *out) {
  std::string calib_bytes = Params().get("CalibrationParams");
  if (calib_bytes.empty()) return false;

  try {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
    auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
    if (calib.getCalStatus() == 0) return false;

    auto ext = calib.getExtrinsicMatrix();
    if (ext.size() != 12) return false;

    const mat3 ground_from_medmodel_frame = {{
       0.00000000e+00,  0.00000000e+00,  1.00000000e+00,
      -1.09890110e-03,  0.00000000e+00,  2.81318681e-01,
      -1.84808520e-20,  9.00738606e-04, -4.28751576e-02,
    }};
    const mat3 ground_from_camera = {{
      ext[0], ext[1], ext[3],
      ext[4], ext[5], ext[7],
      ext[8], ext[9], ext[11],
    }};

    mat3 camera_frame_from_ground = matmul3(fcam_intrinsic_matrix, ground_from_camera);
    mat3 warp = matmul3(camera_frame_from_ground, ground_from_medmodel_frame);
    *out = matmul3(get_model_yuv_transform(), warp);
    return true;
  } catch (kj::Exception) {
    return false;
  }
}

}  // namespace

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
  // What is on screen is the frame blown up to the display with GL_NEAREST, so
  // grabbing the framebuffer saved a magnified copy of the camera rather than
  // the camera. Read the stream instead. modeld consumes the YUV stream, so
  // taking both files out of that one buffer puts them on the same frame.
  VisionIpcClient vipc("camerad", VISION_STREAM_ROAD, true);
  for (int i = 0; i < 10 && !vipc.connected; i++) {
    if (!vipc.connect(false)) util::sleep_for(20);
  }
  if (!vipc.connected) {
    status->setText("camera not streaming");
    return;
  }

  VisionBuf *buf = nullptr;
  for (int i = 0; i < 20 && buf == nullptr; i++) {
    buf = vipc.recv(nullptr, 100);
  }
  if (buf == nullptr) {
    status->setText("no frame yet");
    return;
  }

  const int w = buf->width, h = buf->height;
  const QString stem = CAPTURE_DIR + "/camera_" +
                       QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

  // The frame as camerad publishes it: no rescaling, so this is the camera's
  // own resolution -- 1164x874 on EON, the debayer having already halved the
  // sensor -- and the ceiling on everything downstream of it.
  const QString cam_fn = stem + "_cam.jpg";
  if (!yuv420_to_image(buf->y, buf->u, buf->v, w, h).save(cam_fn, "JPEG", 95)) {
    status->setText("could not write " + cam_fn);
    return;
  }

  // And the same frame after modeld's warp: 512x256, cropped and flattened
  // against the road plane. This is all the model ever sees.
  mat3 projection = {};
  if (!model_projection(&projection)) {
    status->setText("saved " + cam_fn + "\nmodel input needs calibration");
    return;
  }

  const int mw = MODEL_WIDTH, mh = MODEL_HEIGHT;
  std::vector<uint8_t> my(mw * mh), mu(mw / 2 * mh / 2), mv(mw / 2 * mh / 2);
  const mat3 projection_uv = transform_scale_buffer(projection, 0.5);
  warp_plane(buf->y, w, h, my.data(), mw, mh, projection);
  warp_plane(buf->u, w / 2, h / 2, mu.data(), mw / 2, mh / 2, projection_uv);
  warp_plane(buf->v, w / 2, h / 2, mv.data(), mw / 2, mh / 2, projection_uv);

  // PNG, because at this size JPEG would lay its own artifacts over the thing
  // the file exists to show.
  const QString model_fn = stem + "_model.png";
  if (!yuv420_to_image(my.data(), mu.data(), mv.data(), mw, mh).save(model_fn, "PNG")) {
    status->setText("saved " + cam_fn + "\ncould not write " + model_fn);
    return;
  }

  status->setText(QString("saved %1x%2 and %3x%4\n%5_{cam.jpg,model.png}")
                      .arg(w).arg(h).arg(mw).arg(mh).arg(stem));
}
