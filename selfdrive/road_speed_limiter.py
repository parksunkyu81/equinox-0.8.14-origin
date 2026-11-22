import json
import math
import os
import random
import re

import select
import threading
import time
import socket
import fcntl
import struct
from threading import Thread
from cereal import messaging, log
from common.numpy_fast import clip
from common.params import Params
from common.realtime import sec_since_boot
from common.conversions import Conversions as CV
from selfdrive.speed_bump_planner import SpeedBumpPlanner

# 카메라·구간단속 목표 = 제한속도의 95% (50km/h -> 47.5km/h). 2026-11-19 0.9 -> 0.95
# (사용자 요청: 10% 감속은 너무 답답함). 그 전에는 1.05배.
# TmapNda가 다른 값을 보내도 쓰지 않는다.
CAMERA_SPEED_FACTOR = 0.95
# 목표가 이보다 낮아지면 이 값으로 둔다. controlsd는 V_CRUISE_MIN(20km/h) 아래 제한을
# 아예 적용하지 않으므로, 20km/h 구간의 95%(19)는 통째로 무시됐을 것이다.
CAMERA_TARGET_FLOOR = 20.
# 카메라 감속 곡선. 예전에는 '지금 속도로 60초 안에 닿는 거리'(60km/h면 1km)부터
# 제한을 걸고, 목표를 매 순간 '지금 속도' 기준으로 다시 계산했다. 2026-11-19 주행:
#  - 67km/h에서 카메라 890m 앞부터 감속, 170m 만에 45km/h가 되어 남은 720m를 45로 갔다
#    (속도가 줄면 목표도 같이 내려가는 되먹임).
#  - 이미 목표 이하면 처음부터 목표가 상한이라, 50 구간에서 1km 넘게 45에 묶였다.
#  - 한 번 시작하면 카메라를 지날 때까지(다음 카메라로 바로 넘어가면 그 뒤까지) 풀리지
#    않아, 정체 구간에서 1.8km·3분 넘게 45에 묶인 적도 있다.
# 이제 상한은 거리만으로 정한다: 카메라 CAM_REACH_S초 전에 목표가 되는 일정 감속
# CAM_DECEL_MS2 곡선 v = sqrt(vt^2 + 2a(d - vt*CAM_REACH_S)). 이 차는 브레이크를 못
# 쓰므로 a는 타력으로 낼 수 있는 값이어야 한다. 학습된 타력은 50-60km/h에서 0.17-0.20
# m/s^2뿐이라, 그 곡선으로 시뮬레이션하면 0.3은 50 카메라를 50km/h로 지나고 0.25도
# 46-47로 지나지만 0.2는 모든 경우 목표로 지난다 (당시 목표 90%, 45km/h).
# 목표를 95%로 올려 여유가 2.5km/h뿐이라 0.2 -> 0.17 (학습된 타력의 최저치). 0.2는
# 60km/h대 타력(0.17)이 곡선을 못 따라가 67->47.5에서 목표 도달이 카메라 64m 앞까지
# 밀렸고, 0.17은 곡선을 그대로 따라가 86m(약 6.5초) 앞에서 목표가 된다. 목표에 닿는
# 지점은 그대로이고 감속을 시작하는 곳만 앞당겨진다(67->47.5: 589m -> 679m).
# 곡선이 지금 속도 + CAM_START_MARGIN_KPH 아래로 내려올 때 시작하고(예전 60초 기준은
# 1.1km), 곡선이 지금 속도보다 CAM_RELEASE_MARGIN_KPH 넘게 위면(정체로 멈췄을 때,
# 먼 다음 카메라) 푼다. 목표 이하로 달리고 있으면 카메라 CAM_HOLD_S초 전부터만 목표를
# 상한으로 건다.
CAM_DECEL_MS2 = 0.17
CAM_REACH_S = 6.
CAM_START_MARGIN_KPH = 3.
CAM_RELEASE_MARGIN_KPH = 10.
CAM_HOLD_S = 10.
# AutoTurnControl cap floor [km/h] = controlsd V_CRUISE_MIN (see _atc_limit).
ATC_FLOOR_KPH = 20.

UDP_RECV_BUFFER = 65535      # TmapNda는 road_limit + apilot(전체 TBT/SDI)을 한 패킷으로 보내 2KB를 넘을 수 있음
SPEED_BUMP_CAM_TYPE = 22
SPEED_BUMP_DEFAULT_LIMIT = 10  # 방지턱 제한속도가 0으로 올 때 대체값
SECTION_KEYS = ("section_limit_speed", "section_left_dist", "section_avg_speed", "section_left_time")

class Port:
  BROADCAST_PORT = 2899
  RECEIVE_PORT = 2843
  LOCATION_PORT = BROADCAST_PORT


# nda_manager가 보내는 레거시 목적지 명령. 셸로 실행하지 않고 이 두 형태만 인식해 Params로 처리한다.
LEGACY_DEST_SET = re.compile(r"^echo '(\{.*\})' > /data/params/d/NavDestination$", re.DOTALL)
LEGACY_DEST_CLEAR = "rm /data/params/d/NavDestination"


def parse_nav_destination(obj):
  """폰이 보낸 목적지를 검증해 NavDestination 형식 dict로 반환. 잘못된 값이면 None."""
  if not isinstance(obj, dict):
    return None
  try:
    lat = float(obj["latitude"])
    lon = float(obj["longitude"])
  except (KeyError, TypeError, ValueError):
    return None
  if not (-90. <= lat <= 90. and -180. <= lon <= 180.) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
    return None
  name = obj.get("place_name")
  name = str(name)[:200] if name is not None else None
  return {"latitude": lat, "longitude": lon, "place_name": name, "place_details": None}


def apply_nav_destination(dest, params=None):
  """dest가 None이면 목적지 삭제, dict면 검증 후 저장. 저장/삭제했으면 True."""
  params = params if params is not None else Params()
  if dest is None:
    params.delete("NavDestination")
    return True
  parsed = parse_nav_destination(dest)
  if parsed is None:
    return False
  params.put("NavDestination", json.dumps(parsed))
  return True


def handle_legacy_cmd(cmd, params=None):
  """레거시 {"cmd": ...} 처리. 목적지 설정/삭제 두 형태 외에는 무시한다 (임의 셸 실행 금지)."""
  if not isinstance(cmd, str):
    return False
  if cmd.strip() == LEGACY_DEST_CLEAR:
    return apply_nav_destination(None, params)
  m = LEGACY_DEST_SET.match(cmd.strip())
  if m:
    try:
      return apply_nav_destination(json.loads(m.group(1)), params)
    except ValueError:
      return False
  print("road_speed_limiter: ignored unsupported cmd")
  return False


def _int_or_none(v):
  try:
    return int(v)
  except (TypeError, ValueError):
    return None


def normalize_road_limit(road_limit, apilot=None):
  """송신 앱마다 다른 road_limit 해석을 이 수신기 기준으로 맞춘다.

  apilot(TmapNda가 같이 보내는 원본 TBT/SDI)이 있으면:
    - section_*는 버린다. TmapNda는 여기에 '두 번째 카메라'(secondSDIInfo) 값을 넣는데,
      이 수신기는 section_left_dist >= 10이면 카메라 접근 시 점진 감속을 끄기 때문에 급감속이 생긴다.
    - 대신 첫 번째 SDI의 nSdiBlockDist / nSdiBlockSpeed(실제 구간단속)로 section_*를 다시 만든다.
    - is_highway는 apilot.nRoadClass, 없으면 apilot.roadcate(0, 1 = 고속도로)로 판단한다.
      TmapNda는 road_limit.is_highway를 항상 false로 보낸다.
  공통:
    - 방지턱(cam_type 22)의 제한속도가 0 이하면 SPEED_BUMP_DEFAULT_LIMIT로 대체한다.
  nda_manager처럼 apilot이 없는 송신자는 방지턱 대체 외에는 그대로 둔다.
  """
  if not isinstance(road_limit, dict):
    return road_limit
  out = dict(road_limit)

  if isinstance(apilot, dict):
    for k in SECTION_KEYS:
      out.pop(k, None)

    block_dist = _int_or_none(apilot.get("nSdiBlockDist"))
    block_speed = _int_or_none(apilot.get("nSdiBlockSpeed"))
    if block_dist is not None and block_speed is not None and block_dist > 0 and block_speed > 0:
      out["section_limit_speed"] = block_speed
      out["section_left_dist"] = block_dist
      avg = _int_or_none(apilot.get("nSdiBlockAverageSpeed"))
      left_time = _int_or_none(apilot.get("nSdiBlockTime"))
      if avg is not None:
        out["section_avg_speed"] = avg
      if left_time is not None:
        out["section_left_time"] = left_time

    # TmapNda: 실제 TMAP 도로등급(rgData.roadcate)은 nRoadClass로 오고(자동 진출 옵션이 켜졌을 때만, 모르면 -1),
    # apilot.roadcate는 방지턱일 때만 8로 채워진다. 둘 다 0, 1 = 고속도로.
    road_class = _int_or_none(apilot.get("nRoadClass"))
    if road_class is None or road_class < 0:
      road_class = _int_or_none(apilot.get("roadcate"))
    if road_class is not None and road_class >= 0:
      out["is_highway"] = road_class <= 1

  first_is_bump = _int_or_none(out.get("cam_type")) == SPEED_BUMP_CAM_TYPE
  if first_is_bump:
    limit = _int_or_none(out.get("cam_limit_speed"))
    if limit is None or limit <= 0:
      out["cam_limit_speed"] = SPEED_BUMP_DEFAULT_LIMIT

  # bump_dist: the nearest speed bump from either SDI slot. TmapNda puts only
  # the first SDI in cam_type; while a camera or school zone holds that slot, a
  # bump sits in the second (apilot nSdiPlusType/nSdiPlusDist) and used to be
  # seen only once the first item cleared -- on 2026-11-16 at 1132 s, after the
  # driver had already braked for the bump and was past it.
  bump_dist = None
  if first_is_bump:
    bump_dist = _int_or_none(out.get("cam_limit_speed_left_dist"))
  elif isinstance(apilot, dict) and _int_or_none(apilot.get("nSdiPlusType")) == SPEED_BUMP_CAM_TYPE:
    bump_dist = _int_or_none(apilot.get("nSdiPlusDist"))
  out["bump_dist"] = bump_dist if bump_dist is not None and bump_dist > 0 else 0

  return out


class RoadLimitSpeedServer:
  def __init__(self):
    self.json_road_limit = None
    self.active = 0
    self.last_updated = 0
    self.last_updated_active = 0
    self.last_exception = None
    self.lock = threading.Lock()
    self.remote_addr = None
    self.apilot = None   # latest TmapNda apilot not yet consumed by main()

    self.remote_gps_addr = None
    self.last_time_location = 0

    broadcast = Thread(target=self.broadcast_thread, args=[])
    broadcast.setDaemon(True)
    broadcast.start()

    self.gps_sm = messaging.SubMaster(['gpsLocationExternal'], poll=['gpsLocationExternal'])
    self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    self.gps_event = threading.Event()
    gps_thread = Thread(target=self.gps_thread, args=[])
    gps_thread.setDaemon(True)
    gps_thread.start()

  def gps_thread(self):
    try:
      period = 1.0
      wait_time = period
      i = 0.
      frame = 1
      start_time = sec_since_boot()
      while True:
        self.gps_event.wait(wait_time)
        self.gps_timer()

        now = sec_since_boot()
        error = (frame * period - (now - start_time))
        i += error * 0.1
        wait_time = period + error * 0.5 + i
        wait_time = clip(wait_time, 0.8, 1.0)
        frame += 1

    except:
      pass

  def gps_timer(self):
    try:
      if self.remote_gps_addr is not None:
        self.gps_sm.update(0)
        if self.gps_sm.updated['gpsLocationExternal']:
          location = self.gps_sm['gpsLocationExternal']

          if location.accuracy < 10.:
            json_location = json.dumps({"location": [
              location.latitude,
              location.longitude,
              location.altitude,
              location.speed,
              location.bearingDeg,
              location.accuracy,
              location.timestamp,
              # location.source,
              # location.vNED,
              location.verticalAccuracy,
              location.bearingAccuracyDeg,
              location.speedAccuracy,
            ]})

            address = (self.remote_gps_addr[0], Port.LOCATION_PORT)
            self.gps_socket.sendto(json_location.encode(), address)
    except:
      self.remote_gps_addr = None

  def get_broadcast_address(self):
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      ip = fcntl.ioctl(
        s.fileno(),
        0x8919,
        struct.pack('256s', 'wlan0'.encode('utf-8'))
      )[20:24]

      return socket.inet_ntoa(ip)
    except:
      return None

  def broadcast_thread(self):

    broadcast_address = None
    frame = 0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:

          try:

            if broadcast_address is None or frame % 10 == 0:
              broadcast_address = self.get_broadcast_address()

            print('broadcast_address', broadcast_address)

            if broadcast_address is not None:
              address = (broadcast_address, Port.BROADCAST_PORT)
              sock.sendto('EON:ROAD_LIMIT_SERVICE:v1'.encode(), address)
          except:
            pass

          time.sleep(5.)
          frame += 1

      except:
        pass

  def send_sdp(self, sock):
    try:
      sock.sendto('EON:ROAD_LIMIT_SERVICE:v1'.encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
    except:
      pass

  def udp_recv(self, sock, timeout=1.):
    ret = False
    try:
      ready = select.select([sock], [], [], timeout)
      ret = bool(ready[0])
      if ret:
        data, self.remote_addr = sock.recvfrom(UDP_RECV_BUFFER)
        json_obj = json.loads(data.decode())

        # 예전에는 os.system(json_obj['cmd'])로 LAN의 누구든 root 명령을 실행할 수 있었다.
        # 이제 nda_manager의 목적지 명령만 해석하고 셸은 실행하지 않는다.
        if 'cmd' in json_obj:
          try:
            handle_legacy_cmd(json_obj['cmd'])
            ret = False
          except:
            pass

        if 'nav_destination' in json_obj:
          try:
            apply_nav_destination(json_obj['nav_destination'])
            ret = False
          except:
            pass

        if 'request_gps' in json_obj:
          try:
            if json_obj['request_gps'] == 1:
              self.remote_gps_addr = self.remote_addr
            else:
              self.remote_gps_addr = None
            ret = False
          except:
            pass

        if 'echo' in json_obj:
          try:
            echo = json.dumps(json_obj["echo"])
            sock.sendto(echo.encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
            ret = False
          except:
            pass

        try:
          self.lock.acquire()
          try:
            if 'active' in json_obj:
              self.active = json_obj['active']
              self.last_updated_active = sec_since_boot()
          except:
            pass

          if 'road_limit' in json_obj:
            self.json_road_limit = normalize_road_limit(json_obj['road_limit'], json_obj.get('apilot'))
            self.last_updated = sec_since_boot()
            # TmapNda's apilot is the JSON it sends carrotpilot on 7706 (TBT included):
            # handed to AutoTurnControl by main().
            apilot = json_obj.get('apilot')
            if isinstance(apilot, dict):
              self.apilot = apilot

        finally:
          self.lock.release()

    except:

      try:
        self.lock.acquire()
        self.json_road_limit = None
      finally:
        self.lock.release()

    return ret

  def check(self):
    now = sec_since_boot()
    if now - self.last_updated > 6.:
      try:
        self.lock.acquire()
        self.json_road_limit = None
      finally:
        self.lock.release()

    if now - self.last_updated_active > 6.:
      self.active = 0

  def get_limit_val(self, key, default=None):
    return self.get_json_val(self.json_road_limit, key, default)

  def get_json_val(self, json, key, default=None):

    try:
      if json is None:
        return default

      if key in json:
        return json[key]

    except:
      pass

    return default


ATC_LOG_PATH = os.getenv("AUTO_TURN_LOG_PATH", "/data/log/auto_turn.jsonl")
ATC_LOG_INTERVAL_S = 1.0
ATC_LOG_MAX_BYTES = 2 * 1024 * 1024
ATC_PARAMS_INTERVAL_S = 1.0
# The loop used to block up to 1 s on the socket and publish once per packet.
# ATC counts the distance to the turn down between packets and its speed cap
# and lane-change type must follow that, so the loop now runs at 20 Hz.
LOOP_TIMEOUT_S = 0.05
SDP_INTERVAL_S = 1.0


class AtcRunner:
  """Feeds AutoTurnControl from TmapNda packets and carState, and logs it.

  roadLimitSpeed is not in the rlog, so without a log nothing would show what
  the navigation asked for at a turn. One JSON line on every change of
  atcType / xTurnInfo, and one a second while a turn is pending. Background
  thread append, a few lines a minute: nothing measurable.
  """

  def __init__(self, params=None, log_path=ATC_LOG_PATH):
    from selfdrive.controls.lib.auto_turn_control import AutoTurnControl
    self.atc = AutoTurnControl()
    self.params = params if params is not None else Params()
    self.carstate_sock = messaging.sub_sock('carState', conflate=True)
    self.v_ego = 0.
    self.steering_pressed = False
    self.steering_torque = 0.
    self.carstate_t = -1e9
    self.last_t = None
    self.params_t = -1e9
    self.log_path = log_path
    self.log_t = -1e9
    self.prev_key = None
    self.cap_kph = 250.

  def _read_params(self, now):
    if now - self.params_t < ATC_PARAMS_INTERVAL_S:
      return
    self.params_t = now
    from selfdrive.controls.lib import auto_turn_control as atc_mod

    def get_int(key, default):
      try:
        v = self.params.get(key)
        return int(v) if v is not None and len(v) else default
      except (TypeError, ValueError):
        return default
    self.atc.set_params(get_int("AutoTurnControl", atc_mod.DEFAULT_MODE),
                        get_int("AutoTurnControlSpeedTurn", atc_mod.DEFAULT_SPEED_TURN),
                        get_int("AutoTurnControlTurnEnd", atc_mod.DEFAULT_TURN_END),
                        get_int("AutoNaviSpeedDecelRate", atc_mod.DEFAULT_DECEL_RATE))

  def _read_carstate(self, now):
    try:
      dat = messaging.recv_sock(self.carstate_sock, wait=False)
    except Exception:
      dat = None
    if dat is not None:
      cs = dat.carState
      self.v_ego = float(cs.vEgo)
      self.steering_pressed = bool(cs.steeringPressed)
      self.steering_torque = float(cs.steeringTorque)
      self.carstate_t = now
    elif now - self.carstate_t > 1.:
      # No carState (offroad / controls down): nothing moves, no override.
      self.v_ego = 0.
      self.steering_pressed = False
      self.steering_torque = 0.

  def update(self, now, apilot):
    self._read_params(now)
    self._read_carstate(now)
    if apilot is not None:
      self.atc.update_tbt(apilot, now)
    dt = 0. if self.last_t is None else clip(now - self.last_t, 0., 0.5)
    self.last_t = now
    self.cap_kph = self.atc.update(now, self.v_ego, self.v_ego * dt,
                                   self.steering_pressed, self.steering_torque)
    self._log(now)

  def fill(self, msg):
    atc = self.atc
    msg.atcType = atc.atcType
    msg.atcDesiredSpeed = float(self.cap_kph) if self.cap_kph < 250. else 0.
    target_kph, target_dist = atc.speed_target()
    msg.atcTargetSpeed = target_kph
    msg.atcTargetDist = target_dist
    msg.xTurnInfo = int(atc.xTurnInfo)
    msg.xDistToTurn = float(atc.xDistToTurn)

  def _log(self, now):
    atc = self.atc
    key = (atc.atcType, atc.xTurnInfo)
    changed = key != self.prev_key
    self.prev_key = key
    pending = atc.xTurnInfo > 0 and atc.atcType not in ("none",)
    if not (changed or (pending and now - self.log_t >= ATC_LOG_INTERVAL_S)):
      return
    self.log_t = now
    target_kph, target_dist = atc.speed_target()
    line = json.dumps({
      't': round(now, 2), 'event': 'change' if changed else 'tick',
      'mode': atc.autoTurnControl, 'kph': round(self.v_ego * 3.6, 1),
      'tbt_type': atc.nTBTTurnType, 'tbt_dist': atc.nTBTDist,
      'tbt_next_type': atc.nTBTTurnTypeNext, 'road_limit': atc.nRoadLimitSpeed,
      'turn_info': atc.xTurnInfo, 'dist': round(float(atc.xDistToTurn), 1),
      'atc': atc.atcType, 'cap_kph': None if self.cap_kph >= 250. else round(self.cap_kph, 1),
      'target_kph': round(target_kph, 1), 'target_dist': round(target_dist, 1),
      'steer_pressed': self.steering_pressed,
    }, separators=(',', ':'))
    path = self.log_path

    def write():
      try:
        if os.path.exists(path) and os.path.getsize(path) > ATC_LOG_MAX_BYTES:
          os.replace(path, path + ".1")
        with open(path, 'a') as f:
          f.write(line + "\n")
      except Exception:
        pass
    threading.Thread(target=write, daemon=True).start()


def main():
  server = RoadLimitSpeedServer()
  roadLimitSpeed = messaging.pub_sock('roadLimitSpeed')
  atc_runner = AtcRunner()
  last_sdp_t = -1e9

  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    try:

      try:
        sock.bind(('0.0.0.0', 843))
      except:
        sock.bind(('0.0.0.0', Port.RECEIVE_PORT))

      sock.setblocking(False)

      while True:

        server.udp_recv(sock, LOOP_TIMEOUT_S)

        now = sec_since_boot()
        with server.lock:
          apilot, server.apilot = server.apilot, None
        try:
          atc_runner.update(now, apilot)
        except Exception as e:
          print("road_speed_limiter: ATC update failed:", e)

        dat = messaging.new_message()
        dat.init('roadLimitSpeed')
        dat.roadLimitSpeed.active = server.active
        dat.roadLimitSpeed.roadLimitSpeed = server.get_limit_val("road_limit_speed", 0)
        dat.roadLimitSpeed.isHighway = server.get_limit_val("is_highway", False)
        dat.roadLimitSpeed.camType = server.get_limit_val("cam_type", 0)
        dat.roadLimitSpeed.camLimitSpeedLeftDist = server.get_limit_val("cam_limit_speed_left_dist", 0)
        dat.roadLimitSpeed.camLimitSpeed = server.get_limit_val("cam_limit_speed", 0)
        dat.roadLimitSpeed.sectionLimitSpeed = server.get_limit_val("section_limit_speed", 0)
        dat.roadLimitSpeed.sectionLeftDist = server.get_limit_val("section_left_dist", 0)
        dat.roadLimitSpeed.camSpeedFactor = CAMERA_SPEED_FACTOR
        dat.roadLimitSpeed.bumpLeftDist = int(clip(server.get_limit_val("bump_dist", 0) or 0, 0, 32767))

        try:
          json = server.json_road_limit
          if json is not None and "rest_area" in json:

            restAreaList = []
            for rest_area in json["rest_area"]:
              restArea = log.RoadLimitSpeed.RestArea.new_message()
              restArea.image = server.get_json_val(rest_area, "image")
              restArea.title = server.get_json_val(rest_area, "title")
              restArea.oilPrice = server.get_json_val(rest_area, "oilPrice")
              restArea.distance = server.get_json_val(rest_area, "distance")
              restAreaList.append(restArea)

            dat.roadLimitSpeed.restArea = restAreaList
        except:
          pass

        try:
          atc_runner.fill(dat.roadLimitSpeed)
        except Exception as e:
          print("road_speed_limiter: ATC fill failed:", e)

        roadLimitSpeed.send(dat.to_bytes())
        # Was once per loop, i.e. once per packet or per 1 s timeout; the loop
        # is 20 Hz now, so keep the reply to the phone at 1 Hz.
        if now - last_sdp_t >= SDP_INTERVAL_S:
          last_sdp_t = now
          server.send_sdp(sock)
        server.check()

    except Exception as e:
      server.last_exception = e


BUMP_LOG_PATH = os.getenv("SPEED_BUMP_LOG_PATH", "/data/log/speed_bump.jsonl")
BUMP_LOG_INTERVAL_S = 0.5
BUMP_LOG_MAX_BYTES = 2 * 1024 * 1024


class BumpLog:
  """Always-on speed-bump trace for drive analysis.

  roadLimitSpeed is not in the rlog, so the distances TmapNda reports -- the
  one thing needed to judge bump timing -- were lost (the 2026-11-16 analysis
  had to infer them). While a bump is reported or tracked this appends one line
  every 0.5 s, plus one at every tracking start/stop: what arrived (first SDI
  slot, bump distance from either slot), the planner's estimate, and its
  target. Every planner event (detect / slow / pass / release / lost) gets its
  own line at once, tagged with the bump number, so each bump reads as
  detect -> slow -> pass -> (between ->) release. A dict and a background-thread
  append twice a second -- nothing measurable on controlsd.
  """

  def __init__(self, path=BUMP_LOG_PATH):
    self.path = path
    self.last_t = -1e9
    self.prev_phase = 'idle'

  def update(self, now, v_ego_ms, cam_type, cam_limit, cam_dist, bump_dist, target_kph, planner):
    tracking = bool(planner.tracking)
    phase = planner.phase
    events = list(planner.events)
    changed = bool(events) or phase != self.prev_phase
    self.prev_phase = phase
    relevant = phase != 'idle' or bump_dist is not None or cam_type == SPEED_BUMP_CAM_TYPE
    if not (changed or (relevant and now - self.last_t >= BUMP_LOG_INTERVAL_S)):
      return
    self.last_t = now
    since_passed = (round(planner.odo - planner.passed_pos, 1)
                    if planner.passed_pos is not None else None)
    line = json.dumps({
      't': round(now, 2), 'event': '+'.join(events) if events else 'tick',
      'phase': phase, 'bump_no': planner.bump_no,
      'kph': round(v_ego_ms * 3.6, 1),
      'cam_type': int(cam_type), 'cam_limit': int(cam_limit), 'cam_dist': int(cam_dist),
      'bump_dist': bump_dist, 'tracking': tracking,
      'recover_kph': None if planner.recover_kph is None else round(planner.recover_kph, 1),
      'est_dist': round(planner.est_dist, 1) if tracking else None,
      'target_kph': None if target_kph is None else round(target_kph, 1),
      'since_passed_m': since_passed,
      'candidate': None if planner.candidate is None else round(planner.candidate[0] - planner.odo, 1),
    }, separators=(',', ':'))
    path = self.path

    def write():
      try:
        if os.path.exists(path) and os.path.getsize(path) > BUMP_LOG_MAX_BYTES:
          os.replace(path, path + ".1")
        with open(path, 'a') as f:
          f.write(line + "\n")
      except Exception:
        pass
    threading.Thread(target=write, daemon=True).start()


class RoadSpeedLimiter:
  def __init__(self):
    self.slowing_down = False
    self.started_dist = 0

    self.sock = messaging.sub_sock("roadLimitSpeed")
    self.roadLimitSpeed = None

    # 방지턱: 50m 전까지 10km/h, 통과 후 해제. controlsd가 speed_bump_active를 보고
    # V_CRUISE_MIN(20km/h) 아래 목표를 허용한다.
    self.bump_planner = SpeedBumpPlanner()
    self.speed_bump_active = False
    self.bump_log = BumpLog()
    self.atc_slowing_down = False

  def recv(self):
    try:
      dat = messaging.recv_sock(self.sock, wait=False)
      if dat is not None:
        self.roadLimitSpeed = dat.roadLimitSpeed
    except:
      pass

  def get_active(self):
    self.recv()
    if self.roadLimitSpeed is not None:
      return self.roadLimitSpeed.active
    return 0

  def get_max_speed(self, cluster_speed, is_metric):

    log = ""
    self.recv()

    if self.roadLimitSpeed is None:
      return 0, 0, 0, False, ""

    try:

      road_limit_speed = self.roadLimitSpeed.roadLimitSpeed
      is_highway = self.roadLimitSpeed.isHighway

      cam_type = int(self.roadLimitSpeed.camType)

      cam_limit_speed_left_dist = self.roadLimitSpeed.camLimitSpeedLeftDist
      cam_limit_speed = self.roadLimitSpeed.camLimitSpeed

      section_limit_speed = self.roadLimitSpeed.sectionLimitSpeed
      section_left_dist = self.roadLimitSpeed.sectionLeftDist
      camSpeedFactor = CAMERA_SPEED_FACTOR

      if is_highway is not None:
        if is_highway:
          MIN_LIMIT = 40
          MAX_LIMIT = 120
        else:
          MIN_LIMIT = 20
          MAX_LIMIT = 100
      else:
        MIN_LIMIT = 20
        MAX_LIMIT = 120

      # 방지턱은 전용 플래너가 전담한다. 거리는 두 SDI 칸 중 어디서 왔든 bumpLeftDist로
      # 온다(첫 칸만 보던 때는 카메라·스쿨존 뒤의 방지턱이 늦게 잡혔다).
      v_ego_ms = cluster_speed * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
      bump_dist = int(getattr(self.roadLimitSpeed, 'bumpLeftDist', 0) or 0)
      if bump_dist <= 0 and cam_type == SPEED_BUMP_CAM_TYPE:
        bump_dist = cam_limit_speed_left_dist   # bumpLeftDist를 모르는 송신자
      bump_dist = bump_dist if bump_dist > 0 else None
      was_bump_active = self.speed_bump_active
      now = sec_since_boot()
      bump_target_kph = self.bump_planner.update(
        SPEED_BUMP_CAM_TYPE if bump_dist is not None else 0, bump_dist, v_ego_ms, now)
      self.speed_bump_active = bump_target_kph is not None
      self.bump_log.update(now, v_ego_ms, cam_type, cam_limit_speed, cam_limit_speed_left_dist,
                           bump_dist, bump_target_kph, self.bump_planner)

      if cam_type == SPEED_BUMP_CAM_TYPE:
        # 첫 칸이 방지턱 자체: 카메라 감속은 없다 (통과 직후 재보고·150m 고정값 포함)
        self.slowing_down = False
        cam_result = (0, cam_limit_speed, cam_limit_speed_left_dist, False, log)
      else:
        cam_result = self._camera_or_section_limit(
          cluster_speed, is_metric, cam_type, cam_limit_speed, cam_limit_speed_left_dist,
          section_limit_speed, section_left_dist, camSpeedFactor, MIN_LIMIT, MAX_LIMIT, log)

      # AutoTurnControl turn slowdown: the lower of it and the camera/section cap wins.
      atc_result = self._atc_limit(is_metric, cluster_speed, log)
      if atc_result is not None and (cam_result[0] <= 0 or atc_result[0] < cam_result[0]):
        if self.slowing_down and cam_result[0] > 0:
          self.slowing_down = False   # camera hands over to ATC
        cam_result = atc_result

      if self.speed_bump_active:
        kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
        bump_apply = bump_target_kph * kph_to_clu
        # 방지턱이 아직 멀어 목표가 높을 때는 더 낮은 카메라·구간 제한이 이긴다.
        if 0 < cam_result[0] < bump_apply:
          return cam_result
        self.slowing_down = False
        return bump_apply, SpeedBumpPlanner.BUMP_SPEED_KPH * kph_to_clu, \
               self.bump_planner.left_dist, not was_bump_active, log
      return cam_result

    except Exception as e:
      log = "Ex: " + str(e)
      pass

    self.slowing_down = False
    self.speed_bump_active = False
    return 0, 0, 0, False, log

  def _atc_limit(self, is_metric, cluster_speed, log):
    """AutoTurnControl speed cap as a get_max_speed() result, or None.

    The cap is carrot's distance curve (calculate_current_speed). controlsd
    ignores any limit under V_CRUISE_MIN (20 km/h), so a cap that falls below
    it (U-turn / arrival target 1 km/h, or a turn speed under 20) would be
    dropped and the car would speed back up near the point. It is held at
    ATC_FLOOR_KPH instead, like the camera floor.
    """
    rls = self.roadLimitSpeed
    cap_kph = float(getattr(rls, 'atcDesiredSpeed', 0.) or 0.)
    if not (0. < cap_kph < 250.):
      self.atc_slowing_down = False
      return None
    kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
    cap_clu = max(cap_kph, ATC_FLOOR_KPH) * kph_to_clu
    target_clu = max(float(rls.atcTargetSpeed), ATC_FLOOR_KPH) * kph_to_clu
    dist = max(1, int(rls.atcTargetDist))
    first_started = not self.atc_slowing_down and cap_clu < cluster_speed
    if cap_clu < cluster_speed:
      self.atc_slowing_down = True
    return cap_clu, target_clu, dist, first_started, log

  @staticmethod
  def _camera_target(limit, factor, is_metric):
    """Camera / section target in cluster units: limit x factor, floored at
    CAMERA_TARGET_FLOOR km/h (but never above the limit itself)."""
    floor_clu = CAMERA_TARGET_FLOOR * (1. if is_metric else CV.KPH_TO_MPH)
    return max(limit * factor, min(limit, floor_clu))

  def _camera_cap(self, cluster_speed, is_metric, cam_limit_speed, left_dist, factor, in_section=False):
    """(speed ceiling in cluster units, whether it applies now) for a camera left_dist m ahead.

    The ceiling depends on the distance only: the constant-decel curve that
    reaches the target CAM_REACH_S before the camera, never below the target.
    """
    clu_to_ms = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
    kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
    target_clu = self._camera_target(cam_limit_speed, factor, is_metric)
    if in_section:
      # 구간단속 안: 곡선 없이 목표가 바로 상한 (예전과 같음).
      return target_clu, True
    vt = target_clu * clu_to_ms
    d = float(left_dist) - vt * CAM_REACH_S
    cap_ms = math.sqrt(vt * vt + 2. * CAM_DECEL_MS2 * max(0., d))
    cap_clu = max(target_clu, cap_ms / clu_to_ms)
    v_ms = max(0., float(cluster_speed)) * clu_to_ms

    if cap_clu <= cluster_speed + CAM_START_MARGIN_KPH * kph_to_clu:
      active = True            # 지금 줄이기 시작해야 곡선을 지킨다
    elif float(left_dist) <= max(v_ms, vt) * CAM_HOLD_S:
      active = True            # 목표 이하로 달리는 중: 카메라 직전에만 목표를 상한으로
    elif self.slowing_down:
      active = cap_clu <= cluster_speed + CAM_RELEASE_MARGIN_KPH * kph_to_clu
    else:
      active = False
    return cap_clu, active

  def _camera_or_section_limit(self, cluster_speed, is_metric, cam_type, cam_limit_speed,
                               cam_limit_speed_left_dist, section_limit_speed, section_left_dist,
                               camSpeedFactor, MIN_LIMIT, MAX_LIMIT, log):
    """Camera / section-enforcement slowdown from the first SDI slot (bumps excluded)."""
    if cam_limit_speed_left_dist is not None and cam_limit_speed is not None and cam_limit_speed_left_dist > 0:

      if MIN_LIMIT <= cam_limit_speed <= MAX_LIMIT:
        cap_clu, active = self._camera_cap(
          cluster_speed, is_metric, cam_limit_speed, cam_limit_speed_left_dist, camSpeedFactor,
          in_section=(section_left_dist is not None and section_left_dist >= 10))
        if active:
          first_started = not self.slowing_down
          self.slowing_down = True
          self.started_dist = cam_limit_speed_left_dist if first_started else self.started_dist
          return cap_clu, cam_limit_speed, cam_limit_speed_left_dist, first_started, log

      self.slowing_down = False
      return 0, cam_limit_speed, cam_limit_speed_left_dist, False, log

    elif section_left_dist is not None and section_limit_speed is not None and section_left_dist > 0:
      if MIN_LIMIT <= section_limit_speed <= MAX_LIMIT:

        if not self.slowing_down:
          self.slowing_down = True
          first_started = True
        else:
          first_started = False

        return self._camera_target(section_limit_speed, camSpeedFactor, is_metric), \
               section_limit_speed, section_left_dist, first_started, log

      self.slowing_down = False
      return 0, section_limit_speed, section_left_dist, False, log

    self.slowing_down = False
    return 0, 0, 0, False, log


road_speed_limiter = None


def road_speed_limiter_get_active():
  global road_speed_limiter
  if road_speed_limiter is None:
    road_speed_limiter = RoadSpeedLimiter()

  return road_speed_limiter.get_active()


def road_speed_limiter_get_max_speed(cluster_speed, is_metric):
  global road_speed_limiter
  if road_speed_limiter is None:
    road_speed_limiter = RoadSpeedLimiter()

  return road_speed_limiter.get_max_speed(cluster_speed, is_metric)


def get_road_speed_limiter():
  global road_speed_limiter
  if road_speed_limiter is None:
    road_speed_limiter = RoadSpeedLimiter()
  return road_speed_limiter


if __name__ == "__main__":
  main()