import json
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

CAMERA_SPEED_FACTOR = 1.05
SAFE_STARTING_DIST = 60.

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

  if _int_or_none(out.get("cam_type")) == SPEED_BUMP_CAM_TYPE:
    limit = _int_or_none(out.get("cam_limit_speed"))
    if limit is None or limit <= 0:
      out["cam_limit_speed"] = SPEED_BUMP_DEFAULT_LIMIT

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

  def udp_recv(self, sock):
    ret = False
    try:
      ready = select.select([sock], [], [], 1.)
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


def main():
  server = RoadLimitSpeedServer()
  roadLimitSpeed = messaging.pub_sock('roadLimitSpeed')

  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    try:

      try:
        sock.bind(('0.0.0.0', 843))
      except:
        sock.bind(('0.0.0.0', Port.RECEIVE_PORT))

      sock.setblocking(False)

      while True:

        server.udp_recv(sock)

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
        dat.roadLimitSpeed.camSpeedFactor = server.get_limit_val("cam_speed_factor", CAMERA_SPEED_FACTOR)

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

        roadLimitSpeed.send(dat.to_bytes())
        server.send_sdp(sock)
        server.check()

    except Exception as e:
      server.last_exception = e


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
      camSpeedFactor = clip(self.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)

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

      # 방지턱은 전용 플래너가 전담한다 (아래 일반 카메라 로직을 타지 않음).
      v_ego_ms = cluster_speed * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
      bump_dist = cam_limit_speed_left_dist if cam_type == SPEED_BUMP_CAM_TYPE else None
      was_bump_active = self.speed_bump_active
      bump_target_kph = self.bump_planner.update(cam_type, bump_dist, v_ego_ms, sec_since_boot())
      self.speed_bump_active = bump_target_kph is not None
      if self.speed_bump_active:
        self.slowing_down = False
        kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
        return bump_target_kph * kph_to_clu, SpeedBumpPlanner.BUMP_SPEED_KPH * kph_to_clu, \
               max(0., self.bump_planner.est_dist), not was_bump_active, log
      if cam_type == SPEED_BUMP_CAM_TYPE:
        # 통과 직후 재보고 / 고정값(150m) 등 플래너가 무시한 방지턱 정보
        self.slowing_down = False
        return 0, cam_limit_speed, cam_limit_speed_left_dist, False, log

      if cam_limit_speed_left_dist is not None and cam_limit_speed is not None and cam_limit_speed_left_dist > 0:

        v_ego = cluster_speed * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
        diff_speed = cluster_speed - (cam_limit_speed * camSpeedFactor)
        #cam_limit_speed_ms = cam_limit_speed * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)

        starting_dist = v_ego * SAFE_STARTING_DIST  # def : 30

        if cam_type == 22:
          safe_dist = v_ego * 3.
        else:
          safe_dist = v_ego * 6.

        if MIN_LIMIT <= cam_limit_speed <= MAX_LIMIT and (self.slowing_down or cam_limit_speed_left_dist < starting_dist):
          if not self.slowing_down:
            self.started_dist = cam_limit_speed_left_dist
            self.slowing_down = True
            first_started = True
          else:
            first_started = False

          td = self.started_dist - safe_dist
          d = cam_limit_speed_left_dist - safe_dist

          if d > 0. and td > 0. and diff_speed > 0. and (section_left_dist is None or section_left_dist < 10):
            pp = (d / td) ** 0.6
          else:
            pp = 0

          return cam_limit_speed * camSpeedFactor + int(pp * diff_speed), \
                 cam_limit_speed, cam_limit_speed_left_dist, first_started, log

        self.slowing_down = False
        return 0, cam_limit_speed, cam_limit_speed_left_dist, False, log

      elif section_left_dist is not None and section_limit_speed is not None and section_left_dist > 0:
        if MIN_LIMIT <= section_limit_speed <= MAX_LIMIT:

          if not self.slowing_down:
            self.slowing_down = True
            first_started = True
          else:
            first_started = False

          return section_limit_speed * camSpeedFactor, section_limit_speed, section_left_dist, first_started, log

        self.slowing_down = False
        return 0, section_limit_speed, section_left_dist, False, log

    except Exception as e:
      log = "Ex: " + str(e)
      pass

    self.slowing_down = False
    self.speed_bump_active = False
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