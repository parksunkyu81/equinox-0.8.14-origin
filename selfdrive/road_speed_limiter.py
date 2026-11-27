import json
import math
import os
import random

import select
import threading
import time
import socket
import fcntl
import struct
import subprocess
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
# A turn whose own target lies between this and ATC_FLOOR_KPH (the intersection
# turn set to 15-19 km/h, auto_turn_control.TURN_SPEED_MIN_KPH) is floored here instead,
# and controlsd lets it under V_CRUISE_MIN like a speed bump. U-turn / arrival
# targets (1 km/h) still stop at ATC_FLOOR_KPH.
ATC_LOW_FLOOR_KPH = 15.

UDP_RECV_BUFFER = 65535      # TmapNda는 road_limit + apilot(전체 TBT/SDI)을 한 패킷으로 보내 2KB를 넘을 수 있음
SPEED_BUMP_CAM_TYPE = 22
SPEED_BUMP_DEFAULT_LIMIT = 10  # 방지턱 제한속도가 0으로 올 때 대체값
SECTION_KEYS = ("section_limit_speed", "section_left_dist", "section_avg_speed", "section_left_time")

class Port:
  BROADCAST_PORT = 2899
  RECEIVE_PORT = 2843
  LOCATION_PORT = BROADCAST_PORT


def _int_or_none(v):
  try:
    return int(v)
  except (TypeError, ValueError):
    return None


def _i16(v, default=0):
  """A phone value for an Int16 field: numbers and numeric strings rounded and
  clamped, anything else the default. capnp raises on None, text, a fraction
  or an out-of-range value, and one such write used to end main()."""
  try:
    return int(clip(int(round(float(v))), -32768, 32767))
  except (TypeError, ValueError, OverflowError):
    return default


def _flag(v):
  if isinstance(v, str):
    return v.strip().lower() in ("1", "true", "yes")
  try:
    return bool(v)
  except Exception:
    return False


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


ANNOUNCE_SEARCH_S = 1.0     # announcement period while no phone is talking
ANNOUNCE_LINKED_S = 5.0     # and once one is (main() also answers it every 1 s)
LINK_TIMEOUT_S = 6.0        # same as check(): no packet this long = not linked
RX_STATS_WINDOW_S = 60.
RX_STATS_LONG_GAP_S = 1.5   # TmapNda sends about 2 road_limit packets a second
ANNOUNCE_MAX_NEIGHBOURS = 16
LAST_PEER_PATH = os.getenv("NDA_LAST_PEER_PATH", "/data/log/nda_last_peer")
NDA_LINK_LOG_PATH = os.getenv("NDA_LINK_LOG_PATH", "/data/log/nda_link.jsonl")
NDA_LINK_LOG_MAX_BYTES = 512 * 1024


PEERS_MAX = 4               # phones remembered, one per network in practice

# Wi-Fi recovery. The long link losses were not the phone: the EON's wlan0 had
# no address at all. Leaving home Wi-Fi it sat unassociated for 174 s before
# Android joined the car network (2026-11-2x boot t 497 -> 671), because a
# disconnected Android scans less and less often. While wlan0 has no IPv4 the
# announcer now asks wpa_supplicant to scan and reassociate every
# WIFI_KICK_EVERY_S, and if that has not helped after WIFI_TOGGLE_AFTER_S it
# turns Wi-Fi off and on (at most every WIFI_TOGGLE_EVERY_S). Nothing runs
# while wlan0 has an address, so a working link is never touched.
WIFI_KICK_AFTER_S = 5.
WIFI_KICK_EVERY_S = 5.
WIFI_TOGGLE_AFTER_S = 60.
WIFI_TOGGLE_EVERY_S = 120.
WPA_CLI = ['wpa_cli', '-p', '/data/misc/wifi/sockets', '-i', 'wlan0']


def _ip4(ip):
  return struct.unpack('!I', socket.inet_aton(ip))[0]


def load_last_peers(path=None):
  """Recent phone addresses, newest first. The file used to hold one address,
  so in the car the home phone's address was the only one remembered and it
  was dropped as off-net: the car network started from broadcast alone."""
  try:
    with open(path or LAST_PEER_PATH) as f:
      text = f.read().strip()
    peers = json.loads(text) if text.startswith('[') else [text]
    out = []
    for ip in peers:
      socket.inet_aton(ip)
      if ip not in out:
        out.append(ip)
    return out[:PEERS_MAX]
  except Exception:
    return []


def save_last_peers(peers, path=None):
  try:
    with open(path or LAST_PEER_PATH, 'w') as f:
      f.write(json.dumps(peers[:PEERS_MAX]))
  except Exception:
    pass


def announce_unicast_targets(peers, iface_addr, arp_path='/proc/net/arp', iface='wlan0'):
  """Hosts to send the announcement to directly: the remembered phones (DHCP
  usually hands a phone the same address on the same network) and the complete
  wlan0 ARP entries, all inside wlan0's own subnet.

  iface_addr is (ip, netmask) of wlan0, or None when unknown (no filter). The
  filter used to compare the first three octets with the broadcast address,
  which on the car network's /16 (broadcast 172.16.255.255, phone 172.16.0.100)
  rejected every host, so nothing was unicast there at all."""
  if isinstance(peers, str):
    peers = [peers]
  peers = peers or []

  def same_net(ip):
    if iface_addr is None:
      return True
    try:
      my_ip, mask = iface_addr
      return (_ip4(ip) & _ip4(mask)) == (_ip4(my_ip) & _ip4(mask)) and ip != my_ip
    except Exception:
      return False

  out = []
  for ip in peers:
    if ip and ip not in out and same_net(ip):
      out.append(ip)
  try:
    with open(arp_path) as f:
      next(f)
      for line in f:
        p = line.split()
        if len(p) >= 6 and p[5] == iface and p[2] == '0x2' and p[0] not in out and same_net(p[0]):
          out.append(p[0])
          if len(out) >= ANNOUNCE_MAX_NEIGHBOURS:
            break
  except Exception:
    pass
  return out


class NdaLinkLog:
  """/data/log/nda_link.jsonl: when the process started, which broadcast address
  it announced on, when the phone's first packet came and from where, and when
  the link went silent. roadLimitSpeed is not in the rlog, so without this a slow
  NDA connection leaves no trace. A few lines per drive."""

  def __init__(self, path=None):
    self.path = path or NDA_LINK_LOG_PATH
    self.t0 = sec_since_boot()
    self.lost_logged = False
    self._once = set()

  def event(self, name, once=False, **kw):
    if once:
      if name in self._once:
        return
      self._once.add(name)
    if name == 'lost':
      self.lost_logged = True
    elif name == 'linked':
      self.lost_logged = False
    now = sec_since_boot()
    rec = {'t': round(now, 2), 'since_start_s': round(now - self.t0, 1), 'event': name}
    rec.update(kw)
    line = json.dumps(rec, separators=(',', ':'))
    path = self.path

    def write():
      try:
        if os.path.exists(path) and os.path.getsize(path) > NDA_LINK_LOG_MAX_BYTES:
          os.replace(path, path + ".1")
        with open(path, 'a') as f:
          f.write(line + "\n")
      except Exception:
        pass
    threading.Thread(target=write, daemon=True).start()


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

    self.last_rx = -1e9
    # per-minute receive statistics (see RX_STATS_*)
    self.rx_win_t = None
    self.rx_n = 0
    self.rx_last_pkt = None
    self.rx_max_gap = 0.
    self.rx_gaps_long = 0
    self.navi_ref = None
    self.peers = load_last_peers()
    self.last_peer_ip = self.peers[0] if self.peers else None
    self.link_log = NdaLinkLog()
    self.link_log.event('start', last_peer=self.last_peer_ip, peers=self.peers)

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

  @staticmethod
  def get_iface_addr(iface='wlan0'):
    """(ipv4, netmask) of iface, or None when it has no address."""
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      try:
        req = struct.pack('256s', iface.encode('utf-8'))
        ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, req)[20:24])    # SIOCGIFADDR
        mask = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x891b, req)[20:24])  # SIOCGIFNETMASK
        return ip, mask
      finally:
        s.close()
    except Exception:
      return None

  def wifi_recover(self, now, has_addr):
    """Nudge Android back onto a Wi-Fi network while wlan0 has no address (see WIFI_*)."""
    if has_addr:
      if self.wifi_lost_t is not None and self.wifi_kick_t >= self.wifi_lost_t:
        self.link_log.event('wifi_back', down_s=round(now - self.wifi_lost_t, 1))
      self.wifi_lost_t = None
      return
    if self.wifi_lost_t is None:
      self.wifi_lost_t = now
      return
    down = now - self.wifi_lost_t
    if down < WIFI_KICK_AFTER_S:
      return
    if down >= WIFI_TOGGLE_AFTER_S and now - self.wifi_toggle_t >= WIFI_TOGGLE_EVERY_S:
      self.wifi_toggle_t = now
      self.link_log.event('wifi_toggle', down_s=round(down, 1))
      cmd = 'svc wifi disable; sleep 2; svc wifi enable'
      subprocess.Popen(['sh', '-c', cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      self.wifi_kick_t = now
      return
    if now - self.wifi_kick_t >= WIFI_KICK_EVERY_S:
      if self.wifi_kick_t < self.wifi_lost_t:
        self.link_log.event('wifi_kick', down_s=round(down, 1))   # first kick of this outage
      self.wifi_kick_t = now
      for args in (['scan'], ['reassociate']):
        try:
          subprocess.Popen(WPA_CLI + args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
          pass

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
    # The phone only learns the EON's address from this announcement. It used to
    # go out every 5 s as a broadcast only, with the broadcast address re-read
    # every 50 s. Phones in Wi-Fi power save drop most broadcast frames, and on
    # 2026-11-24 the first TmapNda packet came 348 s after start. Until a phone
    # talks: every 1 s, as a broadcast plus unicasts to the last phone and to the
    # wlan0 neighbours (unicast is not filtered). Once linked, main() answers the
    # phone every 1 s already, so only the 5 s broadcast stays.
    broadcast_address = None
    last_sent = -1e9
    msg = 'EON:ROAD_LIMIT_SERVICE:v1'.encode()
    self.wifi_lost_t = None
    self.wifi_kick_t = -1e9
    self.wifi_toggle_t = -1e9

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
          now = sec_since_boot()
          linked = now - self.last_rx < LINK_TIMEOUT_S
          interval = ANNOUNCE_LINKED_S if linked else ANNOUNCE_SEARCH_S
          if now - last_sent >= interval:
            last_sent = now
            try:
              address = self.get_broadcast_address()
              if address != broadcast_address:
                self.link_log.event('bcast_addr', addr=address, prev=broadcast_address)
                broadcast_address = address
              iface_addr = self.get_iface_addr()
              try:
                self.wifi_recover(now, iface_addr is not None)
              except Exception as e:
                self.link_log.event('wifi_recover_error', once=True, err=str(e)[:80])
              if broadcast_address is not None:
                sock.sendto(msg, (broadcast_address, Port.BROADCAST_PORT))
                if not linked:
                  for ip in announce_unicast_targets(self.peers, iface_addr):
                    try:
                      sock.sendto(msg, (ip, Port.BROADCAST_PORT))
                    except Exception:
                      pass
            except Exception as e:
              self.link_log.event('bcast_error', err=str(e)[:80], once=True)
          time.sleep(ANNOUNCE_SEARCH_S)

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
        self._note_rx(self.remote_addr[0])
        try:
          json_obj = json.loads(data.decode())
        except (UnicodeDecodeError, ValueError) as e:
          json_obj = None
          bad = 'not JSON: %s' % str(e)[:60]
        else:
          bad = None if isinstance(json_obj, dict) else 'JSON %s, not an object' % type(json_obj).__name__
        if bad is not None:
          # Drop this packet only. Clearing json_road_limit here (as the old
          # catch-all did) lifted every limit until the next good packet.
          self.link_log.event('bad_packet', once=True, why=bad, size=len(data))
          return False

        # Destination packets (nda_manager's legacy 'cmd', 'nav_destination') fed
        # the Mapbox navd, which is gone: ignore them, and never run 'cmd' in a shell.
        if 'cmd' in json_obj or 'nav_destination' in json_obj:
          ret = False

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
            self._rx_stat(self.last_updated)
            # TmapNda's apilot is the JSON it sends carrotpilot on 7706 (TBT included):
            # handed to AutoTurnControl by main().
            apilot = json_obj.get('apilot')
            if isinstance(apilot, dict):
              self.apilot = apilot

        finally:
          self.lock.release()

    except Exception as e:
      # Keep the last good data; check() still clears it after 6 s of silence.
      self.link_log.event('recv_error', once=True, err=str(e)[:80])

    return ret

  def _note_rx(self, ip):
    now = sec_since_boot()
    gap = now - self.last_rx
    if gap >= LINK_TIMEOUT_S:
      # First packet since start, or back after a silence: how long it took.
      self.link_log.event('linked', ip=ip, after_s=None if self.last_rx < 0 else round(gap, 1))
    self.last_rx = now
    if ip != self.last_peer_ip:
      self.last_peer_ip = ip
      self.peers = [ip] + [p for p in self.peers if p != ip]
      self.peers = self.peers[:PEERS_MAX]
      save_last_peers(self.peers)

  def _rx_stat(self, now):
    if self.rx_win_t is None:
      self.rx_win_t = now
    if self.rx_last_pkt is not None:
      gap = now - self.rx_last_pkt
      if gap < LINK_TIMEOUT_S:          # longer ones are logged as lost / linked
        self.rx_max_gap = max(self.rx_max_gap, gap)
        if gap >= RX_STATS_LONG_GAP_S:
          self.rx_gaps_long += 1
    self.rx_last_pkt = now
    self.rx_n += 1

  def _rx_stats_flush(self, now):
    """One nda_link.jsonl line a minute while packets come: how many, the
    longest gap under the 6 s loss limit, gaps over RX_STATS_LONG_GAP_S, and
    route packets with a 0 distance the turn box rode over. Pauses shorter
    than 6 s left no trace before."""
    if self.rx_win_t is None or now - self.rx_win_t < RX_STATS_WINDOW_S:
      return
    held = 0
    if self.navi_ref is not None:
      held, self.navi_ref.zero_held = self.navi_ref.zero_held, 0
    if self.rx_n > 0:
      self.link_log.event('rx_stats', pkts=self.rx_n, hz=round(self.rx_n / (now - self.rx_win_t), 2),
                          max_gap_s=round(self.rx_max_gap, 2), gaps_over_1_5s=self.rx_gaps_long,
                          navi_zero_held=held)
    self.rx_win_t = now if self.rx_n > 0 else None
    self.rx_n = 0
    self.rx_max_gap = 0.
    self.rx_gaps_long = 0

  def check(self):
    now = sec_since_boot()
    self._rx_stats_flush(now)
    if self.last_rx > 0 and now - self.last_rx >= LINK_TIMEOUT_S and not self.link_log.lost_logged:
      self.link_log.event('lost', silent_s=round(now - self.last_rx, 1))
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


# carrot_serv.py CarrotServ._get_sdi_descr() Korean table (nSdiType -> text).
SDI_DESCR_KO = {
  75: "후면 과속 단속", 76: "후면 신호·과속 단속",
  0: "신호과속", 1: "과속 (고정식)", 2: "구간단속 시작", 3: "구간단속 끝", 4: "구간단속중",
  5: "꼬리물기단속카메라", 6: "신호 단속", 7: "과속 (이동식)", 8: "고정식 과속위험 구간(박스형)",
  9: "버스전용차로구간", 10: "가변 차로 단속", 11: "갓길 감시 지점", 12: "끼어들기 금지",
  13: "교통정보 수집지점", 14: "방범용cctv", 15: "과적차량 위험구간", 16: "적재 불량 단속",
  17: "주차단속 지점", 18: "일방통행도로", 19: "철길 건널목", 20: "어린이 보호구역(스쿨존 시작 구간)",
  21: "어린이 보호구역(스쿨존 끝 구간)", 22: "과속방지턱", 23: "lpg충전소", 24: "터널 구간",
  25: "휴게소", 26: "톨게이트", 27: "안개주의 지역", 28: "유해물질 지역", 29: "사고다발",
  30: "급커브지역", 31: "급커브구간1", 32: "급경사구간", 33: "야생동물 교통사고 잦은 구간",
  34: "우측시야불량지점", 35: "시야불량지점", 36: "좌측시야불량지점", 37: "신호위반다발구간",
  38: "과속운행다발구간", 39: "교통혼잡지역", 40: "방향별차로선택지점", 41: "무단횡단사고다발지점",
  42: "갓길 사고 다발 지점", 43: "과속 사발 다발 지점", 44: "졸음 사고 다발 지점", 45: "사고다발지점",
  46: "보행자 사고다발지점", 47: "차량도난사고 상습발생지점", 48: "낙석주의지역", 49: "결빙주의지역",
  50: "병목지점", 51: "합류 도로", 52: "추락주의지역", 53: "지하차도 구간",
  54: "주택밀집지역(교통진정지역)", 55: "인터체인지", 56: "분기점", 57: "휴게소(lpg충전가능)",
  58: "교량", 59: "제동장치사고다발지점", 60: "중앙선침범사고다발지점", 61: "통행위반사고다발지점",
  62: "목적지 건너편 안내", 63: "졸음 쉼터 안내", 64: "노후경유차단속", 65: "터널내 차로변경단속",
  66: "",
}


def _apilot_int(apilot, key, default=0):
  try:
    return int(apilot.get(key, default))
  except (TypeError, ValueError):
    return default


def _apilot_text(apilot, key):
  v = apilot.get(key)
  if v is None:
    return ""
  v = str(v)
  return "" if v == "null" else v[:100]


# The turn-info box is drawn only while goPosDist and goPosTime are both > 0,
# so one packet with a 0 there (TMAP re-routing, a hiccup) blanked the whole
# box until the next packet, 0.5 s later. Up to NAVI_ZERO_HOLD_S of such
# packets keep the last route values; a route that has really ended goes
# after that.
NAVI_ZERO_HOLD_S = 3.0


class NaviInfo:
  """What the onroad turn-info box shows besides the turn itself (carrot carrotMan fields)."""

  def __init__(self):
    self.route_t = None
    self.zero_held = 0        # packets bridged by NAVI_ZERO_HOLD_S (for the rx stats)
    self.clear()

  def clear(self):
    self.tbt_main_text = ""
    self.go_pos_dist = 0
    self.go_pos_time = 0
    self.sdi_descr = ""
    self.pos_road_name = ""
    self.route_t = None

  def update(self, apilot, now=None):
    # carrot reads these in the same GuidePoint block as the TBT fields.
    if not isinstance(apilot, dict) or "nRoadLimitSpeed" not in apilot:
      return
    go_dist = _apilot_int(apilot, "nGoPosDist")
    go_time = _apilot_int(apilot, "nGoPosTime")
    if go_dist > 0 and go_time > 0:
      self.route_t = now
    elif (self.go_pos_dist > 0 and now is not None and self.route_t is not None and
          now - self.route_t < NAVI_ZERO_HOLD_S):
      self.zero_held += 1
      return                  # keep the last route values (and text) for now
    self.tbt_main_text = _apilot_text(apilot, "szTBTMainText")
    self.go_pos_dist = go_dist
    self.go_pos_time = go_time
    self.pos_road_name = _apilot_text(apilot, "szPosRoadName")
    sdi_type = _apilot_int(apilot, "nSdiType", -1)
    sdi_dist = _apilot_int(apilot, "nSdiDist")
    # carrot: _get_sdi_descr(-1 if nSdiType == 0 and nSdiDist == 0 else nSdiType)
    self.sdi_descr = SDI_DESCR_KO.get(-1 if sdi_type == 0 and sdi_dist == 0 else sdi_type, "")


class AtcRunner:
  """Feeds AutoTurnControl from TmapNda packets and carState, and logs it.

  roadLimitSpeed is not in the rlog, so without a log nothing would show what
  the navigation asked for at a turn. One JSON line on every change of
  atcType / xTurnInfo, and one a second while a turn is pending. Background
  thread append, a few lines a minute: nothing measurable.
  """

  def __init__(self, params=None, log_path=ATC_LOG_PATH):
    from selfdrive.controls.lib.auto_turn_control import AutoTurnControl, STALE_S
    self.atc = AutoTurnControl()
    self.atc_stale_s = STALE_S
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
    self.navi = NaviInfo()

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
      self.navi.update(apilot, now)
    if now - self.atc.last_packet_t > self.atc_stale_s:
      self.navi.clear()
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
    navi = self.navi
    msg.tbtMainText = navi.tbt_main_text
    msg.goPosDist = int(clip(navi.go_pos_dist, 0, 2**31 - 1))
    msg.goPosTime = int(clip(navi.go_pos_time, 0, 2**31 - 1))
    msg.sdiDescr = navi.sdi_descr
    msg.posRoadName = navi.pos_road_name

  def _log(self, now):
    atc = self.atc
    key = (atc.atcType, atc.xTurnInfo)
    changed = key != self.prev_key
    self.prev_key = key
    pending = (atc.xTurnInfo > 0 and atc.atcType not in ("none",)) or atc.turn_hold_m > 0.
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
      'turn_hold_m': round(atc.turn_hold_m, 1),
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


SECTION_LOG_PATH = os.getenv("SECTION_LOG_PATH", "/data/log/section.jsonl")
SECTION_LOG_INTERVAL_S = 1.0
SECTION_LOG_TAIL_S = 30.       # keep logging this long after the section ends
SECTION_LOG_MAX_BYTES = 1024 * 1024
SECTION_SDI_TYPES = (2, 3, 4)  # 구간단속 시작 / 끝 / 구간단속중


class SectionLog:
  """/data/log/section.jsonl: section enforcement (구간단속) as TmapNda reports it.

  roadLimitSpeed is not in the rlog, and controlsState only shows the cap the
  EON applied, so when the car stays slow after a section nothing showed
  whether the phone was still sending it. While a section is reported (block
  distance > 0 or a section SDI in the first slot) and for SECTION_LOG_TAIL_S
  after, one line a second plus one at start and end: the phone's raw values,
  how old the last packet is, and how far the car has gone since the phone's
  block distance last changed (a frozen value shows as that number growing
  while the distance stays put). A dict and a background append once a second.
  """

  def __init__(self, path=SECTION_LOG_PATH):
    self.path = path
    self.raw = {}
    self.last_pkt_t = None
    self.active = False
    self.end_t = None
    self.last_t = -1e9
    self.odo = 0.
    self.block_dist = None
    self.block_changed_odo = 0.
    self.block_changed_t = None
    self.prev_t = None

  def update(self, now, apilot, road_limit, v_ego):
    dt = 0. if self.prev_t is None else clip(now - self.prev_t, 0., 0.5)
    self.prev_t = now
    self.odo += max(0., v_ego) * dt
    if isinstance(apilot, dict) and "nRoadLimitSpeed" in apilot:
      self.last_pkt_t = now
      self.raw = {k: _int_or_none(apilot.get(k)) for k in (
        "nSdiBlockDist", "nSdiBlockSpeed", "nSdiBlockAverageSpeed", "nSdiBlockTime",
        "nSdiType", "nSdiDist", "nSdiSpeedLimit")}
      bd = self.raw["nSdiBlockDist"]
      if bd != self.block_dist:
        self.block_dist = bd
        self.block_changed_odo = self.odo
        self.block_changed_t = now
    rl = road_limit if isinstance(road_limit, dict) else {}
    sec_left = _int_or_none(rl.get("section_left_dist")) or 0
    cam_type = _int_or_none(rl.get("cam_type"))
    in_section = sec_left > 0 or (self.raw.get("nSdiBlockDist") or 0) > 0 or cam_type in SECTION_SDI_TYPES

    event = None
    if in_section and not self.active:
      event = 'start'
    elif self.active and not in_section:
      event = 'end'
      self.end_t = now
    self.active = in_section
    tail = self.end_t is not None and now - self.end_t < SECTION_LOG_TAIL_S
    if event is None:
      if not (in_section or tail) or now - self.last_t < SECTION_LOG_INTERVAL_S:
        return
      event = 'tick' if in_section else 'tail'
    self.last_t = now

    rec = {
      't': round(now, 2), 'event': event, 'kph': round(v_ego * 3.6, 1),
      'pkt_age_s': None if self.last_pkt_t is None else round(now - self.last_pkt_t, 1),
      'rl_none': road_limit is None,
      'sec_limit': _int_or_none(rl.get("section_limit_speed")), 'sec_left': sec_left,
      'sec_avg': _int_or_none(rl.get("section_avg_speed")), 'sec_time': _int_or_none(rl.get("section_left_time")),
      'cam_type': cam_type, 'cam_limit': _int_or_none(rl.get("cam_limit_speed")),
      'cam_dist': _int_or_none(rl.get("cam_limit_speed_left_dist")),
      'road_limit': _int_or_none(rl.get("road_limit_speed")),
      'block_same_m': None if self.block_changed_t is None else round(self.odo - self.block_changed_odo, 1),
      'block_same_s': None if self.block_changed_t is None else round(now - self.block_changed_t, 1),
    }
    rec.update(self.raw)
    line = json.dumps(rec, separators=(',', ':'))
    path = self.path

    def write():
      try:
        if os.path.exists(path) and os.path.getsize(path) > SECTION_LOG_MAX_BYTES:
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
  server.navi_ref = atc_runner.navi
  section_log = SectionLog()
  last_sdp_t = -1e9

  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    try:

      try:
        sock.bind(('0.0.0.0', 843))
      except:
        sock.bind(('0.0.0.0', Port.RECEIVE_PORT))

      sock.setblocking(False)

      loop_errors = {}
      while True:
        try:
          server.udp_recv(sock, LOOP_TIMEOUT_S)

          now = sec_since_boot()
          with server.lock:
            apilot, server.apilot = server.apilot, None
          try:
            atc_runner.update(now, apilot)
          except Exception as e:
            print("road_speed_limiter: ATC update failed:", e)
          try:
            section_log.update(now, apilot, server.json_road_limit, atc_runner.v_ego)
          except Exception as e:
            print("road_speed_limiter: section log failed:", e)

          dat = messaging.new_message()
          dat.init('roadLimitSpeed')
          dat.roadLimitSpeed.active = _i16(server.active)
          # TmapNda's road_limit_speed is not taken (2026-11-27, on request): during
          # Kakao route guidance TmapNda can keep sending one value for kilometres,
          # and only the SPEED LIMIT board read it. roadLimitSpeed stays 0, so the
          # board does not show. ATC still reads apilot.nRoadLimitSpeed (fork
          # speed, turn start distance); that is unchanged.
          dat.roadLimitSpeed.roadLimitSpeed = 0
          dat.roadLimitSpeed.isHighway = _flag(server.get_limit_val("is_highway", False))
          dat.roadLimitSpeed.camType = _i16(server.get_limit_val("cam_type", 0))
          dat.roadLimitSpeed.camLimitSpeedLeftDist = _i16(server.get_limit_val("cam_limit_speed_left_dist", 0))
          dat.roadLimitSpeed.camLimitSpeed = _i16(server.get_limit_val("cam_limit_speed", 0))
          dat.roadLimitSpeed.sectionLimitSpeed = _i16(server.get_limit_val("section_limit_speed", 0))
          dat.roadLimitSpeed.sectionLeftDist = _i16(server.get_limit_val("section_left_dist", 0))
          dat.roadLimitSpeed.camSpeedFactor = CAMERA_SPEED_FACTOR
          dat.roadLimitSpeed.bumpLeftDist = max(0, _i16(server.get_limit_val("bump_dist", 0)))

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
          # One bad iteration used to end main() through the except below with
          # nothing logged, and nothing restarts this process (manager.py starts
          # it once), so controlsd kept the last limit until the next boot.
          now = sec_since_boot()
          key = type(e).__name__ + ':' + str(e)[:60]
          if now - loop_errors.get(key, -1e9) >= 60.:
            loop_errors[key] = now
            server.link_log.event('loop_error', err=key)
            print("road_speed_limiter: loop error:", key)
          time.sleep(LOOP_TIMEOUT_S)

    except Exception as e:
      server.last_exception = e
      server.link_log.event('main_exit', err=str(e)[:80])


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


RLS_STALE_S = 3.   # controlsd: a roadLimitSpeed older than this is dropped


# SECTION START BRIDGE. Passing the section's start camera, TmapNda drops the
# camera (first SDI slot) a packet or more before it sends the section block
# (nSdiBlockDist), so for 1-3 s nothing was reported and the cap lifted: on
# 2026-11-26--00-21-48 at 1999-2002 s it went back to 90 and the car sped up
# 73 -> 78 km/h before the 80 km/h section's 76 came back. Now, once a start
# camera (구간단속 시작) has been seen within SECTION_BRIDGE_PASS_M, its limit
# is held for up to SECTION_BRIDGE_S until the section data arrives. Anything
# section-shaped (block distance, an in-section or end camera) ends the bridge,
# so the release at the section's end is unchanged.
SECTION_START_CAM_TYPE = 2
SECTION_IN_CAM_TYPES = (3, 4)      # 구간단속 끝, 구간단속중
SECTION_BRIDGE_PASS_M = 150
SECTION_BRIDGE_S = 10.
SECTION_BRIDGE_LEFT_DIST_M = 1000  # left_dist reported while bridging (no distance is known)


class RoadSpeedLimiter:
  def __init__(self):
    self.slowing_down = False
    self.started_dist = 0
    self.section_start = None   # (limit, last distance, time) of the last start camera seen

    self.sock = messaging.sub_sock("roadLimitSpeed")
    self.roadLimitSpeed = None
    self.rx_t = -1e9

    # 방지턱: 20m 전부터 20km/h, 마지막 방지턱을 넘고 25m 뒤 해제. controlsd가 speed_bump_active를 보고
    # 목표를 V_CRUISE_MIN 예외로 받는다.
    self.bump_planner = SpeedBumpPlanner()
    self.speed_bump_active = False
    self.bump_log = BumpLog()
    self.atc_slowing_down = False
    # ATC limit below V_CRUISE_MIN is in force (intersection turn); controlsd reads it.
    self.atc_low_active = False

  def recv(self):
    try:
      dat = messaging.recv_sock(self.sock, wait=False)
      now = sec_since_boot()
      if dat is not None:
        self.roadLimitSpeed = dat.roadLimitSpeed
        self.rx_t = now
      elif self.roadLimitSpeed is not None and now - self.rx_t > RLS_STALE_S:
        # The receiver publishes at 20 Hz. Without this a dead receiver left its
        # last message -- a section or camera cap, an ATC turn -- in force for
        # the rest of the drive.
        self.roadLimitSpeed = None
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
    self.atc_low_active = False

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

      # TmapNda always sends is_highway false (the real road class only comes
      # with TMAP's auto-exit option), so the old non-highway ceiling of 100
      # dropped every 110 / 120 km/h camera and section. Those limits exist on
      # expressways only, so the ceiling is 120 everywhere; the floor stays 40
      # on a road known to be a highway.
      MIN_LIMIT = 40 if is_highway else 20
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
        kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
        self.atc_low_active = atc_result[0] < ATC_FLOOR_KPH * kph_to_clu

      if self.speed_bump_active:
        kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
        bump_apply = bump_target_kph * kph_to_clu
        # 방지턱이 아직 멀어 목표가 높을 때는 더 낮은 카메라·구간 제한이 이긴다.
        if 0 < cam_result[0] < bump_apply:
          return cam_result
        self.slowing_down = False
        self.atc_low_active = False
        return bump_apply, SpeedBumpPlanner.BUMP_SPEED_KPH * kph_to_clu, \
               self.bump_planner.left_dist, not was_bump_active, log
      return cam_result

    except Exception as e:
      log = "Ex: " + str(e)
      pass

    self.slowing_down = False
    self.speed_bump_active = False
    self.atc_low_active = False
    return 0, 0, 0, False, log

  def _atc_limit(self, is_metric, cluster_speed, log):
    """AutoTurnControl speed cap as a get_max_speed() result, or None.

    The cap is carrot's distance curve (calculate_current_speed). controlsd
    ignores any limit under V_CRUISE_MIN (20 km/h), so a cap that falls below
    it (U-turn / arrival target 1 km/h, or a turn speed under 20) would be
    dropped and the car would speed back up near the point. It is held at
    ATC_FLOOR_KPH instead, like the camera floor -- or at the turn's own
    target when that is ATC_LOW_FLOOR_KPH or more (intersection turn, 15 km/h).
    """
    rls = self.roadLimitSpeed
    cap_kph = float(getattr(rls, 'atcDesiredSpeed', 0.) or 0.)
    if not (0. < cap_kph < 250.):
      self.atc_slowing_down = False
      return None
    kph_to_clu = 1. if is_metric else CV.KPH_TO_MPH
    target_kph = float(rls.atcTargetSpeed)
    floor_kph = target_kph if ATC_LOW_FLOOR_KPH <= target_kph < ATC_FLOOR_KPH else ATC_FLOOR_KPH
    cap_clu = max(cap_kph, floor_kph) * kph_to_clu
    target_clu = max(target_kph, floor_kph) * kph_to_clu
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
    now = sec_since_boot()
    has_cam = cam_limit_speed_left_dist is not None and cam_limit_speed is not None and cam_limit_speed_left_dist > 0
    has_section = section_left_dist is not None and section_limit_speed is not None and section_left_dist > 0
    if has_section or (has_cam and cam_type in SECTION_IN_CAM_TYPES):
      self.section_start = None
    elif has_cam and cam_type == SECTION_START_CAM_TYPE and MIN_LIMIT <= cam_limit_speed <= MAX_LIMIT:
      self.section_start = (cam_limit_speed, cam_limit_speed_left_dist, now)

    def bridge():
      """The start camera's limit while the section data has not arrived yet, else None."""
      st = self.section_start
      if st is None:
        return None
      limit, last_dist, seen_t = st
      if last_dist > SECTION_BRIDGE_PASS_M or now - seen_t > SECTION_BRIDGE_S:
        if now - seen_t > SECTION_BRIDGE_S:
          self.section_start = None
        return None
      first_started = not self.slowing_down
      self.slowing_down = True
      return (self._camera_target(limit, camSpeedFactor, is_metric), limit,
              SECTION_BRIDGE_LEFT_DIST_M, first_started, log)

    if has_cam:

      if MIN_LIMIT <= cam_limit_speed <= MAX_LIMIT:
        cap_clu, active = self._camera_cap(
          cluster_speed, is_metric, cam_limit_speed, cam_limit_speed_left_dist, camSpeedFactor,
          in_section=(section_left_dist is not None and section_left_dist >= 10))
        if active:
          first_started = not self.slowing_down
          self.slowing_down = True
          self.started_dist = cam_limit_speed_left_dist if first_started else self.started_dist
          return cap_clu, cam_limit_speed, cam_limit_speed_left_dist, first_started, log

      if cam_type != SECTION_START_CAM_TYPE:
        held = bridge()
        if held is not None:
          return held
      self.slowing_down = False
      return 0, cam_limit_speed, cam_limit_speed_left_dist, False, log

    elif has_section:
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

    held = bridge()
    if held is not None:
      return held
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