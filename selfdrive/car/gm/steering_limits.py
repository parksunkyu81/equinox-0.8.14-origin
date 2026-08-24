GM_MIN_STEER_SPEED_KPH = 10.0
GM_MIN_STEER_SPEED_MS = GM_MIN_STEER_SPEED_KPH / 3.6

# Torque rate limits are the official 0.8.13/0.8.14 GM values and are fixed.
# They live in CarControllerParams (selfdrive/car/gm/values.py) and must stay in
# step with GM_MAX_RATE_UP / GM_MAX_RATE_DOWN in panda/board/safety/safety_gm.h,
# which enforces the same numbers.
STEER_DELTA_UP = 7
STEER_DELTA_DOWN = 17
