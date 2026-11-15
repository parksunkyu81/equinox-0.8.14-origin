"""Keep the corner pedal lift from bleeding speed on a hill.

Two lifts take the pedal off around tight corners so the car does not add
speed the steering cannot answer: the steer-saturation hold in controlsd and
PreCornerCoast. Both used to set the pedal scale to 0 outright. On the flat
that is a gentle coast, but uphill gravity adds to the drag: at 5 % grade the
car loses 3-4 km/h every second, and over a 5-10 s corner it slowed enough
for the car behind to sound its horn.

While a lift is active this returns a pedal scale that starts at 0 (the old
behaviour). On the flat the coast stays gentler than HILL_DECEL and the scale
stays 0. Once the car slows harder than that a grade is pulling it back, and
the scale is raised until the car holds its speed (aEgo near 0), then trimmed
so it does not gain speed either. The scale never goes above 1, so it never
adds pedal beyond what the planner asked for.
"""

HILL_DECEL = -0.6      # m/s^2: a flat coast stays above this; slowing harder means a grade
HOLD_LOW = -0.1        # m/s^2: in hill mode, add pedal while slowing harder than this
HOLD_HIGH = 0.1        # m/s^2: in hill mode, take pedal off while gaining faster than this
RISE_RATE = 1.0        # scale/s: 0 -> full pedal in 1 s
FALL_RATE = 0.5        # scale/s


class CornerLiftSpeedHold:
  def __init__(self):
    self.reset()

  def reset(self):
    self.scale = 0.0
    self.hill = False

  def update(self, lift_active, a_ego, dt):
    """Call every control frame. Returns the pedal scale ceiling (0..1) while a
    lift is active, 1.0 otherwise."""
    if not lift_active:
      self.reset()
      return 1.0

    if not self.hill and a_ego < HILL_DECEL:
      self.hill = True

    if self.hill:
      if a_ego < HOLD_LOW:
        self.scale = min(1.0, self.scale + RISE_RATE * dt)
      elif a_ego > HOLD_HIGH:
        self.scale = max(0.0, self.scale - FALL_RATE * dt)
        if self.scale == 0.0:
          # Back to a plain coast: the grade is gone.
          self.hill = False
    return self.scale
