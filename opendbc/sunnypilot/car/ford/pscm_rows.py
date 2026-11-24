"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The numbers a Ford steering module's own data file gives its lane-centering law, for an exact match of the module's
program (F188) and data file (F10A), both read from the module at car init (ford/values.py FW_QUERY_CONFIG).

The law is the 2021-23 F-150's older lane-centering library (ML3V-14D003-BA and -BC run it instruction for
instruction). The module turns the path angle it receives (c1) into target curvature with a gain K2 it looks up by a
preview distance d = 0.28 * speed(km/h) * T, clamped between two tables over speed:

  target curvature = K2(d) * c1 + K1(d) * (c0 + its integral) + slewed(c2 + lead * c3)

Angle mode sends c0 = c2 = c3 = 0, so the module's target is K2 * path_angle, and T stays at its maximum less the
request mode's reduction (no curvature input and no precision weight to lower it). The path angle that makes the module
target exactly the curvature openpilot asks for is therefore curvature / K2.

Every number in a row is read from that data file through the program's own layout; the law and its clamp order are the
program's, held equal to the module's executed code. A row is used only on an exact match: another data file, even one
for the same program, carries other tables.

The same row bounds angle mode in the panda (safety/modes/ford.h, which holds the same numbers): the implied curvature,
the largest K2 the module can use at a speed times a margin times path angle, stays inside comma's lateral acceleration
cap. The car controller holds itself to that bound read more tightly, so the panda never refuses a frame it sends.
"""
from dataclasses import dataclass

import numpy as np

from opendbc.car import structs
from opendbc.car.ford.values import FORD_DATA_FILE_REQUEST
from opendbc.car.fw_query_definitions import StdQueries
from opendbc.car.lateral import MAX_LATERAL_ACCEL

Ecu = structs.CarParams.Ecu

# How much higher the module's K2 may be than the panda's lookup: the module reads its own speed, which may be a few
# percent below the panda's (1.027 needed on the executed fixed point and speed chain of the 2021-23 F-150's newer
# library, plus 6 % allowed for chains not run).
PANDA_K2_MARGIN = 1.10

# One count of the path angle signal, rad: the controller's bound keeps at least this far inside the panda's.
PATH_ANGLE_COUNT = 0.0005


@dataclass(frozen=True)
class PscmRow:
  program: bytes                    # F188, the steering module's strategy
  data_file: bytes                  # F10A, the data file every number below comes from
  panda_row: int                    # the row's index in the panda (safety/modes/ford.h FORD_PSCM_ROW_*), 1-3
  t_max: float                      # preview time T at rest, s
  t_min: float                      # T's floor, s
  t_mode_2: float                   # T's reduction while the request is mode 2 (control states 2-3), s
  speeds_kph: tuple[float, ...]     # the axis of both distance tables
  d_low: tuple[float, ...]          # preview distance floor, m
  d_high: tuple[float, ...]         # preview distance ceiling, m
  k2_distances: tuple[float, ...]   # m
  k2: tuple[float, ...]             # target curvature per radian of path angle, 1/m, by preview distance
  release_rates: tuple[float, ...]  # torque ramp-out on a request 0, Nm/s, by LatCtlRampType 0-3

  def t_for_mode(self, mode: int) -> float:
    return self.t_max - (self.t_mode_2 if mode == 2 else 0.0)

  def distance(self, v_kph: float, t: float) -> float:
    d = 0.28 * v_kph * t
    d_high = float(np.interp(v_kph, self.speeds_kph, self.d_high))
    return d_high if d > d_high else max(d, float(np.interp(v_kph, self.speeds_kph, self.d_low)))

  def k2_at(self, v_kph: float, t: float) -> float:
    return float(np.interp(self.distance(v_kph, t), self.k2_distances, self.k2))

  def hold_kph(self, t: float) -> float:
    """Below this speed the module's distance is short of the K2 table's first point, so K2 sits at the end of the
    table and the gain the law asks for grows without bound as speed falls to zero. The gain is held at its value here
    instead, which keeps the path angle proportional to speed at a crawl, as it always has been."""
    v_kph = 0.1
    while v_kph < self.speeds_kph[-1] and self.distance(v_kph, t) < self.k2_distances[0]:
      v_kph += 0.1
    return v_kph

  def angle_gain(self, v_ego: float, t: float, hold_kph: float) -> float:
    """Path angle per unit of curvature x speed that makes the module target exactly the curvature: 1 / (K2 v)."""
    v_kph = max(v_ego * 3.6, hold_kph)
    return 1.0 / (self.k2_at(v_kph, t) * v_kph / 3.6)

  def k2_max(self, speed: float) -> float:
    """The largest K2 the module can use at this speed (m/s), times the margin, never above the table's largest gain.
    Over T from its floor up the preview distance is never shorter than the lesser of the ceiling and the floor at T's
    minimum (ford.h ford_pscm_k2_max)."""
    v_kph = speed * 3.6
    d_high = float(np.interp(v_kph, self.speeds_kph, self.d_high))
    d_low = float(np.interp(v_kph, self.speeds_kph, self.d_low))
    d = min(d_high, max(0.28 * v_kph * self.t_min, d_low))
    return min(float(np.interp(d, self.k2_distances, self.k2)) * PANDA_K2_MARGIN, self.k2[0])

  def panda_max_path_angle(self, speed_low: float) -> float:
    """The panda's bound on path angle, rad, at its lowest recent speed (ford.h ford_pscm_path_angle_checks, before its
    one-count rounding allowance): comma's cap at that speed less 1 m/s over the largest K2 there."""
    fudged_speed = max(speed_low - 1.0, 1.0)
    return MAX_LATERAL_ACCEL / fudged_speed ** 2 / self.k2_max(speed_low)

  def max_path_angle(self, v_ego: float) -> float:
    """The controller's own bound, rad: the cap at its own speed over the largest K2 there, one signal count under.
    Inside the panda's whenever the panda's lowest recent speed is within 1 m/s of v_ego: a lower speed opens more
    room through the panda's 1 m/s fudge than its larger K2 takes, and a higher one has the smaller K2."""
    return MAX_LATERAL_ACCEL / max(v_ego, 1.0) ** 2 / self.k2_max(v_ego) - PATH_ANGLE_COUNT


# Read from each data file through the program's layout (RAM = data address + 0xFED4F58C for ML3V-14D003-BC), the
# values as the file stores them (float32).
PSCM_ROWS = {(row.program, row.data_file): row for row in (
  # 2023 F-150 Raptor: its own tables, not the 2021 F-150's. K2 is 0.866 of the 2021 file's 4/d and the distance
  # ceiling is 4 s of travel, so the module wants about 1.11-1.15 rad of path angle per (1/m x m/s): the 0.95-1.0 the
  # platform gains send leaves it targeting 0.84-0.89 of the curvature asked for.
  PscmRow(
    program=b'ML3V-14D003-BC',
    data_file=b'PL3V-14D007-CB',
    panda_row=1,
    t_max=5.0,
    t_min=3.0,
    t_mode_2=0.3,
    speeds_kph=(0.0, 7.0, 18.0, 30.0, 60.0, 100.0, 160.0, 200.0),
    d_low=(45.0, 45.0, 45.0, 49.85, 80.1515, 100.0, 160.0, 200.0),
    d_high=(0.0, 7.77, 20.0, 33.33, 66.66, 111.111, 177.0, 222.222),
    k2_distances=(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 150.0),
    k2=(0.3464, 0.1732, 0.1155, 0.0866, 0.0693, 0.0577, 0.0495, 0.0433, 0.0346, 0.0231),
    release_rates=(7.0, 7.0, 10.0, 50.0),
  ),
)}


def _eps_version(car_fw, request: bytes) -> bytes | None:
  for fw in car_fw:
    if fw.ecu == Ecu.eps and request in fw.request:
      return bytes(fw.fwVersion).rstrip(b'\x00')
  return None


def pscm_row(car_fw) -> PscmRow | None:
  """The row for this car's steering module, or None unless both its program and its data file are a known pair."""
  return PSCM_ROWS.get((_eps_version(car_fw, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST),
                        _eps_version(car_fw, FORD_DATA_FILE_REQUEST)))
