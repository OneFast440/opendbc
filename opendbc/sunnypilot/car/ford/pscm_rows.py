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
"""
from dataclasses import dataclass

import numpy as np

from opendbc.car import structs
from opendbc.car.ford.values import FORD_DATA_FILE_REQUEST
from opendbc.car.fw_query_definitions import StdQueries

Ecu = structs.CarParams.Ecu


@dataclass(frozen=True)
class PscmRow:
  program: bytes                    # F188, the steering module's strategy
  data_file: bytes                  # F10A, the data file every number below comes from
  t_max: float                      # preview time T at rest, s
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


# Read from each data file through the program's layout (RAM = data address + 0xFED4F58C for ML3V-14D003-BC), the
# values as the file stores them (float32).
PSCM_ROWS = {(row.program, row.data_file): row for row in (
  # 2023 F-150 Raptor: its own tables, not the 2021 F-150's. K2 is 0.866 of the 2021 file's 4/d and the distance
  # ceiling is 4 s of travel, so the module wants about 1.11-1.15 rad of path angle per (1/m x m/s): the 0.95-1.0 the
  # platform gains send leaves it targeting 0.84-0.89 of the curvature asked for.
  PscmRow(
    program=b'ML3V-14D003-BC',
    data_file=b'PL3V-14D007-CB',
    t_max=5.0,
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
