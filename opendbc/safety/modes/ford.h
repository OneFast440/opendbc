#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

static const CurvatureSteeringLimits FORD_STEERING_LIMITS = {
  .max_curvature = 1000,              // 0.02 rad/m * curvature_to_can
  .curvature_to_can = 50000,          // CAN units per rad/m
  .frequency = 20,                    // Hz
  .max_curvature_error = 100,         // 0.002 rad/m * curvature_to_can
  .curvature_error_min_speed = 10.0,  // m/s
  .max_steer_power = 0,               // disabled, Ford has no steed power signal
};

// *** sunnypilot: BluePilot lateral control ***
//
// Ford's PSCM does not take a steering command. It takes a third-order polynomial describing the
// road centerline -- path_offset (c0), path_angle (c1), curvature (c2), curvature_rate (c3) --
// and runs its own lateral planner on it. Upstream openpilot drives c2 alone and leaves the rest
// at their inactive sentinels, which is what the stock branch below enforces.
//
// BluePilot adds two strategies, selected once at car init via the sunnypilot safety param
// (current_safety_param_sp, delivered over USB 0xdf before the safety model is set -- a separate
// uint16 from ford_init's own param; same pattern as Subaru STOP_AND_GO):
//
//   curvature  all four signals. c2 is still the actuator and keeps every stock check; c1 trims
//              lane position within a tight cap, c3 anticipates the curve, c0 stays inactive.
//   angle      c1 alone. The PSCM low-pass filters c2 for up to a second and keeps acting on a
//              stale command, so angle mode holds c2/c3/c0 inactive and steers with c1, which
//              has no filter state.
//
// With the mode set to stock, not one byte of the original path changes.
#define FORD_LAT_STOCK 0U
#define FORD_LAT_CURVATURE 1U
#define FORD_LAT_ANGLE 2U

// LatCtlPath_An_Actl scaling: 0.0005 rad per LSB.
// The signal is 11 bits, so the extraction below yields 0..2047, and subtracting
// FORD_INACTIVE_PATH_ANGLE gives exactly [-1000, 1047] CAN units, i.e. the DBC's own
// [-0.5, 0.5235] rad range. Angle mode, where c1 is the actuator, may use all of it and needs no
// value check because the wire format is the limit. Curvature mode only trims lane position with
// c1 and is held to a far tighter cap.
#define FORD_PATH_ANGLE_TO_CAN 2000.0f
#define FORD_CURV_MODE_MAX_PATH_ANGLE 500  // 0.25 rad

static uint16_t ford_lateral_mode = FORD_LAT_STOCK;
static bool ford_angle_mode_engaged = false;  // latched out of Lane_Assist_Data1 in ford_tx_hook
static int ford_shadow_curvature_raw = 0;     // wire units, scale 1e-6 1/m
static int ford_desired_path_angle_last = 0;

// shadow_curvature is sent at a wire scale of 1e-6 1/m (see fordcan_ext.py); convert to the CAN
// units the curvature checks use, matching FORD_STEERING_LIMITS.curvature_to_can (50000):
// raw * 1e-6 * 50000 = raw * 0.05.
static int ford_shadow_curvature_to_can(int raw) {
  return ROUND((float)raw * 0.05f);
}

// path_angle rate-of-change limit. Rate is anchored to the command sequence rather than to a
// measurement -- there is no "measured path angle" -- so the previous command is recorded even on
// a violation: holding it stale would desync this window from openpilot's own (tighter) limiter
// and turn one blocked frame into a runaway of blocked frames.
//
// Both tables mirror openpilot's own limiter scaled x1.02, so this backstop is always slightly
// looser and never blocks a well-behaved command. Values are per call, and
// LateralMotionControl(2) is sent once per CarControllerParams.STEER_STEP (5), i.e. at 20 Hz.
static bool ford_path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, int max_path_angle) {
  // angle mode: mirrors lateral_angle_ext.py _SOFT_ROC_*. lookup_t holds three points;
  // openpilot's 9 and 10 m/s nodes are both 0.055 (flat top), so {10, 15, 25} reproduces the
  // curve exactly and lower speeds clamp to the first point.
  static const struct lookup_t FORD_ANGLE_PATH_ANGLE_ROC = {
    {10., 15., 25.},
    {0.0561, 0.04335, 0.00918}
  };
  // curvature mode: mirrors lateral_curv_ext.py _LC_PATH_ANGLE_ROC_*, where c1 is only a
  // centering trim and moves two orders of magnitude more slowly.
  static const struct lookup_t FORD_CURV_PATH_ANGLE_ROC = {
    {5., 15., 25.},
    {0.00306, 0.00153, 0.00204}
  };

  bool violation = false;

  if (steer_control_enabled) {
    if (max_path_angle > 0) {
      violation |= safety_max_limit_check(desired_path_angle, max_path_angle, -max_path_angle);
    }

    // fudge the speed by 1 m/s so the limit is always slightly above openpilot's, in case a
    // newer speed is read between two commands
    const float fudged_speed = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
    const struct lookup_t *roc = (ford_lateral_mode == FORD_LAT_ANGLE) ? &FORD_ANGLE_PATH_ANGLE_ROC : &FORD_CURV_PATH_ANGLE_ROC;
    const int delta = (safety_interpolate(*roc, fudged_speed) * FORD_PATH_ANGLE_TO_CAN) + 1.;
    violation |= safety_max_limit_check(desired_path_angle, ford_desired_path_angle_last + delta,
                                        ford_desired_path_angle_last - delta);
  } else {
    // path_angle must be at its inactive sentinel while not steering
    violation |= desired_path_angle != 0;
  }

  ford_desired_path_angle_last = desired_path_angle;
  if (!(controls_allowed || controls_allowed_lateral)) {
    ford_desired_path_angle_last = 0;
  }

  return violation;
}

// Angle mode pins the curvature signal at its inactive sentinel, so steer_curvature_cmd_checks
// has nothing to compare against measured curvature and the deviation protection would be lost
// entirely. openpilot therefore publishes the curvature its path_angle was derived from
// alongside the LKA message (see fordcan_ext.create_lka_msg), and it is checked here.
//
// Deliberately narrower than steer_curvature_cmd_checks: the DBC signal cap and the
// deviation-from-measured band, with no rate-of-change term. shadow_curvature is not an actuator
// -- path_angle is, and it carries its own tuned rate limit above. Imposing a second,
// curvature-tuned rate limit on a pure cross-check value blocks at low speed for reasons
// unrelated to how the car is actually steering.
//
// The ISO lateral acceleration ceiling is not applied here either, for the same reason it is not
// applied to c2 in curvature mode: see FORD_BP_CURVATURE_ROC in ford_curvature_cmd_checks below.
static bool ford_shadow_curvature_checks(int shadow_curvature, bool steer_control_enabled,
                                         const CurvatureSteeringLimits limits) {
  bool violation = false;

  if (steer_control_enabled) {
    violation |= safety_max_limit_check(shadow_curvature, limits.max_curvature, -limits.max_curvature);

    if ((limits.max_curvature_error != 0) &&
        ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
      const int lowest_allowed = curvature_state.meas.min - limits.max_curvature_error - 1;
      const int highest_allowed = curvature_state.meas.max + limits.max_curvature_error + 1;
      violation |= safety_max_limit_check(shadow_curvature, highest_allowed, lowest_allowed);
    }
  }

  return violation;
}

// c3 (curvature_rate) needs no value check of its own: the signal is 13 bits on CAN and 11 on
// CAN FD, so subtracting the inactive sentinel yields exactly the DBC's [-0.001024, 0.00102375]
// and [-0.001024, 0.001023] 1/m^2 ranges. The wire format is the limit, and at the ~1.4 m
// lookahead the PSCM actually evaluates, c3 contributes below a millimeter of lateral offset.
// c2 checks for the BluePilot modes. Everything steer_curvature_cmd_checks does except the ISO
// lateral acceleration and jerk envelope, which FORD_BP_CURVATURE_ROC replaces.
static bool ford_curvature_cmd_checks(int desired_curvature, bool steer_control_enabled,
                                      const CurvatureSteeringLimits limits) {
  // BluePilot's own curvature rate limit, which replaces the ISO lateral acceleration and jerk
  // envelope in both BluePilot modes. It is looser than that envelope between roughly 11 and
  // 28 m/s and tighter outside it, and it carries no acceleration ceiling below the DBC's own
  // 0.02 1/m cap.
  //
  // This is a deliberate, explicitly requested relaxation for Ford's BluePilot modes only: stock
  // mode still goes through steer_curvature_cmd_checks and keeps the full envelope. What still
  // bounds these modes is the DBC signal range, this rate table, the deviation-from-measured
  // band, the real-time message rate limit and the controls_allowed gate.
  //
  // Mirrors BluePilot's panda: the looser of openpilot's two tables (lateral_curv_ext.py's
  // _BP_ANGLE_RATE_DOWN) is used in both directions, so the stricter wind-up table openpilot
  // applies always sits inside it.
  static const struct lookup_t FORD_BP_CURVATURE_ROC = {
    {5., 16., 25.},
    {0.0025, 0.0014, 0.00018}
  };

  bool violation = false;

  speed_mismatch_check((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR);

  if ((controls_allowed || controls_allowed_lateral) && steer_control_enabled) {
    // *** DBC signal cap ***
    violation |= safety_max_limit_check(desired_curvature, limits.max_curvature, -limits.max_curvature);

    // *** rate limit ***
    // fudge the speed by 1 m/s so the limit is always slightly above openpilot's, in case a
    // newer speed is read between two commands
    const float fudged_speed = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
    const int delta = (safety_interpolate(FORD_BP_CURVATURE_ROC, fudged_speed) * limits.curvature_to_can) + 1.;
    violation |= safety_max_limit_check(desired_curvature, curvature_state.desired_last + delta,
                                        curvature_state.desired_last - delta);

    // *** curvature error from measured ***
    // openpilot clips its command to the same band, so this only fires on a genuine divergence
    if ((limits.max_curvature_error != 0) &&
        ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
      const int lowest_allowed = curvature_state.meas.min - limits.max_curvature_error - 1;
      const int highest_allowed = curvature_state.meas.max + limits.max_curvature_error + 1;
      violation |= safety_max_limit_check(desired_curvature, highest_allowed, lowest_allowed);
    }

    // *** real time rate limit ***
    violation |= rt_curvature_rate_limit_check(limits);
  }

  curvature_state.desired_last = desired_curvature;

  // Curvature must be 0 while not steering: the signal range is not wide enough to enforce it
  // tracking measured instead.
  if (!steer_control_enabled) {
    violation |= desired_curvature != 0;
  }

  violation |= !(controls_allowed || controls_allowed_lateral) && steer_control_enabled;

  // re-centre the rate window on the car's actual curvature so a blocked frame can recover
  if (violation || !(controls_allowed || controls_allowed_lateral)) {
    curvature_state.desired_last = SAFETY_CLAMP(curvature_state.meas.values[0],
                                                -limits.max_curvature, limits.max_curvature);
  }

  return violation;
}

static bool ford_bp_tx_checks(bool steer_control_enabled, unsigned int raw_curvature,
                              unsigned int raw_path_angle, unsigned int raw_path_offset,
                              unsigned int raw_curvature_rate, unsigned int inactive_curvature_rate) {
  bool violation = false;

  // c0 is computed by both strategies but never sent: c0 and c1 fight each other on this
  // platform, and the ride is worse with both.
  violation |= raw_path_offset != FORD_INACTIVE_PATH_OFFSET;

  const int desired_curvature = (int)raw_curvature - (int)FORD_INACTIVE_CURVATURE;
  const int desired_path_angle = (int)raw_path_angle - (int)FORD_INACTIVE_PATH_ANGLE;

  if (ford_lateral_mode == FORD_LAT_ANGLE) {
    // Ford adjusts its limits by speed, so it checks two speed sources against each other and
    // drops controls when they disagree. The other two modes do this inside their own c2 check;
    // angle mode has no c2 command, so do it here, before the controls_allowed gate below sees
    // the result.
    speed_mismatch_check((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR);

    // c2 and c3 are held inactive; c1 is the actuator and may use the whole signal range
    violation |= raw_curvature != FORD_INACTIVE_CURVATURE;
    violation |= raw_curvature_rate != inactive_curvature_rate;
    violation |= ford_path_angle_cmd_checks(desired_path_angle, steer_control_enabled, 0);
    violation |= ford_shadow_curvature_checks(ford_shadow_curvature_to_can(ford_shadow_curvature_raw),
                                              steer_control_enabled, FORD_STEERING_LIMITS);

    // path_angle's rate limit is per message, so without this openpilot could slew five times
    // faster than intended simply by sending LateralMotionControl at 100 Hz instead of 20 Hz.
    // steer_curvature_cmd_checks does this for the other two modes; angle mode does not call it,
    // and curvature_state's real-time window is otherwise unused here.
    if (steer_control_enabled) {
      violation |= rt_curvature_rate_limit_check(FORD_STEERING_LIMITS);
    }

    // Corroboration: the LKA message must independently say angle mode is engaged before
    // path_angle is allowed to leave its sentinel, so one crafted LateralMotionControl frame
    // cannot steer on its own.
    violation |= steer_control_enabled && !ford_angle_mode_engaged;

    // No lateral control at all when controls are not allowed
    violation |= steer_control_enabled && !(controls_allowed || controls_allowed_lateral);

    // keep the stock path's state coherent: nothing is ever commanded on c2 here
    curvature_state.desired_last = 0;
  } else {
    // c2 is still the actuator and keeps the controls_allowed gate, the measured-curvature band
    // and the real-time rate limit; only the ISO envelope is replaced by BluePilot's rate table.
    // c1 only trims lane position, so it is held to a far tighter cap than the signal allows.
    violation |= ford_curvature_cmd_checks(desired_curvature, steer_control_enabled, FORD_STEERING_LIMITS);
    violation |= ford_path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_CURV_MODE_MAX_PATH_ANGLE);
  }

  return violation;
}

static void ford_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == FORD_MAIN_BUS) {
    // Update in motion state from standstill signal
    if (msg->addr == FORD_DesiredTorqBrk) {
      // Signal: VehStop_D_Stat
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    // Update vehicle speed
    if (msg->addr == FORD_BrakeSysFeatures) {
      // Signal: Veh_V_ActlBrk
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    // Check vehicle speed against a second source
    if (msg->addr == FORD_EngVehicleSpThrottle2) {
      // Disable controls if speeds from ABS and PCM ECUs are too far apart.
      // Signal: Veh_V_ActlEng
      float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED_2(filtered_pcm_speed);
    }

    // Update vehicle yaw rate
    if (msg->addr == FORD_Yaw_Data_FD1) {
      // FIXME: safety can receive yaw before new vehicle speed, it should recompute meas on either received
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
    }

    // Update gas pedal
    if (msg->addr == FORD_EngVehicleSpThrottle) {
      // Pedal position: (0.1 * val) in percent
      // Signal: ApedPos_Pc_ActlArb
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    // Update brake pedal and cruise state
    if (msg->addr == FORD_EngBrakeData) {
      // Signal: BpedDrvAppl_D_Actl
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

      // Signal: CcStat_D_Actl
      unsigned int cruise_state = msg->data[1] & 0x07U;
      bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
      pcm_cruise_check(cruise_engaged);
      acc_main_on = (cruise_state == 3U) || cruise_engaged;
    }

    if (msg->addr == FORD_Steering_Data_FD1) {
      mads_button_press = GET_BIT(msg, 40U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = ((msg->data[6] >> 6) & 1U) || ((msg->data[6] >> 7) & 1U);

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny; // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !controls_allowed;     // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }

    // sunnypilot: angle control carries angle_mode_engaged and shadow_curvature in bits of this
    // message that no DBC signal maps to (byte 4 bit 0, bytes 5-6; see fordcan_ext.create_lka_msg
    // for the layout). Read straight out of the frame being transmitted, in the same tx_hook call
    // that already reads LkaActvStats_D2_Req above -- panda does not self-receive its own TX, so
    // a dedicated CAN id would never arrive.
    if (ford_lateral_mode == FORD_LAT_ANGLE) {
      ford_angle_mode_engaged = (msg->data[4] & 0x1U) != 0U;
      const uint32_t shadow_unsigned = ((uint32_t)msg->data[5] << 8) | (uint32_t)msg->data[6];
      ford_shadow_curvature_raw = (shadow_unsigned > 32767U) ? ((int)shadow_unsigned - 65536) : (int)shadow_unsigned;
    }
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);

    bool violation;
    if (ford_lateral_mode != FORD_LAT_STOCK) {
      violation = ford_bp_tx_checks(steer_control_enabled, raw_curvature, raw_path_angle,
                                    raw_path_offset, raw_curvature_rate, FORD_INACTIVE_CURVATURE_RATE);
    } else {
      // These signals are not yet tested with the current safety limits
      violation = (raw_curvature_rate != FORD_INACTIVE_CURVATURE_RATE) || (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

      // Check angle error and steer_control_enabled
      int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
      violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];

    bool violation;
    if (ford_lateral_mode != FORD_LAT_STOCK) {
      violation = ford_bp_tx_checks(steer_control_enabled, raw_curvature, raw_path_angle,
                                    raw_path_offset, raw_curvature_rate, FORD_CANFD_INACTIVE_CURVATURE_RATE);
    } else {
      // These signals are not yet tested with the current safety limits
      violation = (raw_curvature_rate != FORD_CANFD_INACTIVE_CURVATURE_RATE) || (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

      // Check angle error and steer_control_enabled
      int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
      violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  static RxCheck ford_rx_checks[] = {
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
    // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
    // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},
    // These messages have no counter or checksum
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  // sunnypilot: which signal steers, from the SP safety param. openpilot sets the matching
  // CP_SP.fordLateralTuning from the same read, so the two layers can never disagree.
  // An unknown value falls back to stock rather than picking a mode at random.
  const uint16_t FORD_PARAM_SP_LATERAL_MODE_MASK = 0x3U;
  ford_lateral_mode = current_safety_param_sp & FORD_PARAM_SP_LATERAL_MODE_MASK;
  if ((ford_lateral_mode != FORD_LAT_CURVATURE) && (ford_lateral_mode != FORD_LAT_ANGLE)) {
    ford_lateral_mode = FORD_LAT_STOCK;
  }
  ford_angle_mode_engaged = false;
  ford_shadow_curvature_raw = 0;
  ford_desired_path_angle_last = 0;

  // sunnypilot: openpilot longitudinal follows the alpha longitudinal toggle on every Ford, so
  // ACCDATA is in the allowlist only when the toggle is on. Two changes from upstream:
  // a CAN vehicle on Ford's own ACC no longer has ACCDATA available at all, and a CAN FD vehicle
  // can use openpilot longitudinal on a release build rather than only on a debug panda. This
  // is BluePilot's policy; see opendbc/sunnypilot/car/ford/interfaces_ext.py, which sets the
  // matching flag.
  const uint16_t FORD_PARAM_LONGITUDINAL = 1;
  const bool ford_longitudinal = GET_FLAG(param, FORD_PARAM_LONGITUDINAL);

  safety_config ret;
  if (ford_canfd) {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
  } else {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_STOCK_TX_MSGS);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
