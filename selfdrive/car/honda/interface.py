#!/usr/bin/env python3
import numpy as np
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.honda.carstate import CarState, get_can_parser, get_cam_can_parser
from selfdrive.car.honda.values import CruiseButtons, CAR, HONDA_BOSCH, VISUAL_HUD, ECU, ECU_FINGERPRINT, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, CivicParams, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.controls.lib.planner import _A_CRUISE_MAX_V
from selfdrive.car.interfaces import CarInterfaceBase

A_ACC_MAX = max(_A_CRUISE_MAX_V)

ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter

def compute_gb_honda(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  return float(accel) / 4.8 - creep_brake

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.last_enable_pressed = 0
    self.last_enable_sent = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False

    self.cp = get_can_parser(CP)
    self.cp_cam = get_cam_can_parser(CP)

    # *** init the major players ***
    self.CS = CarState(CP)
    self.VM = VehicleModel(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name)

    self.compute_gb = compute_gb_honda

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):

    # normalized max accel. Allowing max accel at low speed causes speed overshoots
    max_accel_bp = [10, 20]    # m/s
    max_accel_v = [0.714, 1.0] # unit of max accel
    max_accel = interp(v_ego, max_accel_bp, max_accel_v)

    # limit the pcm accel cmd if:
    # - v_ego exceeds v_target, or
    # - a_ego exceeds a_target and v_ego is close to v_target

    eA = a_ego - a_target
    valuesA = [1.0, 0.1]
    bpA = [0.3, 1.1]

    eV = v_ego - v_target
    valuesV = [1.0, 0.1]
    bpV = [0.0, 0.5]

    valuesRangeV = [1., 0.]
    bpRangeV = [-1., 0.]

    # only limit if v_ego is close to v_target
    speedLimiter = interp(eV, bpV, valuesV)
    accelLimiter = max(interp(eA, bpA, valuesA), interp(eV, bpRangeV, valuesRangeV))

    # accelOverride is more or less the max throttle allowed to pcm: usually set to a constant
    # unless aTargetMax is very high and then we scale with it; this help in quicker restart

    return float(max(max_accel, a_target / A_ACC_MAX)) * min(speedLimiter, accelLimiter)

  @staticmethod
  def get_params(candidate, has_relay=False):

    ret = car.CarParams.new_message()
    ret.carName = "honda"
    ret.carFingerprint = candidate
    ret.carVin = vin
    ret.isPandaBlack = has_relay

    if candidate in HONDA_BOSCH:
      ret.safetyModel = car.CarParams.SafetyModel.hondaBosch
      rdr_bus = 0 if has_relay else 2
      ret.enableCamera = is_ecu_disconnected(fingerprint[rdr_bus], FINGERPRINTS, ECU_FINGERPRINT, candidate, ECU.CAM) or has_relay
      ret.radarOffCan = True
      ret.openpilotLongitudinalControl = False
    else:
      ret.safetyModel = car.CarParams.SafetyModel.honda
      ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, ECU.CAM) or has_relay
      ret.enableGasInterceptor = 0x201 in fingerprint[0]
      ret.openpilotLongitudinalControl = ret.enableCamera

    cloudlog.warning("ECU Camera Simulated: %r", ret.enableCamera)
    cloudlog.warning("ECU Gas Interceptor: %r", ret.enableGasInterceptor)

    ret.enableCruise = not ret.enableGasInterceptor
    ret.communityFeature = ret.enableGasInterceptor

    # Certain Hondas have an extra steering sensor at the bottom of the steering rack,
    # which improves controls quality as it removes the steering column torsion from feedback.
    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"

    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kf = 0.00006 # conservative feed-forward

    if candidate in [CAR.CIVIC, CAR.CIVIC_BOSCH]:
      stop_and_go = True
      ret.mass = CivicParams.MASS
      ret.wheelbase = CivicParams.WHEELBASE
      ret.centerToFront = CivicParams.CENTER_TO_FRONT
      ret.steerRatio = 15.38  # 10.93 is end-to-end spec
      tire_stiffness_factor = 1.

      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.54, 0.36]

    elif candidate in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH):
      stop_and_go = True
      if not candidate == CAR.ACCORDH: # Hybrid uses same brake msg as hatch
        ret.safetyParam = 1  # Accord and CRV 5G use an alternate user brake msg
      ret.mass = 3279. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.centerToFront = ret.wheelbase * 0.39
      ret.steerRatio = 16.33  # 11.82 is spec end-to-end
      tire_stiffness_factor = 0.8467
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]
    
    elif candidate in [CAR.CIVIC, CAR.VROUM, CAR.ESSAI]:
      stop_and_go = False
      ret.mass =  1215 + STD_CARGO_KG
      ret.wheelbase = 2.589
      ret.centerToFront = ret.wheelbase * 0.37
      ret.steerRatio = 15.2
      tire_stiffness_factor = 0.72
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.4], [0.12]]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.12]

    else:
      raise ValueError("unsupported car %s" % candidate)

    ret.steerControlType = car.CarParams.SteerControlType.torque

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 25.5 * CV.MPH_TO_MS

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.

    # no max steer limit VS speed
    ret.steerMaxBP = [0.]  # m/s
    ret.steerMaxV = [1.]   # max steer allowed

    ret.gasMaxBP = [0.]  # m/s
    ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.] # max gas allowed
    ret.brakeMaxBP = [5., 20.]  # m/s
    ret.brakeMaxV = [1., 0.8]   # max brake allowed

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]

    ret.stoppingControl = True
    ret.startAccel = 0.5

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.8

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    self.CS.update(self.cp, self.cp_cam)

    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal
    ret.gas = self.CS.car_gas / 256.0
    if not self.CP.enableGasInterceptor:
      ret.gasPressed = self.CS.pedal_gas > 0
    else:
      ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0
    # FIXME: read sendcan for brakelights
    ret.brakeLights = False

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # gear shifter lever
    ret.gearShifter = self.CS.gear_shifter

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringTorqueEps = self.CS.steer_torque_motor
    ret.steeringPressed = self.CS.steer_override

    # cruise state
    ret.cruiseState.enabled = self.CS.pcm_acc_status != 0
    ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.KPH_TO_MS
    ret.cruiseState.available = bool(self.CS.main_on) and not bool(self.CS.cruise_mode)
    ret.cruiseState.speedOffset = self.CS.cruise_speed_offset
    ret.cruiseState.standstill = False

    # TODO: button presses
    buttonEvents = []
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    ret.stockAeb = self.CS.stock_aeb
    ret.stockFcw = self.CS.stock_fcw

    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.leftBlinker
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.rightBlinker
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == CruiseButtons.DECEL_SET:
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    if self.CS.cruise_setting != self.CS.prev_cruise_setting:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_setting != 0:
        be.pressed = True
        but = self.CS.cruise_setting
      else:
        be.pressed = False
        but = self.CS.prev_cruise_setting
      if but == 1:
        be.type = ButtonType.altButton1
      # TODO: more buttons?
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    # events
    events = []
    if self.CP.enableCruise and ret.vEgo < self.CP.minEnableSpeed:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev) or \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    # it can happen that car cruise disables while comma system is enabled: need to
    # keep braking if needed or if the speed is very low
    if self.CP.enableCruise and not ret.cruiseState.enabled and (c.actuators.brake <= 0. or not self.CP.openpilotLongitudinalControl):
      # non loud alert if cruise disbales below 25mph as expected (+ a little margin)
      if ret.vEgo < self.CP.minEnableSpeed + 2.:
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      else:
        events.append(create_event("cruiseDisabled", [ET.IMMEDIATE_DISABLE]))
    if self.CS.CP.minEnableSpeed > 0 and ret.vEgo < 0.001:
      events.append(create_event('manualRestart', [ET.WARNING]))

    cur_time = self.frame * DT_CTRL
    enable_pressed = False
    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and not b.pressed:
        self.last_enable_pressed = cur_time
        enable_pressed = True

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    if self.CP.enableCruise:
      # KEEP THIS EVENT LAST! send enable event if button is pressed and there are
      # NO_ENTRY events, so controlsd will display alerts. Also not send enable events
      # too close in time, so a no_entry will not be followed by another one.
      # TODO: button press should be the only thing that triggers enable
      if ((cur_time - self.last_enable_pressed) < 0.2 and
          (cur_time - self.last_enable_sent) > 0.2 and
          ret.cruiseState.enabled) or \
         (enable_pressed and get_events(events, [ET.NO_ENTRY])):
        events.append(create_event('buttonEnable', [ET.ENABLE]))
        self.last_enable_sent = cur_time
    elif enable_pressed:
      events.append(create_event('buttonEnable', [ET.ENABLE]))

    ret.events = events

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

    # cast to reader so it can't be modified
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    if c.hudControl.speedVisible:
      hud_v_cruise = c.hudControl.setSpeed * CV.MS_TO_KPH
    else:
      hud_v_cruise = 255

    hud_alert = VISUAL_HUD[c.hudControl.visualAlert.raw]

    pcm_accel = int(clip(c.cruiseControl.accelOverride, 0, 1) * 0xc6)

    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators,
                               c.cruiseControl.speedOverride,
                               c.cruiseControl.override,
                               c.cruiseControl.cancel,
                               pcm_accel,
                               hud_v_cruise,
                               c.hudControl.lanesVisible,
                               hud_show_car=c.hudControl.leadVisible,
                               hud_alert=hud_alert)

    self.frame += 1
    return can_sends
