from cereal import car
from collections import defaultdict
from common.numpy_fast import interp
from common.kalman.simple_kalman import KF1D
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR, HONDA_BOSCH

# valeurs forcees :
# standstill depuis toyota
# door_all_closed
# seatbelt

GearShifter = car.CarState.GearShifter

def parse_gear_shifter(gear, vals):

  val_to_capnp = {'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
                  'D': GearShifter.drive, 'S': GearShifter.sport, 'L': GearShifter.low}
  try:
    return val_to_capnp[vals[gear]]
  except KeyError:
    return "unknown"


def calc_cruise_offset(offset, speed):
  # euristic formula so that speed is controlled to ~ 0.3m/s below pid_speed
  # constraints to solve for _K0, _K1, _K2 are:
  # - speed = 0m/s, out = -0.3
  # - speed = 34m/s, offset = 20, out = -0.25
  # - speed = 34m/s, offset = -2.5, out = -1.8
  _K0 = -0.3
  _K1 = -0.01879
  _K2 = 0.01013
  return min(_K0 + _K1 * speed + _K2 * speed * offset, 0.)


def get_can_signals(CP):
# this function generates lists for signal, messages and initial values
  signals = [
      ("SPEED1", "SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),

      ("STEER_ANGLE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_SIGN", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_RATE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_RATE_SIGN", "STEERING_SENSORS", 0),
      ("STEER_TORQUE", "STEERING_CONTROL", 0),

      ("PEDAL_GAS", "POWERTRAIN_DATA", 0),
      ("BRAKE_PRESSED", "POWERTRAIN_DATA", 0),
      
      ("STEER_TORQUE_DRIVER", "MACCHINA", 0),
      ("ACC_STATUS", "MACCHINA", 0),   #???
      ("MAIN_ON", "MACCHINA", 0),
      ("GEAR", "MACCHINA", 0),
      ("GEAR_SHIFTER", "MACCHINA", 0),
      ("STEER_STATUS", "MACCHINA", 0),  # etait a 5 : "fault_1"
      ("CRUISE_BUTTONS", "MACCHINA", 0),           #???
      ("CRUISE_SETTING", "MACCHINA", 0),   #???
      ("CRUISE_SPEED_OFFSET", "MACCHINA", 0),
      ("CRUISE_SPEED_PCM", "MACCHINA", 0),
  ]

  checks = [
      ("POWERTRAIN_DATA", 100),
      ("WHEEL_SPEEDS", 100),
      ("SPEEDS", 100),
      ("STEERING_SENSORS", 100),
      ("STEERING_CONTROL", 100),
      ("MACCHINA", 100),
  ]

  # add gas interceptor reading if we are using it
  # add gas interceptor reading if we are using it
  if CP.enableGasInterceptor: # ne se declenche que si l'id du gas interceptor est dedans
    signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
    checks.append(("GAS_SENSOR", 100)) #100 car a 10ms indexe sur 0x208

  return signals, checks


def get_can_parser(CP):
  signals, checks = get_can_signals(CP)
  bus_pt = 1 if CP.isPandaBlack and CP.carFingerprint in HONDA_BOSCH else 0
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, bus_pt)


def get_cam_can_parser(CP):
  signals = []

  # all hondas except CRV, RDX and 2019 Odyssey@China use 0xe4 for steering
  checks = [(0xe4, 100)]
  bus_cam = 1
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, bus_cam)

class CarState():
  def __init__(self, CP):
    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = self.can_define.dv["MACCHINA"]["GEAR_SHIFTER"]
    
    self.user_gas, self.user_gas_pressed = 0., 0
    self.brake_switch_prev = 0
    self.brake_switch_ts = 0

    self.cruise_buttons = 0
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0
    self.blinker_on = 0

    self.left_blinker_on = 0
    self.right_blinker_on = 0

    self.cruise_mode = 0
    self.stopped = 0

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

  def update(self, cp, cp_cam):

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_setting = self.cruise_setting
    self.prev_blinker_on = self.blinker_on

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # ******************* parse out can *******************
    # 2 = temporary; 3 = TBD; 4 = temporary, hit a bump; 5 = (permanent); 6 = temporary; 7 = (permanent)
    # TODO: Use values from DBC to parse this field
    self.steer_error = False #cp.vl["MACCHINA"]['STEER_STATUS'] not in [0, 2, 3, 4, 6]
    self.steer_not_allowed = False #cp.vl["MACCHINA"]['STEER_STATUS']  not in [0, 4]  # 4 can be caused by bump OR steering nudge from driver
    self.steer_warning = False #cp.vl["MACCHINA"]['STEER_STATUS'] not in [0, 3, 4] # 3 is low speed lockout, not worth a warning
    if self.CP.radarOffCan:
      self.brake_error = 0
    else:
      self.brake_error = 0
    self.esp_disabled = 0

    # calc best v_ego estimate, by averaging two opposite corners
    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel = (self.v_wheel_fl+self.v_wheel_fr+self.v_wheel_rl+self.v_wheel_rr)/4.

    if self.CP.carFingerprint in (CAR.ACCORD, CAR.ESSAI): # TODO: find wheels moving bit in dbc
      #self.standstill = cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] < 0.1
      self.standstill = not self.v_wheel > 0.001
      #self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DRIVERS_DOOR_OPEN']
      self.door_all_closed = 1
    else:
      #self.standstill = not cp.vl["STANDSTILL"]['WHEELS_MOVING']
      self.standstill = not self.v_wheel > 0.001
      #self.door_all_closed = not any([cp.vl["DOORS_STATUS"]['DOOR_OPEN_FL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_FR'],
      #                                cp.vl["DOORS_STATUS"]['DOOR_OPEN_RL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_RR']])
      self.door_all_closed = 1
    #self.seatbelt = not cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LAMP'] and cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LATCHED']
    self.seatbelt = 1

    # blend in transmission speed at low speed, since it has more low speed accuracy
    self.v_weight = interp(self.v_wheel, v_weight_bp, v_weight_v)
    #speed = (1. - self.v_weight) * cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] * CV.KPH_TO_MS * speed_factor + \
    speed = (1. - self.v_weight) * cp.vl["SPEEDS"]['SPEED1'] * CV.KPH_TO_MS * speed_factor + \
      self.v_weight * self.v_wheel

    if abs(speed - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_x = [[speed], [0.0]]

    self.v_ego_raw = speed
    v_ego_x = self.v_ego_kf.update(speed)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    # this is a hack for the interceptor. This is now only used in the simulation
    # TODO: Replace tests by toyota so this can go away
    if self.CP.enableGasInterceptor:
      self.user_gas = cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']
      self.user_gas_pressed = self.user_gas > 0 # this works because interceptor read < 0 when pedal position is 0. Once calibrated, this will change

    self.gear = 0 if self.CP.carFingerprint == CAR.CIVIC else cp.vl["MACCHINA"]['GEAR']
    #self.gear = 4 # cf DBC Honda
    self.angle_steers = (cp.vl["STEERING_SENSORS"]['STEER_ANGLE']) * (2*cp.vl["STEERING_SENSORS"]['STEER_ANGLE_SIGN']-1) * CV.RAD_TO_DEG
    self.angle_steers_rate = (cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']) * (2*cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE_SIGN']-1) * CV.RAD_TO_DEG

    self.cruise_setting = cp.vl["MACCHINA"]['CRUISE_SETTING']
    self.cruise_buttons = cp.vl["MACCHINA"]['CRUISE_BUTTONS']

    self.blinker_on = 0
    self.left_blinker_on = 0
    self.right_blinker_on = 0
    self.brake_hold = 0

    if self.CP.carFingerprint in (CAR.CIVIC,CAR.ESSAI):
      self.park_brake = 0
      self.main_on = cp.vl["MACCHINA"]['MAIN_ON']
    else:
      self.park_brake = 0
      self.main_on = cp.vl["MACCHINA"]['MAIN_ON']

    can_gear_shifter = int(cp.vl["MACCHINA"]['GEAR_SHIFTER'])
    self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)

    self.pedal_gas = cp.vl["POWERTRAIN_DATA"]['PEDAL_GAS']
    self.car_gas = self.pedal_gas

    self.steer_torque_driver = cp.vl["MACCHINA"]['STEER_TORQUE_DRIVER']
    self.steer_torque_motor = cp.vl["MACCHINA"]['STEER_TORQUE_DRIVER']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.CP.carFingerprint]

    # ATTENTION a changer:
    self.brake_switch = 0 # voir si pas de pb, pas de brake switch sur Toyota

    if self.CP.radarOffCan:
      self.stopped = 0
      self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
      if self.CP.carFingerprint in (CAR.ACCORD, CAR.CIVIC):
        self.brake_switch = 0
        self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED']
        self.brake_switch_prev = self.brake_switch
        self.brake_switch_ts = 0
      else:
        self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED']
      # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
      self.v_cruise_pcm = self.v_cruise_pcm_prev if cp.vl["ACC_HUD"]['CRUISE_SPEED'] > 160.0 else cp.vl["ACC_HUD"]['CRUISE_SPEED']
      self.v_cruise_pcm_prev = self.v_cruise_pcm
    else:
      self.brake_switch = 0
      #self.cruise_speed_offset = calc_cruise_offset(cp.vl["CRUISE_PARAMS"]['CRUISE_SPEED_OFFSET'], self.v_ego)
      self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
      #self.v_cruise_pcm = cp.vl["CRUISE"]['CRUISE_SPEED_PCM']
      self.v_cruise_pcm = 0
      # brake switch has shown some single time step noise, so only considered when
      # switch is on for at least 2 consecutive CAN samples
      self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED']
      self.brake_switch_prev = 0
      self.brake_switch_ts = 0

    self.user_brake = 0
    self.pcm_acc_status = cp.vl["MACCHINA"]['ACC_STATUS']
    self.hud_lead = 1


    # TODO: discover the CAN msg that has the imperial unit bit for all other cars
    self.is_metric = 1

    self.stock_aeb = False
    
    self.stock_hud = False
    self.stock_fcw = False
    
