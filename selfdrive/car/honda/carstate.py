from common.numpy_fast import interp
from common.kalman.simple_kalman import KF1D
from selfdrive.can.parser import CANParser, CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR, HONDA_BOSCH

# valeurs forcees :
# standstill depuis toyota
# door_all_closed
# seatbelt

def parse_gear_shifter(gear, vals):

  val_to_capnp = {'P': 'park', 'R': 'reverse', 'N': 'neutral',
                  'D': 'drive', 'S': 'sport', 'L': 'low'}
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
      #("XMISSION_SPEED", "ENGINE_DATA", 0),       ???
      ("SPEED_ABS", "ABS", 0),
      #("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      #("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FT", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("STEER_ANGLE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_RATE", "STEERING_SENSORS", 0),
      #("STEER_TORQUE_SENSOR", "STEER_STATUS", 0),
      ("STEER_TORQUE_SENSOR", "EPS", 0),
      #("LEFT_BLINKER", "SCM_FEEDBACK", 0),
      #("RIGHT_BLINKER", "SCM_FEEDBACK", 0),
      #("GEAR", "GEARBOX", 0),                           ???
      #("SEATBELT_DRIVER_LAMP", "SEATBELT_STATUS", 1),
      #("SEATBELT_DRIVER_LATCHED", "SEATBELT_STATUS", 0),
      ("BRAKE_PRESSED", "POWERTRAIN_DATA", 0),
      #("CRUISE_BUTTONS", "SCM_BUTTONS", 0),           ???
      ("CRUISE_BUTTONS", "MACCHINA", 0),           #???
      #("HUD_LEAD", "ACC_HUD", 0),
      #("USER_BRAKE", "VSA_STATUS", 0),
      #("STEER_STATUS", "STEER_STATUS", 5),
      ("STEER_STATUS", "MACCHINA", 0),  # etait a 5 : "fault_1"
      #("GEAR_SHIFTER", "GEARBOX", 0),         ???
      ("PEDAL_GAS", "POWERTRAIN_DATA", 0),
      #("CRUISE_SETTING", "SCM_BUTTONS", 0),   ???
      ("CRUISE_SETTING", "MACCHINA", 0),   #???
      #("ACC_STATUS", "POWERTRAIN_DATA", 0),   ???
      ("ACC_STATUS", "MACCHINA", 0),   #???
      ("MAIN_ON", "MACCHINA", 0),
      ("STEER_TORQUE_DRIVER", "MACCHINA", 0),

  ]

  checks = [
      ("POWERTRAIN_DATA", 100),
      ("WHEEL_SPEEDS", 25),
      ("STEERING_SENSORS", 100),
      ("EPS", 100),
      #("SCM_FEEDBACK", 10),
      #("GEARBOX", 100),
      #("SEATBELT_STATUS", 10),
      ("ABS", 25),
      ("MACCHINA", 100),
      #("VSA_STATUS", 50),
      #("SCM_BUTTONS", 25),
  ]

  if CP.radarOffCan:
    # Civic is only bosch to use the same brake message as other hondas.
    """if CP.carFingerprint not in (CAR.ACCORDH, CAR.CIVIC_BOSCH):
      signals += [("BRAKE_PRESSED", "BRAKE_MODULE", 0)]
      checks += [("BRAKE_MODULE", 50)]"""
    # Ignore car apparemment le GAS_PEDAL_2 ne sert a rien dans le code
    """signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                #("MAIN_ON", "SCM_FEEDBACK", 0),
                ("MAIN_ON", "MACCHINA", 0),
                #("EPB_STATE", "EPB_STATUS", 0),
                #("CRUISE_SPEED", "ACC_HUD", 0)]
    checks += [("GAS_PEDAL_2", 100)]"""
  """else:
    # Nidec signals.
    signals += [("BRAKE_ERROR_1", "STANDSTILL", 1),
                ("BRAKE_ERROR_2", "STANDSTILL", 1),
                ("CRUISE_SPEED_PCM", "CRUISE", 0),
                ("CRUISE_SPEED_OFFSET", "CRUISE_PARAMS", 0)]
    checks += [("CRUISE_PARAMS", 50),
               ("STANDSTILL", 50)]"""

  # car specific, voir si utile dans mon cas
  """if CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_BOSCH):
    signals += [("DRIVERS_DOOR_OPEN", "SCM_FEEDBACK", 1)]
  else:
    signals += [("DOOR_OPEN_FL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_FR", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RR", "DOORS_STATUS", 1),
                ("WHEELS_MOVING", "STANDSTILL", 1)]
    checks += [("DOORS_STATUS", 3)]

  if CP.carFingerprint == CAR.CIVIC:
    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0)]
  elif CP.carFingerprint == CAR.ACURA_ILX:
    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_BUTTONS", 0)]
  elif CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX, CAR.PILOT_2019, CAR.RIDGELINE):
    signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
  elif CP.carFingerprint == CAR.ODYSSEY:
    signals += [("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0)]
    checks += [("EPB_STATUS", 50)]
  elif CP.carFingerprint == CAR.PILOT:
    signals += [("MAIN_ON", "SCM_BUTTONS", 0),
                ("CAR_GAS", "GAS_PEDAL_2", 0)]"""

  # add gas interceptor reading if we are using it
  if CP.enableGasInterceptor: # ne se declenche que si l'id du gas interceptor est dedans
    signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
    checks.append(("GAS_SENSOR", 100)) #100 car a 10ms indexe sur 0x208

  return signals, checks


def get_can_parser(CP):
  signals, checks = get_can_signals(CP)
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

def get_cam_can_parser(CP):
  signals = []

  # all hondas except CRV and RDX use 0xe4 for steering
  checks = [(0xe4, 100)]
  #if CP.carFingerprint in [CAR.CRV, CAR.ACURA_RDX]:
  #  checks = [(0x194, 100)]

#  cam_bus = 1 if CP.carFingerprint in HONDA_BOSCH else 2
  cam_bus = 2 # le macchina envoie sur le bus 2 du panda l'id 0xe4

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, cam_bus)

class CarState(object):
  def __init__(self, CP):
    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = 'D'

    self.user_gas, self.user_gas_pressed = 0., 0
    self.brake_switch_prev = 0
    self.brake_switch_ts = 0

    self.cruise_buttons = 0
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0
    self.blinker_on = 0

    self.left_blinker_on = 0
    self.right_blinker_on = 0

    self.stopped = 0

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[[1.0, 0.0]],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

  def update(self, cp, cp_cam):

    # copy can_valid on buses 0 and 2
    self.can_valid = cp.can_valid
    self.cam_can_valid = cp_cam.can_valid

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
    self.steer_error = cp.vl["MACCHINA"]['STEER_STATUS'] not in [0, 2, 3, 4, 6]
    self.steer_not_allowed = cp.vl["MACCHINA"]['STEER_STATUS']  not in [0, 4]  # 4 can be caused by bump OR steering nudge from driver
    self.steer_warning = cp.vl["MACCHINA"]['STEER_STATUS'] not in [0, 3, 4] # 3 is low speed lockout, not worth a warning
    if self.CP.radarOffCan:
      self.brake_error = 0
    else:
      self.brake_error = 0
    self.esp_disabled = 0

    # calc best v_ego estimate, by averaging two opposite corners
    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FT'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FT'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel = (self.v_wheel_fl+self.v_wheel_fr+self.v_wheel_rl+self.v_wheel_rr)/4.

    if self.CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_BOSCH): # TODO: find wheels moving bit in dbc
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
    speed = (1. - self.v_weight) * cp.vl["ABS"]['SPEED_ABS'] * CV.KPH_TO_MS * speed_factor + \
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

    #self.gear = 0 if self.CP.carFingerprint == CAR.CIVIC else cp.vl["GEARBOX"]['GEAR']
    self.gear = 4 # cf DBC Honda
    self.angle_steers = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']

    self.cruise_setting = cp.vl["MACCHINA"]['CRUISE_SETTING']
    self.cruise_buttons = cp.vl["MACCHINA"]['CRUISE_BUTTONS']

    self.blinker_on = 0
    self.left_blinker_on = 0
    self.right_blinker_on = 0
    self.brake_hold = 0

    if self.CP.carFingerprint in (CAR.CIVIC, CAR.ODYSSEY, CAR.CRV_5G, CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_BOSCH):
      self.park_brake = 0
      self.main_on = cp.vl["MACCHINA"]['MAIN_ON']
    else:
      self.park_brake = 0
      self.main_on = cp.vl["MACCHINA"]['MAIN_ON']

    #can_gear_shifter = int(cp.vl["GEARBOX"]['GEAR_SHIFTER'])
    can_gear_shifter = int('D')
    self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)

    self.pedal_gas = cp.vl["POWERTRAIN_DATA"]['PEDAL_GAS']
    self.car_gas = self.pedal_gas

    self.steer_torque_driver = cp.vl["MACCHINA"]['STEER_TORQUE_DRIVER']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.CP.carFingerprint]

    # ATTENTION a changer:
    self.brake_switch = 0 # voir si pas de pb, pas de brake switch sur Toyota

    if self.CP.radarOffCan:
      self.stopped = 0
      self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
      if self.CP.carFingerprint in (CAR.CIVIC_BOSCH, CAR.ACCORDH):
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
      self.cruise_speed_offset = calc_cruise_offset(cp.vl["MACCHINA"]['CRUISE_SPEED_OFFSET'], self.v_ego)
      #self.v_cruise_pcm = cp.vl["CRUISE"]['CRUISE_SPEED_PCM']
      self.v_cruise_pcm = cp.vl["MACCHINA"]['CRUISE_SPEED_PCM']
      # brake switch has shown some single time step noise, so only considered when
      # switch is on for at least 2 consecutive CAN samples
      self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED']
      self.brake_switch_prev = 0
      self.brake_switch_ts = 0

    self.user_brake = 0
    self.pcm_acc_status = cp.vl["MACCHINA"]['ACC_STATUS']
    self.hud_lead = 1

    # Gets rid of Pedal Grinding noise when brake is pressed at slow speeds for some models
    # TODO: this should be ok for all cars. Verify it.
    #if self.CP.carFingerprint in (CAR.PILOT, CAR.PILOT_2019, CAR.RIDGELINE):
    #  if self.user_brake > 0.05:
    #    self.brake_pressed = 1

# carstate standalone tester
if __name__ == '__main__':
  import zmq
  context = zmq.Context()

  class CarParams(object):
    def __init__(self):
      self.carFingerprint = "HONDA CIVIC 2016 TOURING"
      self.enableGasInterceptor = 0
  CP = CarParams()
  CS = CarState(CP)

  # while 1:
  #   CS.update()
  #   time.sleep(0.01)
