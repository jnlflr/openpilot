from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, HONDA_BOSCH


#def get_pt_bus(car_fingerprint, has_relay):
#  return 1 if car_fingerprint in HONDA_BOSCH and has_relay else 0


#def get_lkas_cmd_bus(car_fingerprint, has_relay):
#  return 2 if car_fingerprint in HONDA_BOSCH and not has_relay else 0


def create_brake_command(packer, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, fcw, idx, car_fingerprint, has_relay, stock_brake):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_1": 1,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": stock_brake["CHIME"],  # chime issued when disabling FCM
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw?
    "AEB_REQ_1": 0,
    "AEB_REQ_2": 0,
    "AEB_STATUS": 0,
  }
  bus = 0
  return packer.make_can_msg("BRAKE_COMMAND", bus, values, idx)


def create_steering_control(packer, apply_steer, angle_des, lkas_active, enabled, lkMode, car_fingerprint, idx):
  values = {
    "STEER_TORQUE": apply_steer,
    "LKAS_ACTIVE": lkas_active,
	  "LK_MODE": lkMode,
	  "ST_NOT_ALL": 0x01,
	  "ANGLE_DES": angle_des,
  }
  # Set bus 2 for accord and new crv.
  bus = 0
  return packer.make_can_msg("STEERING_CONTROL", bus, values, idx)

"""
def create_ui_commands(packer, pcm_speed, hud, car_fingerprint, is_metric, idx, has_relay, stock_hud):
  commands = []
  bus_pt = get_pt_bus(car_fingerprint, has_relay)
  bus_lkas = get_lkas_cmd_bus(car_fingerprint, has_relay)
  return commands


def spam_buttons_command(packer, button_val, idx, car_fingerprint, has_relay):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  bus = get_pt_bus(car_fingerprint, has_relay)
  return packer.make_can_msg("SCM_BUTTONS", bus, values, idx)"""
