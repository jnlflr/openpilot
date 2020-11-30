from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, HONDA_BOSCH


#def get_pt_bus(car_fingerprint, has_relay):
#  return 1 if car_fingerprint in HONDA_BOSCH and has_relay else 0


#def get_lkas_cmd_bus(car_fingerprint, has_relay):
#  return 2 if car_fingerprint in HONDA_BOSCH and not has_relay else 0


def create_brake_command(packer, apply_brake, idx, car_fingerprint):
  values = {
    "COMPUTER_BRAKE": apply_brake,
  }
  bus = 0
  return packer.make_can_msg("GAS_COMMAND", bus, values, idx)

def create_left_lane(packer, idx, car_fingerprint,l_poly):
  values = {
    "C0_L": l_poly[0],
    "C1_L": l_poly[1],
    "C2_L": l_poly[2],
    "C3_L": l_poly[3],
  }
  bus = 0
  return packer.make_can_msg("LEFT_LANE", bus, values, idx)

def create_right_lane(packer, idx, car_fingerprint,r_poly):
  values = {
    "C0_R": r_poly[0],
    "C1_R": r_poly[1],
    "C2_R": r_poly[2],
    "C3_R": r_poly[3],
  }
  bus = 0
  return packer.make_can_msg("RIGHT_LANE", bus, values, idx)

def create_path_lane(packer, idx, car_fingerprint,p_poly):
  values = {
    "C0_P": p_poly[0],
    "C1_P": p_poly[1],
    "C2_P": p_poly[2],
    "C3_P": p_poly[3],
  }
  bus = 0
  return packer.make_can_msg("PATH_LANE", bus, values, idx)

def create_d_lane(packer, idx, car_fingerprint,d_poly):
  values = {
    "C0_D": d_poly[0],
    "C1_D": d_poly[1],
    "C2_D": d_poly[2],
    "C3_D": d_poly[3],
  }
  bus = 0
  return packer.make_can_msg("D_LANE", bus, values, idx)

def create_lane_prob(packer, idx, car_fingerprint,l_prob,r_prob,lane_width):
  values = {
    "L_PROB": l_prob,
    "R_PROB": r_prob,
    "LANE_WIDTH": lane_width,
  }
  bus = 0
  return packer.make_can_msg("LANE_PROB", bus, values, idx)

def create_params(packer, idx, car_fingerprint,angleOffset,angleOffsetAverage,stiffness,sR,curvature):
  values = {
    "AO_OFFSET": angleOffset,
    "AO_OFFSET_AV": angleOffsetAverage,
    "STIFFNESS": stiffness,
    "STEER_RATIO": sR,
    "CURV_FACTOR": curvature,
  }
  bus = 0
  return packer.make_can_msg("PARAMS", bus, values, idx)

def create_mpc(packer, idx, car_fingerprint,l_prob,r_prob,lane_width):
  values = {
    "L_PROB": l_prob,
    "R_PROB": r_prob,
    "LANE_WIDTH": lane_width,
  }
  bus = 0
  return packer.make_can_msg("MPC", bus, values, idx)


def create_steering_control(packer, apply_steer, angle_des, lkas_active, enabled, car_fingerprint, idx):
  values = {
    "STEER_TORQUE": apply_steer,
    "LKAS_ACTIVE": lkas_active,
	  "LK_MODE": False,
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
