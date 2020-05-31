from selfdrive.car.volvo.values import PLATFORM

def cancelACC(packer, car_fingerprint, CS):
  # Send cancel button to disengage ACC
  # TODO add support for EUCD
  msg = {}

  if car_fingerprint in PLATFORM.C1:
    msg["ACCStopBtn"] = 1
  
  elif car_fingerprint in PLATFORM.EUCD:
    msg["ACCOnOffBtn"] = 1
    msg["ACCOnOffBtnInv"] = 0

  return packer.make_can_msg("CCButtons", 0, msg)


def manipulateServo(packer, car_fingerprint, CS):
  # Manipulate data from servo to FSM
  # Zero active and torque bits.
  msg = {
      "LKATorque" : 0,
      "SteeringAngleServo" : CS.PSCMInfo.SteeringAngleServo,
      "byte0" : CS.PSCMInfo.byte0,
      "byte4" : CS.PSCMInfo.byte4,
      "byte7" : CS.PSCMInfo.byte7,
  }

  if car_fingerprint in PLATFORM.C1: 
    msg["LKAActive"] = CS.PSCMInfo.LKAActive & 0xFD
    msg["byte3"] = CS.PSCMInfo.byte3
  elif car_fingerprint in PLATFORM.EUCD:
    msg["LKAActive"] = CS.PSCMInfo.LKAActive
    msg["SteeringWheelRateOfChange"] = CS.PSCMInfo.SteeringWheelRateOfChange

  return packer.make_can_msg("PSCM1", 2, msg)


def create_chksum(dat, car_fingerprint):
  # Input: dat byte array, and fingerprint
  # Steering direction = 0 -> 3
  # TrqLim = 0 -> 255
  # Steering angle request = -360 -> 360
    
  # Extract LKAAngleRequest, LKADirection and Unknown
  if car_fingerprint in PLATFORM.C1: 
    steer_angle_request = ((dat[4] & 0x3F) << 8) + dat[5]
    steering_direction_request = dat[7] & 0x03  
    trqlim = dat[3]
  elif car_fingerprint in PLATFORM.EUCD:
    steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
    steering_direction_request = dat[5] & 0x03  
    trqlim = dat[2]
  
  # Sum of all bytes, carry ignored.
  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF


def create_steering_control(packer, frame, car_fingerprint, SteerCommand, FSMInfo):  
 
  # Set common parameters
  # TODO: DEBUG, REMOVE WHEN DONE
  if car_fingerprint in PLATFORM.C1 or True:
    values = {
      "LKAAngleReq": SteerCommand.angle_request,
      "LKASteerDirection": SteerCommand.steer_direction,
      "TrqLim": SteerCommand.trqlim,
    }
  elif car_fingerprint in PLATFORM.EUCD and False:
    values = {
      "LKAAngleReq": FSMInfo.LKAAngleReq,
      "LKASteerDirection": FSMInfo.LKASteerDirection,
      "TrqLim": FSMInfo.TrqLim,
      "Checksum": FSMInfo.Checksum,      
    }
  
  # Set car specific parameters
  if car_fingerprint in PLATFORM.C1:
    values_static = {
      "SET_X_E3": 0xE3,
      "SET_X_B4": 0xB4,
      "SET_X_08": 0x08,
      "SET_X_02": 0x02,
      "SET_X_25": 0x25,
    }
  elif car_fingerprint in PLATFORM.EUCD:
    values_static = {
      "SET_X_22": 0x25,
      "SET_X_02": 0,
      "SET_X_10": 0x10,
      "SET_X_A4": 0xa7,
      #"SET_X_22": FSMInfo.SET_X_22,
      #"SET_X_02": FSMInfo.SET_X_02,
      #"SET_X_10": FSMInfo.SET_X_10,
      #"SET_X_A4": FSMInfo.SET_X_A4,
    }

  # Combine common and static parameters
  values.update(values_static)

  # Create can message with "translated" can bytes.
  if car_fingerprint in PLATFORM.C1:
    dat = packer.make_can_msg("FSM1", 0, values)[2]
  elif car_fingerprint in PLATFORM.EUCD:
    dat = packer.make_can_msg("FSM2", 0, values)[2]

  values["Checksum"] = create_chksum(dat, car_fingerprint)

  if car_fingerprint in PLATFORM.C1:
    return packer.make_can_msg("FSM1", 0, values)
  elif car_fingerprint in PLATFORM.EUCD:
    return packer.make_can_msg("FSM2", 0, values)

