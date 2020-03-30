from selfdrive.car.volvo.values import CAR

def create_chksum(dat, car_fingerprint):
  # Input: dat byte array
  # Steering direction = 0 -> 3
  # Unkown = 128
  # Steering angle request = -360 -> 360
    
  # Extract LKAAngleRequest, LKADirection and Unknown
  if car_fingerprint == CAR.V40: 
    steer_angle_request = ((dat[4] & 0x3F) << 8) + dat[5]
    steering_direction_request = dat[7] & 0x03  
    unkown = dat[3]
  elif car_fingerprint == CAR.V60:
    steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
    steering_direction_request = dat[5] & 0x03  
    unkown = dat[2]
  
  # Sum of all bytes, carry ignored.
  s = (unkown + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF

def create_steering_control(packer, car_fingerprint, apply_steer, frame, steer_on, steer_direction, unkown):  
 
  # Set common parameters
  values = {
    "LKAAngleRequest": apply_steer,
    "LKADirection": steer_direction,
    "Unkown": unkown,
  }
  
  # Set car specific parameters
  if car_fingerprint == CAR.V40:
    values_static = {
      "SET_X_E3": 0xE3,
      "SET_X_B4": 0xB4,
      "SET_X_08": 0x08,
      "SET_X_02": 0x02,
      "SET_X_25": 0x25,
    }
  elif car_fingerprint == CAR.V60:
    values_static = {
      "SET_X_22": 0x22,
      "SET_X_02": 0x02,
      "SET_X_10": 0x10,
      "SET_X_A4": 0xA4,
    }
  else:
    print("ERROR: Car model not supported.")
    return [] 

  # Combine common and static parameters
  values.update(values_static)

  # Create can message with "translated" can bytes.
  dat = packer.make_can_msg("fromFSMSteeringRequest", 0, values)[2]
  values["Checksum"] = create_chksum(dat, car_fingerprint)
    
  return packer.make_can_msg("fromFSMSteeringRequest", 0, values)

