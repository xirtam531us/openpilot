from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volvo.values import CAR, DBC, BUTTON_STATES
from selfdrive.kegman_conf import kegman_conf

class diagInfo():
  def __init__(self):
    self.diagFSMResp = 0
    self.diagCEMResp = 0
    self.diagPSCMResp = 0

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # Live tuning
    self.kegman = kegman_conf()
    self.diag = diagInfo() # diagInfo

    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.buttonStates = BUTTON_STATES.copy()

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    # Speeds
    ret.vEgoRaw = cp.vl["VehicleSpeed1"]['VehicleSpeed'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1
    
     # Steering
    ret.steeringAngle = cp.vl["fromServo1"]['SteeringAngleServo']
    ret.steeringTorque = 0 # Needed? No signal to check against
    ret.steeringPressed = bool(cp.vl["CCButtons"]['ACCSetBtn'] or \
      cp.vl["CCButtons"]['ACCMinusBtn'] or \
      cp.vl["CCButtons"]['ACCResumeBtn']) 
    
    # Update gas and brake
    if self.CP.carFingerprint == CAR.V40:
      ret.gas = cp.vl["PedalandBrake"]['AccPedal'] / 102.3
    elif self.CP.carFingerprint == CAR.V60:
      ret.gas = cp.vl["AccPedal"]['AccPedal'] / 102.3
    ret.gasPressed = ret.gas > 0
    ret.brakePressed = False
    #ret.brakePressed = cp.vl["PedalandBrake"]['BrakePedalActive2'] != 0 # old way.
    #ret.brakePressed = (cp.vl["BrakeMessages"]['BrakeStatus'] == 0) and (cp.vl["BrakeMessages"]['BrakePress0'] != 0) # doesnt false detect during acc braking.

    # Update gear position
    ret.gearShifter = self.parse_gear_shifter('D') # TODO

    # Belt and doors
    ret.doorOpen = False

    # Check seatbelts
    ret.seatbeltUnlatched = False # No signal yet.

    # ACC status from camera
    if self.CP.carFingerprint == CAR.V40:
      ret.cruiseState.available = bool(cp_cam.vl["fromFSM0"]['ACCStatusOnOff'])
      ret.cruiseState.enabled = bool(cp_cam.vl["fromFSM0"]['ACCStatusActive'])
      ret.cruiseState.speed = cp.vl["ACC"]['SpeedTargetACC'] * CV.KPH_TO_MS
    
    elif self.CP.carFingerprint == CAR.V60:
      accStatus = cp_cam.vl["fromFSM0"]['ACCStatus']  # TODO
      #accStatus = cp.vl["fromFSM0"]['ACCStatus']  # Only for testing TODO
      if accStatus == 2:
        # Acc in ready mode
        ret.cruiseState.available = True
        ret.cruiseState.enabled = False
      elif accStatus >= 6:
        # Acc active
        ret.cruiseState.available = True
        ret.cruiseState.enabled = True
      else:
        # Acc in a unkown mode
        ret.cruiseState.available = False
        ret.cruiseState.enabled = False

    # Button and blinkers.
    self.buttonStates['altButton1'] = bool(cp.vl["CCButtons"]['ACCOnOffBtn'])
    self.buttonStates['accelCruise'] = bool(cp.vl["CCButtons"]['ACCSetBtn'])
    self.buttonStates['decelCruise'] = bool(cp.vl["CCButtons"]['ACCMinusBtn'])
    self.buttonStates['setCruise'] = bool(cp.vl["CCButtons"]['ACCSetBtn'])
    self.buttonStates['resumeCruise'] = bool(cp.vl["CCButtons"]['ACCResumeBtn'])
    #self.buttonStates['cancel'] = bool(cp.vl["CCButtons"]['ACCStopBtn']) No cancel button in V60.
    self.buttonStates['gapAdjustCruise'] = bool(cp.vl["CCButtons"]['TimeGapIncreaseBtn']) or bool(cp.vl["CCButtons"]['TimeGapDecreaseBtn'])
    ret.leftBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 1
    ret.rightBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 3

    # Diagnostics, for debugging
    self.diag.diagFSMResp = int(cp_cam.vl["diagFSMResp"]["byte03"])
    self.diag.diagCEMResp = int(cp.vl["diagCEMResp"]["byte03"])
    self.diag.diagPSCMResp = int(cp.vl["diagPSCMResp"]["byte03"])
    
    return ret

  @staticmethod
  def get_can_parser(CP):
    # ptcan on bus 0
    # # this function generates lists for signal, messages and initial values
    
    # Common signals for all cars
    signals = [
      # sig_name, sig_address, default
      ("VehicleSpeed", "VehicleSpeed1", 0),
      ("SteeringAngleServo", "fromServo1", 0),
      ("LKAActive", "fromServo1", 0),
      ("TurnSignal", "MiscCarInfo", 0),
      ("ACCOnOffBtn", "CCButtons", 0),
      ("ACCResumeBtn", "CCButtons", 0),
      ("ACCSetBtn", "CCButtons", 0),
      ("ACCMinusBtn", "CCButtons", 0),
      ("ACCStopBtn", "CCButtons", 0),
      ("TimeGapIncreaseBtn", "CCButtons", 0),
      ("TimeGapDecreaseBtn", "CCButtons", 0),
      # diagnostic
      ("byte03", "diagCEMResp", 0),
      ("byte47", "diagCEMResp", 0),
      ("byte03", "diagPSCMResp", 0),
      ("byte47", "diagPSCMResp", 0),
    ]
   
    checks = [
      # sig_address, frequency
      ("CCButtons", 100),
      ("fromServo1", 50),   
      ("VehicleSpeed1", 50),
      ("MiscCarInfo", 25),
    ]

    # Car specific signals
    if CP.carFingerprint == CAR.V40:
      signals.append(("SpeedTargetACC", "ACC", 0))
      signals.append(("BrakePedalActive2", "PedalandBrake", 0))
      signals.append(("AccPedal", "PedalandBrake", 0))
      signals.append(("BrakePress0", "BrakeMessages", 0))
      signals.append(("BrakePress1", "BrakeMessages", 0))
      signals.append(("BrakeStatus", "BrakeMessages", 0))
     
      checks.append(("BrakeMessages", 50))
      checks.append(("ACC", 17))
      #checks.append(("PedalandBrake", ) # TODO
    
    if CP.carFingerprint == CAR.V60:
      signals.append(("AccPedal", "AccPedal", 0))
      signals.append(("BrakePedal", "BrakePedal", 0))
  
      #signals.append(("ACCStatus", "fromFSM0", 0x00)) # Just used in debugging TODO
      
      checks.append(("AccPedal", 100))
      checks.append(("BrakePedal", 50))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_adas_can_parser(CP):
    # radar on bus 1, not decoded yet
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
    ]

    checks = [
      # sig_address, frequency
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)

  @staticmethod
  def get_cam_can_parser(CP):
    # camera on bus 2
    # Common signals 
    signals = [
      # sig_name, sig_address, default
      ("byte03", "diagFSMResp", 0),
      ("byte47", "diagFSMResp", 0),

   ]
    # Common checks
    checks = [
      # sig_address, frequency
      #('fromFSM0', 100),
      #('fromFSMSteeringRequest', 50),
    ]
    
    # Car specific
    if CP.carFingerprint == CAR.V40:
      signals.append(("SET_X_E3", "fromFSMSteeringRequest", 0xE3))
      signals.append(("SET_X_B3", "fromFSMSteeringRequest", 0xB3))
      signals.append(("SET_X_08", "fromFSMSteeringRequest", 0x08))
      signals.append(("SET_X_02", "fromFSMSteeringRequest", 0x02))
      signals.append(("SET_X_25", "fromFSMSteeringRequest", 0x25))
      signals.append(("ACCStatusOnOff", "fromFSM0", 0x00))
      signals.append(("ACCStatusActive", "fromFSM0", 0x00))

      # TODO only for testing when i havent got a good v60 recording. Move later
      checks.append(('fromFSM0', 100))
      checks.append(('fromFSMSteeringRequest', 50))
      #checks.append()
    
    # TODO add checks and signals nescessary
    # TODO Comment out during testing
    elif CP.carFingerprint == CAR.V60:
      signals.append(("ACCStatus", "fromFSM0", 0))

      checks.append(('fromFSM0', 100))

    
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
