from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volvo.values import CAR, PLATFORM, DBC, BUTTON_STATES 
from selfdrive.kegman_conf import kegman_conf

class diagInfo():
  def __init__(self):
    self.diagFSMResp = 0
    self.diagCEMResp = 0
    self.diagPSCMResp = 0
    self.diagCVMResp = 0

class PSCMInfo():
  def __init__(self):
    # Common
    self.byte0 = 0
    self.byte4 = 0
    self.byte7 = 0
    self.LKAActive = 0
    self.LKATorque = 0
    self.SteeringAngleServo = 0

    # C1
    self.byte3 = 0

    # EUCD
    self.SteeringWheelRateOfChange = 0

class FSMInfo():
  def __init__(self):
    # C1
    self.SET_X_E3 = 0
    self.SET_X_B4 = 0
    self.SET_X_08 = 0
    self.Unkown = 0
    self.LKAAngleReq = 0
    self.SET_X_02 = 0
    self.Checksum = 0
    self.LKASteerDirection = 0
    self.SET_X_25 = 0

    # EUCD
    self.SET_X_22 = 0
    self.SET_X_02 = 0
    self.SET_X_10 = 0
    self.SET_X_A4 = 0


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # Live tuning
    self.kegman = kegman_conf()
    self.diag = diagInfo() # diagInfo
    self.PSCMInfo = PSCMInfo() # PSCMInfo try remove fault code in FSM
    self.FSMInfo = FSMInfo()

    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.buttonStates = BUTTON_STATES.copy()
     
  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    
    # Speeds
    ret.vEgoRaw = cp.vl["VehicleSpeed1"]['VehicleSpeed'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1
    
     # Steering
    ret.steeringAngle = cp.vl["PSCM1"]['SteeringAngleServo']
    ret.steeringTorque = 0 # Needed? No signal to check against yet
    ret.steeringPressed = bool(cp.vl["CCButtons"]['ACCSetBtn'] or \
      cp.vl["CCButtons"]['ACCMinusBtn'] or \
      cp.vl["CCButtons"]['ACCResumeBtn']) 
    
    # Update gas and brake
    if self.CP.carFingerprint in PLATFORM.C1:
      ret.gas = cp.vl["PedalandBrake"]['AccPedal'] / 102.3
      ret.gasPressed = ret.gas > 0.05
    elif self.CP.carFingerprint in PLATFORM.EUCD:
      ret.gas = cp.vl["AccPedal"]['AccPedal'] / 102.3
      ret.gasPressed = ret.gas > 0.1
    ret.brakePressed = False

    # Update gear position
    if self.CP.carFingerprint in PLATFORM.C1:
      self.shifter_values = self.can_define.dv["TCM0"]['GearShifter']
      can_gear = int(cp.vl["TCM0"]["GearShifter"])
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    elif self.CP.carFingerprint in PLATFORM.EUCD:
      ret.gearShifter = self.parse_gear_shifter('D')

    # Belt and doors
    ret.doorOpen = False

    # Check seatbelts
    ret.seatbeltUnlatched = False # No signal yet.

    # ACC status from camera
    if self.CP.carFingerprint in PLATFORM.C1:
      ret.cruiseState.available = bool(cp_cam.vl["FSM0"]['ACCStatusOnOff'])
      ret.cruiseState.enabled = bool(cp_cam.vl["FSM0"]['ACCStatusActive'])
      ret.cruiseState.speed = cp.vl["ACC"]['SpeedTargetACC'] * CV.KPH_TO_MS
    
    elif self.CP.carFingerprint in PLATFORM.EUCD:
      accStatus = cp_cam.vl["FSM0"]['ACCStatus']  # TODO
      
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
    self.diag.diagCVMResp = int(cp.vl["diagCVMResp"]["byte03"])
    self.diag.diagPSCMResp = int(cp.vl["diagPSCMResp"]["byte03"])

  
    # PSCMInfo
    # Common
    self.PSCMInfo.byte0 = int(cp.vl['PSCM1']['byte0']) 
    self.PSCMInfo.byte4 = int(cp.vl['PSCM1']['byte4']) 
    self.PSCMInfo.byte7 = int(cp.vl['PSCM1']['byte7']) 
    self.PSCMInfo.LKATorque = int(cp.vl['PSCM1']['LKATorque']) 
    self.PSCMInfo.LKAActive = int(cp.vl['PSCM1']['LKAActive']) 
    self.PSCMInfo.SteeringAngleServo = int(cp.vl['PSCM1']['SteeringAngleServo']) 

    # Specifics  
    if self.CP.carFingerprint in PLATFORM.C1:
      self.PSCMInfo.byte3 = int(cp.vl['PSCM1']['byte3']) 
    elif self.CP.carFingerprint in PLATFORM.EUCD:
      self.PSCMInfo.SteeringWheelRateOfChange = int(cp.vl['PSCM1']['SteeringWheelRateOfChange'])

    # FSMInfo
    if self.CP.carFingerprint in PLATFORM.C1:
      self.FSMInfo.SET_X_E3 = int(cp_cam.vl['FSM1']['SET_X_E3']) 
      self.FSMInfo.SET_X_B4 = int(cp_cam.vl['FSM1']['SET_X_B4']) 
      self.FSMInfo.SET_X_08 = int(cp_cam.vl['FSM1']['SET_X_08']) 
      self.FSMInfo.SET_X_02 = int(cp_cam.vl['FSM1']['SET_X_02']) 
      self.FSMInfo.SET_X_25 = int(cp_cam.vl['FSM1']['SET_X_25']) 
      self.FSMInfo.Unkown = int(cp_cam.vl['FSM1']['Unkown']) 
      self.FSMInfo.LKAAngleReq = int(cp_cam.vl['FSM1']['LKAAngleReq']) 
      self.FSMInfo.Checksum = int(cp_cam.vl['FSM1']['Checksum']) 
      self.FSMInfo.LKASteerDirection = int(cp_cam.vl['FSM1']['LKASteerDirection'])
    
    elif self.CP.carFingerprint in PLATFORM.EUCD:
      self.FSMInfo.SET_X_22 = int(cp_cam.vl['FSM2']['SET_X_22']) 
      self.FSMInfo.SET_X_02 = int(cp_cam.vl['FSM2']['SET_X_02']) 
      self.FSMInfo.SET_X_A4 = int(cp_cam.vl['FSM2']['SET_X_A4']) 
      self.FSMInfo.SET_X_10 = int(cp_cam.vl['FSM2']['SET_X_10']) 
      
    return ret

  @staticmethod
  def get_can_parser(CP):
    # ptcan on bus 0
    # # this function generates lists for signal, messages and initial values
    
    # Common signals for all cars
    signals = [
      # sig_name, sig_address, default
      ("VehicleSpeed", "VehicleSpeed1", 0),
      ("TurnSignal", "MiscCarInfo", 0),
      ("ACCOnOffBtn", "CCButtons", 0),
      ("ACCResumeBtn", "CCButtons", 0),
      ("ACCSetBtn", "CCButtons", 0),
      ("ACCMinusBtn", "CCButtons", 0),
      ("ACCStopBtn", "CCButtons", 0),
      ("TimeGapIncreaseBtn", "CCButtons", 0),
      ("TimeGapDecreaseBtn", "CCButtons", 0),
      
      # Common PSCM signals
      ("SteeringAngleServo", "PSCM1", 0),
      ("LKATorque", "PSCM1", 0),
      ("LKAActive", "PSCM1", 0),
      ("byte0", "PSCM1", 0),
      ("byte4", "PSCM1", 0),
      ("byte7", "PSCM1", 0),
      
      # diagnostic
      ("byte03", "diagCEMResp", 0),
      ("byte47", "diagCEMResp", 0),
      ("byte03", "diagPSCMResp", 0),
      ("byte47", "diagPSCMResp", 0),
      ("byte03", "diagCVMResp", 0),
      ("byte47", "diagCVMResp", 0),
    ]
   
    checks = [
      # sig_address, frequency
      ("CCButtons", 100),
      ("PSCM1", 50),   
      ("VehicleSpeed1", 50),
      ("MiscCarInfo", 25),
    ]

    # Car specific signals
    if CP.carFingerprint in PLATFORM.C1:
      signals.append(("SpeedTargetACC", "ACC", 0))
      signals.append(("BrakePedalActive2", "PedalandBrake", 0))
      signals.append(("AccPedal", "PedalandBrake", 0))
      signals.append(("BrakePress0", "BrakeMessages", 0))
      signals.append(("BrakePress1", "BrakeMessages", 0))
      signals.append(("BrakeStatus", "BrakeMessages", 0))
      signals.append(("GearShifter", "TCM0", 0))
      
      # Car specific Servo1 
      signals.append(("byte3", "PSCM1", 0))
     
      checks.append(("BrakeMessages", 50))
      checks.append(("ACC", 17))
      #checks.append(("PedalandBrake", ) # TODO
    
    if CP.carFingerprint in PLATFORM.EUCD:
      signals.append(("AccPedal", "AccPedal", 0))
      signals.append(("BrakePedal", "BrakePedal", 0))

      signals.append(("SteeringWheelRateOfChange", "PSCM1", 0))

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
    ]
    
    # Car specific
    if CP.carFingerprint in PLATFORM.C1:
      signals.append(("SET_X_E3", "FSM1", 0xE3))
      signals.append(("SET_X_B4", "FSM1", 0xB4))
      signals.append(("SET_X_08", "FSM1", 0x08))
      signals.append(("SET_X_02", "FSM1", 0x02))
      signals.append(("SET_X_25", "FSM1", 0x25))
      # Test to get rid of faultcodes in FSM & PSCM
      signals.append(("Unkown", "FSM1", 0x80))
      signals.append(("LKAAngleReq", "FSM1", 0x2000))
      signals.append(("Checksum", "FSM1", 0x5f))
      signals.append(("LKASteerDirection", "FSM1", 0x00))

      signals.append(("ACCStatusOnOff", "FSM0", 0x00))
      signals.append(("ACCStatusActive", "FSM0", 0x00))

      checks.append(('FSM0', 100))
      checks.append(('FSM1', 50))
    
    # TODO add checks and signals nescessary
    elif CP.carFingerprint in PLATFORM.EUCD:
      signals.append(("ACCStatus", "FSM0", 0))

      # Test to forward heartbeat. TODO:
      signals.append(("SET_X_22", "FSM2", 0x00))
      signals.append(("SET_X_02", "FSM2", 0x00))
      signals.append(("SET_X_10", "FSM2", 0x00))
      signals.append(("SET_X_A4", "FSM2", 0x00))

      checks.append(('FSM0', 100))
      checks.append(('FSM1', 50))

    
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
