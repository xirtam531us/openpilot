#!/usr/bin/env python3
from cereal import car
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.car.volvo.values import CAR, BUTTON_STATES
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    
    # Create variables
    self.low_speed_alert = False 
    self.cruiseState_enabled_prev = False
    self.buttonStatesPrev = BUTTON_STATES.copy()
    
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    
    ret.radarOffCan = True  # No radar objects on can
    
    if candidate == CAR.V40 or candidate == CAR.V60:
      # Set specific paramter
      if candidate == CAR.V40:
        ret.safetyParam = 1
        ret.safetyModel = car.CarParams.SafetyModel.volvoC1
      if candidate == CAR.V60:
        ret.safetyParam = 2
        ret.safetyModel = car.CarParams.SafetyModel.volvoEUCD
      # Set common parameters
      ret.carName = "volvo"
      ret.enableCruise = True # Stock ACC
      ret.openpilotLongitudinalControl = False
      ret.steerControlType = car.CarParams.SteerControlType.angle
      ret.steerLimitAlert = False # Steer torque is strongly rate limit and max value is decently high. Off to avoid false positives
      ret.minSteerSpeed = 30. * CV.KPH_TO_MS
      ret.enableCamera = True # force openpilot to fake the stock camera
      
      # Steering settings
      ret.steerRateCost = 0.5
      ret.steerActuatorDelay = 0.25
      ret.steerLimitTimer = 0.4
      ret.steerMaxBP = [0.] # m/s
      ret.steerMaxV = [1.]

      # Speed adjusted lateral tuning breakpoint
      #ret.lateralTuning.pid.kpBP = [0., 15 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS]
      #ret.lateralTuning.pid.kiBP = [0., 15 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS]
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      # Tuning factors
      ret.lateralTuning.pid.kf = 0.00006
      #ret.lateralTuning.pid.kpV  = [0.01, 0.01, 0.005]
      #ret.lateralTuning.pid.kiV = [0.005, 0.005, 0.005]
      ret.lateralTuning.pid.kpV  = [0.01]
      ret.lateralTuning.pid.kiV = [0.005]
      
      # Technical specifications
      ret.mass = 1610 + STD_CARGO_KG
      ret.wheelbase = 2.647
      ret.centerToFront = ret.wheelbase * 0.44
      ret.steerRatio = 14.7
          
    # Assuming all is automatic
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)    

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    canMonoTimes = []
    buttonEvents = []
    params = Params()
   
    # Process the most recent CAN message traffic, and check for validity
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
   
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
    
    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)
    
    # Events 
    events = self.create_common_events(ret)
    
    # Decide when to activate / deactivate openpilot
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    
    # Engage when ACC goes on if the speed is above 70 km/h. 
    # If ACC is already engaged, only engage openpilot when ACC is on 
    # and above 70 km/h and a rising edge appears on +,- or resume button.
    if ret.cruiseState.enabled and not self.cruiseState_enabled_prev and ret.vEgo > self.CP.minEnableSpeed \
       or ret.cruiseState.enabled and ret.vEgo > self.CP.minEnableSpeed \
       and (self.CS.buttonStates['accelCruise'] and not self.buttonStatesPrev['accelCruise'] \
       or self.CS.buttonStates['decelCruise'] and not self.buttonStatesPrev['decelCruise'] \
       or self.CS.buttonStates['resumeCruise'] and not self.buttonStatesPrev['resumeCruise']):
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    
    if not ret.cruiseState.enabled or ret.brakePressed:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))
    
    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 1.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    # update previous values 
    self.gas_pressed_prev = ret.gasPressed
    self.cruiseState_enabled_prev = ret.cruiseState.enabled
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    # cast to reader so it can't be modified
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(self.CS, self.frame, c.actuators)
    self.frame += 1
    return can_sends
