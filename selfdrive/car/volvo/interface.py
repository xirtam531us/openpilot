#!/usr/bin/env python3
from cereal import car
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.car.volvo.values import CAR, PLATFORM, BUTTON_STATES
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
    
    # Set specific paramter
    if candidate in PLATFORM.C1:
      ret.safetyParam = 1
      ret.safetyModel = car.CarParams.SafetyModel.volvoC1
    if candidate in PLATFORM.EUCD:
      ret.safetyParam = 2
      ret.safetyModel = car.CarParams.SafetyModel.volvoEUCD
   
    if candidate:
       # Set common parameters
      ret.carName = "volvo"
      ret.radarOffCan = True  # No radar objects on can
      ret.steerControlType = car.CarParams.SteerControlType.angle
      #ret.steerLimitAlert = False # Do this do anything?
      #ret.minSteerSpeed = 30. * CV.KPH_TO_MS
      ret.enableCamera = True # force openpilot to fake the stock camera
      
      # Steering settings - tuning parameters for lateral control.
      ret.steerRateCost = 0.5 # Used in pathplanner for punishing? Steering movements?
      ret.steerActuatorDelay = 0.1 # Actuator delay from input to output.
      
      # No PID control used. Set to 0, otherwise pid loop crashes.
      #ret.steerMaxBP = [0.] # m/s
      #ret.steerMaxV = [1.]
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      # Tuning factors
      ret.lateralTuning.pid.kf = 0.0
      ret.lateralTuning.pid.kpV  = [0.0]
      ret.lateralTuning.pid.kiV = [0.0]
      
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
    
    # Engagement and longitudinal control using stock ACC. Make sure OP is
    # disengaged if stock ACC is disengaged.
    if not ret.cruiseState.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
    # Attempt OP engagement only on rising edge of stock ACC engagement.
    elif not self.cruiseState_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))

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
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                               c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)
    self.frame += 1
    return can_sends
