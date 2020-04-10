from common.numpy_fast import clip
from selfdrive.car.volvo.values import CAR, DBC, CarControllerParams as CCP 
from selfdrive.car.volvo import volvocan
from opendbc.can.packer import CANPacker
from collections import deque
from selfdrive.kegman_conf import kegman_conf
from selfdrive.controls.lib.pid import apply_deadzone
kegman = kegman_conf()

class SteerCommand:
    angle_request = 0
    steer_direction = 0
    unkown = 0


class CarController():
  def __init__(self, dbc_name, CP, VW):
    
    # state
    self.acc_enabled_prev = 0

    # steering related
    # angle
    self.lka_angle_request_prev = 0
    self.rel_angle_change = 0 # The change in angle request (do we want to steer to left or right of last position?)
    self.rel_angle_change_prev = 0
    
    # direction
    self.BLOCK_LEN = 4
    self.DEQ_LEN = 3
    self.block_steering = 0
    self.stored_steer_direction = 0
 
    self.steer_direction_prev = 0
    self.steer_direction_deque = deque(maxlen=self.DEQ_LEN)
    self.steer_right_ratio_prev = 0

    # Direction change statemachine
    self.UNBLOCKED = 0
    self.BLOCKED = 1
    self.dir_state = 0
    self.steer_direction_bf_block = 0
    self.des_steer_direction_prev = 0

    # SteerCommand
    self.SteerCommand = SteerCommand
    
    # Diag
    self.doDTCRequests = True # Turn on and off DTC requests
    self.clearDtcs = True # Set false to stop sending diagnostic requests 
    self.timeout = 999
    self.diagRequest = { 
      "byte0": 0x03,
      "byte1": 0x19,
      "byte2": 0x02,
      "byte3": 0x0f,
      }
    self.flowControl = { 
      "byte0": 0x30,
      "byte1": 0x00,
      "byte2": 0x00,
      "byte3": 0x00,
      }
    self.clearDTC = {
      "byte0": 0x04,
      "byte1": 0x14,
      "byte2": 0xFF,
      "byte3": 0xFF,
      "byte4": 0xFF,
      }

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.CP = CP
    #self.params = CarControllerParams(CP.carFingerprint)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])


  def max_angle_req(self, current_steer_angle, lka_angle_request_prev, CCP):
    """ 
    Calculate maximum angle request delta/offset from current steering angle. 
    
    This is just a helper function that calculates the boundary for min and max
    steering angle request. It uses the parameters CCP.MAX_ACT_ANGLE_REQUEST_DIFF
    and CCP.STEER_ANGLE_DELTA_REQ_DIFF. To calculate the max and min allowed delta/offset request.

    The delta request is just a rate limiter. The request angle cant change more 
    than CCP.STEER_ANGLE_DELTA_REQ_DIFF per loop. 
    
    """

    # determine max and min allowed lka angle request
    # based on delta per sample
    max_delta_right = lka_angle_request_prev-CCP.STEER_ANGLE_DELTA_REQ_DIFF 
    max_delta_left = lka_angle_request_prev+CCP.STEER_ANGLE_DELTA_REQ_DIFF

    # based on distance from actual steering angle
    max_right = current_steer_angle-CCP.MAX_ACT_ANGLE_REQUEST_DIFF 
    max_left = current_steer_angle+CCP.MAX_ACT_ANGLE_REQUEST_DIFF

    return max_right, max_left, max_delta_right, max_delta_left


  def direction_change(self, SteerCommand):
    """
    Briefly pauses the steering request when switching direction to steer.
    
    When switching desired steering direction.
    I.e from left to right, a small pause is needed. 
    Otherwise the servo wont change steering direction.

    This function filters the steering input a little 
    so if we try to change direction, at a high rate, 
    every 4th processing loop. It will be filtered out and hold
    the same direction.

    CCP.Parameters:
    SteerCommand.steer_direction (int) = should be set to CCP.STEER_RIGHT or CCP.STEER_LEFT

    Returns:
    SteerCommand.steer_direction (int) = will return the value CCP.STEER_NO, CCP.STEER_LEFT, 
                            CCP.STEER_RIGHT
    """
    # Only run when not blocking a steering direction change
    if not self.block_steering:
      reengage = 1 if (self.steer_right_ratio_prev == -1) else 0

      if not self.acc_enabled_prev or reengage:
        self.steer_right_ratio_prev = self.DEQ_LEN if SteerCommand.steer_direction == CCP.STEER_RIGHT else 0
        # Fill list first time
        for _ in range(self.DEQ_LEN):
          self.steer_direction_deque.appendleft(SteerCommand.steer_direction)
      else:
        self.steer_direction_deque.appendleft(SteerCommand.steer_direction)
      # Calculate the ratio over last steering direction.  
      if self.steer_direction_deque.count(CCP.STEER_LEFT): # protect division by zero
        steer_right_ratio = float(self.steer_direction_deque.count(CCP.STEER_RIGHT)) / self.steer_direction_deque.count(CCP.STEER_LEFT)
      else:
        steer_right_ratio = self.DEQ_LEN

      # Set steering_direction based on the majority of SteerCommand.steer_direction in steer_direction_deque.
      SteerCommand.steer_direction = CCP.STEER_RIGHT if steer_right_ratio > 1 else CCP.STEER_LEFT
      
      # Detect time to change steering direction
      if (( steer_right_ratio > 1 and self.steer_right_ratio_prev < 1 ) or ( steer_right_ratio < 1 and self.steer_right_ratio_prev > 1 )):
        # Todo: Add check of torque from servo. When torque is zero allow switch of direction.
        change_detected = True
        self.block_steering = self.BLOCK_LEN # Block steering change
      else:
        change_detected = False
      
      # Reset deque when change is detected and fill with new steering direction.
      if change_detected:
        self.steer_direction_deque.clear()
        self.steer_right_ratio_prev = self.DEQ_LEN if SteerCommand.steer_direction == CCP.STEER_RIGHT else 0
        
        for _ in range(self.DEQ_LEN):
          self.steer_direction_deque.appendleft(SteerCommand.steer_direction)
      
    # When blocking set SteerCommand.steer_direction to no steering.
    if self.block_steering:
      #self.stored_steer_direction = SteerCommand.steer_direction if self.block_steering == self.BLOCK_LEN else self.stored_steer_direction # not needed right now
      self.block_steering -= 1
      SteerCommand.steer_direction = CCP.STEER_NO
      steer_right_ratio = -1 if not self.block_steering else 0 # Trigger reengage
    
    # Update stored values  
    self.steer_right_ratio_prev = steer_right_ratio  

    return SteerCommand.steer_direction


  def dir_change(self, steer_direction, error):
    """ Filters out direction changes
    
    Uses a simple state machine to determine if we should 
    block or allow the steer_direction bits to pass thru.

    """
    
    dessd = steer_direction
    dzError = 0 if abs(error) < CCP.DEADZONE else error 
    tState = -1 

    # Update prev with desired if just enabled.
    self.des_steer_direction_prev = steer_direction if not self.acc_enabled_prev else self.des_steer_direction_prev
    
    # Check conditions for state change
    if self.dir_state == self.UNBLOCKED:
      tState = self.BLOCKED if (steer_direction != self.des_steer_direction_prev and dzError != 0) else tState
    elif self.dir_state == self.BLOCKED:
      if (steer_direction == self.steer_direction_bf_block) or (self.block_steering <= 0) or (dzError == 0):
        tState = self.UNBLOCKED

    # State transition
    if tState == self.UNBLOCKED:
      self.dir_state = self.UNBLOCKED
    elif tState == self.BLOCKED:
      self.steer_direction_bf_block = self.des_steer_direction_prev  
      self.block_steering = self.BLOCK_LEN
      self.dir_state = self.BLOCKED

    #  Run actions in state
    if self.dir_state == self.UNBLOCKED:
      if dzError == 0:
        steer_direction = self.des_steer_direction_prev # Set old request when inside deadzone
    if self.dir_state == self.BLOCKED:
      self.block_steering -= 1
      steer_direction = CCP.STEER_NO

    #print("State:{} Sd:{} Sdp:{} Bs:{} Dz:{:.2f} Err:{:.2f}".format(self.dir_state, steer_direction, self.des_steer_direction_prev, self.block_steering, dzError, error))
    return steer_direction


  def update(self, CS, frame, actuators): #, pcm_cancel_cmd, visual_alert, left_line, right_line):
    """ Controls thread """
    
    # Send CAN commands.
    can_sends = []

    acc_enabled = CS.out.cruiseState.enabled
    
    # run at 50hz
    if (frame % 2 == 0):
      
      if acc_enabled and CS.out.vEgo > self.CP.minSteerSpeed:
        current_steer_angle = CS.out.steeringAngle
        self.SteerCommand.angle_request = actuators.steerAngle # Desired value from pathplanner
        
        self.SteerCommand.steer_direction = CCP.STEER_RIGHT if current_steer_angle > self.SteerCommand.angle_request else CCP.STEER_LEFT
        self.SteerCommand.steer_direction = self.dir_change(self.SteerCommand.steer_direction, current_steer_angle-self.SteerCommand.angle_request) # Filter the direction change 
        
        # get maximum allowed steering angle request
        max_right, max_left, max_delta_right, max_delta_left = self.max_angle_req(current_steer_angle, self.lka_angle_request_prev, CCP)
        
        # set clipped lka angle request
        # activating from disabled is allowed to bypass the delta change requirement
        # TODO: Fix this with minSteerSpeed.
        self.SteerCommand.angle_request = clip(self.SteerCommand.angle_request, max_delta_right, max_delta_left) if self.acc_enabled_prev else self.SteerCommand.angle_request
        self.SteerCommand.angle_request = clip(self.SteerCommand.angle_request, max_right, max_left)

      else:
        self.SteerCommand.steer_direction = CCP.STEER_NO
        self.SteerCommand.angle_request = 0

      # set unkown field to fix value
      self.SteerCommand.unkown = 0

      # update stored values
      self.acc_enabled_prev = acc_enabled
      self.lka_angle_request_prev = self.SteerCommand.angle_request
      self.steer_direction_prev = self.SteerCommand.steer_direction
      if self.SteerCommand.steer_direction == CCP.STEER_RIGHT or self.SteerCommand.steer_direction == CCP.STEER_LEFT:
        self.des_steer_direction_prev = self.SteerCommand.steer_direction
      
      # Manipulate data from servo to FSM
      # based on if we are steering or not
      can_sends = volvocan.manipulateServo(self.packer, self.CP.carFingerprint, CS, can_sends)
    
      # send can, add to list.
      can_sends.append(volvocan.create_steering_control(self.packer, frame, self.CP.carFingerprint, self.SteerCommand, CS.FSMInfo))

    # Send diagnostic requests
    if(self.doDTCRequests):
      if(frame % 100 == 0) and (not self.clearDtcs):
        # Request diagnostic codes, 2 Hz
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.diagRequest))
        can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.diagRequest))
        can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.diagRequest))
        self.timeout = frame + 5 # Set wait time 
      
      # Handle flow control in case of many DTC
      if frame > self.timeout: # Wait fix time before sending flow control, otherwise just spamming...
        self.timeout = frame-1 
        if (CS.diag.diagFSMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.flowControl))
        if (CS.diag.diagCEMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.flowControl))
        if (CS.diag.diagPSCMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.flowControl))
        
      # Clear DTCs in FSM on start
      # TODO check for engine running before clearing dtc.
      if(self.clearDtcs and (frame > 0) and (frame % 500 == 0)):
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.clearDTC))
        can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.clearDTC))
        can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.clearDTC))
        self.clearDtcs = False
      
    return can_sends
