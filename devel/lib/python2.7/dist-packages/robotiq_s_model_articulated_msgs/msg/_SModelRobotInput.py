# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robotiq_s_model_articulated_msgs/SModelRobotInput.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SModelRobotInput(genpy.Message):
  _md5sum = "4d0701156e580a420c48833f57bc83f3"
  _type = "robotiq_s_model_articulated_msgs/SModelRobotInput"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# gACT : Initialization status, echo of the rACT bit (activation bit).
# 0x0 - Gripper reset.
# 0x1 - Gripper activation.

uint8 gACT

# gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).
# 0x00 - Basic mode.
# 0x01 - Pinch mode.
# 0x02 - Wide mode.
# 0x03 - Scissor mode.

uint8 gMOD

# gGTO : Action status. echo of the rGTO bit (go to bit).
# 0x0 - Stopped (or performing activation / grasping mode change / automatic release)
# 0x1 - Go to Position Request

uint8 gGTO

# gIMC : Gripper status, returns the current status of the Gripper.
# 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.
# 0x01 - Activation is in progress.
# 0x02 - Mode change is in progress.
# 0x03 - Activation and mode change are completed.

uint8 gIMC

# gSTA : Motion status, returns the current motion of the Gripper fingers.
# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
# 0x01 - Gripper is stopped. One or two fingers stopped before requested position
# 0x02 - Gripper is stopped. All fingers stopped before requested position
# 0x03 - Gripper is stopped. All fingers reached requested position

uint8 gSTA

# gDTA : Finger A object detection status returns information on possible object contact from finger A.
# 0x00 - Finger A is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger A has stopped due to a contact while opening.
# 0x02 - Finger A has stopped due to a contact while closing.
# 0x03 - Finger A is at requested position.

uint8 gDTA

# gDTB : Finger B object detection status returns information on possible object contact from finger B.
# 0x00 - Finger B is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger B has stopped due to a contact while opening.
# 0x02 - Finger B has stopped due to a contact while closing.
# 0x03 - Finger B is at requested position.

uint8 gDTB

# gDTC : Finger C object detection status returns information on possible object contact from finger A.
# 0x00 - Finger C is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger C has stopped due to a contact while opening.
# 0x02 - Finger C has stopped due to a contact while closing.
# 0x03 - Finger C is at requested position.

uint8 gDTC

# gDTS : Scissor object detection status returns information on possible object contact from scissor.
# 0x00 - Scissor is in motion (only meaningful if gGTO = 1).
# 0x01 - Scissor has stopped due to a contact while opening.
# 0x02 - Scissor has stopped due to a contact while closing.
# 0x03 - Scissor is at requested position.

uint8 gDTS

# gFLT : Fault status returns general error messages useful for troubleshooting.
#   0x00 - No fault (fault LED off)
#   Priority faults (fault LED off)
#     0x05 - Action delayed, activation (reactivation) must be completed prior to action.
#     0x06 - Action delayed, mode change must be completed prior to action.
#     0x07 - The activation bit must be set prior to action.
#   Minor faults (fault LED continuous red)
#     0x09 - The communication chip is not ready (may be booting).
#     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).
#     0x0B - Automatic release in progress.
#   Major faults (fault LED blinking red) - Reset is required
#     0x0D - Activation fault, verify that no interference or other error occurred.
#     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).
#     0x0F - Automatic release completed. Reset and activation is required.

uint8 gFLT


# gPRA : Echo of the requested position of the Gripper (rPRA),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).
# If commanding the Gripper in
#   individual control mode, gPRA is the echo of finger A,
#   otherwise it is the general position requested to all fingers.

uint8 gPRA

# gPOA : Returns the actual position of the Gripper finger A,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOA

# gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUA

# gPRB : Echo of the requested position of finger B (rPRB),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRB

# gPOB : Returns the actual position of the Gripper finger B,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOB

# gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUB

# gPRC : Echo of the requested position of finger C (rPRC),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRC

# gPOC : Returns the actual position of the Gripper finger C,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOC

# gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUC

# gPRS : Echo of the requested position of scissor (rPRS),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRS

# gPOS : Returns the actual position of the Gripper scissor,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOS

# gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.
#   0x00 is ...
#   0xFF is ...

uint8 gCUS
"""
  __slots__ = ['gACT','gMOD','gGTO','gIMC','gSTA','gDTA','gDTB','gDTC','gDTS','gFLT','gPRA','gPOA','gCUA','gPRB','gPOB','gCUB','gPRC','gPOC','gCUC','gPRS','gPOS','gCUS']
  _slot_types = ['uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gACT,gMOD,gGTO,gIMC,gSTA,gDTA,gDTB,gDTC,gDTS,gFLT,gPRA,gPOA,gCUA,gPRB,gPOB,gCUB,gPRC,gPOC,gCUC,gPRS,gPOS,gCUS

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SModelRobotInput, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.gACT is None:
        self.gACT = 0
      if self.gMOD is None:
        self.gMOD = 0
      if self.gGTO is None:
        self.gGTO = 0
      if self.gIMC is None:
        self.gIMC = 0
      if self.gSTA is None:
        self.gSTA = 0
      if self.gDTA is None:
        self.gDTA = 0
      if self.gDTB is None:
        self.gDTB = 0
      if self.gDTC is None:
        self.gDTC = 0
      if self.gDTS is None:
        self.gDTS = 0
      if self.gFLT is None:
        self.gFLT = 0
      if self.gPRA is None:
        self.gPRA = 0
      if self.gPOA is None:
        self.gPOA = 0
      if self.gCUA is None:
        self.gCUA = 0
      if self.gPRB is None:
        self.gPRB = 0
      if self.gPOB is None:
        self.gPOB = 0
      if self.gCUB is None:
        self.gCUB = 0
      if self.gPRC is None:
        self.gPRC = 0
      if self.gPOC is None:
        self.gPOC = 0
      if self.gCUC is None:
        self.gCUC = 0
      if self.gPRS is None:
        self.gPRS = 0
      if self.gPOS is None:
        self.gPOS = 0
      if self.gCUS is None:
        self.gCUS = 0
    else:
      self.gACT = 0
      self.gMOD = 0
      self.gGTO = 0
      self.gIMC = 0
      self.gSTA = 0
      self.gDTA = 0
      self.gDTB = 0
      self.gDTC = 0
      self.gDTS = 0
      self.gFLT = 0
      self.gPRA = 0
      self.gPOA = 0
      self.gCUA = 0
      self.gPRB = 0
      self.gPOB = 0
      self.gCUB = 0
      self.gPRC = 0
      self.gPOC = 0
      self.gCUC = 0
      self.gPRS = 0
      self.gPOS = 0
      self.gCUS = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_22B().pack(_x.gACT, _x.gMOD, _x.gGTO, _x.gIMC, _x.gSTA, _x.gDTA, _x.gDTB, _x.gDTC, _x.gDTS, _x.gFLT, _x.gPRA, _x.gPOA, _x.gCUA, _x.gPRB, _x.gPOB, _x.gCUB, _x.gPRC, _x.gPOC, _x.gCUC, _x.gPRS, _x.gPOS, _x.gCUS))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 22
      (_x.gACT, _x.gMOD, _x.gGTO, _x.gIMC, _x.gSTA, _x.gDTA, _x.gDTB, _x.gDTC, _x.gDTS, _x.gFLT, _x.gPRA, _x.gPOA, _x.gCUA, _x.gPRB, _x.gPOB, _x.gCUB, _x.gPRC, _x.gPOC, _x.gCUC, _x.gPRS, _x.gPOS, _x.gCUS,) = _get_struct_22B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_22B().pack(_x.gACT, _x.gMOD, _x.gGTO, _x.gIMC, _x.gSTA, _x.gDTA, _x.gDTB, _x.gDTC, _x.gDTS, _x.gFLT, _x.gPRA, _x.gPOA, _x.gCUA, _x.gPRB, _x.gPOB, _x.gCUB, _x.gPRC, _x.gPOC, _x.gCUC, _x.gPRS, _x.gPOS, _x.gCUS))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 22
      (_x.gACT, _x.gMOD, _x.gGTO, _x.gIMC, _x.gSTA, _x.gDTA, _x.gDTB, _x.gDTC, _x.gDTS, _x.gFLT, _x.gPRA, _x.gPOA, _x.gCUA, _x.gPRB, _x.gPOB, _x.gCUB, _x.gPRC, _x.gPOC, _x.gCUC, _x.gPRS, _x.gPOS, _x.gCUS,) = _get_struct_22B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_22B = None
def _get_struct_22B():
    global _struct_22B
    if _struct_22B is None:
        _struct_22B = struct.Struct("<22B")
    return _struct_22B
