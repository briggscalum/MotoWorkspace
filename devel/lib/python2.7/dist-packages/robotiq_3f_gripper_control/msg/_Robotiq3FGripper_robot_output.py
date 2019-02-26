# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robotiq_3f_gripper_control/Robotiq3FGripper_robot_output.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Robotiq3FGripper_robot_output(genpy.Message):
  _md5sum = "31ba91390a569c669af204c3d006a806"
  _type = "robotiq_3f_gripper_control/Robotiq3FGripper_robot_output"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 rACT 
uint8 rMOD 
uint8 rGTO 
uint8 rATR 
uint8 rGLV 
uint8 rICF 
uint8 rICS 
uint8 rPRA 
uint8 rSPA 
uint8 rFRA 
uint8 rPRB 
uint8 rSPB 
uint8 rFRB 
uint8 rPRC 
uint8 rSPC 
uint8 rFRC 
uint8 rPRS 
uint8 rSPS 
uint8 rFRS
"""
  __slots__ = ['rACT','rMOD','rGTO','rATR','rGLV','rICF','rICS','rPRA','rSPA','rFRA','rPRB','rSPB','rFRB','rPRC','rSPC','rFRC','rPRS','rSPS','rFRS']
  _slot_types = ['uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       rACT,rMOD,rGTO,rATR,rGLV,rICF,rICS,rPRA,rSPA,rFRA,rPRB,rSPB,rFRB,rPRC,rSPC,rFRC,rPRS,rSPS,rFRS

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Robotiq3FGripper_robot_output, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.rACT is None:
        self.rACT = 0
      if self.rMOD is None:
        self.rMOD = 0
      if self.rGTO is None:
        self.rGTO = 0
      if self.rATR is None:
        self.rATR = 0
      if self.rGLV is None:
        self.rGLV = 0
      if self.rICF is None:
        self.rICF = 0
      if self.rICS is None:
        self.rICS = 0
      if self.rPRA is None:
        self.rPRA = 0
      if self.rSPA is None:
        self.rSPA = 0
      if self.rFRA is None:
        self.rFRA = 0
      if self.rPRB is None:
        self.rPRB = 0
      if self.rSPB is None:
        self.rSPB = 0
      if self.rFRB is None:
        self.rFRB = 0
      if self.rPRC is None:
        self.rPRC = 0
      if self.rSPC is None:
        self.rSPC = 0
      if self.rFRC is None:
        self.rFRC = 0
      if self.rPRS is None:
        self.rPRS = 0
      if self.rSPS is None:
        self.rSPS = 0
      if self.rFRS is None:
        self.rFRS = 0
    else:
      self.rACT = 0
      self.rMOD = 0
      self.rGTO = 0
      self.rATR = 0
      self.rGLV = 0
      self.rICF = 0
      self.rICS = 0
      self.rPRA = 0
      self.rSPA = 0
      self.rFRA = 0
      self.rPRB = 0
      self.rSPB = 0
      self.rFRB = 0
      self.rPRC = 0
      self.rSPC = 0
      self.rFRC = 0
      self.rPRS = 0
      self.rSPS = 0
      self.rFRS = 0

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
      buff.write(_get_struct_19B().pack(_x.rACT, _x.rMOD, _x.rGTO, _x.rATR, _x.rGLV, _x.rICF, _x.rICS, _x.rPRA, _x.rSPA, _x.rFRA, _x.rPRB, _x.rSPB, _x.rFRB, _x.rPRC, _x.rSPC, _x.rFRC, _x.rPRS, _x.rSPS, _x.rFRS))
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
      end += 19
      (_x.rACT, _x.rMOD, _x.rGTO, _x.rATR, _x.rGLV, _x.rICF, _x.rICS, _x.rPRA, _x.rSPA, _x.rFRA, _x.rPRB, _x.rSPB, _x.rFRB, _x.rPRC, _x.rSPC, _x.rFRC, _x.rPRS, _x.rSPS, _x.rFRS,) = _get_struct_19B().unpack(str[start:end])
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
      buff.write(_get_struct_19B().pack(_x.rACT, _x.rMOD, _x.rGTO, _x.rATR, _x.rGLV, _x.rICF, _x.rICS, _x.rPRA, _x.rSPA, _x.rFRA, _x.rPRB, _x.rSPB, _x.rFRB, _x.rPRC, _x.rSPC, _x.rFRC, _x.rPRS, _x.rSPS, _x.rFRS))
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
      end += 19
      (_x.rACT, _x.rMOD, _x.rGTO, _x.rATR, _x.rGLV, _x.rICF, _x.rICS, _x.rPRA, _x.rSPA, _x.rFRA, _x.rPRB, _x.rSPB, _x.rFRB, _x.rPRC, _x.rSPC, _x.rFRC, _x.rPRS, _x.rSPS, _x.rFRS,) = _get_struct_19B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_19B = None
def _get_struct_19B():
    global _struct_19B
    if _struct_19B is None:
        _struct_19B = struct.Struct("<19B")
    return _struct_19B
