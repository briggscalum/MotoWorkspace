// Auto-generated. Do not edit!

// (in-package robotiq_s_model_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SModel_robot_output {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rACT = null;
      this.rMOD = null;
      this.rGTO = null;
      this.rATR = null;
      this.rGLV = null;
      this.rICF = null;
      this.rICS = null;
      this.rPRA = null;
      this.rSPA = null;
      this.rFRA = null;
      this.rPRB = null;
      this.rSPB = null;
      this.rFRB = null;
      this.rPRC = null;
      this.rSPC = null;
      this.rFRC = null;
      this.rPRS = null;
      this.rSPS = null;
      this.rFRS = null;
    }
    else {
      if (initObj.hasOwnProperty('rACT')) {
        this.rACT = initObj.rACT
      }
      else {
        this.rACT = 0;
      }
      if (initObj.hasOwnProperty('rMOD')) {
        this.rMOD = initObj.rMOD
      }
      else {
        this.rMOD = 0;
      }
      if (initObj.hasOwnProperty('rGTO')) {
        this.rGTO = initObj.rGTO
      }
      else {
        this.rGTO = 0;
      }
      if (initObj.hasOwnProperty('rATR')) {
        this.rATR = initObj.rATR
      }
      else {
        this.rATR = 0;
      }
      if (initObj.hasOwnProperty('rGLV')) {
        this.rGLV = initObj.rGLV
      }
      else {
        this.rGLV = 0;
      }
      if (initObj.hasOwnProperty('rICF')) {
        this.rICF = initObj.rICF
      }
      else {
        this.rICF = 0;
      }
      if (initObj.hasOwnProperty('rICS')) {
        this.rICS = initObj.rICS
      }
      else {
        this.rICS = 0;
      }
      if (initObj.hasOwnProperty('rPRA')) {
        this.rPRA = initObj.rPRA
      }
      else {
        this.rPRA = 0;
      }
      if (initObj.hasOwnProperty('rSPA')) {
        this.rSPA = initObj.rSPA
      }
      else {
        this.rSPA = 0;
      }
      if (initObj.hasOwnProperty('rFRA')) {
        this.rFRA = initObj.rFRA
      }
      else {
        this.rFRA = 0;
      }
      if (initObj.hasOwnProperty('rPRB')) {
        this.rPRB = initObj.rPRB
      }
      else {
        this.rPRB = 0;
      }
      if (initObj.hasOwnProperty('rSPB')) {
        this.rSPB = initObj.rSPB
      }
      else {
        this.rSPB = 0;
      }
      if (initObj.hasOwnProperty('rFRB')) {
        this.rFRB = initObj.rFRB
      }
      else {
        this.rFRB = 0;
      }
      if (initObj.hasOwnProperty('rPRC')) {
        this.rPRC = initObj.rPRC
      }
      else {
        this.rPRC = 0;
      }
      if (initObj.hasOwnProperty('rSPC')) {
        this.rSPC = initObj.rSPC
      }
      else {
        this.rSPC = 0;
      }
      if (initObj.hasOwnProperty('rFRC')) {
        this.rFRC = initObj.rFRC
      }
      else {
        this.rFRC = 0;
      }
      if (initObj.hasOwnProperty('rPRS')) {
        this.rPRS = initObj.rPRS
      }
      else {
        this.rPRS = 0;
      }
      if (initObj.hasOwnProperty('rSPS')) {
        this.rSPS = initObj.rSPS
      }
      else {
        this.rSPS = 0;
      }
      if (initObj.hasOwnProperty('rFRS')) {
        this.rFRS = initObj.rFRS
      }
      else {
        this.rFRS = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SModel_robot_output
    // Serialize message field [rACT]
    bufferOffset = _serializer.uint8(obj.rACT, buffer, bufferOffset);
    // Serialize message field [rMOD]
    bufferOffset = _serializer.uint8(obj.rMOD, buffer, bufferOffset);
    // Serialize message field [rGTO]
    bufferOffset = _serializer.uint8(obj.rGTO, buffer, bufferOffset);
    // Serialize message field [rATR]
    bufferOffset = _serializer.uint8(obj.rATR, buffer, bufferOffset);
    // Serialize message field [rGLV]
    bufferOffset = _serializer.uint8(obj.rGLV, buffer, bufferOffset);
    // Serialize message field [rICF]
    bufferOffset = _serializer.uint8(obj.rICF, buffer, bufferOffset);
    // Serialize message field [rICS]
    bufferOffset = _serializer.uint8(obj.rICS, buffer, bufferOffset);
    // Serialize message field [rPRA]
    bufferOffset = _serializer.uint8(obj.rPRA, buffer, bufferOffset);
    // Serialize message field [rSPA]
    bufferOffset = _serializer.uint8(obj.rSPA, buffer, bufferOffset);
    // Serialize message field [rFRA]
    bufferOffset = _serializer.uint8(obj.rFRA, buffer, bufferOffset);
    // Serialize message field [rPRB]
    bufferOffset = _serializer.uint8(obj.rPRB, buffer, bufferOffset);
    // Serialize message field [rSPB]
    bufferOffset = _serializer.uint8(obj.rSPB, buffer, bufferOffset);
    // Serialize message field [rFRB]
    bufferOffset = _serializer.uint8(obj.rFRB, buffer, bufferOffset);
    // Serialize message field [rPRC]
    bufferOffset = _serializer.uint8(obj.rPRC, buffer, bufferOffset);
    // Serialize message field [rSPC]
    bufferOffset = _serializer.uint8(obj.rSPC, buffer, bufferOffset);
    // Serialize message field [rFRC]
    bufferOffset = _serializer.uint8(obj.rFRC, buffer, bufferOffset);
    // Serialize message field [rPRS]
    bufferOffset = _serializer.uint8(obj.rPRS, buffer, bufferOffset);
    // Serialize message field [rSPS]
    bufferOffset = _serializer.uint8(obj.rSPS, buffer, bufferOffset);
    // Serialize message field [rFRS]
    bufferOffset = _serializer.uint8(obj.rFRS, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SModel_robot_output
    let len;
    let data = new SModel_robot_output(null);
    // Deserialize message field [rACT]
    data.rACT = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rMOD]
    data.rMOD = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rGTO]
    data.rGTO = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rATR]
    data.rATR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rGLV]
    data.rGLV = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rICF]
    data.rICF = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rICS]
    data.rICS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rPRA]
    data.rPRA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rSPA]
    data.rSPA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rFRA]
    data.rFRA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rPRB]
    data.rPRB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rSPB]
    data.rSPB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rFRB]
    data.rFRB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rPRC]
    data.rPRC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rSPC]
    data.rSPC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rFRC]
    data.rFRC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rPRS]
    data.rPRS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rSPS]
    data.rSPS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rFRS]
    data.rFRS = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotiq_s_model_control/SModel_robot_output';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '31ba91390a569c669af204c3d006a806';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 rACT 
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SModel_robot_output(null);
    if (msg.rACT !== undefined) {
      resolved.rACT = msg.rACT;
    }
    else {
      resolved.rACT = 0
    }

    if (msg.rMOD !== undefined) {
      resolved.rMOD = msg.rMOD;
    }
    else {
      resolved.rMOD = 0
    }

    if (msg.rGTO !== undefined) {
      resolved.rGTO = msg.rGTO;
    }
    else {
      resolved.rGTO = 0
    }

    if (msg.rATR !== undefined) {
      resolved.rATR = msg.rATR;
    }
    else {
      resolved.rATR = 0
    }

    if (msg.rGLV !== undefined) {
      resolved.rGLV = msg.rGLV;
    }
    else {
      resolved.rGLV = 0
    }

    if (msg.rICF !== undefined) {
      resolved.rICF = msg.rICF;
    }
    else {
      resolved.rICF = 0
    }

    if (msg.rICS !== undefined) {
      resolved.rICS = msg.rICS;
    }
    else {
      resolved.rICS = 0
    }

    if (msg.rPRA !== undefined) {
      resolved.rPRA = msg.rPRA;
    }
    else {
      resolved.rPRA = 0
    }

    if (msg.rSPA !== undefined) {
      resolved.rSPA = msg.rSPA;
    }
    else {
      resolved.rSPA = 0
    }

    if (msg.rFRA !== undefined) {
      resolved.rFRA = msg.rFRA;
    }
    else {
      resolved.rFRA = 0
    }

    if (msg.rPRB !== undefined) {
      resolved.rPRB = msg.rPRB;
    }
    else {
      resolved.rPRB = 0
    }

    if (msg.rSPB !== undefined) {
      resolved.rSPB = msg.rSPB;
    }
    else {
      resolved.rSPB = 0
    }

    if (msg.rFRB !== undefined) {
      resolved.rFRB = msg.rFRB;
    }
    else {
      resolved.rFRB = 0
    }

    if (msg.rPRC !== undefined) {
      resolved.rPRC = msg.rPRC;
    }
    else {
      resolved.rPRC = 0
    }

    if (msg.rSPC !== undefined) {
      resolved.rSPC = msg.rSPC;
    }
    else {
      resolved.rSPC = 0
    }

    if (msg.rFRC !== undefined) {
      resolved.rFRC = msg.rFRC;
    }
    else {
      resolved.rFRC = 0
    }

    if (msg.rPRS !== undefined) {
      resolved.rPRS = msg.rPRS;
    }
    else {
      resolved.rPRS = 0
    }

    if (msg.rSPS !== undefined) {
      resolved.rSPS = msg.rSPS;
    }
    else {
      resolved.rSPS = 0
    }

    if (msg.rFRS !== undefined) {
      resolved.rFRS = msg.rFRS;
    }
    else {
      resolved.rFRS = 0
    }

    return resolved;
    }
};

module.exports = SModel_robot_output;
