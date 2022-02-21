// Auto-generated. Do not edit!

// (in-package commissioning_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MechanismReport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.running_phase = null;
      this.running_mode = null;
      this.state_limit = null;
      this.state_pulse = null;
      this.reference = null;
      this.current = null;
    }
    else {
      if (initObj.hasOwnProperty('running_phase')) {
        this.running_phase = initObj.running_phase
      }
      else {
        this.running_phase = 0;
      }
      if (initObj.hasOwnProperty('running_mode')) {
        this.running_mode = initObj.running_mode
      }
      else {
        this.running_mode = [];
      }
      if (initObj.hasOwnProperty('state_limit')) {
        this.state_limit = initObj.state_limit
      }
      else {
        this.state_limit = [];
      }
      if (initObj.hasOwnProperty('state_pulse')) {
        this.state_pulse = initObj.state_pulse
      }
      else {
        this.state_pulse = [];
      }
      if (initObj.hasOwnProperty('reference')) {
        this.reference = initObj.reference
      }
      else {
        this.reference = [];
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MechanismReport
    // Serialize message field [running_phase]
    bufferOffset = _serializer.uint8(obj.running_phase, buffer, bufferOffset);
    // Serialize message field [running_mode]
    bufferOffset = _arraySerializer.uint8(obj.running_mode, buffer, bufferOffset, null);
    // Serialize message field [state_limit]
    bufferOffset = _arraySerializer.uint8(obj.state_limit, buffer, bufferOffset, null);
    // Serialize message field [state_pulse]
    bufferOffset = _arraySerializer.int64(obj.state_pulse, buffer, bufferOffset, null);
    // Serialize message field [reference]
    bufferOffset = _arraySerializer.float64(obj.reference, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.float64(obj.current, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MechanismReport
    let len;
    let data = new MechanismReport(null);
    // Deserialize message field [running_phase]
    data.running_phase = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [running_mode]
    data.running_mode = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [state_limit]
    data.state_limit = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [state_pulse]
    data.state_pulse = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [reference]
    data.reference = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.running_mode.length;
    length += object.state_limit.length;
    length += 8 * object.state_pulse.length;
    length += 8 * object.reference.length;
    length += 8 * object.current.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commissioning_robot/MechanismReport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4efe817f8f990d484fcf48fdff687678';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 running_phase
    uint8[] running_mode
    uint8[] state_limit
    int64[] state_pulse
    float64[] reference
    float64[] current
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MechanismReport(null);
    if (msg.running_phase !== undefined) {
      resolved.running_phase = msg.running_phase;
    }
    else {
      resolved.running_phase = 0
    }

    if (msg.running_mode !== undefined) {
      resolved.running_mode = msg.running_mode;
    }
    else {
      resolved.running_mode = []
    }

    if (msg.state_limit !== undefined) {
      resolved.state_limit = msg.state_limit;
    }
    else {
      resolved.state_limit = []
    }

    if (msg.state_pulse !== undefined) {
      resolved.state_pulse = msg.state_pulse;
    }
    else {
      resolved.state_pulse = []
    }

    if (msg.reference !== undefined) {
      resolved.reference = msg.reference;
    }
    else {
      resolved.reference = []
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = []
    }

    return resolved;
    }
};

module.exports = MechanismReport;
