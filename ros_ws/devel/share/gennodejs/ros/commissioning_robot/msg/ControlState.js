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

class ControlState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.reference = null;
      this.manual = null;
      this.offset = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = [];
      }
      if (initObj.hasOwnProperty('reference')) {
        this.reference = initObj.reference
      }
      else {
        this.reference = [];
      }
      if (initObj.hasOwnProperty('manual')) {
        this.manual = initObj.manual
      }
      else {
        this.manual = [];
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlState
    // Serialize message field [mode]
    bufferOffset = _arraySerializer.uint8(obj.mode, buffer, bufferOffset, null);
    // Serialize message field [reference]
    bufferOffset = _arraySerializer.float64(obj.reference, buffer, bufferOffset, null);
    // Serialize message field [manual]
    bufferOffset = _arraySerializer.float64(obj.manual, buffer, bufferOffset, null);
    // Serialize message field [offset]
    bufferOffset = _arraySerializer.float64(obj.offset, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlState
    let len;
    let data = new ControlState(null);
    // Deserialize message field [mode]
    data.mode = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [reference]
    data.reference = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [manual]
    data.manual = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [offset]
    data.offset = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mode.length;
    length += 8 * object.reference.length;
    length += 8 * object.manual.length;
    length += 8 * object.offset.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commissioning_robot/ControlState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37618245187afeac8a81955f76bd2d07';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] mode
    float64[] reference
    float64[] manual
    float64[] offset
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlState(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = []
    }

    if (msg.reference !== undefined) {
      resolved.reference = msg.reference;
    }
    else {
      resolved.reference = []
    }

    if (msg.manual !== undefined) {
      resolved.manual = msg.manual;
    }
    else {
      resolved.manual = []
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = []
    }

    return resolved;
    }
};

module.exports = ControlState;
