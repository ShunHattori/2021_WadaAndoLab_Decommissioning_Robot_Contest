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

class FeedbackState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_ended = null;
      this.current = null;
      this.reference_feedbackside = null;
      this.mode_feedbackside = null;
    }
    else {
      if (initObj.hasOwnProperty('is_ended')) {
        this.is_ended = initObj.is_ended
      }
      else {
        this.is_ended = [];
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = [];
      }
      if (initObj.hasOwnProperty('reference_feedbackside')) {
        this.reference_feedbackside = initObj.reference_feedbackside
      }
      else {
        this.reference_feedbackside = [];
      }
      if (initObj.hasOwnProperty('mode_feedbackside')) {
        this.mode_feedbackside = initObj.mode_feedbackside
      }
      else {
        this.mode_feedbackside = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FeedbackState
    // Serialize message field [is_ended]
    bufferOffset = _arraySerializer.uint8(obj.is_ended, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.float64(obj.current, buffer, bufferOffset, null);
    // Serialize message field [reference_feedbackside]
    bufferOffset = _arraySerializer.float64(obj.reference_feedbackside, buffer, bufferOffset, null);
    // Serialize message field [mode_feedbackside]
    bufferOffset = _arraySerializer.uint8(obj.mode_feedbackside, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FeedbackState
    let len;
    let data = new FeedbackState(null);
    // Deserialize message field [is_ended]
    data.is_ended = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [reference_feedbackside]
    data.reference_feedbackside = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [mode_feedbackside]
    data.mode_feedbackside = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.is_ended.length;
    length += 8 * object.current.length;
    length += 8 * object.reference_feedbackside.length;
    length += object.mode_feedbackside.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commissioning_robot/FeedbackState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73c7091a2c6badbe1419f63c8f01c277';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] is_ended
    float64[] current
    float64[] reference_feedbackside
    uint8[] mode_feedbackside
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FeedbackState(null);
    if (msg.is_ended !== undefined) {
      resolved.is_ended = msg.is_ended;
    }
    else {
      resolved.is_ended = []
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = []
    }

    if (msg.reference_feedbackside !== undefined) {
      resolved.reference_feedbackside = msg.reference_feedbackside;
    }
    else {
      resolved.reference_feedbackside = []
    }

    if (msg.mode_feedbackside !== undefined) {
      resolved.mode_feedbackside = msg.mode_feedbackside;
    }
    else {
      resolved.mode_feedbackside = []
    }

    return resolved;
    }
};

module.exports = FeedbackState;
