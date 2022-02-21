// Auto-generated. Do not edit!

// (in-package ds4_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.set_led = null;
      this.led_r = null;
      this.led_g = null;
      this.led_b = null;
      this.set_led_flash = null;
      this.led_flash_on = null;
      this.led_flash_off = null;
      this.set_rumble = null;
      this.rumble_duration = null;
      this.rumble_small = null;
      this.rumble_big = null;
    }
    else {
      if (initObj.hasOwnProperty('set_led')) {
        this.set_led = initObj.set_led
      }
      else {
        this.set_led = false;
      }
      if (initObj.hasOwnProperty('led_r')) {
        this.led_r = initObj.led_r
      }
      else {
        this.led_r = 0.0;
      }
      if (initObj.hasOwnProperty('led_g')) {
        this.led_g = initObj.led_g
      }
      else {
        this.led_g = 0.0;
      }
      if (initObj.hasOwnProperty('led_b')) {
        this.led_b = initObj.led_b
      }
      else {
        this.led_b = 0.0;
      }
      if (initObj.hasOwnProperty('set_led_flash')) {
        this.set_led_flash = initObj.set_led_flash
      }
      else {
        this.set_led_flash = false;
      }
      if (initObj.hasOwnProperty('led_flash_on')) {
        this.led_flash_on = initObj.led_flash_on
      }
      else {
        this.led_flash_on = 0.0;
      }
      if (initObj.hasOwnProperty('led_flash_off')) {
        this.led_flash_off = initObj.led_flash_off
      }
      else {
        this.led_flash_off = 0.0;
      }
      if (initObj.hasOwnProperty('set_rumble')) {
        this.set_rumble = initObj.set_rumble
      }
      else {
        this.set_rumble = false;
      }
      if (initObj.hasOwnProperty('rumble_duration')) {
        this.rumble_duration = initObj.rumble_duration
      }
      else {
        this.rumble_duration = 0.0;
      }
      if (initObj.hasOwnProperty('rumble_small')) {
        this.rumble_small = initObj.rumble_small
      }
      else {
        this.rumble_small = 0.0;
      }
      if (initObj.hasOwnProperty('rumble_big')) {
        this.rumble_big = initObj.rumble_big
      }
      else {
        this.rumble_big = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Feedback
    // Serialize message field [set_led]
    bufferOffset = _serializer.bool(obj.set_led, buffer, bufferOffset);
    // Serialize message field [led_r]
    bufferOffset = _serializer.float32(obj.led_r, buffer, bufferOffset);
    // Serialize message field [led_g]
    bufferOffset = _serializer.float32(obj.led_g, buffer, bufferOffset);
    // Serialize message field [led_b]
    bufferOffset = _serializer.float32(obj.led_b, buffer, bufferOffset);
    // Serialize message field [set_led_flash]
    bufferOffset = _serializer.bool(obj.set_led_flash, buffer, bufferOffset);
    // Serialize message field [led_flash_on]
    bufferOffset = _serializer.float32(obj.led_flash_on, buffer, bufferOffset);
    // Serialize message field [led_flash_off]
    bufferOffset = _serializer.float32(obj.led_flash_off, buffer, bufferOffset);
    // Serialize message field [set_rumble]
    bufferOffset = _serializer.bool(obj.set_rumble, buffer, bufferOffset);
    // Serialize message field [rumble_duration]
    bufferOffset = _serializer.float32(obj.rumble_duration, buffer, bufferOffset);
    // Serialize message field [rumble_small]
    bufferOffset = _serializer.float32(obj.rumble_small, buffer, bufferOffset);
    // Serialize message field [rumble_big]
    bufferOffset = _serializer.float32(obj.rumble_big, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Feedback
    let len;
    let data = new Feedback(null);
    // Deserialize message field [set_led]
    data.set_led = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [led_r]
    data.led_r = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led_g]
    data.led_g = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led_b]
    data.led_b = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_led_flash]
    data.set_led_flash = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [led_flash_on]
    data.led_flash_on = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led_flash_off]
    data.led_flash_off = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [set_rumble]
    data.set_rumble = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rumble_duration]
    data.rumble_duration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rumble_small]
    data.rumble_small = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rumble_big]
    data.rumble_big = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 35;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ds4_driver/Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e6ff655a0461aca2c8725df811fe5c33';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message file for controlling DualShock4
    # LED intensities [0.0, 1.0]
    bool set_led
    float32 led_r
    float32 led_g
    float32 led_b
    
    # LED flash durations in seconds (max is 2.5 sec)
    bool set_led_flash
    # Note: Set led_flash_off to 0 to stop flashing
    float32 led_flash_on
    float32 led_flash_off
    
    # Rumble intensities [0.0, 1.0] (continues rumbling for 5 sec by default)
    bool set_rumble
    float32 rumble_duration
    float32 rumble_small
    float32 rumble_big
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Feedback(null);
    if (msg.set_led !== undefined) {
      resolved.set_led = msg.set_led;
    }
    else {
      resolved.set_led = false
    }

    if (msg.led_r !== undefined) {
      resolved.led_r = msg.led_r;
    }
    else {
      resolved.led_r = 0.0
    }

    if (msg.led_g !== undefined) {
      resolved.led_g = msg.led_g;
    }
    else {
      resolved.led_g = 0.0
    }

    if (msg.led_b !== undefined) {
      resolved.led_b = msg.led_b;
    }
    else {
      resolved.led_b = 0.0
    }

    if (msg.set_led_flash !== undefined) {
      resolved.set_led_flash = msg.set_led_flash;
    }
    else {
      resolved.set_led_flash = false
    }

    if (msg.led_flash_on !== undefined) {
      resolved.led_flash_on = msg.led_flash_on;
    }
    else {
      resolved.led_flash_on = 0.0
    }

    if (msg.led_flash_off !== undefined) {
      resolved.led_flash_off = msg.led_flash_off;
    }
    else {
      resolved.led_flash_off = 0.0
    }

    if (msg.set_rumble !== undefined) {
      resolved.set_rumble = msg.set_rumble;
    }
    else {
      resolved.set_rumble = false
    }

    if (msg.rumble_duration !== undefined) {
      resolved.rumble_duration = msg.rumble_duration;
    }
    else {
      resolved.rumble_duration = 0.0
    }

    if (msg.rumble_small !== undefined) {
      resolved.rumble_small = msg.rumble_small;
    }
    else {
      resolved.rumble_small = 0.0
    }

    if (msg.rumble_big !== undefined) {
      resolved.rumble_big = msg.rumble_big;
    }
    else {
      resolved.rumble_big = 0.0
    }

    return resolved;
    }
};

module.exports = Feedback;
