// Auto-generated. Do not edit!

// (in-package ds4_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Report {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_analog_x = null;
      this.left_analog_y = null;
      this.right_analog_x = null;
      this.right_analog_y = null;
      this.l2_analog = null;
      this.r2_analog = null;
      this.dpad_up = null;
      this.dpad_down = null;
      this.dpad_left = null;
      this.dpad_right = null;
      this.button_cross = null;
      this.button_circle = null;
      this.button_square = null;
      this.button_triangle = null;
      this.button_l1 = null;
      this.button_l2 = null;
      this.button_l3 = null;
      this.button_r1 = null;
      this.button_r2 = null;
      this.button_r3 = null;
      this.button_share = null;
      this.button_options = null;
      this.button_trackpad = null;
      this.button_ps = null;
      this.lin_acc_x = null;
      this.lin_acc_y = null;
      this.lin_acc_z = null;
      this.ang_vel_x = null;
      this.ang_vel_y = null;
      this.ang_vel_z = null;
      this.trackpad_touch0_id = null;
      this.trackpad_touch0_active = null;
      this.trackpad_touch0_x = null;
      this.trackpad_touch0_y = null;
      this.trackpad_touch1_id = null;
      this.trackpad_touch1_active = null;
      this.trackpad_touch1_x = null;
      this.trackpad_touch1_y = null;
      this.timestamp = null;
      this.battery = null;
      this.plug_usb = null;
      this.plug_audio = null;
      this.plug_mic = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_analog_x')) {
        this.left_analog_x = initObj.left_analog_x
      }
      else {
        this.left_analog_x = 0;
      }
      if (initObj.hasOwnProperty('left_analog_y')) {
        this.left_analog_y = initObj.left_analog_y
      }
      else {
        this.left_analog_y = 0;
      }
      if (initObj.hasOwnProperty('right_analog_x')) {
        this.right_analog_x = initObj.right_analog_x
      }
      else {
        this.right_analog_x = 0;
      }
      if (initObj.hasOwnProperty('right_analog_y')) {
        this.right_analog_y = initObj.right_analog_y
      }
      else {
        this.right_analog_y = 0;
      }
      if (initObj.hasOwnProperty('l2_analog')) {
        this.l2_analog = initObj.l2_analog
      }
      else {
        this.l2_analog = 0;
      }
      if (initObj.hasOwnProperty('r2_analog')) {
        this.r2_analog = initObj.r2_analog
      }
      else {
        this.r2_analog = 0;
      }
      if (initObj.hasOwnProperty('dpad_up')) {
        this.dpad_up = initObj.dpad_up
      }
      else {
        this.dpad_up = false;
      }
      if (initObj.hasOwnProperty('dpad_down')) {
        this.dpad_down = initObj.dpad_down
      }
      else {
        this.dpad_down = false;
      }
      if (initObj.hasOwnProperty('dpad_left')) {
        this.dpad_left = initObj.dpad_left
      }
      else {
        this.dpad_left = false;
      }
      if (initObj.hasOwnProperty('dpad_right')) {
        this.dpad_right = initObj.dpad_right
      }
      else {
        this.dpad_right = false;
      }
      if (initObj.hasOwnProperty('button_cross')) {
        this.button_cross = initObj.button_cross
      }
      else {
        this.button_cross = false;
      }
      if (initObj.hasOwnProperty('button_circle')) {
        this.button_circle = initObj.button_circle
      }
      else {
        this.button_circle = false;
      }
      if (initObj.hasOwnProperty('button_square')) {
        this.button_square = initObj.button_square
      }
      else {
        this.button_square = false;
      }
      if (initObj.hasOwnProperty('button_triangle')) {
        this.button_triangle = initObj.button_triangle
      }
      else {
        this.button_triangle = false;
      }
      if (initObj.hasOwnProperty('button_l1')) {
        this.button_l1 = initObj.button_l1
      }
      else {
        this.button_l1 = false;
      }
      if (initObj.hasOwnProperty('button_l2')) {
        this.button_l2 = initObj.button_l2
      }
      else {
        this.button_l2 = false;
      }
      if (initObj.hasOwnProperty('button_l3')) {
        this.button_l3 = initObj.button_l3
      }
      else {
        this.button_l3 = false;
      }
      if (initObj.hasOwnProperty('button_r1')) {
        this.button_r1 = initObj.button_r1
      }
      else {
        this.button_r1 = false;
      }
      if (initObj.hasOwnProperty('button_r2')) {
        this.button_r2 = initObj.button_r2
      }
      else {
        this.button_r2 = false;
      }
      if (initObj.hasOwnProperty('button_r3')) {
        this.button_r3 = initObj.button_r3
      }
      else {
        this.button_r3 = false;
      }
      if (initObj.hasOwnProperty('button_share')) {
        this.button_share = initObj.button_share
      }
      else {
        this.button_share = false;
      }
      if (initObj.hasOwnProperty('button_options')) {
        this.button_options = initObj.button_options
      }
      else {
        this.button_options = false;
      }
      if (initObj.hasOwnProperty('button_trackpad')) {
        this.button_trackpad = initObj.button_trackpad
      }
      else {
        this.button_trackpad = false;
      }
      if (initObj.hasOwnProperty('button_ps')) {
        this.button_ps = initObj.button_ps
      }
      else {
        this.button_ps = false;
      }
      if (initObj.hasOwnProperty('lin_acc_x')) {
        this.lin_acc_x = initObj.lin_acc_x
      }
      else {
        this.lin_acc_x = 0;
      }
      if (initObj.hasOwnProperty('lin_acc_y')) {
        this.lin_acc_y = initObj.lin_acc_y
      }
      else {
        this.lin_acc_y = 0;
      }
      if (initObj.hasOwnProperty('lin_acc_z')) {
        this.lin_acc_z = initObj.lin_acc_z
      }
      else {
        this.lin_acc_z = 0;
      }
      if (initObj.hasOwnProperty('ang_vel_x')) {
        this.ang_vel_x = initObj.ang_vel_x
      }
      else {
        this.ang_vel_x = 0;
      }
      if (initObj.hasOwnProperty('ang_vel_y')) {
        this.ang_vel_y = initObj.ang_vel_y
      }
      else {
        this.ang_vel_y = 0;
      }
      if (initObj.hasOwnProperty('ang_vel_z')) {
        this.ang_vel_z = initObj.ang_vel_z
      }
      else {
        this.ang_vel_z = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch0_id')) {
        this.trackpad_touch0_id = initObj.trackpad_touch0_id
      }
      else {
        this.trackpad_touch0_id = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch0_active')) {
        this.trackpad_touch0_active = initObj.trackpad_touch0_active
      }
      else {
        this.trackpad_touch0_active = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch0_x')) {
        this.trackpad_touch0_x = initObj.trackpad_touch0_x
      }
      else {
        this.trackpad_touch0_x = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch0_y')) {
        this.trackpad_touch0_y = initObj.trackpad_touch0_y
      }
      else {
        this.trackpad_touch0_y = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch1_id')) {
        this.trackpad_touch1_id = initObj.trackpad_touch1_id
      }
      else {
        this.trackpad_touch1_id = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch1_active')) {
        this.trackpad_touch1_active = initObj.trackpad_touch1_active
      }
      else {
        this.trackpad_touch1_active = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch1_x')) {
        this.trackpad_touch1_x = initObj.trackpad_touch1_x
      }
      else {
        this.trackpad_touch1_x = 0;
      }
      if (initObj.hasOwnProperty('trackpad_touch1_y')) {
        this.trackpad_touch1_y = initObj.trackpad_touch1_y
      }
      else {
        this.trackpad_touch1_y = 0;
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0;
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = 0;
      }
      if (initObj.hasOwnProperty('plug_usb')) {
        this.plug_usb = initObj.plug_usb
      }
      else {
        this.plug_usb = false;
      }
      if (initObj.hasOwnProperty('plug_audio')) {
        this.plug_audio = initObj.plug_audio
      }
      else {
        this.plug_audio = false;
      }
      if (initObj.hasOwnProperty('plug_mic')) {
        this.plug_mic = initObj.plug_mic
      }
      else {
        this.plug_mic = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Report
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left_analog_x]
    bufferOffset = _serializer.uint8(obj.left_analog_x, buffer, bufferOffset);
    // Serialize message field [left_analog_y]
    bufferOffset = _serializer.uint8(obj.left_analog_y, buffer, bufferOffset);
    // Serialize message field [right_analog_x]
    bufferOffset = _serializer.uint8(obj.right_analog_x, buffer, bufferOffset);
    // Serialize message field [right_analog_y]
    bufferOffset = _serializer.uint8(obj.right_analog_y, buffer, bufferOffset);
    // Serialize message field [l2_analog]
    bufferOffset = _serializer.uint8(obj.l2_analog, buffer, bufferOffset);
    // Serialize message field [r2_analog]
    bufferOffset = _serializer.uint8(obj.r2_analog, buffer, bufferOffset);
    // Serialize message field [dpad_up]
    bufferOffset = _serializer.bool(obj.dpad_up, buffer, bufferOffset);
    // Serialize message field [dpad_down]
    bufferOffset = _serializer.bool(obj.dpad_down, buffer, bufferOffset);
    // Serialize message field [dpad_left]
    bufferOffset = _serializer.bool(obj.dpad_left, buffer, bufferOffset);
    // Serialize message field [dpad_right]
    bufferOffset = _serializer.bool(obj.dpad_right, buffer, bufferOffset);
    // Serialize message field [button_cross]
    bufferOffset = _serializer.bool(obj.button_cross, buffer, bufferOffset);
    // Serialize message field [button_circle]
    bufferOffset = _serializer.bool(obj.button_circle, buffer, bufferOffset);
    // Serialize message field [button_square]
    bufferOffset = _serializer.bool(obj.button_square, buffer, bufferOffset);
    // Serialize message field [button_triangle]
    bufferOffset = _serializer.bool(obj.button_triangle, buffer, bufferOffset);
    // Serialize message field [button_l1]
    bufferOffset = _serializer.bool(obj.button_l1, buffer, bufferOffset);
    // Serialize message field [button_l2]
    bufferOffset = _serializer.bool(obj.button_l2, buffer, bufferOffset);
    // Serialize message field [button_l3]
    bufferOffset = _serializer.bool(obj.button_l3, buffer, bufferOffset);
    // Serialize message field [button_r1]
    bufferOffset = _serializer.bool(obj.button_r1, buffer, bufferOffset);
    // Serialize message field [button_r2]
    bufferOffset = _serializer.bool(obj.button_r2, buffer, bufferOffset);
    // Serialize message field [button_r3]
    bufferOffset = _serializer.bool(obj.button_r3, buffer, bufferOffset);
    // Serialize message field [button_share]
    bufferOffset = _serializer.bool(obj.button_share, buffer, bufferOffset);
    // Serialize message field [button_options]
    bufferOffset = _serializer.bool(obj.button_options, buffer, bufferOffset);
    // Serialize message field [button_trackpad]
    bufferOffset = _serializer.bool(obj.button_trackpad, buffer, bufferOffset);
    // Serialize message field [button_ps]
    bufferOffset = _serializer.bool(obj.button_ps, buffer, bufferOffset);
    // Serialize message field [lin_acc_x]
    bufferOffset = _serializer.int16(obj.lin_acc_x, buffer, bufferOffset);
    // Serialize message field [lin_acc_y]
    bufferOffset = _serializer.int16(obj.lin_acc_y, buffer, bufferOffset);
    // Serialize message field [lin_acc_z]
    bufferOffset = _serializer.int16(obj.lin_acc_z, buffer, bufferOffset);
    // Serialize message field [ang_vel_x]
    bufferOffset = _serializer.int16(obj.ang_vel_x, buffer, bufferOffset);
    // Serialize message field [ang_vel_y]
    bufferOffset = _serializer.int16(obj.ang_vel_y, buffer, bufferOffset);
    // Serialize message field [ang_vel_z]
    bufferOffset = _serializer.int16(obj.ang_vel_z, buffer, bufferOffset);
    // Serialize message field [trackpad_touch0_id]
    bufferOffset = _serializer.uint16(obj.trackpad_touch0_id, buffer, bufferOffset);
    // Serialize message field [trackpad_touch0_active]
    bufferOffset = _serializer.uint16(obj.trackpad_touch0_active, buffer, bufferOffset);
    // Serialize message field [trackpad_touch0_x]
    bufferOffset = _serializer.uint16(obj.trackpad_touch0_x, buffer, bufferOffset);
    // Serialize message field [trackpad_touch0_y]
    bufferOffset = _serializer.uint16(obj.trackpad_touch0_y, buffer, bufferOffset);
    // Serialize message field [trackpad_touch1_id]
    bufferOffset = _serializer.uint16(obj.trackpad_touch1_id, buffer, bufferOffset);
    // Serialize message field [trackpad_touch1_active]
    bufferOffset = _serializer.uint16(obj.trackpad_touch1_active, buffer, bufferOffset);
    // Serialize message field [trackpad_touch1_x]
    bufferOffset = _serializer.uint16(obj.trackpad_touch1_x, buffer, bufferOffset);
    // Serialize message field [trackpad_touch1_y]
    bufferOffset = _serializer.uint16(obj.trackpad_touch1_y, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.uint8(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = _serializer.uint8(obj.battery, buffer, bufferOffset);
    // Serialize message field [plug_usb]
    bufferOffset = _serializer.bool(obj.plug_usb, buffer, bufferOffset);
    // Serialize message field [plug_audio]
    bufferOffset = _serializer.bool(obj.plug_audio, buffer, bufferOffset);
    // Serialize message field [plug_mic]
    bufferOffset = _serializer.bool(obj.plug_mic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Report
    let len;
    let data = new Report(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_analog_x]
    data.left_analog_x = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [left_analog_y]
    data.left_analog_y = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [right_analog_x]
    data.right_analog_x = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [right_analog_y]
    data.right_analog_y = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [l2_analog]
    data.l2_analog = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [r2_analog]
    data.r2_analog = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dpad_up]
    data.dpad_up = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dpad_down]
    data.dpad_down = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dpad_left]
    data.dpad_left = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dpad_right]
    data.dpad_right = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_cross]
    data.button_cross = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_circle]
    data.button_circle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_square]
    data.button_square = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_triangle]
    data.button_triangle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_l1]
    data.button_l1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_l2]
    data.button_l2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_l3]
    data.button_l3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_r1]
    data.button_r1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_r2]
    data.button_r2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_r3]
    data.button_r3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_share]
    data.button_share = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_options]
    data.button_options = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_trackpad]
    data.button_trackpad = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [button_ps]
    data.button_ps = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lin_acc_x]
    data.lin_acc_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [lin_acc_y]
    data.lin_acc_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [lin_acc_z]
    data.lin_acc_z = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ang_vel_x]
    data.ang_vel_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ang_vel_y]
    data.ang_vel_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ang_vel_z]
    data.ang_vel_z = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch0_id]
    data.trackpad_touch0_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch0_active]
    data.trackpad_touch0_active = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch0_x]
    data.trackpad_touch0_x = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch0_y]
    data.trackpad_touch0_y = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch1_id]
    data.trackpad_touch1_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch1_active]
    data.trackpad_touch1_active = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch1_x]
    data.trackpad_touch1_x = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trackpad_touch1_y]
    data.trackpad_touch1_y = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [plug_usb]
    data.plug_usb = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [plug_audio]
    data.plug_audio = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [plug_mic]
    data.plug_mic = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ds4_driver/Report';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec2c37165ced5aec5b7a50d72696b7dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Raw report from DualShock 4
    Header header
    
    # Left: 0, Right: 255
    uint8 left_analog_x
    # Up: 0, Down: 255
    uint8 left_analog_y
    uint8 right_analog_x
    uint8 right_analog_y
    
    # Released: 0, Pressed: 255
    uint8 l2_analog
    # Released: 0, Pressed: 255
    uint8 r2_analog
    
    # Released: 0, Pressed: 1
    bool dpad_up
    bool dpad_down
    bool dpad_left
    bool dpad_right
    bool button_cross
    bool button_circle
    bool button_square
    bool button_triangle
    bool button_l1
    bool button_l2
    bool button_l3
    bool button_r1
    bool button_r2
    bool button_r3
    bool button_share
    bool button_options
    bool button_trackpad
    bool button_ps
    
    # IMU
    int16 lin_acc_x
    int16 lin_acc_y
    int16 lin_acc_z
    int16 ang_vel_x
    int16 ang_vel_y
    int16 ang_vel_z
    
    # Top-left: (0, 0), Bottom-right: (1919, 942)
    uint16 trackpad_touch0_id
    uint16 trackpad_touch0_active
    uint16 trackpad_touch0_x
    uint16 trackpad_touch0_y
    uint16 trackpad_touch1_id
    uint16 trackpad_touch1_active
    uint16 trackpad_touch1_x
    uint16 trackpad_touch1_y
    
    uint8 timestamp
    # Full: 8, Full (and charging): 11
    uint8 battery
    
    # Unused: 0, Plugged: 1
    bool plug_usb
    bool plug_audio
    bool plug_mic
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Report(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_analog_x !== undefined) {
      resolved.left_analog_x = msg.left_analog_x;
    }
    else {
      resolved.left_analog_x = 0
    }

    if (msg.left_analog_y !== undefined) {
      resolved.left_analog_y = msg.left_analog_y;
    }
    else {
      resolved.left_analog_y = 0
    }

    if (msg.right_analog_x !== undefined) {
      resolved.right_analog_x = msg.right_analog_x;
    }
    else {
      resolved.right_analog_x = 0
    }

    if (msg.right_analog_y !== undefined) {
      resolved.right_analog_y = msg.right_analog_y;
    }
    else {
      resolved.right_analog_y = 0
    }

    if (msg.l2_analog !== undefined) {
      resolved.l2_analog = msg.l2_analog;
    }
    else {
      resolved.l2_analog = 0
    }

    if (msg.r2_analog !== undefined) {
      resolved.r2_analog = msg.r2_analog;
    }
    else {
      resolved.r2_analog = 0
    }

    if (msg.dpad_up !== undefined) {
      resolved.dpad_up = msg.dpad_up;
    }
    else {
      resolved.dpad_up = false
    }

    if (msg.dpad_down !== undefined) {
      resolved.dpad_down = msg.dpad_down;
    }
    else {
      resolved.dpad_down = false
    }

    if (msg.dpad_left !== undefined) {
      resolved.dpad_left = msg.dpad_left;
    }
    else {
      resolved.dpad_left = false
    }

    if (msg.dpad_right !== undefined) {
      resolved.dpad_right = msg.dpad_right;
    }
    else {
      resolved.dpad_right = false
    }

    if (msg.button_cross !== undefined) {
      resolved.button_cross = msg.button_cross;
    }
    else {
      resolved.button_cross = false
    }

    if (msg.button_circle !== undefined) {
      resolved.button_circle = msg.button_circle;
    }
    else {
      resolved.button_circle = false
    }

    if (msg.button_square !== undefined) {
      resolved.button_square = msg.button_square;
    }
    else {
      resolved.button_square = false
    }

    if (msg.button_triangle !== undefined) {
      resolved.button_triangle = msg.button_triangle;
    }
    else {
      resolved.button_triangle = false
    }

    if (msg.button_l1 !== undefined) {
      resolved.button_l1 = msg.button_l1;
    }
    else {
      resolved.button_l1 = false
    }

    if (msg.button_l2 !== undefined) {
      resolved.button_l2 = msg.button_l2;
    }
    else {
      resolved.button_l2 = false
    }

    if (msg.button_l3 !== undefined) {
      resolved.button_l3 = msg.button_l3;
    }
    else {
      resolved.button_l3 = false
    }

    if (msg.button_r1 !== undefined) {
      resolved.button_r1 = msg.button_r1;
    }
    else {
      resolved.button_r1 = false
    }

    if (msg.button_r2 !== undefined) {
      resolved.button_r2 = msg.button_r2;
    }
    else {
      resolved.button_r2 = false
    }

    if (msg.button_r3 !== undefined) {
      resolved.button_r3 = msg.button_r3;
    }
    else {
      resolved.button_r3 = false
    }

    if (msg.button_share !== undefined) {
      resolved.button_share = msg.button_share;
    }
    else {
      resolved.button_share = false
    }

    if (msg.button_options !== undefined) {
      resolved.button_options = msg.button_options;
    }
    else {
      resolved.button_options = false
    }

    if (msg.button_trackpad !== undefined) {
      resolved.button_trackpad = msg.button_trackpad;
    }
    else {
      resolved.button_trackpad = false
    }

    if (msg.button_ps !== undefined) {
      resolved.button_ps = msg.button_ps;
    }
    else {
      resolved.button_ps = false
    }

    if (msg.lin_acc_x !== undefined) {
      resolved.lin_acc_x = msg.lin_acc_x;
    }
    else {
      resolved.lin_acc_x = 0
    }

    if (msg.lin_acc_y !== undefined) {
      resolved.lin_acc_y = msg.lin_acc_y;
    }
    else {
      resolved.lin_acc_y = 0
    }

    if (msg.lin_acc_z !== undefined) {
      resolved.lin_acc_z = msg.lin_acc_z;
    }
    else {
      resolved.lin_acc_z = 0
    }

    if (msg.ang_vel_x !== undefined) {
      resolved.ang_vel_x = msg.ang_vel_x;
    }
    else {
      resolved.ang_vel_x = 0
    }

    if (msg.ang_vel_y !== undefined) {
      resolved.ang_vel_y = msg.ang_vel_y;
    }
    else {
      resolved.ang_vel_y = 0
    }

    if (msg.ang_vel_z !== undefined) {
      resolved.ang_vel_z = msg.ang_vel_z;
    }
    else {
      resolved.ang_vel_z = 0
    }

    if (msg.trackpad_touch0_id !== undefined) {
      resolved.trackpad_touch0_id = msg.trackpad_touch0_id;
    }
    else {
      resolved.trackpad_touch0_id = 0
    }

    if (msg.trackpad_touch0_active !== undefined) {
      resolved.trackpad_touch0_active = msg.trackpad_touch0_active;
    }
    else {
      resolved.trackpad_touch0_active = 0
    }

    if (msg.trackpad_touch0_x !== undefined) {
      resolved.trackpad_touch0_x = msg.trackpad_touch0_x;
    }
    else {
      resolved.trackpad_touch0_x = 0
    }

    if (msg.trackpad_touch0_y !== undefined) {
      resolved.trackpad_touch0_y = msg.trackpad_touch0_y;
    }
    else {
      resolved.trackpad_touch0_y = 0
    }

    if (msg.trackpad_touch1_id !== undefined) {
      resolved.trackpad_touch1_id = msg.trackpad_touch1_id;
    }
    else {
      resolved.trackpad_touch1_id = 0
    }

    if (msg.trackpad_touch1_active !== undefined) {
      resolved.trackpad_touch1_active = msg.trackpad_touch1_active;
    }
    else {
      resolved.trackpad_touch1_active = 0
    }

    if (msg.trackpad_touch1_x !== undefined) {
      resolved.trackpad_touch1_x = msg.trackpad_touch1_x;
    }
    else {
      resolved.trackpad_touch1_x = 0
    }

    if (msg.trackpad_touch1_y !== undefined) {
      resolved.trackpad_touch1_y = msg.trackpad_touch1_y;
    }
    else {
      resolved.trackpad_touch1_y = 0
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = 0
    }

    if (msg.plug_usb !== undefined) {
      resolved.plug_usb = msg.plug_usb;
    }
    else {
      resolved.plug_usb = false
    }

    if (msg.plug_audio !== undefined) {
      resolved.plug_audio = msg.plug_audio;
    }
    else {
      resolved.plug_audio = false
    }

    if (msg.plug_mic !== undefined) {
      resolved.plug_mic = msg.plug_mic;
    }
    else {
      resolved.plug_mic = false
    }

    return resolved;
    }
};

module.exports = Report;
