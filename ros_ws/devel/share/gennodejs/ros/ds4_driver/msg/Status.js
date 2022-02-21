// Auto-generated. Do not edit!

// (in-package ds4_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Trackpad = require('./Trackpad.js');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.axis_left_x = null;
      this.axis_left_y = null;
      this.axis_right_x = null;
      this.axis_right_y = null;
      this.axis_l2 = null;
      this.axis_r2 = null;
      this.button_dpad_up = null;
      this.button_dpad_down = null;
      this.button_dpad_left = null;
      this.button_dpad_right = null;
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
      this.imu = null;
      this.battery_percentage = null;
      this.battery_full_charging = null;
      this.touch0 = null;
      this.touch1 = null;
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
      if (initObj.hasOwnProperty('axis_left_x')) {
        this.axis_left_x = initObj.axis_left_x
      }
      else {
        this.axis_left_x = 0.0;
      }
      if (initObj.hasOwnProperty('axis_left_y')) {
        this.axis_left_y = initObj.axis_left_y
      }
      else {
        this.axis_left_y = 0.0;
      }
      if (initObj.hasOwnProperty('axis_right_x')) {
        this.axis_right_x = initObj.axis_right_x
      }
      else {
        this.axis_right_x = 0.0;
      }
      if (initObj.hasOwnProperty('axis_right_y')) {
        this.axis_right_y = initObj.axis_right_y
      }
      else {
        this.axis_right_y = 0.0;
      }
      if (initObj.hasOwnProperty('axis_l2')) {
        this.axis_l2 = initObj.axis_l2
      }
      else {
        this.axis_l2 = 0.0;
      }
      if (initObj.hasOwnProperty('axis_r2')) {
        this.axis_r2 = initObj.axis_r2
      }
      else {
        this.axis_r2 = 0.0;
      }
      if (initObj.hasOwnProperty('button_dpad_up')) {
        this.button_dpad_up = initObj.button_dpad_up
      }
      else {
        this.button_dpad_up = 0;
      }
      if (initObj.hasOwnProperty('button_dpad_down')) {
        this.button_dpad_down = initObj.button_dpad_down
      }
      else {
        this.button_dpad_down = 0;
      }
      if (initObj.hasOwnProperty('button_dpad_left')) {
        this.button_dpad_left = initObj.button_dpad_left
      }
      else {
        this.button_dpad_left = 0;
      }
      if (initObj.hasOwnProperty('button_dpad_right')) {
        this.button_dpad_right = initObj.button_dpad_right
      }
      else {
        this.button_dpad_right = 0;
      }
      if (initObj.hasOwnProperty('button_cross')) {
        this.button_cross = initObj.button_cross
      }
      else {
        this.button_cross = 0;
      }
      if (initObj.hasOwnProperty('button_circle')) {
        this.button_circle = initObj.button_circle
      }
      else {
        this.button_circle = 0;
      }
      if (initObj.hasOwnProperty('button_square')) {
        this.button_square = initObj.button_square
      }
      else {
        this.button_square = 0;
      }
      if (initObj.hasOwnProperty('button_triangle')) {
        this.button_triangle = initObj.button_triangle
      }
      else {
        this.button_triangle = 0;
      }
      if (initObj.hasOwnProperty('button_l1')) {
        this.button_l1 = initObj.button_l1
      }
      else {
        this.button_l1 = 0;
      }
      if (initObj.hasOwnProperty('button_l2')) {
        this.button_l2 = initObj.button_l2
      }
      else {
        this.button_l2 = 0;
      }
      if (initObj.hasOwnProperty('button_l3')) {
        this.button_l3 = initObj.button_l3
      }
      else {
        this.button_l3 = 0;
      }
      if (initObj.hasOwnProperty('button_r1')) {
        this.button_r1 = initObj.button_r1
      }
      else {
        this.button_r1 = 0;
      }
      if (initObj.hasOwnProperty('button_r2')) {
        this.button_r2 = initObj.button_r2
      }
      else {
        this.button_r2 = 0;
      }
      if (initObj.hasOwnProperty('button_r3')) {
        this.button_r3 = initObj.button_r3
      }
      else {
        this.button_r3 = 0;
      }
      if (initObj.hasOwnProperty('button_share')) {
        this.button_share = initObj.button_share
      }
      else {
        this.button_share = 0;
      }
      if (initObj.hasOwnProperty('button_options')) {
        this.button_options = initObj.button_options
      }
      else {
        this.button_options = 0;
      }
      if (initObj.hasOwnProperty('button_trackpad')) {
        this.button_trackpad = initObj.button_trackpad
      }
      else {
        this.button_trackpad = 0;
      }
      if (initObj.hasOwnProperty('button_ps')) {
        this.button_ps = initObj.button_ps
      }
      else {
        this.button_ps = 0;
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('battery_percentage')) {
        this.battery_percentage = initObj.battery_percentage
      }
      else {
        this.battery_percentage = 0.0;
      }
      if (initObj.hasOwnProperty('battery_full_charging')) {
        this.battery_full_charging = initObj.battery_full_charging
      }
      else {
        this.battery_full_charging = 0;
      }
      if (initObj.hasOwnProperty('touch0')) {
        this.touch0 = initObj.touch0
      }
      else {
        this.touch0 = new Trackpad();
      }
      if (initObj.hasOwnProperty('touch1')) {
        this.touch1 = initObj.touch1
      }
      else {
        this.touch1 = new Trackpad();
      }
      if (initObj.hasOwnProperty('plug_usb')) {
        this.plug_usb = initObj.plug_usb
      }
      else {
        this.plug_usb = 0;
      }
      if (initObj.hasOwnProperty('plug_audio')) {
        this.plug_audio = initObj.plug_audio
      }
      else {
        this.plug_audio = 0;
      }
      if (initObj.hasOwnProperty('plug_mic')) {
        this.plug_mic = initObj.plug_mic
      }
      else {
        this.plug_mic = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Status
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [axis_left_x]
    bufferOffset = _serializer.float32(obj.axis_left_x, buffer, bufferOffset);
    // Serialize message field [axis_left_y]
    bufferOffset = _serializer.float32(obj.axis_left_y, buffer, bufferOffset);
    // Serialize message field [axis_right_x]
    bufferOffset = _serializer.float32(obj.axis_right_x, buffer, bufferOffset);
    // Serialize message field [axis_right_y]
    bufferOffset = _serializer.float32(obj.axis_right_y, buffer, bufferOffset);
    // Serialize message field [axis_l2]
    bufferOffset = _serializer.float32(obj.axis_l2, buffer, bufferOffset);
    // Serialize message field [axis_r2]
    bufferOffset = _serializer.float32(obj.axis_r2, buffer, bufferOffset);
    // Serialize message field [button_dpad_up]
    bufferOffset = _serializer.int32(obj.button_dpad_up, buffer, bufferOffset);
    // Serialize message field [button_dpad_down]
    bufferOffset = _serializer.int32(obj.button_dpad_down, buffer, bufferOffset);
    // Serialize message field [button_dpad_left]
    bufferOffset = _serializer.int32(obj.button_dpad_left, buffer, bufferOffset);
    // Serialize message field [button_dpad_right]
    bufferOffset = _serializer.int32(obj.button_dpad_right, buffer, bufferOffset);
    // Serialize message field [button_cross]
    bufferOffset = _serializer.int32(obj.button_cross, buffer, bufferOffset);
    // Serialize message field [button_circle]
    bufferOffset = _serializer.int32(obj.button_circle, buffer, bufferOffset);
    // Serialize message field [button_square]
    bufferOffset = _serializer.int32(obj.button_square, buffer, bufferOffset);
    // Serialize message field [button_triangle]
    bufferOffset = _serializer.int32(obj.button_triangle, buffer, bufferOffset);
    // Serialize message field [button_l1]
    bufferOffset = _serializer.int32(obj.button_l1, buffer, bufferOffset);
    // Serialize message field [button_l2]
    bufferOffset = _serializer.int32(obj.button_l2, buffer, bufferOffset);
    // Serialize message field [button_l3]
    bufferOffset = _serializer.int32(obj.button_l3, buffer, bufferOffset);
    // Serialize message field [button_r1]
    bufferOffset = _serializer.int32(obj.button_r1, buffer, bufferOffset);
    // Serialize message field [button_r2]
    bufferOffset = _serializer.int32(obj.button_r2, buffer, bufferOffset);
    // Serialize message field [button_r3]
    bufferOffset = _serializer.int32(obj.button_r3, buffer, bufferOffset);
    // Serialize message field [button_share]
    bufferOffset = _serializer.int32(obj.button_share, buffer, bufferOffset);
    // Serialize message field [button_options]
    bufferOffset = _serializer.int32(obj.button_options, buffer, bufferOffset);
    // Serialize message field [button_trackpad]
    bufferOffset = _serializer.int32(obj.button_trackpad, buffer, bufferOffset);
    // Serialize message field [button_ps]
    bufferOffset = _serializer.int32(obj.button_ps, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [battery_percentage]
    bufferOffset = _serializer.float32(obj.battery_percentage, buffer, bufferOffset);
    // Serialize message field [battery_full_charging]
    bufferOffset = _serializer.int32(obj.battery_full_charging, buffer, bufferOffset);
    // Serialize message field [touch0]
    bufferOffset = Trackpad.serialize(obj.touch0, buffer, bufferOffset);
    // Serialize message field [touch1]
    bufferOffset = Trackpad.serialize(obj.touch1, buffer, bufferOffset);
    // Serialize message field [plug_usb]
    bufferOffset = _serializer.int32(obj.plug_usb, buffer, bufferOffset);
    // Serialize message field [plug_audio]
    bufferOffset = _serializer.int32(obj.plug_audio, buffer, bufferOffset);
    // Serialize message field [plug_mic]
    bufferOffset = _serializer.int32(obj.plug_mic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Status
    let len;
    let data = new Status(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [axis_left_x]
    data.axis_left_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axis_left_y]
    data.axis_left_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axis_right_x]
    data.axis_right_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axis_right_y]
    data.axis_right_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axis_l2]
    data.axis_l2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [axis_r2]
    data.axis_r2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [button_dpad_up]
    data.button_dpad_up = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_dpad_down]
    data.button_dpad_down = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_dpad_left]
    data.button_dpad_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_dpad_right]
    data.button_dpad_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_cross]
    data.button_cross = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_circle]
    data.button_circle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_square]
    data.button_square = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_triangle]
    data.button_triangle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_l1]
    data.button_l1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_l2]
    data.button_l2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_l3]
    data.button_l3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_r1]
    data.button_r1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_r2]
    data.button_r2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_r3]
    data.button_r3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_share]
    data.button_share = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_options]
    data.button_options = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_trackpad]
    data.button_trackpad = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [button_ps]
    data.button_ps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [battery_percentage]
    data.battery_percentage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery_full_charging]
    data.battery_full_charging = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [touch0]
    data.touch0 = Trackpad.deserialize(buffer, bufferOffset);
    // Deserialize message field [touch1]
    data.touch1 = Trackpad.deserialize(buffer, bufferOffset);
    // Deserialize message field [plug_usb]
    data.plug_usb = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plug_audio]
    data.plug_audio = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plug_mic]
    data.plug_mic = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu);
    return length + 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ds4_driver/Status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '410582f2082b7ace5412b7159665c723';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Human-readable and more ROS-compatible status of the device
    Header header
    
    # Stick
    # Left: 1.0, Right: -1.0
    float32 axis_left_x
    # Up: 1.0, Down: -1.0
    float32 axis_left_y
    float32 axis_right_x
    float32 axis_right_y
    
    # Shoulder buttons [0, 1.0]
    float32 axis_l2
    float32 axis_r2
    
    # Buttons (0: Not pressed, 1: Pressed)
    int32 button_dpad_up
    int32 button_dpad_down
    int32 button_dpad_left
    int32 button_dpad_right
    int32 button_cross
    int32 button_circle
    int32 button_square
    int32 button_triangle
    int32 button_l1
    int32 button_l2
    int32 button_l3
    int32 button_r1
    int32 button_r2
    int32 button_r3
    int32 button_share
    int32 button_options
    int32 button_trackpad
    int32 button_ps
    
    # Gyro
    sensor_msgs/Imu imu
    
    # Battery
    float32 battery_percentage  # [0.0, 1.0]
    int32 battery_full_charging # 0: No, 1: Yes
    
    # Trackpads
    ds4_driver/Trackpad touch0
    ds4_driver/Trackpad touch1
    
    # Plugs
    int32 plug_usb              # 0: No, 1: Yes
    int32 plug_audio            # 0: No, 1: Yes
    int32 plug_mic              # 0: No, 1: Yes
    
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
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: ds4_driver/Trackpad
    # Trackpad message for DualShock 4
    uint16 id       # Touch ID (increments every touch)
    int32 active    # 0: inactive, 1: active
    float32 x       # 0.0: left edge, 1.0: right edge
    float32 y       # 0.0: left edge, 1.0: right edge
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Status(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.axis_left_x !== undefined) {
      resolved.axis_left_x = msg.axis_left_x;
    }
    else {
      resolved.axis_left_x = 0.0
    }

    if (msg.axis_left_y !== undefined) {
      resolved.axis_left_y = msg.axis_left_y;
    }
    else {
      resolved.axis_left_y = 0.0
    }

    if (msg.axis_right_x !== undefined) {
      resolved.axis_right_x = msg.axis_right_x;
    }
    else {
      resolved.axis_right_x = 0.0
    }

    if (msg.axis_right_y !== undefined) {
      resolved.axis_right_y = msg.axis_right_y;
    }
    else {
      resolved.axis_right_y = 0.0
    }

    if (msg.axis_l2 !== undefined) {
      resolved.axis_l2 = msg.axis_l2;
    }
    else {
      resolved.axis_l2 = 0.0
    }

    if (msg.axis_r2 !== undefined) {
      resolved.axis_r2 = msg.axis_r2;
    }
    else {
      resolved.axis_r2 = 0.0
    }

    if (msg.button_dpad_up !== undefined) {
      resolved.button_dpad_up = msg.button_dpad_up;
    }
    else {
      resolved.button_dpad_up = 0
    }

    if (msg.button_dpad_down !== undefined) {
      resolved.button_dpad_down = msg.button_dpad_down;
    }
    else {
      resolved.button_dpad_down = 0
    }

    if (msg.button_dpad_left !== undefined) {
      resolved.button_dpad_left = msg.button_dpad_left;
    }
    else {
      resolved.button_dpad_left = 0
    }

    if (msg.button_dpad_right !== undefined) {
      resolved.button_dpad_right = msg.button_dpad_right;
    }
    else {
      resolved.button_dpad_right = 0
    }

    if (msg.button_cross !== undefined) {
      resolved.button_cross = msg.button_cross;
    }
    else {
      resolved.button_cross = 0
    }

    if (msg.button_circle !== undefined) {
      resolved.button_circle = msg.button_circle;
    }
    else {
      resolved.button_circle = 0
    }

    if (msg.button_square !== undefined) {
      resolved.button_square = msg.button_square;
    }
    else {
      resolved.button_square = 0
    }

    if (msg.button_triangle !== undefined) {
      resolved.button_triangle = msg.button_triangle;
    }
    else {
      resolved.button_triangle = 0
    }

    if (msg.button_l1 !== undefined) {
      resolved.button_l1 = msg.button_l1;
    }
    else {
      resolved.button_l1 = 0
    }

    if (msg.button_l2 !== undefined) {
      resolved.button_l2 = msg.button_l2;
    }
    else {
      resolved.button_l2 = 0
    }

    if (msg.button_l3 !== undefined) {
      resolved.button_l3 = msg.button_l3;
    }
    else {
      resolved.button_l3 = 0
    }

    if (msg.button_r1 !== undefined) {
      resolved.button_r1 = msg.button_r1;
    }
    else {
      resolved.button_r1 = 0
    }

    if (msg.button_r2 !== undefined) {
      resolved.button_r2 = msg.button_r2;
    }
    else {
      resolved.button_r2 = 0
    }

    if (msg.button_r3 !== undefined) {
      resolved.button_r3 = msg.button_r3;
    }
    else {
      resolved.button_r3 = 0
    }

    if (msg.button_share !== undefined) {
      resolved.button_share = msg.button_share;
    }
    else {
      resolved.button_share = 0
    }

    if (msg.button_options !== undefined) {
      resolved.button_options = msg.button_options;
    }
    else {
      resolved.button_options = 0
    }

    if (msg.button_trackpad !== undefined) {
      resolved.button_trackpad = msg.button_trackpad;
    }
    else {
      resolved.button_trackpad = 0
    }

    if (msg.button_ps !== undefined) {
      resolved.button_ps = msg.button_ps;
    }
    else {
      resolved.button_ps = 0
    }

    if (msg.imu !== undefined) {
      resolved.imu = sensor_msgs.msg.Imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new sensor_msgs.msg.Imu()
    }

    if (msg.battery_percentage !== undefined) {
      resolved.battery_percentage = msg.battery_percentage;
    }
    else {
      resolved.battery_percentage = 0.0
    }

    if (msg.battery_full_charging !== undefined) {
      resolved.battery_full_charging = msg.battery_full_charging;
    }
    else {
      resolved.battery_full_charging = 0
    }

    if (msg.touch0 !== undefined) {
      resolved.touch0 = Trackpad.Resolve(msg.touch0)
    }
    else {
      resolved.touch0 = new Trackpad()
    }

    if (msg.touch1 !== undefined) {
      resolved.touch1 = Trackpad.Resolve(msg.touch1)
    }
    else {
      resolved.touch1 = new Trackpad()
    }

    if (msg.plug_usb !== undefined) {
      resolved.plug_usb = msg.plug_usb;
    }
    else {
      resolved.plug_usb = 0
    }

    if (msg.plug_audio !== undefined) {
      resolved.plug_audio = msg.plug_audio;
    }
    else {
      resolved.plug_audio = 0
    }

    if (msg.plug_mic !== undefined) {
      resolved.plug_mic = msg.plug_mic;
    }
    else {
      resolved.plug_mic = 0
    }

    return resolved;
    }
};

module.exports = Status;
