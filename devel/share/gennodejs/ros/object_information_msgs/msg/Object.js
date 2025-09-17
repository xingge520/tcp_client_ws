// Auto-generated. Do not edit!

// (in-package object_information_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Object {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detect_sequence = null;
      this.object_total = null;
      this.object_sequence = null;
      this.label = null;
      this.probability = null;
      this.position = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detect_sequence')) {
        this.detect_sequence = initObj.detect_sequence
      }
      else {
        this.detect_sequence = 0;
      }
      if (initObj.hasOwnProperty('object_total')) {
        this.object_total = initObj.object_total
      }
      else {
        this.object_total = 0;
      }
      if (initObj.hasOwnProperty('object_sequence')) {
        this.object_sequence = initObj.object_sequence
      }
      else {
        this.object_sequence = 0;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('probability')) {
        this.probability = initObj.probability
      }
      else {
        this.probability = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Object
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detect_sequence]
    bufferOffset = _serializer.uint64(obj.detect_sequence, buffer, bufferOffset);
    // Serialize message field [object_total]
    bufferOffset = _serializer.uint16(obj.object_total, buffer, bufferOffset);
    // Serialize message field [object_sequence]
    bufferOffset = _serializer.uint16(obj.object_sequence, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [probability]
    bufferOffset = _serializer.float32(obj.probability, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Object
    let len;
    let data = new Object(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detect_sequence]
    data.detect_sequence = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [object_total]
    data.object_total = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [object_sequence]
    data.object_sequence = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [probability]
    data.probability = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.label);
    return length + 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_information_msgs/Object';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d8f12d51af0cd884816fbc62b3e1cd4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Face bounding box with marker positions
    Header header
    uint64 detect_sequence
    uint16 object_total
    uint16 object_sequence
    string label
    float32 probability
    geometry_msgs/Pose position
    geometry_msgs/Vector3 size
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Object(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detect_sequence !== undefined) {
      resolved.detect_sequence = msg.detect_sequence;
    }
    else {
      resolved.detect_sequence = 0
    }

    if (msg.object_total !== undefined) {
      resolved.object_total = msg.object_total;
    }
    else {
      resolved.object_total = 0
    }

    if (msg.object_sequence !== undefined) {
      resolved.object_sequence = msg.object_sequence;
    }
    else {
      resolved.object_sequence = 0
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.probability !== undefined) {
      resolved.probability = msg.probability;
    }
    else {
      resolved.probability = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Pose.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Pose()
    }

    if (msg.size !== undefined) {
      resolved.size = geometry_msgs.msg.Vector3.Resolve(msg.size)
    }
    else {
      resolved.size = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = Object;
