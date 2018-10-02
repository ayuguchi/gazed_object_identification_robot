// Auto-generated. Do not edit!

// (in-package test_headpose_estimate.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ActionUnit {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.presence = null;
      this.intensity = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('presence')) {
        this.presence = initObj.presence
      }
      else {
        this.presence = 0.0;
      }
      if (initObj.hasOwnProperty('intensity')) {
        this.intensity = initObj.intensity
      }
      else {
        this.intensity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionUnit
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [presence]
    bufferOffset = _serializer.float64(obj.presence, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.float64(obj.intensity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionUnit
    let len;
    let data = new ActionUnit(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [presence]
    data.presence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'test_headpose_estimate/ActionUnit';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa8dca63187d7a12cf7a6c27efaaaa8d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    float64 presence
    float64 intensity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionUnit(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.presence !== undefined) {
      resolved.presence = msg.presence;
    }
    else {
      resolved.presence = 0.0
    }

    if (msg.intensity !== undefined) {
      resolved.intensity = msg.intensity;
    }
    else {
      resolved.intensity = 0.0
    }

    return resolved;
    }
};

module.exports = ActionUnit;
