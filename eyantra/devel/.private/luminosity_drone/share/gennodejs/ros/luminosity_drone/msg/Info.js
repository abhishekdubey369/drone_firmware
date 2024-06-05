// Auto-generated. Do not edit!

// (in-package luminosity_drone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sub_ms = null;
      this.sub_mst = null;
      this.sub_ls = null;
    }
    else {
      if (initObj.hasOwnProperty('sub_ms')) {
        this.sub_ms = initObj.sub_ms
      }
      else {
        this.sub_ms = 0;
      }
      if (initObj.hasOwnProperty('sub_mst')) {
        this.sub_mst = initObj.sub_mst
      }
      else {
        this.sub_mst = 0;
      }
      if (initObj.hasOwnProperty('sub_ls')) {
        this.sub_ls = initObj.sub_ls
      }
      else {
        this.sub_ls = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Info
    // Serialize message field [sub_ms]
    bufferOffset = _serializer.int32(obj.sub_ms, buffer, bufferOffset);
    // Serialize message field [sub_mst]
    bufferOffset = _serializer.int32(obj.sub_mst, buffer, bufferOffset);
    // Serialize message field [sub_ls]
    bufferOffset = _serializer.int32(obj.sub_ls, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Info
    let len;
    let data = new Info(null);
    // Deserialize message field [sub_ms]
    data.sub_ms = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sub_mst]
    data.sub_mst = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sub_ls]
    data.sub_ls = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'luminosity_drone/Info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe8702018e11ba267a7b52a7fb524fc7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 sub_ms
    int32 sub_mst
    int32 sub_ls
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Info(null);
    if (msg.sub_ms !== undefined) {
      resolved.sub_ms = msg.sub_ms;
    }
    else {
      resolved.sub_ms = 0
    }

    if (msg.sub_mst !== undefined) {
      resolved.sub_mst = msg.sub_mst;
    }
    else {
      resolved.sub_mst = 0
    }

    if (msg.sub_ls !== undefined) {
      resolved.sub_ls = msg.sub_ls;
    }
    else {
      resolved.sub_ls = 0
    }

    return resolved;
    }
};

module.exports = Info;
