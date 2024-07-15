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

class Biolocation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.organism_type = null;
      this.whycon_x = null;
      this.whycon_y = null;
      this.whycon_z = null;
    }
    else {
      if (initObj.hasOwnProperty('organism_type')) {
        this.organism_type = initObj.organism_type
      }
      else {
        this.organism_type = '';
      }
      if (initObj.hasOwnProperty('whycon_x')) {
        this.whycon_x = initObj.whycon_x
      }
      else {
        this.whycon_x = 0.0;
      }
      if (initObj.hasOwnProperty('whycon_y')) {
        this.whycon_y = initObj.whycon_y
      }
      else {
        this.whycon_y = 0.0;
      }
      if (initObj.hasOwnProperty('whycon_z')) {
        this.whycon_z = initObj.whycon_z
      }
      else {
        this.whycon_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Biolocation
    // Serialize message field [organism_type]
    bufferOffset = _serializer.string(obj.organism_type, buffer, bufferOffset);
    // Serialize message field [whycon_x]
    bufferOffset = _serializer.float64(obj.whycon_x, buffer, bufferOffset);
    // Serialize message field [whycon_y]
    bufferOffset = _serializer.float64(obj.whycon_y, buffer, bufferOffset);
    // Serialize message field [whycon_z]
    bufferOffset = _serializer.float64(obj.whycon_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Biolocation
    let len;
    let data = new Biolocation(null);
    // Deserialize message field [organism_type]
    data.organism_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [whycon_x]
    data.whycon_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [whycon_y]
    data.whycon_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [whycon_z]
    data.whycon_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.organism_type);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'luminosity_drone/Biolocation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '101e3ee1346387705dd3316e6b7bd9bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string organism_type
    float64 whycon_x
    float64 whycon_y
    float64 whycon_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Biolocation(null);
    if (msg.organism_type !== undefined) {
      resolved.organism_type = msg.organism_type;
    }
    else {
      resolved.organism_type = ''
    }

    if (msg.whycon_x !== undefined) {
      resolved.whycon_x = msg.whycon_x;
    }
    else {
      resolved.whycon_x = 0.0
    }

    if (msg.whycon_y !== undefined) {
      resolved.whycon_y = msg.whycon_y;
    }
    else {
      resolved.whycon_y = 0.0
    }

    if (msg.whycon_z !== undefined) {
      resolved.whycon_z = msg.whycon_z;
    }
    else {
      resolved.whycon_z = 0.0
    }

    return resolved;
    }
};

module.exports = Biolocation;
