#include "MavlinkMessenger.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstring>


MavlinkMessenger::MavlinkMessenger(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough)
    : _mavlink_passthrough(mavlink_passthrough),
      _system_id(1),
      _component_id(MAV_COMP_ID_MISSIONPLANNER),
      _initialized(false) {
}

MavlinkMessenger::~MavlinkMessenger() {
    if (_initialized && _mavlink_passthrough) {
        std::lock_guard<std::mutex> lock(_subscription_mutex);
        for (auto& pair : _subscription_handles) {
            _mavlink_passthrough->unsubscribe_message(static_cast<uint16_t>(pair.first), pair.second);
        }
        _subscription_handles.clear();
    }
}

bool MavlinkMessenger::initialize() {
    if (!_mavlink_passthrough) {
        std::cerr << "MavlinkPassthrough instance is null" << std::endl;
        return false;
    }
    
    std::lock_guard<std::mutex> lock(_mutex);
    _initialized = true;
    std::cout << "MavlinkMessenger initialized successfully" << std::endl;
    return true;
}

void MavlinkMessenger::setSystemId(uint8_t system_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    _system_id = system_id;
}

void MavlinkMessenger::setComponentId(uint8_t component_id) {
    std::lock_guard<std::mutex> lock(_mutex);
    _component_id = component_id;
}

bool MavlinkMessenger::registerMessageCallback(uint32_t message_id, MessageCallback callback) {
    if (!callback) {
        std::cerr << "Null callback provided for message ID: " << message_id << std::endl;
        return false;
    }
    
    // Subscribe to the message. The lambda is provided directly by the caller.
    auto handle = _mavlink_passthrough->subscribe_message(static_cast<uint16_t>(message_id), callback);
    {
        std::lock_guard<std::mutex> lock(_subscription_mutex);
        _subscription_handles[message_id] = handle;
    }
    return true;
}

bool MavlinkMessenger::unregisterMessageCallback(uint32_t message_id) {
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    auto it = _subscription_handles.find(message_id);
    if (it != _subscription_handles.end()) {
        _mavlink_passthrough->unsubscribe_message(static_cast<uint16_t>(message_id), it->second);
        _subscription_handles.erase(it);
        return true;
    }
    std::cerr << "No subscription found for message ID: " << message_id << std::endl;
    return false;
}

bool MavlinkMessenger::sendCommand(uint8_t target_system, uint8_t target_component, 
                                  uint16_t command, const std::vector<float>& params) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot send command: messenger not initialized" << std::endl;
        return false;
    }

    mavlink_message_t message;
    mavlink_command_long_t cmd_long = {};

    cmd_long.target_system = target_system;
    cmd_long.target_component = target_component;
    cmd_long.command = command;
    cmd_long.confirmation = 0;

    // Copy up to 7 parameters.
    size_t param_count = std::min(params.size(), static_cast<size_t>(7));
    for (size_t i = 0; i < param_count; ++i) {
        // Using a safer approach than pointer arithmetic
        switch (i) {
            case 0: cmd_long.param1 = params[i]; break;
            case 1: cmd_long.param2 = params[i]; break;
            case 2: cmd_long.param3 = params[i]; break;
            case 3: cmd_long.param4 = params[i]; break;
            case 4: cmd_long.param5 = params[i]; break;
            case 5: cmd_long.param6 = params[i]; break;
            case 6: cmd_long.param7 = params[i]; break;
        }
    }
    
    // Fill remaining parameters with NaN.
    for (size_t i = param_count; i < 7; ++i) {
        switch (i) {
            case 0: cmd_long.param1 = NAN; break;
            case 1: cmd_long.param2 = NAN; break;
            case 2: cmd_long.param3 = NAN; break;
            case 3: cmd_long.param4 = NAN; break;
            case 4: cmd_long.param5 = NAN; break;
            case 5: cmd_long.param6 = NAN; break;
            case 6: cmd_long.param7 = NAN; break;
        }
    }

    mavlink_msg_command_long_encode(_system_id, _component_id, &message, &cmd_long);
    return sendMessage(message);
}

bool MavlinkMessenger::sendMessage(const mavlink_message_t& message) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot send message: messenger not initialized" << std::endl;
        return false;
    }
    
    // Use the correct signature with MavlinkAddress and channel
    // Capture message by value for safety
    mavsdk::MavlinkPassthrough::Result result = _mavlink_passthrough->queue_message(
        [message](MavlinkAddress mavlink_address, uint8_t channel) -> mavlink_message_t {
            return message;
        }
    );
    
    // Convert the Result enum to a boolean
    bool success = (result == mavsdk::MavlinkPassthrough::Result::Success);
    if (!success) {
        std::cerr << "Failed to queue message" << std::endl;
    }
    return success;
}

bool MavlinkMessenger::sendHeartbeat(uint8_t type, uint8_t autopilot, uint8_t base_mode, 
                                    uint32_t custom_mode, uint8_t system_status) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot send heartbeat: messenger not initialized" << std::endl;
        return false;
    }
    
    mavlink_message_t message;
    mavlink_heartbeat_t heartbeat = {};

    heartbeat.type = type;
    heartbeat.autopilot = autopilot;
    heartbeat.base_mode = base_mode;
    heartbeat.custom_mode = custom_mode;
    heartbeat.system_status = system_status;

    mavlink_msg_heartbeat_encode(_system_id, _component_id, &message, &heartbeat);
    return sendMessage(message);
}

bool MavlinkMessenger::requestParameterList(uint8_t target_system, uint8_t target_component) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot request parameter list: messenger not initialized" << std::endl;
        return false;
    }
    
    mavlink_message_t message;
    mavlink_param_request_list_t param_request = {};

    param_request.target_system = target_system;
    param_request.target_component = target_component;

    mavlink_msg_param_request_list_encode(_system_id, _component_id, &message, &param_request);
    return sendMessage(message);
}

bool MavlinkMessenger::setParameter(uint8_t target_system, uint8_t target_component,
                                   const std::string& param_id, float param_value, uint8_t param_type) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot set parameter: messenger not initialized" << std::endl;
        return false;
    }
    
    mavlink_message_t message;
    mavlink_param_set_t param_set = {};

    param_set.target_system = target_system;
    param_set.target_component = target_component;
    param_set.param_value = param_value;
    param_set.param_type = param_type;

    // Ensure param_id is null-terminated and fits in the buffer.
    strncpy(param_set.param_id, param_id.c_str(), sizeof(param_set.param_id) - 1);
    param_set.param_id[sizeof(param_set.param_id) - 1] = '\0';

    mavlink_msg_param_set_encode(_system_id, _component_id, &message, &param_set);
    return sendMessage(message);
}

bool MavlinkMessenger::sendSetPositionTarget(uint8_t target_system, uint8_t target_component, 
                                           uint8_t coordinate_frame, uint16_t type_mask,
                                           float x, float y, float z,
                                           float vx, float vy, float vz,
                                           float afx, float afy, float afz,
                                           float yaw, float yaw_rate) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_initialized || !_mavlink_passthrough) {
        std::cerr << "Cannot send position target: messenger not initialized" << std::endl;
        return false;
    }
    
    mavlink_message_t message;
    mavlink_set_position_target_local_ned_t pos_target = {};

    // Get the current time in milliseconds.
    uint32_t time_boot_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());

    pos_target.time_boot_ms = time_boot_ms;
    pos_target.target_system = target_system;
    pos_target.target_component = target_component;
    pos_target.coordinate_frame = coordinate_frame;
    pos_target.type_mask = type_mask;
    pos_target.x = x;
    pos_target.y = y;
    pos_target.z = z;
    pos_target.vx = vx;
    pos_target.vy = vy;
    pos_target.vz = vz;
    pos_target.afx = afx;
    pos_target.afy = afy;
    pos_target.afz = afz;
    pos_target.yaw = yaw;
    pos_target.yaw_rate = yaw_rate;

    mavlink_msg_set_position_target_local_ned_encode(_system_id, _component_id, &message, &pos_target);
    return sendMessage(message);
}

// ---- DECODE HELPER FUNCTIONS IMPLEMENTATION ----

void MavlinkMessenger::decodeHeartbeat(const mavlink_message_t& message, mavlink_heartbeat_t* heartbeat) {
    mavlink_msg_heartbeat_decode(&message, heartbeat);
}

void MavlinkMessenger::decodeParamValue(const mavlink_message_t& message, mavlink_param_value_t* param_value) {
    mavlink_msg_param_value_decode(&message, param_value);
}

void MavlinkMessenger::decodeCommandAck(const mavlink_message_t& message, mavlink_command_ack_t* command_ack) {
    mavlink_msg_command_ack_decode(&message, command_ack);
}

void MavlinkMessenger::decodeLocalPosition(const mavlink_message_t& message, mavlink_local_position_ned_t* position) {
    mavlink_msg_local_position_ned_decode(&message, position);
}

void MavlinkMessenger::decodeGlobalPosition(const mavlink_message_t& message, mavlink_global_position_int_t* position) {
    mavlink_msg_global_position_int_decode(&message, position);
}

void MavlinkMessenger::decodeAttitude(const mavlink_message_t& message, mavlink_attitude_t* attitude) {
    mavlink_msg_attitude_decode(&message, attitude);
}

// ---- TYPED CALLBACK REGISTRATION IMPLEMENTATION ----

bool MavlinkMessenger::registerHeartbeatCallback(std::function<void(const mavlink_heartbeat_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for heartbeat message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_HEARTBEAT, 
        [callback](const mavlink_message_t& message) {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            callback(heartbeat);
        }
    );
}

bool MavlinkMessenger::registerParamValueCallback(std::function<void(const mavlink_param_value_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for param_value message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_PARAM_VALUE, 
        [callback](const mavlink_message_t& message) {
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&message, &param_value);
            callback(param_value);
        }
    );
}

bool MavlinkMessenger::registerCommandAckCallback(std::function<void(const mavlink_command_ack_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for command_ack message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_COMMAND_ACK, 
        [callback](const mavlink_message_t& message) {
            mavlink_command_ack_t command_ack;
            mavlink_msg_command_ack_decode(&message, &command_ack);
            callback(command_ack);
        }
    );
}

bool MavlinkMessenger::registerLocalPositionCallback(std::function<void(const mavlink_local_position_ned_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for local_position_ned message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
        [callback](const mavlink_message_t& message) {
            mavlink_local_position_ned_t position;
            mavlink_msg_local_position_ned_decode(&message, &position);
            callback(position);
        }
    );
}

bool MavlinkMessenger::registerGlobalPositionCallback(std::function<void(const mavlink_global_position_int_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for global_position_int message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 
        [callback](const mavlink_message_t& message) {
            mavlink_global_position_int_t position;
            mavlink_msg_global_position_int_decode(&message, &position);
            callback(position);
        }
    );
}

bool MavlinkMessenger::registerAttitudeCallback(std::function<void(const mavlink_attitude_t&)> callback) {
    if (!callback) {
        std::cerr << "Null callback provided for attitude message" << std::endl;
        return false;
    }
    
    return registerMessageCallback(MAVLINK_MSG_ID_ATTITUDE, 
        [callback](const mavlink_message_t& message) {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&message, &attitude);
            callback(attitude);
        }
    );
}

