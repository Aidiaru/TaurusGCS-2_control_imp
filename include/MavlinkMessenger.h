#ifndef MAVLINK_MESSENGER_H
#define MAVLINK_MESSENGER_H

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/mavlink_address.h>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <cstdint>

/**
 * @brief The MavlinkMessenger class serves as an interface for MAVLink communication
 * with drones or other MAVLink-enabled devices.
 *
 * This class handles sending and receiving MAVLink messages, manages communication
 * channels, and acts as a bridge between the MAVLink protocol and higher-level
 * application components.
 */
class MavlinkMessenger {
public:
    /**
     * @brief Constructor for MavlinkMessenger.
     * @param mavlink_passthrough A shared pointer to the MavlinkPassthrough instance.
     */
    MavlinkMessenger(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

    /**
     * @brief Destructor for MavlinkMessenger.
     */
    ~MavlinkMessenger();

    /**
     * @brief Initialize the MavlinkMessenger.
     * @return true if initialization was successful, false otherwise.
     */
    bool initialize();

    /**
     * @brief Set the system ID for outgoing messages.
     * @param system_id The system ID to use.
     */
    void setSystemId(uint8_t system_id);

    /**
     * @brief Set the component ID for outgoing messages.
     * @param component_id The component ID to use.
     */
    void setComponentId(uint8_t component_id);

    /**
     * @brief Callback function type for handling received MAVLink messages.
     */
    using MessageCallback = std::function<void(const mavlink_message_t&)>;

    /**
     * @brief Register a callback for a specific MAVLink message ID.
     *
     * This method subscribes to the given message ID using the MavlinkPassthrough API
     * and registers a callback that will be invoked when a message with that ID is received.
     *
     * @param message_id The MAVLink message ID to register the callback for.
     * @param callback The callback function to be called when a message with the given ID is received.
     * @return true if registration was successful, false otherwise.
     */
    bool registerMessageCallback(uint32_t message_id, MessageCallback callback);

    /**
     * @brief Unregister a callback for a specific MAVLink message ID.
     *
     * This method unsubscribes from the given message ID.
     *
     * @param message_id The MAVLink message ID to unregister the callback for.
     * @return true if unregistration was successful, false otherwise.
     */
    bool unregisterMessageCallback(uint32_t message_id);

    /**
     * @brief Send a MAVLink command to a target system and component.
     *
     * @param target_system The target system ID.
     * @param target_component The target component ID.
     * @param command The command ID to send.
     * @param params Parameters for the command (up to 7).
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendCommand(uint8_t target_system, uint8_t target_component, uint16_t command, 
                     const std::vector<float>& params);

    /**
     * @brief Send a raw MAVLink message.
     *
     * @param message The MAVLink message to send.
     * @return true if the message was sent successfully, false otherwise.
     */
    bool sendMessage(const mavlink_message_t& message);

    /**
     * @brief Send a heartbeat message.
     *
     * @param type The system type.
     * @param autopilot The autopilot type.
     * @param base_mode The base mode.
     * @param custom_mode The custom mode.
     * @param system_status The system status.
     * @return true if the heartbeat was sent successfully, false otherwise.
     */
    bool sendHeartbeat(uint8_t type, uint8_t autopilot, uint8_t base_mode, 
                       uint32_t custom_mode, uint8_t system_status);

    /**
     * @brief Request the parameter list from a system.
     *
     * @param target_system The target system ID.
     * @param target_component The target component ID.
     * @return true if the request was sent successfully, false otherwise.
     */
    bool requestParameterList(uint8_t target_system, uint8_t target_component);

    /**
     * @brief Set a parameter on a remote system.
     *
     * @param target_system The target system ID.
     * @param target_component The target component ID.
     * @param param_id The parameter ID.
     * @param param_value The parameter value.
     * @param param_type The parameter type.
     * @return true if the parameter set message was sent successfully, false otherwise.
     */
    bool setParameter(uint8_t target_system, uint8_t target_component,
                      const std::string& param_id, float param_value, uint8_t param_type);

    /**
     * @brief Send a set position target message.
     *
     * @param target_system The target system ID.
     * @param target_component The target component ID.
     * @param coordinate_frame The coordinate frame.
     * @param type_mask Type mask for position target.
     * @param x X position/velocity.
     * @param y Y position/velocity.
     * @param z Z position/velocity.
     * @param vx X velocity.
     * @param vy Y velocity.
     * @param vz Z velocity.
     * @param afx X acceleration.
     * @param afy Y acceleration.
     * @param afz Z acceleration.
     * @param yaw Yaw angle.
     * @param yaw_rate Yaw rate.
     * @return true if the set position target message was sent successfully, false otherwise.
     */
    bool sendSetPositionTarget(uint8_t target_system, uint8_t target_component, 
                               uint8_t coordinate_frame, uint16_t type_mask,
                               float x, float y, float z,
                               float vx, float vy, float vz,
                               float afx, float afy, float afz,
                               float yaw, float yaw_rate);


    // ---- DECODE HELPER FUNCTIONS ----

    /**
     * @brief Helper function to decode a heartbeat message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param heartbeat Pointer to a heartbeat struct where the decoded data will be stored.
     */
    static void decodeHeartbeat(const mavlink_message_t& message, mavlink_heartbeat_t* heartbeat);

    /**
     * @brief Helper function to decode a parameter value message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param param_value Pointer to a param_value struct where the decoded data will be stored.
     */
    static void decodeParamValue(const mavlink_message_t& message, mavlink_param_value_t* param_value);

    /**
     * @brief Helper function to decode a command acknowledgment message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param command_ack Pointer to a command_ack struct where the decoded data will be stored.
     */
    static void decodeCommandAck(const mavlink_message_t& message, mavlink_command_ack_t* command_ack);

    /**
     * @brief Helper function to decode a position message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param position Pointer to a position struct where the decoded data will be stored.
     */
    static void decodeLocalPosition(const mavlink_message_t& message, mavlink_local_position_ned_t* position);

    /**
     * @brief Helper function to decode a global position message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param position Pointer to a global position struct where the decoded data will be stored.
     */
    static void decodeGlobalPosition(const mavlink_message_t& message, mavlink_global_position_int_t* position);

    /**
     * @brief Helper function to decode an attitude message.
     * 
     * @param message The raw MAVLink message to decode.
     * @param attitude Pointer to an attitude struct where the decoded data will be stored.
     */
    static void decodeAttitude(const mavlink_message_t& message, mavlink_attitude_t* attitude);

    // ---- TYPED CALLBACK REGISTRATION HELPERS ----

    /**
     * @brief Register a callback for heartbeat messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded heartbeat data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerHeartbeatCallback(std::function<void(const mavlink_heartbeat_t&)> callback);

    /**
     * @brief Register a callback for parameter value messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded parameter value data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerParamValueCallback(std::function<void(const mavlink_param_value_t&)> callback);

    /**
     * @brief Register a callback for command acknowledgment messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded command acknowledgment data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerCommandAckCallback(std::function<void(const mavlink_command_ack_t&)> callback);

    /**
     * @brief Register a callback for local position messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded local position data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerLocalPositionCallback(std::function<void(const mavlink_local_position_ned_t&)> callback);

    /**
     * @brief Register a callback for global position messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded global position data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerGlobalPositionCallback(std::function<void(const mavlink_global_position_int_t&)> callback);

    /**
     * @brief Register a callback for attitude messages with decoded data.
     * 
     * @param callback The callback function to be called with decoded attitude data.
     * @return true if registration was successful, false otherwise.
     */
    bool registerAttitudeCallback(std::function<void(const mavlink_attitude_t&)> callback);

private:
    std::mutex _mutex; // Protects _initialized, _system_id, _component_id
    std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;
    uint8_t _system_id;
    uint8_t _component_id;
    bool _initialized;

    // Map of message IDs to subscription handles.
    // The MessageHandle type is defined by the MavlinkPassthrough API.
    std::unordered_map<uint32_t, mavsdk::MavlinkPassthrough::MessageHandle> _subscription_handles;

    // Mutex to protect access to _subscription_handles.
    std::mutex _subscription_mutex;
};

#endif // MAVLINK_MESSENGER_H