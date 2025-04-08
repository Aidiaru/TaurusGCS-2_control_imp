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

    /**
     * @brief Process received MAVLink messages.
     *
     * This method is provided for compatibility but is no longer required
     * since incoming messages are handled via subscription callbacks.
     */
    void processMessages();

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
