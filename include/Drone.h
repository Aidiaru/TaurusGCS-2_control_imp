#pragma once

#ifndef DRONE_H
#define DRONE_H

#include "TelemetryManager.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

struct Drone
{
    std::shared_ptr<mavsdk::Mavsdk> mavsdk;
    std::shared_ptr<mavsdk::System> system;
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough;
#ifdef TELEMETRY_MANAGER_H
    std::shared_ptr<TelemetryManager> telemetry_manager;
#endif
#ifdef MAVLINK_MESSENGER_H
    std::shared_ptr<MavlinkMessenger> mavlink_messenger;
#endif
};

bool init_drone(std::shared_ptr<Drone> drone, const QString& connection_url);

#endif // DRONE_H