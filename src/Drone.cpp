#include <Drone.h>

#include <thread>
#include <chrono>

#define CONNECTION_TIMEOUT 8000 // ms
#define CONNECTION_PERIOD 500 // ms

using namespace mavsdk;

bool init_drone(std::shared_ptr<Drone> drone, const QString& connection_url) {
    qDebug() << "MAVSDK starting...";
    drone->mavsdk = std::make_shared<Mavsdk>(Mavsdk::Configuration(ComponentType::GroundStation));

    qDebug() << "Trying to connect : " << connection_url;
    ConnectionResult connection_result = drone->mavsdk->add_any_connection(connection_url.toStdString());

    if (connection_result != ConnectionResult::Success) {
        qDebug() << "Connection failed";
        return false;
    }

    // Sistem algılanmasını bekle
    qDebug() << "Searching for system...";
    for (int j = 0; j < CONNECTION_TIMEOUT / CONNECTION_PERIOD; ++j) {  // Daha uzun süre beklemek gerekiyordu
        auto systems = drone->mavsdk->systems();
        if (!systems.empty()) {
            drone->system = systems.at(0);
            qDebug() << "System found: " << connection_url;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(CONNECTION_PERIOD));
    }

    if (!drone->system) {
        qDebug() << "ERROR! Connection established but no system found!";
        return false;
    }

    // MAVSDK bileşenlerini başlat
    qDebug() << "Telemetry and Action starting...";
    drone->telemetry = std::make_shared<Telemetry>(drone->system);
    drone->action = std::make_shared<Action>(drone->system);
    drone->telemetry_manager = std::make_unique<TelemetryManager>(drone->telemetry);
    drone->telemetry_manager->start();

    #ifdef MAVLINK_MESSENGER_H
    drone->mavlink_messenger = std::make_unique<MavlinkMessenger>(drone->mavlink_passthrough);
    if (!drone->mavlink_messenger->initialize()) {
        std::cerr << "Failed to initialize MavlinkMessenger" << std::endl;
        return false;
    } 
#endif
    qDebug() << "Drone succesfully initialized!";
    return true;
}