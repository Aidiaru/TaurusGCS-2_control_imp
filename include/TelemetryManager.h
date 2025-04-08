#pragma once

#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>

#include <QObject>
#include <QDebug>
#include <QTimer>

using namespace mavsdk;

struct TelemetryData {
    Telemetry::AccelerationFrd m_accelerationFrd;
    Telemetry::ActuatorControlTarget m_actuatorControlTarget;
    Telemetry::ActuatorOutputStatus m_actuatorOutputStatus;
    Telemetry::Altitude m_altitude;
    Telemetry::AngularVelocityBody m_angularVelocityBody;
    Telemetry::AngularVelocityFrd m_angularVelocityFrd;
    Telemetry::Battery m_battery;
    Telemetry::Covariance m_covariance;
    Telemetry::DistanceSensor m_distanceSensor;
    Telemetry::EulerAngle m_eulerAngle;
    Telemetry::FixedwingMetrics m_fixedwingMetrics;
    Telemetry::GpsGlobalOrigin m_gpsGlobalOrigin;
    Telemetry::GpsInfo m_gpsInfo;
    Telemetry::GroundTruth m_groundTruth;
    Telemetry::Heading m_heading;
    Telemetry::Health m_health;
    Telemetry::Imu m_imu;
    Telemetry::MagneticFieldFrd m_magneticFieldFrd;
    Telemetry::Odometry m_odometry;
    Telemetry::Position m_position;
    Telemetry::PositionBody m_positionBody;
    Telemetry::PositionNed m_positionNed;
    Telemetry::PositionVelocityNed m_positionVelocityNed;
    Telemetry::Quaternion m_quaternion;
    Telemetry::RawGps m_rawGps;
    Telemetry::RcStatus m_rcStatus;
    Telemetry::ScaledPressure m_scaledPressure;
    Telemetry::StatusText m_statusText;
    Telemetry::VelocityBody m_velocityBody;
    Telemetry::VelocityNed m_velocityNed;
    Telemetry::FlightMode m_flightMode;
    bool m_inAir;
    Telemetry::LandedState m_landedState;
    bool m_armed;
    Telemetry::VtolState m_vtolState;
    bool m_healthAllOk;
    uint64_t m_unixEpochTime;
    Telemetry::Position m_homePosition;
    Telemetry::Imu m_scaledImu;
    Telemetry::Imu m_rawImu;
};

class TelemetryManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(TelemetryData telemetryData READ telemetryData NOTIFY telemetryDataChanged)
public:
    explicit TelemetryManager(std::shared_ptr<Telemetry> telemetry);
    ~TelemetryManager();

    void start();
    void stop();
    TelemetryData& telemetryData() { return m_telemetryData; }

signals:
    void telemetryDataChanged(TelemetryData telemetryData);

private:
    TelemetryData m_telemetryData;
    std::shared_ptr<Telemetry> m_telemetry;
    QTimer m_stalenessTimer;
    
};

#endif // TELEMETRY_MANAGER_H