#include <TelemetryManager.h>

TelemetryManager::TelemetryManager(std::shared_ptr<Telemetry> telemetry) 
    : m_telemetry(telemetry)
{
    qDebug() << "TelemetryManager created";
}

TelemetryManager::~TelemetryManager()
{
    stop();
    qDebug() << "TelemetryManager destroyed";
}

void TelemetryManager::start()
{
    if (!m_telemetry) {
        qDebug() << "ERROR! Telemetry plugin is not loaded";
        return;
    }

    qDebug() << "TelemetryManager is starting...";

    // Drone’dan telemetry verisi almayı başlat
    m_telemetry->subscribe_battery([this](Telemetry::Battery battery) {
        m_telemetryData.m_battery = battery;
        emit telemetryDataChanged(m_telemetryData);
    });

    m_telemetry->subscribe_position([this](Telemetry::Position position) {
        m_telemetryData.m_position = position;
        emit telemetryDataChanged(m_telemetryData);
    });

    m_telemetry->subscribe_flight_mode([this](Telemetry::FlightMode mode) {
        m_telemetryData.m_flightMode = mode;
        emit telemetryDataChanged(m_telemetryData);
    });

    m_telemetry->subscribe_armed([this](bool armed) {
        m_telemetryData.m_armed = armed;
        emit telemetryDataChanged(m_telemetryData);
    });

    m_telemetry->subscribe_health_all_ok([this](bool health_ok) {
        m_telemetryData.m_healthAllOk = health_ok;
        emit telemetryDataChanged(m_telemetryData);
    });

    qDebug() << "TelemetryManager started!";
}

void TelemetryManager::stop()
{
    if (!m_telemetry) {
        qDebug() << "ERROR! Telemetry plugin is not loaded";
        return;
    }

    qDebug() << "TelemetryManager is stopping...";

    // Telemetry güncellemelerinden ayrıl
    m_telemetry->subscribe_battery(nullptr);
    m_telemetry->subscribe_position(nullptr);
    m_telemetry->subscribe_flight_mode(nullptr);
    m_telemetry->subscribe_armed(nullptr);
    m_telemetry->subscribe_health_all_ok(nullptr);

    qDebug() << "TelemetryManager is stopped!";
}

