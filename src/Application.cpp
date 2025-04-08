#include <Application.h>

#include <QDebug>
#include <QQmlContext>
#include <QLoggingCategory>

Application::Application(int argc, char *argv[]) : QGuiApplication(argc, argv) 
{
    m_swarm = new Swarm();
    m_droneControl = new DroneControl(m_swarm);
}

Application::~Application()
{
    shutdown();
}

void Application::init() 
{
    this->setApplicationName(APP_NAME);
    this->setOrganizationName(ORG_NAME);
    this->setOrganizationDomain(ORG_DOMAIN);

    m_engine = new QQmlApplicationEngine();

    // Make all necessary QML components available
    m_engine->addImportPath("qrc:/");
    
    // Register the custom C++ types with QML
    qmlRegisterType<DroneControl>("TaurusGCS", 1, 0, "DroneControl");
    qRegisterMetaType<FormationType>("FormationType");
    
    // Set context properties
    m_engine->rootContext()->setContextProperty("swarm", m_swarm);
    m_engine->rootContext()->setContextProperty("droneControl", m_droneControl);
    
    // Enable debug output for QML loading issues
    QLoggingCategory::setFilterRules("qt.qml.binding.removal.info=true");
    
    // Main QML file
    const QUrl mainQmlUrl(QStringLiteral("qrc:/qml/Main.qml"));
    
    QObject::connect(
        m_engine, &QQmlApplicationEngine::objectCreated,
        this, [mainQmlUrl](QObject *obj, const QUrl &objUrl) {
            if (!obj && mainQmlUrl == objUrl) {
                qCritical() << "Failed to load QML file:" << objUrl;
                QCoreApplication::exit(-1);
            } else if (obj && mainQmlUrl == objUrl) {
                qDebug() << "Successfully loaded main QML file";
            }
        },
        Qt::QueuedConnection);

    m_engine->load(mainQmlUrl);
}

void Application::shutdown()
{
    delete m_droneControl;
}