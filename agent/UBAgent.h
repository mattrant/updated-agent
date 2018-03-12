#ifndef UBAGENT_H
#define UBAGENT_H

#include <QObject>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QHash>
#include <QWaitCondition>
#include <QSet>
#include <QQueue>
#include <QSharedMemory>

#include "message.h"
#include "builder.h"

#include <QObject>
#include <QGeoCoordinate>

#define TIME_DISCOUNT_FACTOR .95
#define REWARD 10
#define MAX_SPEED 10

#define MAX_ASSIGNMENTS 5
#define NUM_AGENTS 2
#define DELTA_T 5
#define POINT_ZONE 1

class QTimer;

class ArduPilotMegaMAV;

class UBNetwork;
class UBVision;
class Builder;

class Vehicle;

class UBAgent : public QObject
{
    Q_OBJECT
public:
    explicit UBAgent(QObject *parent = 0);

    QVector<QVector<Message*>*> incoming;
    QHash<int,Message*> outgoing;

    QMutex incoming_lock;
    int curr_buff;

    QVector<Location*> task_locations;
    QWaitCondition new_message;

    QSharedMemory shared_mem;

    qint64 start_time;

    Vehicle* m_mav;


public slots:
    void pathChanged(QVector<int> path);
    void startAgent();

    void dataReadyEvent(quint32 srcID, QByteArray data);
    void missionTracker();
    void work();
signals:
    void sendData(quint32 srcId,QByteArray data);
    void startWork();
protected slots:
  void setMAV(Vehicle* mav);

  void vehicleAddedEvent(Vehicle* mav);
  void vehicleRemovedEvent(Vehicle* mav);

  void armedChangedEvent(bool armed);
  void flightModeChangedEvent(QString mode);
public:

    void init_task_locations(unsigned seed);
    void read_in_task_locations();

protected:
    QTimer* m_timer;

    UBNetwork* m_net;
    UBVision* m_sensor;

    QQueue<int> task_list;
    QSet<int> completed_tasks;
    Builder* builder;

    QMutex work_lock;
    bool is_working;
    int current_task;
    QWaitCondition finished_task;

    enum EMissionStage {
            STAGE_IDLE,
            STAGE_BEGIN,
            STAGE_MISSION
        } m_mission_stage;


protected:
    double distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);
    void set_waypoint_to_location(Location &l);
    bool inPointZone(double lat, double lon,double alt);
    void stage_mission();
    void stage_idle();
    void setup();
};

#endif // UBAGENT_H
